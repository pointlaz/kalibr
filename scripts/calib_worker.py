#!/usr/bin/env python3
"""Watch-folder calibration worker for the kalibr-slim image.

Polls a jobs directory for calibration job requests and runs them one at a
time. This lets an external stack (e.g. the lazaruss processing modules)
drive kalibr with pure file I/O over a shared volume — no docker exec, no
docker cp, no container orchestration from the client side.

Job protocol — the client creates a subdirectory per job:

    <jobs-dir>/<job-id>/job.yaml

  job.yaml (calibrate_cameras):
    type: calibrate_cameras
    bag: /data/recording.mcap        # path as seen inside this container
    topics: [sensor/camera/cam_1/image, sensor/camera/cam_2/image]
    models: [pinhole-radtan, pinhole-radtan]   # optional, default replicated
    target: /data/april_6x6.yaml
    extra_args: ["--mi-tol", "0.1"]            # optional passthrough
    timeout_s: 7200                            # optional, default none

  job.yaml (calibrate_imu_camera):
    type: calibrate_imu_camera
    bag: /data/recording.mcap
    target: /data/april_6x6.yaml
    camchain: /data/xxx-camchain.yaml
    imu_config: /data/imu_VN100.yaml
    imu_models: [calibrated]                   # optional
    extra_args: []                             # optional
    timeout_s: 14400                           # optional

The worker claims a job by creating status.yaml (exclusive create), symlinks
the input bag into the job dir (kalibr derives output names from the bag
path, so outputs land in the job dir), streams combined output to log.txt,
and finishes by rewriting status.yaml:

    state: done | failed
    exit_code: <int>
    started_at / finished_at: <iso8601 utc>
    outputs: [<files produced in the job dir>]

Client contract: write job.yaml last (or write to a temp name and rename)
so a partially-written spec is never picked up; poll status.yaml.
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys
import time

import yaml

VALID_TYPES = ("calibrate_cameras", "calibrate_imu_camera")

_shutdown = False


def _utcnow():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _log(msg):
    print("[calib-worker {0}] {1}".format(_utcnow(), msg), flush=True)


def _write_status(job_dir, status):
    tmp = os.path.join(job_dir, ".status.yaml.tmp")
    with open(tmp, "w") as f:
        yaml.safe_dump(status, f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, os.path.join(job_dir, "status.yaml"))


def _build_command(spec, job_dir):
    """Validate the job spec and return (argv, error_message)."""
    jtype = spec.get("type")
    if jtype not in VALID_TYPES:
        return None, "unknown job type: {0!r} (expected one of {1})".format(jtype, VALID_TYPES)

    bag = spec.get("bag")
    if not bag or not os.path.exists(bag):
        return None, "bag missing or not found: {0!r}".format(bag)

    target = spec.get("target")
    if not target or not os.path.isfile(target):
        return None, "target missing or not found: {0!r}".format(target)

    # Symlink the bag into the job dir: kalibr writes its outputs next to the
    # bag path it was given, and the job dir is where outputs belong.
    bag_link = os.path.join(job_dir, os.path.basename(bag))
    if not os.path.lexists(bag_link):
        os.symlink(bag, bag_link)

    extra = [str(a) for a in spec.get("extra_args", [])]

    if jtype == "calibrate_cameras":
        topics = spec.get("topics")
        if not topics or not isinstance(topics, list):
            return None, "topics must be a non-empty list"
        models = spec.get("models") or ["pinhole-radtan"] * len(topics)
        if len(models) != len(topics):
            return None, "models length {0} != topics length {1}".format(len(models), len(topics))
        # Optional pre-computed intrinsics camchain (materialized from camchain_yaml/
        # camchain_path). Forward it as --intrinsics so kalibr can load per-camera
        # intrinsics and, with --fix-intrinsics in extra_args, freeze them during the
        # extrinsic solve (fixed-intrinsics profile). Absent = normal re-estimation.
        intrinsics_args = []
        camchain = spec.get("camchain")
        if camchain:
            if not os.path.isfile(camchain):
                return None, "intrinsics camchain not found: {0!r}".format(camchain)
            intrinsics_args = ["--intrinsics", camchain]
        argv = (["kalibr_calibrate_cameras", "--models"] + [str(m) for m in models]
                + ["--target", target, "--bag", bag_link, "--topics"]
                + [str(t) for t in topics]
                + intrinsics_args
                + ["--dont-show-report"] + extra)
        return argv, None

    # calibrate_imu_camera
    camchain = spec.get("camchain")
    if not camchain or not os.path.isfile(camchain):
        return None, "camchain missing or not found: {0!r}".format(camchain)
    imu_config = spec.get("imu_config")
    if not imu_config or not os.path.isfile(imu_config):
        return None, "imu_config missing or not found: {0!r}".format(imu_config)
    imu_models = spec.get("imu_models") or ["calibrated"]
    argv = (["kalibr_calibrate_imu_camera", "--target", target,
             "--imu", imu_config, "--imu-models"] + [str(m) for m in imu_models]
            + ["--cam", camchain, "--bag", bag_link, "--dont-show-report"] + extra)
    return argv, None


def _run_job(job_dir):
    job_file = os.path.join(job_dir, "job.yaml")
    try:
        with open(job_file) as f:
            spec = yaml.safe_load(f)
        if not isinstance(spec, dict):
            raise ValueError("job.yaml is not a mapping")
    except Exception as e:
        _write_status(job_dir, {"state": "failed", "exit_code": -1,
                                "error": "unreadable job.yaml: {0}".format(e),
                                "finished_at": _utcnow()})
        return

    argv, err = _build_command(spec, job_dir)
    if err:
        _log("job {0} rejected: {1}".format(os.path.basename(job_dir), err))
        _write_status(job_dir, {"state": "failed", "exit_code": -1, "error": err,
                                "finished_at": _utcnow()})
        return

    started = _utcnow()
    _write_status(job_dir, {"state": "running", "started_at": started,
                            "command": " ".join(argv)})
    _log("job {0}: {1}".format(os.path.basename(job_dir), " ".join(argv)))

    before = set(os.listdir(job_dir))
    timeout_s = spec.get("timeout_s")
    env = dict(os.environ, MPLBACKEND="Agg")

    log_path = os.path.join(job_dir, "log.txt")
    with open(log_path, "a") as logf:
        try:
            proc = subprocess.run(argv, cwd=job_dir, env=env,
                                  stdout=logf, stderr=subprocess.STDOUT,
                                  timeout=timeout_s)
            exit_code = proc.returncode
            error = None
        except subprocess.TimeoutExpired:
            exit_code = -1
            error = "timed out after {0}s".format(timeout_s)
        except Exception as e:  # missing binary etc.
            exit_code = -1
            error = str(e)

    outputs = sorted(set(os.listdir(job_dir)) - before - {"log.txt", "status.yaml"})
    status = {
        "state": "done" if exit_code == 0 else "failed",
        "exit_code": exit_code,
        "started_at": started,
        "finished_at": _utcnow(),
        "outputs": outputs,
    }
    if error:
        status["error"] = error
    _write_status(job_dir, status)
    _log("job {0} finished: {1} (exit {2})".format(
        os.path.basename(job_dir), status["state"], exit_code))


def _claim(job_dir):
    """Atomically claim a job by creating status.yaml exclusively."""
    try:
        fd = os.open(os.path.join(job_dir, "status.yaml"),
                     os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        return False
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump({"state": "claimed", "claimed_at": _utcnow()}, f)
    return True


def _recover_stale(jobs_dir):
    """Mark jobs left 'running'/'claimed' by a previous worker as failed."""
    for name in sorted(os.listdir(jobs_dir)):
        job_dir = os.path.join(jobs_dir, name)
        status_file = os.path.join(job_dir, "status.yaml")
        if not os.path.isfile(status_file):
            continue
        try:
            with open(status_file) as f:
                status = yaml.safe_load(f) or {}
        except Exception:
            continue
        if status.get("state") in ("running", "claimed"):
            _log("marking stale job {0} as failed (worker restart)".format(name))
            status.update({"state": "failed", "exit_code": -1,
                           "error": "worker restarted while job was running",
                           "finished_at": _utcnow()})
            _write_status(job_dir, status)


def _handle_signal(signum, frame):
    global _shutdown
    _shutdown = True
    _log("received signal {0}, finishing current job then exiting".format(signum))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--jobs-dir", required=True)
    parser.add_argument("--poll-interval", type=float, default=2.0)
    args = parser.parse_args()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    os.makedirs(args.jobs_dir, exist_ok=True)
    _recover_stale(args.jobs_dir)
    _log("watching {0} (poll every {1}s)".format(args.jobs_dir, args.poll_interval))

    while not _shutdown:
        try:
            for name in sorted(os.listdir(args.jobs_dir)):
                job_dir = os.path.join(args.jobs_dir, name)
                if not os.path.isdir(job_dir):
                    continue
                if not os.path.isfile(os.path.join(job_dir, "job.yaml")):
                    continue
                if os.path.exists(os.path.join(job_dir, "status.yaml")):
                    continue
                if _claim(job_dir):
                    _run_job(job_dir)
                if _shutdown:
                    break
        except Exception as e:
            _log("scan error: {0}".format(e))
        time.sleep(args.poll_interval)

    return 0


if __name__ == "__main__":
    sys.exit(main())
