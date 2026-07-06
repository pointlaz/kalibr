"""MCAP-backed IMU dataset reader for kalibr.

Mirrors McapImageDatasetReader but for IMU samples. Supports two flavors:

  1. ROS1-encoded MCAP (encoding 'ros1msg'), sensor_msgs/Imu messages.
  2. pointlaz.IMU flatbuffer MCAP (encoding 'flatbuffer',
     schema name 'pointlaz.IMU').

Public surface mirrors BagImuDatasetReader so callers can swap readers based
on input kind.
"""

import glob
import os
import struct

import numpy as np
import aslam_cv as acv
import sm


def _resolve_mcap_files(path):
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not files:
            raise RuntimeError("No *.mcap files found in directory: {0}".format(path))
        return files
    if os.path.isfile(path):
        return [path]
    raise RuntimeError("MCAP path does not exist: {0}".format(path))


def _read_table(buf, root):
    """Common flatbuffer table preamble: returns (offsets_array, slot_count)."""
    soffset = struct.unpack_from("<i", buf, root)[0]
    vt = root - soffset
    vt_size, _tbl_size = struct.unpack_from("<HH", buf, vt)
    slots = (vt_size - 4) // 2
    off = struct.unpack_from("<" + "H" * slots, buf, vt + 4)
    return off, slots


def _parse_imu_sample(buf, sample_root):
    """Parse one pointlaz.IMUSample table at position sample_root.

    IMUSample fields (id order):
      0 timestamp:           ulong
      1 frame_id:            string
      2 linear_acceleration: Vector3 (struct, 3xf64 inline)
      3 angular_velocity:    Vector3 (struct, 3xf64 inline)
      4 orientation:         Quaternion (struct, 4xf64 inline)
      5 orientation_covariance: [double]
    """
    off, slots = _read_table(buf, sample_root)

    def u64(slot_idx):
        o = off[slot_idx] if slot_idx < slots else 0
        return struct.unpack_from("<Q", buf, sample_root + o)[0] if o else 0

    def vec3(slot_idx):
        # Vector3 is a flatbuffer STRUCT: 3 doubles stored inline at the field
        # position (no extra indirection like a table).
        o = off[slot_idx] if slot_idx < slots else 0
        if not o:
            return np.zeros(3, dtype=np.float64)
        x, y, z = struct.unpack_from("<ddd", buf, sample_root + o)
        return np.array([x, y, z], dtype=np.float64)

    return {
        "timestamp_ns": u64(0),
        "alpha": vec3(2),  # linear_acceleration
        "omega": vec3(3),  # angular_velocity
    }


def _parse_pointlaz_imu(buf):
    """Parse a pointlaz.IMU root table (single sample per message)."""
    root = struct.unpack_from("<I", buf, 0)[0]
    return _parse_imu_sample(buf, root)


def _parse_pointlaz_imu_samples(buf):
    """Parse a pointlaz.IMUSamples root table.

    IMUSamples fields:
      0 timestamp: ulong
      1 frame_id:  string
      2 samples:   [IMUSample]

    Returns list of dicts (one per IMUSample).
    """
    root = struct.unpack_from("<I", buf, 0)[0]
    off, slots = _read_table(buf, root)

    samples_field_off = off[2] if 2 < slots else 0
    if not samples_field_off:
        return []

    samples_field_pos = root + samples_field_off
    vec_uoffset = struct.unpack_from("<I", buf, samples_field_pos)[0]
    vec_pos = samples_field_pos + vec_uoffset
    vec_len = struct.unpack_from("<I", buf, vec_pos)[0]

    out = []
    for i in range(vec_len):
        elem_pos = vec_pos + 4 + i * 4
        elem_uoffset = struct.unpack_from("<I", buf, elem_pos)[0]
        sample_root = elem_pos + elem_uoffset
        out.append(_parse_imu_sample(buf, sample_root))
    return out


def _decode_ros1_imu(ros_msg, log_time_ns):
    ts_ns = ros_msg.header.stamp.secs * int(1e9) + ros_msg.header.stamp.nsecs
    if ts_ns == 0:
        ts_ns = log_time_ns
    omega = np.array([ros_msg.angular_velocity.x,
                      ros_msg.angular_velocity.y,
                      ros_msg.angular_velocity.z], dtype=np.float64)
    alpha = np.array([ros_msg.linear_acceleration.x,
                      ros_msg.linear_acceleration.y,
                      ros_msg.linear_acceleration.z], dtype=np.float64)
    return ts_ns, omega, alpha


class McapImuDatasetReaderIterator(object):
    def __init__(self, dataset, indices=None):
        self.dataset = dataset
        self.indices = np.arange(dataset.numMessages()) if indices is None else indices
        self.iter = self.indices.__iter__()

    def __iter__(self):
        return self

    def next(self):
        return self.dataset.getMessage(next(self.iter))

    def __next__(self):
        return self.dataset.getMessage(next(self.iter))


class McapImuDatasetReader(object):
    """Drop-in for BagImuDatasetReader, backed by one or more *.mcap shards."""

    def __init__(self, mcap_path, imutopic, bag_from_to=None,
                 perform_synchronization=False):
        if imutopic is None:
            raise RuntimeError(
                "Please pass in a topic name referring to the IMU stream in the MCAP input\n{0}".format(mcap_path))

        from mcap.reader import make_reader

        self.mcap_path = mcap_path
        self.topic = imutopic
        self.perform_synchronization = perform_synchronization

        files = _resolve_mcap_files(mcap_path)

        # Each entry: (timestamp_ns, omega, alpha) - already decoded.
        self._records = []

        for f in files:
            # First pass: open with no decoder factories, just to learn the
            # encoding of the target topic. Keeps mcap_ros1 (and the ROS1
            # message stack it pulls in) out of the import graph entirely
            # for flatbuffer-only inputs.
            with open(f, "rb") as fh:
                reader = make_reader(fh)
                summary = reader.get_summary()
                target_channels = {ch_id: ch for ch_id, ch in summary.channels.items()
                                   if ch.topic == imutopic}
                if not target_channels:
                    continue

                first_ch = next(iter(target_channels.values()))
                sch = summary.schemas.get(first_ch.schema_id)
                msg_enc = first_ch.message_encoding

                if msg_enc == "flatbuffer" and sch and sch.name == "pointlaz.IMU":
                    for schema, channel, message in reader.iter_messages(topics=[imutopic]):
                        parsed = _parse_pointlaz_imu(message.data)
                        ts = parsed["timestamp_ns"] or message.log_time
                        self._records.append((ts, parsed["omega"], parsed["alpha"]))
                    continue
                elif msg_enc == "flatbuffer" and sch and sch.name == "pointlaz.IMUSamples":
                    # Each MCAP message contains a batch of IMU samples; expand
                    # them to one record each so the rest of kalibr sees a flat
                    # stream like a ROS bag would.
                    for schema, channel, message in reader.iter_messages(topics=[imutopic]):
                        for parsed in _parse_pointlaz_imu_samples(message.data):
                            ts = parsed["timestamp_ns"] or message.log_time
                            self._records.append((ts, parsed["omega"], parsed["alpha"]))
                    continue
                elif msg_enc not in ("ros1", "ros1msg"):
                    raise RuntimeError(
                        "Unsupported MCAP message encoding={0!r} schema={1!r} on topic {2!r}".format(
                            msg_enc, sch.name if sch else None, imutopic))

            # ros1/ros1msg: re-open with the ROS1 decoder, imported lazily here
            # so it's never required for a flatbuffer-only run.
            from mcap_ros1.decoder import DecoderFactory as Ros1DecoderFactory
            with open(f, "rb") as fh:
                reader = make_reader(fh, decoder_factories=[Ros1DecoderFactory()])
                for schema, channel, message, ros_msg in reader.iter_decoded_messages(
                        topics=[imutopic]):
                    ts, omega, alpha = _decode_ros1_imu(ros_msg, message.log_time)
                    self._records.append((ts, omega, alpha))

        if not self._records:
            raise RuntimeError("Could not find topic {0} in {1}.".format(
                imutopic, mcap_path))

        self.indices = np.arange(len(self._records))
        self.indices = self.sortByTime(self.indices)

        if bag_from_to:
            self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)

    def sortByTime(self, indices):
        self.timestamp_corrector = sm.DoubleTimestampCorrector()
        keyed = [(self._records[i][0], i) for i in indices]
        keyed.sort()
        return [i for _, i in keyed]

    def truncateIndicesFromTime(self, indices, bag_from_to):
        timestamps_s = [self._records[i][0] / 1.0e9 for i in indices]
        bagstart = min(timestamps_s)
        baglength = max(timestamps_s) - bagstart
        if bag_from_to[0] >= bag_from_to[1]:
            raise RuntimeError("Bag start time must be bigger than end time.")
        if bag_from_to[0] < 0.0:
            sm.logWarn("Bag start time of {0} s is smaller 0".format(bag_from_to[0]))
        if bag_from_to[1] > baglength:
            sm.logWarn("Bag end time of {0} s is bigger than the total length of {1} s".format(
                bag_from_to[1], baglength))
        valid = [i for i, t in zip(indices, timestamps_s)
                 if (bagstart + bag_from_to[0]) <= t <= (bagstart + bag_from_to[1])]
        sm.logWarn("McapImuDatasetReader: truncated {0} / {1} messages.".format(
            len(indices) - len(valid), len(indices)))
        return valid

    def __iter__(self):
        return self.readDataset()

    def readDataset(self):
        return McapImuDatasetReaderIterator(self, self.indices)

    def readDatasetShuffle(self):
        indices = np.array(self.indices)
        np.random.shuffle(indices)
        return McapImuDatasetReaderIterator(self, indices)

    def numMessages(self):
        return len(self.indices)

    # Compatibility shim: BagImuDatasetReader exposes `index` which callers
    # use as `len(reader.index)`. Provide a list of the same length.
    @property
    def index(self):
        return list(self.indices)

    def getMessage(self, idx):
        ts_ns, omega, alpha = self._records[idx]
        secs, nsecs = divmod(ts_ns, int(1e9))
        timestamp = acv.Time(int(secs), int(nsecs))
        return (timestamp, omega, alpha)
