import sm

import numpy as np
import sys
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import time
import copy
import cv2

# Module-level globals set by Pool initializer; lets workers access the
# detector without pickling it on every task.
_worker_detector = None
_worker_clearImages = True
_worker_noTransformation = False


def _init_worker(detector, clearImages, noTransformation):
    global _worker_detector, _worker_clearImages, _worker_noTransformation
    _worker_detector = detector
    _worker_clearImages = clearImages
    _worker_noTransformation = noTransformation


def _extract_one(task):
    idx, stamp, image = task
    if _worker_noTransformation:
        success, obs = _worker_detector.findTargetNoTransformation(stamp, np.array(image))
    else:
        success, obs = _worker_detector.findTarget(stamp, np.array(image))
    if _worker_clearImages:
        obs.clearImage()
    return (idx, success, obs)


def multicoreExtractionWrapper(detector, taskq, resultq, clearImages, noTransformation):
    while 1:
        try:
            task = taskq.get_nowait()
        except queue.Empty:
            return
        idx = task[0]
        stamp = task[1]
        image = task[2]

        if noTransformation:
            success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
        else:
            success, obs = detector.findTarget(stamp, np.array(image))

        if clearImages:
            obs.clearImage()
        if success:
            resultq.put( (obs, idx) )


def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    print("Extracting calibration target corners")
    targetObservations = []
    numImages = dataset.numImages()

    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()

    if multithreading:
        if not numProcesses:
            numProcesses = max(1, multiprocessing.cpu_count()-1)

        # Stream images through Pool.imap_unordered to keep memory bounded.
        # The previous design put all images in a Manager.Queue() upfront,
        # which held ~6GB per camera in a manager process and caused OOM
        # crashes after 3-4 high-res cameras.
        try:
            results = []
            with multiprocessing.Pool(
                processes=numProcesses,
                initializer=_init_worker,
                initargs=(detector, clearImages, noTransformation),
            ) as pool:
                tasks = ((idx, stamp, image)
                         for idx, (stamp, image) in enumerate(dataset.readDataset()))
                for idx, success, obs in pool.imap_unordered(_extract_one, tasks, chunksize=4):
                    if success:
                        results.append((idx, obs))
                    iProgress.sample()
        except Exception as e:
            raise RuntimeError("Exception during multithreaded extraction: {0}".format(e))

        # sort by time index
        results.sort(key=lambda x: x[0])
        targetObservations = [obs for _, obs in results]

    #single threaded implementation
    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

    if len(targetObservations) == 0:
        print("\r")
        sm.logFatal("No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(dataset.topic))
    else:
        print("\r  Extracted corners for %d images (of %d images)                              " % (len(targetObservations), numImages))

    #close all opencv windows that might be open
    cv2.destroyAllWindows()

    return targetObservations
