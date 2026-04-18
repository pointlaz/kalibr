"""MCAP-backed image dataset reader for kalibr.

Supports two MCAP flavors:

  1. ROS1-encoded MCAP (schema encoding = 'ros1msg'), for files produced by
     standard rosbag->mcap converters. Decoded via mcap-ros1-support and
     routed through the shared _decode_image_msg helper.

  2. pointlaz.Image flatbuffer MCAP (schema encoding = 'flatbuffer',
     schema name = 'pointlaz.Image'), our internal camera acquisition format.
     The image payload is JPEG; we decode it with OpenCV and return grayscale.

Timestamps for flatbuffer messages come from the MCAP record's log_time (which
matches the flatbuffer's internal timestamp field in our dataset).

Public surface mirrors BagImageDatasetReader so initBagDataset can swap
readers based on input kind.
"""

import glob
import os
import struct

import cv2
import cv_bridge
import numpy as np
import aslam_cv as acv
import sm

from .ImageDatasetReader import _decode_image_msg


def _resolve_mcap_files(path):
  if os.path.isdir(path):
    files = sorted(glob.glob(os.path.join(path, "*.mcap")))
    if not files:
      raise RuntimeError("No *.mcap files found in directory: {0}".format(path))
    return files
  if os.path.isfile(path):
    return [path]
  raise RuntimeError("MCAP path does not exist: {0}".format(path))


def _to_native_ros_msg(dyn_msg):
  """Copy a genpy.dynamic ROS1 msg into a real sensor_msgs.msg.* instance."""
  t = getattr(dyn_msg, "_type", None)
  if t == "sensor_msgs/Image":
    from sensor_msgs.msg import Image
    out = Image()
    out.header.seq = dyn_msg.header.seq
    out.header.stamp.secs = dyn_msg.header.stamp.secs
    out.header.stamp.nsecs = dyn_msg.header.stamp.nsecs
    out.header.frame_id = dyn_msg.header.frame_id
    out.height = dyn_msg.height
    out.width = dyn_msg.width
    out.encoding = dyn_msg.encoding
    out.is_bigendian = dyn_msg.is_bigendian
    out.step = dyn_msg.step
    out.data = dyn_msg.data
    return out
  if t == "sensor_msgs/CompressedImage":
    from sensor_msgs.msg import CompressedImage
    out = CompressedImage()
    out.header.seq = dyn_msg.header.seq
    out.header.stamp.secs = dyn_msg.header.stamp.secs
    out.header.stamp.nsecs = dyn_msg.header.stamp.nsecs
    out.header.frame_id = dyn_msg.header.frame_id
    out.format = dyn_msg.format
    out.data = dyn_msg.data
    return out
  return dyn_msg


def _parse_pointlaz_image(buf):
  """Manual flatbuffer reader for the pointlaz.Image schema.

  Schema fields (in id order):
    0 timestamp: ulong
    1 frame_id:  string
    2 width:     ushort
    3 height:    ushort
    4 encoding:  string
    5 step:      uint
    6 data:      [ubyte]

  Returns dict with width, height, encoding, timestamp_ns, data (bytes).
  """
  root = struct.unpack_from("<I", buf, 0)[0]
  soffset = struct.unpack_from("<i", buf, root)[0]
  vt = root - soffset
  vt_size, _tbl_size = struct.unpack_from("<HH", buf, vt)
  slots = (vt_size - 4) // 2
  off = struct.unpack_from("<" + "H" * slots, buf, vt + 4)

  def u64(slot_idx):
    o = off[slot_idx] if slot_idx < slots else 0
    return struct.unpack_from("<Q", buf, root + o)[0] if o else 0

  def u32(slot_idx):
    o = off[slot_idx] if slot_idx < slots else 0
    return struct.unpack_from("<I", buf, root + o)[0] if o else 0

  def u16(slot_idx):
    o = off[slot_idx] if slot_idx < slots else 0
    return struct.unpack_from("<H", buf, root + o)[0] if o else 0

  def string(slot_idx):
    o = off[slot_idx] if slot_idx < slots else 0
    if not o:
      return ""
    field_pos = root + o
    uoff = struct.unpack_from("<I", buf, field_pos)[0]
    sp = field_pos + uoff
    slen = struct.unpack_from("<I", buf, sp)[0]
    return bytes(buf[sp + 4: sp + 4 + slen]).decode("utf-8", errors="replace")

  def vector_bytes(slot_idx):
    o = off[slot_idx] if slot_idx < slots else 0
    if not o:
      return b""
    field_pos = root + o
    uoff = struct.unpack_from("<I", buf, field_pos)[0]
    vp = field_pos + uoff
    vlen = struct.unpack_from("<I", buf, vp)[0]
    return bytes(buf[vp + 4: vp + 4 + vlen])

  return {
    "timestamp_ns": u64(0),
    "frame_id": string(1),
    "width": u16(2),
    "height": u16(3),
    "encoding": string(4),
    "step": u32(5),
    "data": vector_bytes(6),
  }


def _decode_pointlaz_image(parsed, log_time_ns):
  """Decode pointlaz.Image dict into a grayscale uint8 ndarray and timestamp.

  Uses log_time_ns (from the mcap record) if the flatbuffer timestamp is 0
  (the first message in our dataset has a zeroed timestamp field).
  """
  ts_ns = parsed["timestamp_ns"] or log_time_ns
  enc = parsed["encoding"].lower()
  data = parsed["data"]
  if enc in ("jpeg", "jpg", "png"):
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if img is None:
      raise RuntimeError("Failed to decode {0} image for pointlaz.Image".format(enc))
    if img.ndim == 3:
      img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  elif enc in ("mono8", "8uc1"):
    img = np.frombuffer(data, dtype=np.uint8).reshape(parsed["height"], parsed["width"])
  elif enc in ("mono16", "16uc1"):
    img16 = np.frombuffer(data, dtype="<u2").reshape(parsed["height"], parsed["width"])
    img = (img16 / 256).astype(np.uint8)
  else:
    raise RuntimeError("Unsupported pointlaz.Image encoding: {0!r}".format(parsed["encoding"]))

  secs, nsecs = divmod(ts_ns, int(1e9))
  timestamp = acv.Time(secs, nsecs)
  return (timestamp, img, ts_ns)


class McapImageDatasetReaderIterator(object):
  def __init__(self, dataset, indices=None):
    self.dataset = dataset
    self.indices = np.arange(dataset.numImages()) if indices is None else indices
    self.iter = self.indices.__iter__()

  def __iter__(self):
    return self

  def next(self):
    return self.dataset.getImage(next(self.iter))

  def __next__(self):
    return self.dataset.getImage(next(self.iter))


class McapImageDatasetReader(object):
  """Drop-in for BagImageDatasetReader, backed by one or more *.mcap shards."""

  def __init__(self, mcap_path, imagetopic, bag_from_to=None,
               perform_synchronization=False, bag_freq=None):
    if imagetopic is None:
      raise RuntimeError(
          "Please pass in a topic name referring to the image stream in the MCAP input\n{0}".format(mcap_path))

    from mcap.reader import make_reader

    self.mcap_path = mcap_path
    self.topic = imagetopic
    self.perform_synchronization = perform_synchronization
    self.CVB = cv_bridge.CvBridge()

    files = _resolve_mcap_files(mcap_path)

    # Each entry: (kind, payload, log_time_ns)
    # kind='ros1'  -> payload is a decoded ROS1 message.
    # kind='fbs'   -> payload is the dict from _parse_pointlaz_image.
    self._records = []

    from mcap_ros1.decoder import DecoderFactory as Ros1DecoderFactory

    for f in files:
      with open(f, "rb") as fh:
        # Always include the ROS1 decoder so iter_decoded_messages works for
        # ros1-encoded channels; flatbuffer channels are read via the raw
        # iter_messages path below and do not need a decoder.
        reader = make_reader(fh, decoder_factories=[Ros1DecoderFactory()])
        summary = reader.get_summary()

        target_channels = {ch_id: ch for ch_id, ch in summary.channels.items()
                           if ch.topic == imagetopic}
        if not target_channels:
          continue

        # Dispatch by encoding of the first matching channel (a single topic
        # should not mix encodings within one file).
        first_ch = next(iter(target_channels.values()))
        sch = summary.schemas.get(first_ch.schema_id)
        msg_enc = first_ch.message_encoding

        if msg_enc in ("ros1", "ros1msg"):
          for schema, channel, message, ros_msg in reader.iter_decoded_messages(
              topics=[imagetopic]):
            self._records.append(("ros1", ros_msg, message.log_time))
        elif msg_enc == "flatbuffer" and sch and sch.name == "pointlaz.Image":
          for schema, channel, message in reader.iter_messages(topics=[imagetopic]):
            parsed = _parse_pointlaz_image(message.data)
            self._records.append(("fbs", parsed, message.log_time))
        else:
          raise RuntimeError(
              "Unsupported MCAP message encoding={0!r} schema={1!r} on topic {2!r}".format(
                  msg_enc, sch.name if sch else None, imagetopic))

    if not self._records:
      raise RuntimeError("Could not find topic {0} in {1}.".format(
          imagetopic, mcap_path))

    self.indices = np.arange(len(self._records))
    self.indices = self.sortByTime(self.indices)

    if bag_from_to:
      self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)

    if bag_freq:
      self.indices = self.truncateIndicesFromFreq(self.indices, bag_freq)

  def _timestamp_ns(self, idx):
    kind, payload, log_time = self._records[idx]
    if kind == "ros1":
      return payload.header.stamp.secs * int(1e9) + payload.header.stamp.nsecs
    # fbs: prefer flatbuffer timestamp, fall back to log_time.
    return payload["timestamp_ns"] or log_time

  def sortByTime(self, indices):
    self.timestamp_corrector = sm.DoubleTimestampCorrector()
    keyed = [(self._timestamp_ns(i), i) for i in indices]
    keyed.sort()
    return [i for _, i in keyed]

  def truncateIndicesFromTime(self, indices, bag_from_to):
    timestamps_s = [self._timestamp_ns(i) / 1.0e9 for i in indices]
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
    sm.logWarn("McapImageDatasetReader: truncated {0} / {1} images (from-to).".format(
        len(indices) - len(valid), len(indices)))
    return valid

  def truncateIndicesFromFreq(self, indices, freq):
    if freq < 0.0:
      raise RuntimeError("Frequency {0} Hz is smaller 0".format(freq))
    timestamp_last = -1.0
    valid = []
    for i in indices:
      t = self._timestamp_ns(i) / 1.0e9
      if timestamp_last < 0.0 or (t - timestamp_last) >= 1.0 / freq:
        valid.append(i)
        timestamp_last = t
    sm.logWarn("McapImageDatasetReader: truncated {0} / {1} images (frequency)".format(
        len(indices) - len(valid), len(indices)))
    return valid

  def __iter__(self):
    return self.readDataset()

  def readDataset(self):
    return McapImageDatasetReaderIterator(self, self.indices)

  def readDatasetShuffle(self):
    indices = np.array(self.indices)
    np.random.shuffle(indices)
    return McapImageDatasetReaderIterator(self, indices)

  def numImages(self):
    return len(self.indices)

  def getImage(self, idx):
    kind, payload, log_time = self._records[idx]
    if kind == "ros1":
      data = _to_native_ros_msg(payload)
      if self.perform_synchronization:
        timestamp = acv.Time(self.timestamp_corrector.getLocalTime(
            data.header.stamp.to_sec()))
      else:
        timestamp = acv.Time(data.header.stamp.secs,
                             data.header.stamp.nsecs)
      return _decode_image_msg(data, timestamp, self.CVB)
    # fbs
    timestamp, img, _ = _decode_pointlaz_image(payload, log_time)
    return (timestamp, img)
