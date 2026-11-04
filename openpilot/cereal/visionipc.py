from enum import IntEnum


class VisionStreamType(IntEnum):
  VISION_STREAM_NARROW_ROAD = 0
  VISION_STREAM_CABIN = 1
  VISION_STREAM_WIDE_ROAD = 2
  VISION_STREAM_MAP = 3
  VISION_STREAM_NARROW_SHORT = 4  # StarPinguPilot: raw short exposures (CameraShortExposure)
  VISION_STREAM_WIDE_SHORT = 5
