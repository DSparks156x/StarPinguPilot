#pragma once

#include "msgq/visionipc/visionbuf.h"

enum VisionStreamValues : VisionStreamType {
  VISION_STREAM_NARROW_ROAD = 0,
  VISION_STREAM_CABIN = 1,
  VISION_STREAM_WIDE_ROAD = 2,
  VISION_STREAM_MAP = 3,
  // StarPinguPilot: raw RAW10 short exposures from camerad's staggered HDR mode (CameraShortExposure), not logged
  VISION_STREAM_NARROW_SHORT = 4,
  VISION_STREAM_WIDE_SHORT = 5,
};
