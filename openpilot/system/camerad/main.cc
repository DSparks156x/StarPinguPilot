#include "system/camerad/cameras/camera_common.h"

#include <unistd.h>

#include <cassert>
#include <cerrno>

#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"

int main(int argc, char *argv[]) {
  // doesn't need RT priority since we're using isolcpus
  int ret = util::set_core_affinity({6});
  assert(ret == 0 || Params().getBool("IsOffroad")); // failure ok while offroad due to offlining cores

  if (camerad_thread()) {
    // everything is closed by now. Same pid, so manager (which doesn't restart native processes) never notices.
    LOGW("camerad: restarting without staggered HDR");
    execv("/proc/self/exe", argv);
    LOGE("camerad: re-exec failed, errno=%d", errno);
    return 1;
  }
  return 0;
}
