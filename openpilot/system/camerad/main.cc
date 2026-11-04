#include "system/camerad/cameras/camera_common.h"

#include <unistd.h>

#include <climits>
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
    // Exec the real path, not /proc/self/exe, so the process is still named camerad.
    LOGW("camerad: restarting without staggered HDR");
    fflush(nullptr);  // exec drops whatever stdio hasn't written yet
    char path[PATH_MAX] = {};
    if (readlink("/proc/self/exe", path, sizeof(path) - 1) > 0) execv(path, argv);
    LOGE("camerad: re-exec failed, errno=%d", errno);
    return 1;
  }
  return 0;
}
