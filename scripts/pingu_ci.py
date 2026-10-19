#!/usr/bin/env python3
"""StarPinguPilot CI helper for .github/workflows/build_pingu.yaml. Stdlib only, runs on the bare runner.

  pingu_ci.py build-args            print the ./build arguments for the default (targeted) CI build
  pingu_ci.py guard [BASE [HEAD]]   fail if native sources changed in BASE..HEAD that the default build doesn't cover
  pingu_ci.py verify                after a build: services.h matches services.py, loggerd knows every logged service

Why: the comma runs the committed binaries (launch_chffrplus.sh skips ./build when `prebuilt` exists), so a native
change only reaches the car if CI rebuilds the binary that contains it. The default CI build is a short explicit list
(BUILD_ARGS). The guard makes a change that falls outside the list fail loudly instead of shipping silently stale.

To add a target: append it to BUILD_ARGS and add the source paths it covers to COVERED, with a comment saying why
those paths affect nothing else that ships. Check with `python3 scripts/pingu_ci.py guard <old>..<new>`-style ranges
(see main) against commits that touched those paths.
"""
import fnmatch
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BOT_AUTHOR = "StarPinguPilot Build Bot"

# Default CI build. Flags are expanded by ./build: --params = common/params_pyx.so (+ common/libcommon.a),
# --panda = the 18 panda/board/obj/*.bin.signed, --cereal = cereal/{services.h,libcereal.a,libsocketmaster.a,
# messaging/bridge}.
BUILD_ARGS = [
  "--params",
  "--panda",
  "--cereal",
  "system/loggerd/loggerd",
  "system/loggerd/encoderd",
  "system/loggerd/bootlog",
]

# Files that count as native: compiled, or generating something compiled.
NATIVE = ["*.c", "*.cc", "*.cpp", "*.h", "*.hpp", "*.pyx", "*.pxd", "*.capnp", "SConscript", "SConstruct",
          "cereal/services.py"]

# Native paths that the default build fully covers. A path is covered only if every shipped binary whose behaviour
# it can change is in BUILD_ARGS. Data-only tables (service list, params keys, capnp schema) also land in binaries we
# don't rebuild (pandad, camerad via libsocketmaster/libcommon), but those only look up their own upstream
# services/keys/fields, so stale copies there don't change what they do. Code (socketmaster.cc, messaging.h,
# params.cc, ...) is NOT covered: it changes every daemon that links it.
COVERED = {
  # cereal: service table + schema + bridge
  "cereal/services.py": "--cereal, loggerd/encoderd/bootlog",
  "cereal/services.h": "--cereal, loggerd/encoderd/bootlog",
  "cereal/*.capnp": "--cereal (libcereal.a)",
  "cereal/SConscript": "--cereal",
  "cereal/messaging/bridge.cc": "--cereal (bridge)",
  "cereal/messaging/msgq_to_zmq.*": "--cereal (bridge)",
  # params
  "common/params_keys.h": "--params",
  "common/params_pyx.pyx": "--params",
  # loggerd (only loggerd/encoderd/bootlog are built from here)
  "system/loggerd/*": "system/loggerd/{loggerd,encoderd,bootlog}",
  # panda firmware (includes the opendbc safety headers)
  "panda/*": "--panda",
  "opendbc_repo/opendbc/safety/*": "--panda",
}

# Native paths that don't end up in anything shipped to the comma.
IGNORED = [
  "tools/*",                     # PC tools (cabana, replay), --extras only
  "tinygrad_repo/*",             # not built by scons
  "starpilot/third_party/*",     # vendored Python packages with optional C speedups, not built by scons
  "*/tests/*", "*/test/*",       # test programs, --extras only
  ".github/*",
]


def _match(path, patterns):
  return any(fnmatch.fnmatchcase(path, p) for p in patterns)


def is_native(path):
  return _match(path, NATIVE) or _match(os.path.basename(path), NATIVE)


def classify(paths):
  """Return the native paths that are neither covered nor ignored."""
  return [p for p in paths if is_native(p) and not _match(p, COVERED) and not _match(p, IGNORED)]


def git(*args):
  return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def is_commit(rev):
  if not rev or set(rev) == {"0"}:
    return False
  return subprocess.run(["git", "cat-file", "-e", f"{rev}^{{commit}}"], cwd=ROOT,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def find_base(head, fallback=None):
  # Last bot build commit in HEAD's history: covers pushes whose own run was cancelled or failed.
  base = git("log", "-1", "--format=%H", f"--author={BOT_AUTHOR}", head)
  if base:
    return base, "last build bot commit"
  if is_commit(fallback):
    return fallback, "push 'before'"
  return None, None


def cmd_guard(base=None, head="HEAD"):
  why = "given"
  if base is None:
    base, why = find_base(head, os.environ.get("GUARD_FALLBACK_BASE"))
  if base is None:
    print("::warning::native-source guard skipped: no build bot commit in history and no usable 'before' commit")
    return 0
  # --no-renames: names only, no blob reads (the CI checkout is blobless)
  changed = git("diff", "--no-renames", "--name-only", base, head).splitlines()
  native = [p for p in changed if is_native(p)]
  bad = classify(changed)
  print(f"guard: {base[:10]}..{head} ({why}): {len(changed)} files changed, {len(native)} native, {len(bad)} uncovered")
  for p in native:
    print(f"  {'UNCOVERED' if p in bad else 'ok':9} {p}")
  if bad:
    for p in bad:
      print(f"::error file={p}::native source {p} changed but isn't in the CI build list; add its target to "
            f"scripts/pingu_ci.py (see the note in .github/workflows/build_pingu.yaml) or dispatch with full_build=true")
    return 1
  return 0


def cmd_verify():
  ok = True
  sys.path.insert(0, os.path.join(ROOT, "cereal"))
  import services  # noqa: E402  (cereal/services.py, stdlib only)

  with open(os.path.join(ROOT, "cereal/services.h")) as f:
    if f.read() != services.build_header() + "\n":
      print("::error file=cereal/services.h::cereal/services.h is out of date with cereal/services.py "
            "(python3 cereal/services.py > cereal/services.h)")
      ok = False
    else:
      print("verify: cereal/services.h matches services.py")

  logged = [n for n, s in services.SERVICE_LIST.items() if s.should_log]
  for binary in ["system/loggerd/loggerd"]:
    with open(os.path.join(ROOT, binary), "rb") as f:
      data = f.read()
    # Names of <= 8 bytes can be compiled into immediate stores (std::string SSO), so they never appear as a string;
    # only longer ones are checkable. All fork services (starpilot*, mapd*) are longer.
    # NUL-terminated match: the linker can tail-merge literals ("can" inside "sendcan").
    checkable = [n for n in logged if len(n) > 8]
    missing = [n for n in checkable if n.encode() + b"\0" not in data]
    if missing:
      print(f"::error file={binary}::{binary} doesn't contain logged services {missing}; it would not log them")
      ok = False
    else:
      print(f"verify: {binary} contains all {len(checkable)} logged services longer than 8 chars "
            f"({len(logged) - len(checkable)} shorter ones not checkable)")
  return 0 if ok else 1


def main(argv):
  if not argv:
    print(__doc__)
    return 2
  cmd, rest = argv[0], argv[1:]
  if cmd == "build-args":
    print(" ".join(BUILD_ARGS))
    return 0
  if cmd == "guard":
    if rest and ".." in rest[0]:
      rest = rest[0].split("..", 1)
    return cmd_guard(*rest)
  if cmd == "verify":
    return cmd_verify()
  print(__doc__)
  return 2


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
