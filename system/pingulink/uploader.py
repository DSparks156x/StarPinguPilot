#!/usr/bin/env python3
import json
import os
import random
import requests
import threading
import time
import traceback
import datetime
from collections.abc import Iterator

from cereal import log
import cereal.messaging as messaging
from openpilot.common.utils import get_upload_stream
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
from openpilot.common.swaglog import cloudlog

from openpilot.system.pingulink.api import PingulinkApi
from http.client import HTTPConnection

# Optimize upload chunk size for requests / urllib3 (default is 8KB)
# Setting this to 512KB reduces CPU usage and speeds up large uploads significantly.
HTTPConnection.__init__.__defaults__ = tuple(x if x != 8192 else 512 * 1024 for x in HTTPConnection.__init__.__defaults__)

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.pingulink_upload'
UPLOAD_ATTR_VALUE = b'1'

MAX_UPLOAD_SIZES = {
  "qlog": 25 * 1e6,
  "qcam": 5 * 1e6,
}

allow_sleep = bool(int(os.getenv("UPLOADER_SLEEP", "1")))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


class FakeRequest:
  def __init__(self):
    self.headers = {"Content-Length": "0"}


class FakeResponse:
  def __init__(self):
    self.status_code = 200
    self.request = FakeRequest()


def get_directory_sort(d: str) -> list[str]:
  # ensure old format is sorted sooner
  o = (
    [
      "0",
    ]
    if d.startswith("2024-")
    else [
      "1",
    ]
  )
  return o + [s.rjust(10, '0') for s in d.rsplit('--', 1)]


def listdir_by_creation(d: str) -> list[str]:
  if not os.path.isdir(d):
    return []

  try:
    paths = [f for f in os.listdir(d) if os.path.isdir(os.path.join(d, f))]
    paths = sorted(paths, key=get_directory_sort)
    return paths
  except OSError:
    cloudlog.exception("listdir_by_creation failed")
    return []


def clear_locks(root: str) -> None:
  for logdir in os.listdir(root):
    path = os.path.join(root, logdir)
    try:
      for fname in os.listdir(path):
        if fname.endswith(".lock"):
          os.unlink(os.path.join(path, fname))
    except OSError:
      cloudlog.exception("clear_locks failed")


class Uploader:
  def __init__(self, dongle_id: str, root: str):
    self.dongle_id = dongle_id
    self.api = PingulinkApi(dongle_id)
    self.root = root

    self.params = Params()

    # stats for last successfully uploaded file
    self.last_filename = ""

    self.immediate_folders = ["crash/", "boot/"]
    self.immediate_priority = {"qlog": 0, "qlog.zst": 0, "rlog": 1, "rlog.zst": 1}

  def list_upload_files(self, metered: bool) -> Iterator[tuple[str, str, str]]:
    for logdir in listdir_by_creation(self.root):
      path = os.path.join(self.root, logdir)
      try:
        names = os.listdir(path)
      except OSError:
        continue

      if any(name.endswith(".lock") for name in names):
        continue

      for name in sorted(names, key=lambda n: self.immediate_priority.get(n, 1000)):
        if name not in self.immediate_priority:
          continue

        # RESTRICTION: On LTE (metered), only auto-upload qlogs
        if metered and ("qcamera" in name or "rlog" in name):
          continue

        key = os.path.join(logdir, name)
        fn = os.path.join(path, name)
        # skip files already uploaded
        try:
          ctime = os.path.getctime(fn)
          is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
        except OSError:
          cloudlog.event("pingulink_uploader_getxattr_failed", key=key, fn=fn)
          # deleter could have deleted, so skip
          continue
        if is_uploaded:
          continue

        # limit uploading on metered connections
        if metered:
          dt = datetime.timedelta(hours=12)
          if logdir in self.immediate_folders and (datetime.datetime.now() - datetime.datetime.fromtimestamp(ctime)) < dt:
            continue

          # For pingulink, we might want different metered logic, but for now we follow loggerd
          if name == "qcamera.ts":
            continue

        yield name, key, fn

  def next_file_to_upload(self, metered: bool) -> tuple[str, str, str] | None:
    upload_files = list(self.list_upload_files(metered))

    for name, key, fn in upload_files:
      if any(f in fn for f in self.immediate_folders):
        return name, key, fn

    for name, key, fn in upload_files:
      if name in self.immediate_priority:
        return name, key, fn

    return None

  def do_upload(self, key: str, fn: str):
    url_resp = self.api.get("api/uploads/v1.4/" + self.dongle_id + "/upload_url/", timeout=10, path=key, access_token=self.api.get_token())
    if url_resp is None:
      return None
    if url_resp.status_code == 412:
      return url_resp

    url_resp_json = json.loads(url_resp.text)
    url = url_resp_json['url']
    headers = url_resp_json['headers']
    cloudlog.debug("pingulink_upload_url v1.4 %s %s", url, str(headers))

    if fake_upload:
      return FakeResponse()

    stream = None
    try:
      compress = key.endswith('.zst') and not fn.endswith('.zst')
      stream, file_size = get_upload_stream(fn, compress)
      # Explicitly set Content-Length so requests sends a standard upload
      # instead of chunked transfer encoding, which nginx/uvicorn drops.
      upload_headers = {**headers, "Content-Length": str(file_size)}
      response = self.api.session.put(url, data=stream, headers=upload_headers, timeout=120)
      return response
    except Exception:
      return None
    finally:
      if stream:
        stream.close()

  def upload(self, name: str, key: str, fn: str, network_type: int, metered: bool) -> bool:
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.exception("pingulink_upload: getsize failed")
      return False

    cloudlog.event("pingulink_upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
    print(f"Uploading {key} ({sz} bytes)...")

    if sz == 0:
      # tag files of 0 size as uploaded
      success = True
    elif name in MAX_UPLOAD_SIZES and sz > MAX_UPLOAD_SIZES[name]:
      cloudlog.event("pingulink_uploader_too_large", key=key, fn=fn, sz=sz)
      success = True
    else:
      start_time = time.monotonic()

      stat = None
      last_exc = None
      try:
        stat = self.do_upload(key, fn)
      except Exception as e:
        last_exc = (e, traceback.format_exc())
        print(f"Upload exception: {e}")

      if stat is not None and stat.status_code in (200, 201, 401, 403, 412):
        self.last_filename = fn
        dt = time.monotonic() - start_time
        if stat.status_code == 412:
          cloudlog.event("pingulink_upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        else:
          content_length = int(stat.request.headers.get("Content-Length", 0))
          speed = (content_length / 1e6) / dt
          cloudlog.event(
            "pingulink_upload_success", key=key, fn=fn, sz=sz, content_length=content_length, network_type=network_type, metered=metered, speed=speed
          )
          print(f"Successfully uploaded {key} at {speed:.2f} MB/s")
        success = True
      else:
        success = False
        status = stat.status_code if stat else "No Response"
        print(f"Upload failed for {key}: Status {status}")
        cloudlog.event("pingulink_upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)

    if success:
      # tag file as uploaded
      try:
        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
      except OSError:
        cloudlog.event("pingulink_uploader_setxattr_failed", exc=last_exc, key=key, fn=fn, sz=sz)

    return success

  def step(self, network_type: int, metered: bool) -> bool | None:
    d = self.next_file_to_upload(metered)
    if d is None:
      return None

    name, key, fn = d

    # qlogs and bootlogs need to be compressed before uploading
    if key.endswith(('qlog', 'rlog')) or (key.startswith('boot/') and not key.endswith('.zst')):
      key += ".zst"

    return self.upload(name, key, fn, network_type, metered)

  def handle_pending_uploads(self, network_type: int, metered: bool) -> tuple[int, int]:
    uploaded_count = 0
    try:
      resp = self.api.get_pending_uploads()
      if resp is None or resp.status_code != 200:
        return 0, 0

      pending_requests = resp.json()
      if not pending_requests:
        return 0, 0

      # Get all local directories to match against base route IDs
      local_dirs = [d for d in os.listdir(self.root) if os.path.isdir(os.path.join(self.root, d))]

      already_queued: set[tuple[str, str]] = set()  # (route_dir, file_type) dedup

      for req in pending_requests:
        req_route = req['route_name']  # Exact segment name OR base route name
        file_type = req['file_type']

        # Normalize file_type aliases
        if file_type == "rlog":
          file_type = "rlog.zst"
        elif file_type == "qlog":
          file_type = "qlog.zst"

        allowed_types = {"fcamera.hevc", "rlog.zst", "qcamera.ts", "dcamera.hevc", "ecamera.hevc", "qlog.zst"}
        if file_type not in allowed_types:
          cloudlog.event("pingulink_pending_unknown_type", file_type=file_type)
          continue

        # Determine which local folders to check.
        # Exact segment match takes priority; fall back to base-route prefix for drive-level requests.
        if req_route in local_dirs:
          target_dirs = [req_route]
        else:
          target_dirs = [d for d in local_dirs if d.startswith(req_route + "--") or d == req_route]

        # Check if the requested file actually exists anywhere on the device
        file_exists_anywhere = False
        if target_dirs:
          for route in target_dirs:
            if file_type == "rlog.zst":
              rlog_zst = os.path.join(self.root, route, "rlog.zst")
              rlog_raw = os.path.join(self.root, route, "rlog")
              if os.path.exists(rlog_zst) or os.path.exists(rlog_raw):
                file_exists_anywhere = True
                break
            elif file_type == "qlog.zst":
              qlog_zst = os.path.join(self.root, route, "qlog.zst")
              qlog_raw = os.path.join(self.root, route, "qlog")
              if os.path.exists(qlog_zst) or os.path.exists(qlog_raw):
                file_exists_anywhere = True
                break
            else:
              fn = os.path.join(self.root, route, file_type)
              if os.path.exists(fn):
                file_exists_anywhere = True
                break

        if not file_exists_anywhere:
          print(f"Pending upload request file no longer available on device: {req_route} / {req['file_type']}. Marking as failed.")
          try:
            self.api.post(f"api/uploads/v1.4/{self.dongle_id}/fail", json={
              "route_name": req_route,
              "file_type": req['file_type']
            }, timeout=10)
          except Exception as e:
            print(f"Failed to report upload failure to server: {e}")
          continue

        # ENFORCE NETWORK POLICY: Full cameras are WiFi-only
        if metered and file_type in ("fcamera.hevc", "dcamera.hevc", "ecamera.hevc"):
          continue

        for route in target_dirs:
          dedup_key = (route, file_type)
          if dedup_key in already_queued:
            continue

          # For rlog.zst, also accept the uncompressed rlog on disk — upload() +
          # do_upload() will compress it on the fly via get_upload_stream.
          if file_type == "rlog.zst":
            rlog_zst = os.path.join(self.root, route, "rlog.zst")
            rlog_raw = os.path.join(self.root, route, "rlog")
            if os.path.exists(rlog_zst):
              fn, disk_name = rlog_zst, "rlog.zst"
            elif os.path.exists(rlog_raw):
              fn, disk_name = rlog_raw, "rlog"
            else:
              continue
          elif file_type == "qlog.zst":
            qlog_zst = os.path.join(self.root, route, "qlog.zst")
            qlog_raw = os.path.join(self.root, route, "qlog")
            if os.path.exists(qlog_zst):
              fn, disk_name = qlog_zst, "qlog.zst"
            elif os.path.exists(qlog_raw):
              fn, disk_name = qlog_raw, "qlog"
            else:
              continue
          else:
            fn = os.path.join(self.root, route, file_type)
            disk_name = file_type
            if not os.path.exists(fn):
              continue

          try:
            is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
          except OSError:
            is_uploaded = False
          if is_uploaded:
            continue

          # Key used for backend storage path — always use the canonical .zst name
          key = os.path.join(route, file_type)
          # Compress rlogs on the fly just like step() does
          if disk_name in ("rlog", "qlog"):
            key += ".zst"

          print(f"Executing pending upload request: {key}")
          already_queued.add(dedup_key)
          if self.upload(disk_name, key, fn, network_type, metered):
            uploaded_count += 1
    except Exception as e:
      print(f"handle_pending_uploads failed: {e}")
    return uploaded_count, len(pending_requests)


def main(exit_event: threading.Event | None = None) -> None:
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  clear_locks(Paths.log_root())

  params = Params()
  if not params.get_bool("PingulinkEnable"):
    cloudlog.info("Pingulink uploader disabled")
    return

  dongle_id = params.get("DongleId")
  if dongle_id is None:
    cloudlog.info("pingulink_uploader missing dongle_id")
    raise Exception("pingulink_uploader can't start without dongle id")

  sm = messaging.SubMaster(['deviceState', 'peripheralState'])
  uploader = Uploader(dongle_id, Paths.log_root())

  # Wait for messaging synchronization (up to 5s)
  print("Pingulink uploader: waiting for messaging synchronization...")
  for _ in range(50):
    if exit_event.is_set():
      return
    sm.update(100)
    if sm.valid['peripheralState'] and sm.valid['deviceState']:
      break

  last_onroad_time = time.monotonic() if not params.get_bool("IsOffroad") else 0
  last_pending_check_time = 0
  last_log_time = 0
  last_upload_time = 0
  success = None
  last_active_power = None
  last_network_type = NetworkType.none

  startup_time = time.monotonic()
  backoff = 0.1
  while not exit_event.is_set():
    try:
      sm.update(100)  # Wait up to 100ms for messages

      offroad = params.get_bool("IsOffroad")
      onroad = not offroad
      if onroad:
        last_onroad_time = time.monotonic()

      # Get device state data
      device_state = sm['deviceState']
      peripheral_state = sm['peripheralState']

      network_type = device_state.networkType if not force_wifi else NetworkType.wifi
      if network_type != last_network_type:
        print(f"Pingulink uploader: Network status changed from {last_network_type} to {network_type}")
        backoff = 0.1
        last_pending_check_time = 0  # Force immediate check
        last_network_type = network_type

      network_metered = device_state.networkMetered
      voltage = peripheral_state.voltage

      # USB Heuristic: ~5V (4V-7V range) is USB/Desk power. 11V+ is Car Battery.
      is_usb_power = 4000 < voltage < 7000
      is_wifi = network_type == NetworkType.wifi

      # Determine active cadence: driving (onroad), force_wifi debug env, is_usb_power (desk), or within 10 minutes of last onroad/upload
      time_since_onroad = time.monotonic() - last_onroad_time
      time_since_upload = time.monotonic() - last_upload_time
      is_active_cadence = onroad or force_wifi or is_usb_power or (time_since_onroad < 600) or (time_since_upload < 600)
      is_low_battery = not is_usb_power and 7000 < voltage < 11800  # Car battery low

      # Track active state transitions to report instantly
      power_transitioned = (last_active_power is not None) and (last_active_power != is_active_cadence)
      last_active_power = is_active_cadence

      # Diagnostic logging and status report cadence
      status_interval = 60 if is_active_cadence else 600

      # Report instantly on transition, at startup, or at regular intervals
      if not is_low_battery and (last_log_time == 0 or power_transitioned or (time.monotonic() - last_log_time > status_interval)):
        print(f"Uploader State: onroad={onroad}, wifi={is_wifi}, metered={network_metered}, voltage={voltage / 1000:.2f}V, active={is_active_cadence}")
        last_log_time = time.monotonic()

        # Report status to Pingulink server
        try:
          r = uploader.api.post(
            f"api/devices/{dongle_id}/status",
            json={
              "is_onroad": onroad,
              "is_wifi": is_wifi,
              "is_metered": network_metered,
              "battery_voltage": round(voltage / 1000.0, 2),
              "uploader_syncing": success is not None,
            },
            timeout=5,
          )

          # Update local UI parameters
          if r is not None and r.status_code == 200:
            params.put("PingulinkLastPingStatus", "Success (200)")
          else:
            status_code = r.status_code if r is not None else "Timeout/Offline"
            params.put("PingulinkLastPingStatus", f"Failed ({status_code})")
        except Exception as e:
          params.put("PingulinkLastPingStatus", f"Error: {str(e)[:40]}")
          print(f"Failed to report status: {e}")

        # Update timestamp
        from datetime import datetime

        params.put("PingulinkLastPing", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

      if is_low_battery and offroad:
        cloudlog.warning("Pingulink uploader: low battery, sleeping")
        time.sleep(600)
        continue

      if network_type == NetworkType.none:
        # If we just booted/started (uploader runtime < 120s), or if deviceState is invalid, or if onroad,
        # sleep for a short duration (10s) to allow Wi-Fi to negotiate and connect.
        is_startup = (time.monotonic() - startup_time) < 120
        sleep_duration = 10 if (is_startup or not sm.valid['deviceState'] or not offroad) else 1800
        
        # Sleep responsively
        start_sleep = time.monotonic()
        while time.monotonic() - start_sleep < sleep_duration and not exit_event.is_set():
          sm.update(1000)
          new_network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
          if new_network_type != NetworkType.none:
            print("Pingulink uploader: Network connected during sleep, waking up")
            backoff = 0.1
            last_pending_check_time = 0
            last_network_type = new_network_type
            break
        continue

      # Polling for pending uploads (15s if active, 30m otherwise)
      poll_interval = 15 if is_active_cadence else 1800
      if time.monotonic() - last_pending_check_time > poll_interval:
        uploaded, total_pending = uploader.handle_pending_uploads(network_type.raw, network_metered)
        last_pending_check_time = time.monotonic()
        if uploaded > 0:
          last_upload_time = time.monotonic()

        # Calculate auto-upload queue size (qlogs)
        auto_queue = len(list(uploader.list_upload_files(network_metered)))
        params.put("PingulinkPendingCount", f"{total_pending},{auto_queue}")

      # Try to upload a file from the queue
      # On LTE/Offroad, we still want to upload qlogs, but maybe with a slower cadence than WiFi
      success = uploader.step(network_type.raw, network_metered)
      if success:
        last_upload_time = time.monotonic()

      if success is None:
        # No files to upload
        backoff = 5 if is_active_cadence else 1800
      elif success:
        # Successful upload, reset backoff to stay snappy
        backoff = 0.1
      else:
        # Failed upload (e.g. server error or network drop)
        cloudlog.info("pingulink_upload backoff %r", backoff)
        backoff = min(backoff * 2, 120)

      if allow_sleep:
        sleep_duration = backoff + random.uniform(0, backoff)
        start_sleep = time.monotonic()
        while time.monotonic() - start_sleep < sleep_duration and not exit_event.is_set():
          sm.update(1000)
          new_network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
          if new_network_type != network_type:
            print(f"Pingulink uploader: Network status changed from {network_type} to {new_network_type}, waking up")
            backoff = 0.1
            last_pending_check_time = 0
            last_network_type = new_network_type
            break

    except Exception as e:
      cloudlog.exception("pingulink_uploader_main_loop_exception")
      print(f"Uploader main loop exception: {e}")
      time.sleep(10)


if __name__ == "__main__":
  main()
