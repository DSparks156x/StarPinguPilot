#!/usr/bin/env python3
import sys
import time
import argparse
import threading
import os
import csv
from subprocess import check_output, CalledProcessError

from panda import Panda
from opendbc.can import CANPacker, CANParser
from opendbc.car.structs import CarParams

# HCA Status values to strings
HCA_STATES = {
  0: "disabled",
  1: "initializing",
  2: "fault",
  3: "ready",
  4: "rejected",
  5: "active",
  7: "active"
}

BIG_FONT = {
  '0': ["  000  ", " 0   0 ", " 0   0 ", " 0   0 ", "  000  "],
  '1': ["   11  ", "  111  ", "   11  ", "   11  ", "  11111"],
  '2': [" 2222  ", "2    2 ", "   22  ", " 22    ", "222222 "],
  '3': [" 33333 ", "     3 ", "  333  ", "     3 ", " 33333 "],
  '4': [" 4   4 ", " 4   4 ", " 44444 ", "     4 ", "     4 "],
  '5': [" 55555 ", " 5     ", " 5555  ", "     5 ", " 5555  "],
  '6': ["  666  ", " 6     ", " 6666  ", " 6   6 ", "  666  "],
  '7': [" 77777 ", "    7  ", "   7   ", "  7    ", " 7     "],
  '8': ["  888  ", " 8   8 ", "  888  ", " 8   8 ", "  888  "],
  '9': ["  999  ", " 9   9 ", "  9999 ", "     9 ", "  999  "],
  '.': ["       ", "       ", "       ", "  ••   ", "  ••   "],
  '-': ["       ", "       ", " ----- ", "       ", "       "],
  '+': ["   +   ", "   +   ", " +++++ ", "   +   ", "   +   "],
  ':': ["   ••  ", "   ••  ", "       ", "   ••  ", "   ••  "],
  ' ': ["       ", "       ", "       ", "       ", "       "],
  'A': ["  AAA  ", " A   A ", " AAAAA ", " A   A ", " A   A "],
  'C': ["  CCC  ", " C     ", " C     ", " C     ", "  CCC  "],
  'D': [" DDDD  ", " D   D ", " D   D ", " D   D ", " DDDD  "],
  'E': [" EEEEE ", " E     ", " EEE   ", " E     ", " EEEEE "],
  'G': ["  GGG  ", " G     ", " G  GG ", " G   G ", "  GGG  "],
  'L': [" L     ", " L     ", " L     ", " L     ", " LLLLL "],
  'M': [" M   M ", " MM MM ", " M M M ", " M   M ", " M   M "],
  'N': [" N   N ", " NN  N ", " N N N ", " N  NN ", " N   N "],
  'o': ["  ooo  ", " o   o ", "  ooo  ", "       ", "       "],
  'S': ["  SSS  ", " S     ", "  SSS  ", "     S ", "  SSS  "],
  'T': [" TTTTT ", "   T   ", "   T   ", "   T   ", "   T   "],
  'U': [" U   U ", " U   U ", " U   U ", " U   U ", "  UUU  "],
  'V': [" V   V ", " V   V ", " V   V ", "  V V  ", "   V   "],
  'c': ["       ", "  ccc  ", " c     ", " c     ", "  ccc  "],
  'd': ["     d ", "  dddd ", " d   d ", " d   d ", "  dddd "],
  'e': ["       ", "  eee  ", " e   e ", " eeeee ", "  eee  "],
  'g': ["  ggg  ", " g   g ", "  gggg ", "     g ", "  ggg  "],
  'l': ["   ll  ", "   ll  ", "   ll  ", "   ll  ", "   ll  "],
  'm': ["       ", " m m m ", " m m m ", " m m m ", " m m m "],
  'n': ["       ", "  nnn  ", " n   n ", " n   n ", " n   n "],
  'r': ["       ", "  rrr  ", " r   r ", " r     ", " r     "],
  's': ["       ", "  sss  ", " s     ", "  sss  ", "     s "],
  't': ["   t   ", "  ttt  ", "   t   ", "   t   ", "   tt  "],
  'u': ["       ", " u   u ", " u   u ", " u  uu ", "  uu u "],
  'v': ["       ", " v   v ", " v   v ", "  v v  ", "   v   "],
  '/': ["    /  ", "   /   ", "  /    ", " /     ", "/      "],
}


def check_pandad():
  try:
    check_output(["pidof", "pandad"])
    print("\033[91mpandad is running, please kill openpilot before running this script! (aborted)\033[0m")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1: # 1 == no process found (pandad not running)
      raise e
  except FileNotFoundError:
    pass

class AudiSteeringTester:
  def __init__(self, bus=0):
    self.bus = bus
    self.panda = None
    self.packer = CANPacker("vw_pq")
    self.parser = CANParser("vw_pq", [
      ("Lenkwinkel_1", 100),
      ("Lenkhilfe_1", 50),
      ("Lenkhilfe_2", 50),
      ("Lenkhilfe_3", 50),
      ("Kombi_1", 100),
    ], self.bus)
    self.keepalive_thread = None
    self.keepalive_active = False
    self.keepalive_type = None
    self.keepalive_val = 0
    self.esp_stat = 0
    self.bremsmoment = 0
    self.keepalive_torque = 0
    self.keepalive_direction = 0
    self.keepalive_status = 3
    self.last_sent_torque = 0
    self.same_torque_frames = 0
    self.daemon_running = False
    self.daemon_thread = None

  def start_keepalive(self, keepalive_type, val=0, esp_stat=0, bremsmoment=0):
    self.keepalive_type = keepalive_type
    self.keepalive_val = val
    self.esp_stat = esp_stat
    self.bremsmoment = bremsmoment
    self.keepalive_torque = 0
    self.keepalive_direction = 0
    self.keepalive_status = 3 if keepalive_type == "HCA" else 0
    self.keepalive_active = True

  def stop_keepalive(self):
    self.keepalive_active = False

  def send_hca_cmd(self, torque, direction, status):
    # stuck-torque watchdog mitigation:
    # VAG EPS will fault if commanded a constant non-zero torque for too long.
    # We reset the watchdog timer by dither-jittering the torque command by 1 cNm every 15 frames (150ms).
    if torque != 0 and torque == self.last_sent_torque:
      self.same_torque_frames += 1
      if self.same_torque_frames >= 15:
        # Subtract 1 cNm to trigger a value change for the rack's watchdog timer
        torque = max(0, torque - 1)
        self.same_torque_frames = 0
    else:
      self.same_torque_frames = 0

    self.last_sent_torque = torque

    addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
      "LM_Offset": torque,
      "LM_OffSign": direction,
      "HCA_Status": status,
      "Vib_Freq": 16,
      "Vib_Amp": 0,
    })
    try:
      self.panda.can_send(addr, dat, bus)
    except Exception:
      pass

  def make_pla_msg(self, status, angle, sign, esp_stat, bremsmoment):
    addr, dat, bus = self.packer.make_can_msg("PLA_1", self.bus, {
      "PL1_Status_EPS": status,
      "PL1_ArcAngleReq": angle,
      "PL1_AngleReqSign": sign,
      "PL1_Stat_PLA_ESP": esp_stat,
      "PL1_Bremsmoment": bremsmoment,
    })
    return addr, dat, bus


  def _bg_daemon_loop(self):
    last_hca_send = 0
    last_pla_send = 0
    while self.daemon_running:
      now = time.perf_counter()
      
      # Keep parser updated and prevent socket buffer backlog
      try:
        self.update_parser()
      except Exception:
        pass
        
      if self.keepalive_active:
        if self.keepalive_type == "HCA":
          if now - last_hca_send >= 0.01:
            self.send_hca_cmd(self.keepalive_torque, self.keepalive_direction, self.keepalive_status)
            last_hca_send = now
        elif self.keepalive_type == "PLA":
          if now - last_pla_send >= 0.02:
            addr, dat, bus = self.make_pla_msg(
              self.keepalive_val, 0.0, 0, self.esp_stat, self.bremsmoment
            )
            try:
              self.panda.can_send(addr, dat, bus)
            except Exception:
              pass
            last_pla_send = now
            
      time.sleep(0.005)

  def connect(self):
    print("Connecting to Panda...")
    try:
      self.panda = Panda()
      self.panda.set_safety_mode(CarParams.SafetyModel.allOutput)
      print("Panda connected successfully. Safety mode set to ALLOUTPUT.")
      
      # Start the background CAN and keepalive daemon thread
      self.daemon_running = True
      self.daemon_thread = threading.Thread(target=self._bg_daemon_loop)
      self.daemon_thread.daemon = True
      self.daemon_thread.start()
    except Exception as e:
      print(f"\033[91mError connecting to Panda: {e}\033[0m")
      print("Please check that the Panda is connected via USB and ignition is ON.")
      sys.exit(1)

  def update_parser(self):
    # If the background daemon thread is active, let it handle CAN parsing
    # to avoid thread-unsafe race conditions on the C++ CANParser and Panda RX buffer.
    if self.daemon_running and threading.current_thread() != self.daemon_thread:
      return []

    msgs = self.panda.can_recv() or []
    if msgs:
      frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
      self.parser.update([(time.time_ns(), frames)])
    return msgs

  def run_pla_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("            PLA STATUS 8 VALIDATION TEST            ")
    print("====================================================")
    
    # Allow user to configure parameters interactively
    try:
      esp_stat_input = input("Enter PL1_Stat_PLA_ESP to send (0-15, default: 0): ").strip()
      esp_stat = int(esp_stat_input) if esp_stat_input else 0
      
      handshake_input = input("Enable PLA handshake initialization (1 -> 8)? (y/N): ").strip().lower()
      use_handshake = handshake_input.startswith('y')
    except ValueError:
      print("Invalid inputs. Using defaults: PL1_Stat_PLA_ESP=0, Handshake=No")
      esp_stat = 0
      use_handshake = False

    print("\n----------------------------------------------------")
    if use_handshake:
      print("Step 1: Sending PLA_1 (status 1 [PLA Init]) to handshake...")
    else:
      print("Sending PLA_1 (status 8) at 50Hz...")
    print("Listening for Lenkhilfe_2 PLA response signals...")
    print("Press Ctrl+C to stop the test and return to menu.\n")

    last_send = 0
    last_render = 0
    success = False
    max_status_seen = 0
    handshake_done = not use_handshake
    handshake_start_time = time.perf_counter()
    current_status_cmd = 1 if use_handshake else 8

    last_pla_status = -1
    last_err = -1
    last_abbr = -1

    try:
      while True:
        now = time.perf_counter()
        self.update_parser()
        
        # Read PLA feedback signals
        pla_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_StatEPS_PLA"])
        pla_err = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Err"])
        pla_abbr = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Abbr"])

        if pla_status > max_status_seen:
          max_status_seen = pla_status

        if pla_status == 8:
          success = True

        # Perform handshake logic
        if use_handshake and not handshake_done:
          if pla_status == 1:
            print("\n[Handshake] EPS acknowledged PLA status 1 (Init)!")
            print("[Handshake] Transitioning command to status 8 (Activatable Inactive)...")
            current_status_cmd = 8
            handshake_done = True
          elif now - handshake_start_time > 2.0:
            print("\n[Handshake] Timeout waiting for status 1! Forcing transition to status 8...")
            current_status_cmd = 8
            handshake_done = True

        # Print live status (on signal change, or at least every 100ms to keep screen alive)
        if (pla_status != last_pla_status or pla_err != last_err or pla_abbr != last_abbr or
            now - last_render >= 0.1):
          sys.stdout.write(
            f"\r[PLA Test] Cmd: {current_status_cmd} | ESP_Stat: {esp_stat} | Feedback: PLA={pla_status}, Err={pla_err}, Abbr={pla_abbr} | Max Seen={max_status_seen} | Success: {'YES!' if success else 'NO'}   "
          )
          sys.stdout.flush()
          last_pla_status = pla_status
          last_err = pla_err
          last_abbr = pla_abbr
          last_render = now

        # Send PLA_1 at 50Hz (every 20ms)
        if now - last_send >= 0.02:
          # Construct PLA_1 frame
          addr, dat, bus = self.make_pla_msg(
            current_status_cmd, 0.0, 0, esp_stat, 0
          )
          self.panda.can_send(addr, dat, bus)
          last_send = now

        time.sleep(0.005)

    except KeyboardInterrupt:
      print("\n\n----------------------------------------------------")
      print("PLA Test Completed!")
      if success:
        print("\033[92mSUCCESS: Steering rack responded by reporting PLA status 8!\033[0m")
      else:
        print("\033[91mFAILURE: Steering rack did not enter status 8. Max status seen: {}\033[0m".format(max_status_seen))
        print(f"Final Feedback Signals:")
        print(f"  - PLA Status  : {last_pla_status}")
        print(f"  - PLA Error   : {last_err} (LH2_PLA_Err)")
        print(f"  - PLA Abort   : {last_abbr} (LH2_PLA_Abbr)")
        print("\n\033[93mDIAGNOSTIC ADVICE:\033[0m")
        if last_err > 0 or last_abbr > 0:
          print(f"  * The steering rack is actively rejecting the command with Error Code {last_err} and Abort Code {last_abbr}.")
        print("  * Check if PLA coding/enable is turned ON in address 44 (Steering Assist) coding via VCDS / ODIS.")
        print("  * Try running again and changing PL1_Stat_PLA_ESP to 1, 5, or 8 (some racks require ESP confirmation).")
        print("  * Try enabling the handshake initialization (1 -> 8 sequence).")
      print("----------------------------------------------------")
      input("\nPress Enter to return to the main menu...")

  def run_pla_sweep_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("         PLA CONTROLLED ANGLE SWEEP TEST            ")
    print("====================================================")
    print("This test will validate PLA angle control by sweeping the wheel to:")
    print("  0 -> +100 degrees -> 0 -> -100 degrees -> 0")
    print("A safe, low ramping speed (delta rate) will be used to avoid sudden motion.\n")
    
    # Configure parameters
    # Status 8 = PLA activatable/idle (used during init, rack must echo 8 before sweep)
    # Status 6 = PLA active in control (used during angle sweep)
    PLA_STATUS_IDLE   = 8
    PLA_STATUS_ACTIVE = 6

    try:
      ramp_input = input("Enter max ramping speed (deg/sec, default: 30): ")
      max_deg_per_sec = float(ramp_input) if ramp_input.strip() else 30.0

      target_angle_input = input("Enter target sweep angle magnitude (deg, default: 100): ")
      sweep_target = float(target_angle_input) if target_angle_input.strip() else 100.0

      esp_stat_input = input("Enter PL1_Stat_PLA_ESP to send (0-15, default: 0): ").strip()
      esp_stat = int(esp_stat_input) if esp_stat_input else 0

      brems_input = input("Enter PL1_Bremsmoment to send (default: 0): ").strip()
      bremsmoment = int(brems_input) if brems_input else 0
    except ValueError:
      print("Invalid inputs. Using defaults: Speed=30 deg/s, Target=100 deg, ESP_Stat=0, Bremsmoment=0")
      max_deg_per_sec = 30.0
      sweep_target = 100.0
      esp_stat = 0
      bremsmoment = 0

    print("\n--- Phase 1: Initializing PLA (status 8 idle) ---")
    print("Sending PLA_1 status=8 with 0 angle (ESP_Stat={}, Brems={})...".format(esp_stat, bremsmoment))
    print("Waiting for rack to echo back status 8 (activatable/idle)...")

    # Clear old buffer
    self.panda.can_recv()

    initialized = False
    init_start = time.perf_counter()

    try:
      # Start keepalive at status 8 (idle) with 0 angle
      self.start_keepalive("PLA", PLA_STATUS_IDLE, esp_stat, bremsmoment)

      while time.perf_counter() - init_start < 4.0:
        self.update_parser()
        pla_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_StatEPS_PLA"])
        pla_err = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Err"])
        pla_abbr = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Abbr"])

        sys.stdout.write(f"\rCurrent PLA Status: {pla_status} | Err: {pla_err} | Abbr: {pla_abbr}   ")
        sys.stdout.flush()

        if pla_status == PLA_STATUS_IDLE:
          initialized = True
          break

        time.sleep(0.01)

      if not initialized:
        self.stop_keepalive()
        print("\n\033[91mPLA Initialization Timeout!\033[0m Rack did not echo status 8.")
        print(f"Final Feedback: PLA Status={pla_status}, Err={pla_err}, Abbr={pla_abbr}")
        print("Please ensure your ignition is on, and the engine is running or in a state that enables PLA.")
        input("\nPress Enter to return to the main menu...")
        return

      print("\n\n\033[92mPLA Ready (status 8)! Switching to status 6 for sweep.\033[0m")
      print("Press Enter to begin the automated sweep test (Ctrl+C to abort)...")
      input()

      # Stop background keepalive — sweep loop will send status 6 directly
      self.stop_keepalive()

      # Automated sequence
      # A list of tuples: (target_angle, hold_seconds)
      sequence = [
        (sweep_target, 1.0),
        (0.0, 1.0),
        (-sweep_target, 1.0),
        (0.0, 1.0)
      ]

      last_send = 0
      current_cmd_angle = 0.0
      last_loop_time = time.perf_counter()

      for seq_idx, (goal_angle, hold_time) in enumerate(sequence):
        print(f"\nMoving to goal: {goal_angle}° (Sequence {seq_idx+1}/{len(sequence)})...")
        
        # Ramping phase
        while True:
          now = time.perf_counter()
          dt = now - last_loop_time
          last_loop_time = now

          # Send PLA_1 at 50Hz with status 6 (active in control)
          if now - last_send >= 0.02:
            sign = 1 if current_cmd_angle < 0 else 0
            addr, dat, bus = self.make_pla_msg(
              PLA_STATUS_ACTIVE, abs(current_cmd_angle), sign, esp_stat, bremsmoment
            )
            self.panda.can_send(addr, dat, bus)
            last_send = now

          # Read current angle
          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          act_angle = -raw_angle if sign_angle == 1 else raw_angle
          
          lh2_pla = int(self.parser.vl["Lenkhilfe_2"]["LH2_StatEPS_PLA"])
          lh2_err = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Err"])
          lh2_abbr = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Abbr"])

          sys.stdout.write(f"\rCmd Angle: {current_cmd_angle:6.1f}° | LWS: {act_angle:6.1f}° | State: {lh2_pla} | Err: {lh2_err} | Abbr: {lh2_abbr}   ")
          sys.stdout.flush()

          # Check if we reached goal
          if abs(current_cmd_angle - goal_angle) < 0.01:
            current_cmd_angle = goal_angle
            break

          # Ramp command
          step = max_deg_per_sec * dt
          if goal_angle > current_cmd_angle:
            current_cmd_angle = min(current_cmd_angle + step, goal_angle)
          else:
            current_cmd_angle = max(current_cmd_angle - step, goal_angle)

          time.sleep(0.005)

        # Holding phase
        hold_start = time.perf_counter()
        while time.perf_counter() - hold_start < hold_time:
          now = time.perf_counter()
          if now - last_send >= 0.02:
            sign = 1 if current_cmd_angle < 0 else 0
            addr, dat, bus = self.make_pla_msg(
              PLA_STATUS_ACTIVE, abs(current_cmd_angle), sign, esp_stat, bremsmoment
            )
            self.panda.can_send(addr, dat, bus)
            last_send = now

          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          act_angle = -raw_angle if sign_angle == 1 else raw_angle
          lh2_pla = int(self.parser.vl["Lenkhilfe_2"]["LH2_StatEPS_PLA"])
          lh2_err = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Err"])
          lh2_abbr = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Abbr"])

          sys.stdout.write(f"\rCmd Angle: {current_cmd_angle:6.1f}° | LWS: {act_angle:6.1f}° | State: {lh2_pla} | Err: {lh2_err} | Abbr: {lh2_abbr}   ")
          sys.stdout.flush()
          time.sleep(0.005)

      # Cleanup disengage command
      print("\n\nSweep complete! Disengaging PLA control...")
      for _ in range(10): # Send 10 times to be sure
        addr, dat, bus = self.make_pla_msg(0, 0.0, 0, 0, 0)
        self.panda.can_send(addr, dat, bus)
        time.sleep(0.01)

      print("\033[92mSUCCESS: Finished PLA sweep test cleanly!\033[0m")

    except KeyboardInterrupt:
      print("\n\n\033[91mInterrupted! Resetting and disengaging PLA steering control for safety.\033[0m")
      try:
        for _ in range(15):
          addr, dat, bus = self.make_pla_msg(0, 0.0, 0, 0, 0)
          self.panda.can_send(addr, dat, bus)
          time.sleep(0.01)
      except:
        pass

    input("\nPress Enter to return to the main menu...")

  def run_hca_step_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("         HCA STEP TORQUE RESPONSE DELAY TEST        ")
    print("====================================================")
    
    # Configure parameters
    try:
      torque_input = input("Enter test torque to apply (cNm, default: 150): ")
      torque_val = int(torque_input) if torque_input.strip() else 150
      
      status_input = input("Enter HCA_Status to send (default: 5 [active]): ")
      hca_send_status = int(status_input) if status_input.strip() else 5

      dir_input = input("Enter direction (L/R, default: L): ").strip().upper()
      direction = 1 if dir_input == 'R' else 0
    except ValueError:
      print("Invalid inputs. Using defaults: Torque=150 cNm, HCA_Status=5, Direction=Left")
      torque_val = 150
      hca_send_status = 5
      direction = 0

    print("\n--- Phase 1: Initializing HCA ---")
    print("Sending HCA_Status={} with 0 torque to initialize...".format(hca_send_status))

    # Clear old buffer
    self.panda.can_recv()

    initialized = False
    init_start = time.perf_counter()
    
    try:
      # Start keepalive in background to constantly satisfy the EPS rack
      self.start_keepalive("HCA")

      while time.perf_counter() - init_start < 4.0:
        self.update_parser()
        hca_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        
        sys.stdout.write(f"\rCurrent HCA Status from EPS: {hca_status} ({HCA_STATES.get(hca_status, 'UNKNOWN')})   ")
        sys.stdout.flush()

        if hca_status in (3, 5, 7): # ready or active
          initialized = True
          break

        time.sleep(0.01)

      if not initialized:
        self.stop_keepalive()
        print("\n\033[91mHCA Initialization Timeout!\033[0m EPS status did not transition to Ready or Active.")
        print("Please ensure your ignition is on, and the engine is running or in a state that enables HCA.")
        input("\nPress Enter to return to the main menu...")
        return

      print("\n\n\033[92mHCA Initialized! Rack is ready.\033[0m")
      
      # We stop the background keepalive thread because we will manually manage HCA frames at 100Hz in our loops.
      self.stop_keepalive()

      # Import raw terminal handling for wait loop
      import termios
      import tty
      import select

      fd = sys.stdin.fileno()
      
      while True:
        print("\n----------------------------------------------------")
        print(f" Ready to apply step torque command of {torque_val} cNm.")
        print(" [Enter] to run test | [q] to return to main menu")
        print("----------------------------------------------------")
        
        # Clear stdin buffer to avoid accidental presses
        termios.tcflush(fd, termios.TCIFLUSH)
        
        old_settings = termios.tcgetattr(fd)
        ch_pressed = None
        try:
          tty.setraw(fd)
          last_send = 0
          last_render = 0
          while True:
            now = time.perf_counter()
            
            if now - last_send >= 0.01:
              self.send_hca_cmd(0, 0, 3)
              last_send = now
              
            self.update_parser()
            
            # Render live angle and velocity at 10Hz
            if now - last_render >= 0.1:
              raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
              sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
              angle = -raw_angle if sign_angle == 1 else raw_angle
              
              raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
              sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
              vel = -raw_vel if sign_vel == 1 else raw_vel
              
              # Return to normal settings momentarily to print safely
              termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
              sys.stdout.write(f"\rLive LWS: Angle={angle:7.2f}° | Velocity={vel:7.2f}°/s   ")
              sys.stdout.flush()
              tty.setraw(fd)
              last_render = now
              
            # Check stdin
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
              ch = sys.stdin.read(1)
              if ch in ('\n', '\r'):
                ch_pressed = 'enter'
                break
              elif ch.lower() == 'q':
                ch_pressed = 'q'
                break
        finally:
          termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
          
        if ch_pressed == 'q':
          break
          
        print("\n\nExecuting step command...")
        
        # Collect baseline for 200ms
        last_send = 0
        baselines_angle = []
        baselines_vel = []
        baseline_start = time.perf_counter()
        while time.perf_counter() - baseline_start < 0.2:
          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          angle = -raw_angle if sign_angle == 1 else raw_angle

          raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
          sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
          vel = -raw_vel if sign_vel == 1 else raw_vel

          baselines_angle.append(angle)
          baselines_vel.append(vel)
          
          # Keep HCA alive with 0 torque
          now = time.perf_counter()
          if now - last_send >= 0.01:
            self.send_hca_cmd(0, 0, 3)
            last_send = now
          time.sleep(0.005)

        base_angle = sum(baselines_angle) / len(baselines_angle) if baselines_angle else 0.0
        base_vel = sum(baselines_vel) / len(baselines_vel) if baselines_vel else 0.0

        # Apply step torque
        t_cmd = time.perf_counter()
        
        data_log = [] # list of tuples: (relative_time, angle_delta, velocity_delta)
        step_duration = 1.0 # test for 1.0 second
        t_resp_angle = None
        t_resp_vel = None
        t_resp_vel_05 = None
        t_resp_vel_15 = None
        t_resp_vel_min = None
        t_resp_angle_min = None

        # Loop during step execution
        while time.perf_counter() - t_cmd < step_duration:
          now = time.perf_counter()
          
          # Send HCA torque command at 100Hz
          if now - last_send >= 0.01:
            self.send_hca_cmd(torque_val, direction, hca_send_status)
            last_send = now

          # Read feedback
          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          angle = -raw_angle if sign_angle == 1 else raw_angle

          raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
          sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
          vel = -raw_vel if sign_vel == 1 else raw_vel

          rel_time = now - t_cmd
          angle_delta = angle - base_angle
          vel_delta = vel - base_vel

          data_log.append((rel_time, angle_delta, vel_delta))

          # Check response threshold:
          if t_resp_vel_min is None and abs(vel_delta) >= 0.04375:
            t_resp_vel_min = rel_time
          if t_resp_angle_min is None and abs(angle_delta) >= 0.04375:
            t_resp_angle_min = rel_time
          if t_resp_vel_05 is None and abs(vel_delta) >= 0.5:
            t_resp_vel_05 = rel_time
          if t_resp_vel_15 is None and abs(vel_delta) >= 1.5:
            t_resp_vel_15 = rel_time
            t_resp_vel = rel_time
          if t_resp_angle is None and abs(angle_delta) >= 0.1:
            t_resp_angle = rel_time

          time.sleep(0.002)

        self.send_hca_cmd(0, 0, 3)

        print("\nStep complete! HCA returned to Ready.")
        print("\n====================================================")
        print("                    TEST RESULTS                    ")
        print("====================================================")
        print(f" Torque Command Sent   : {torque_val} cNm (Direction: {'Right' if direction else 'Left'})")
        print(f" LWS Angle Baseline    : {base_angle:.3f}°")
        print(f" LWS Velocity Baseline : {base_vel:.3f}°/s")
        
        if t_resp_vel_min is not None:
          print(f"\033[92m Velocity Delay (min LSB): {t_resp_vel_min * 1000:.2f} ms\033[0m (threshold: 0.04375°/s change)")
        else:
          print("\033[91m Velocity Delay (min LSB): No Response Detected\033[0m")

        if t_resp_vel_05 is not None:
          print(f"\033[92m Velocity Delay (0.5°/s) : {t_resp_vel_05 * 1000:.2f} ms\033[0m")
        else:
          print("\033[91m Velocity Delay (0.5°/s) : No Response Detected\033[0m")

        if t_resp_vel_15 is not None:
          print(f"\033[92m Velocity Delay (1.5°/s) : {t_resp_vel_15 * 1000:.2f} ms\033[0m")
        else:
          print("\033[91m Velocity Delay (1.5°/s) : No Response Detected\033[0m")

        if t_resp_angle_min is not None:
          print(f"\033[92m Angle Delay (min LSB)   : {t_resp_angle_min * 1000:.2f} ms\033[0m (threshold: 0.04375° change)")
        else:
          print("\033[91m Angle Delay (min LSB)   : No Response Detected\033[0m")

        if t_resp_angle is not None:
          print(f"\033[92m Angle Delay (0.1°)      : {t_resp_angle * 1000:.2f} ms\033[0m")
        else:
          print("\033[91m Angle Delay (0.1°)      : No Response Detected\033[0m")
        
        # Render text-based plot of velocity delta over time!
        self.render_ascii_plot(data_log, t_resp_vel)

    except KeyboardInterrupt:
      # Safety cleanup in case of Ctrl+C
      print("\nInterrupted! Resetting steering control.")
    finally:
      self.stop_keepalive()
      try:
        addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
          "LM_Offset": 0,
          "LM_OffSign": 0,
          "HCA_Status": 3,
          "Vib_Freq": 16,
          "Vib_Amp": 0,
        })
        self.panda.can_send(addr, dat, bus)
      except:
        pass

    input("\nPress Enter to return to the main menu...")

  def run_hca_active_step_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("   HCA ACTIVE STEP RESPONSE DELAY TEST (LOW -> HIGH) ")
    print("====================================================")
    print("This profiles response time while ALREADY engaged in")
    print("Active HCA status (Status 5/7) at a low torque to")
    print("isolate mechanical lag from status transition lag.")
    print("====================================================")
    
    # Configure parameters
    try:
      base_torque = input("Enter baseline active torque (cNm, default: 2): ")
      base_torque_val = int(base_torque) if base_torque.strip() else 2
      
      torque_input = input("Enter step active torque to jump to (cNm, default: 150): ")
      torque_val = int(torque_input) if torque_input.strip() else 150
      
      status_input = input("Enter HCA_Status to send (default: 5 [active]): ")
      hca_send_status = int(status_input) if status_input.strip() else 5

      dir_input = input("Enter direction (L/R, default: L): ").strip().upper()
      direction = 1 if dir_input == 'R' else 0
    except ValueError:
      print("Invalid inputs. Using defaults: Base=2 cNm, Step=150 cNm, HCA_Status=5, Direction=Left")
      base_torque_val = 2
      torque_val = 150
      hca_send_status = 5
      direction = 0

    print("\n--- Phase 1: Initializing HCA ---")
    print("Sending HCA_Status={} with 0 torque to initialize...".format(hca_send_status))

    # Clear old buffer
    self.panda.can_recv()

    initialized = False
    init_start = time.perf_counter()
    
    try:
      # Start keepalive in background to satisfy the EPS rack
      self.start_keepalive("HCA")

      while time.perf_counter() - init_start < 4.0:
        self.update_parser()
        hca_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        
        sys.stdout.write(f"\rCurrent HCA Status from EPS: {hca_status} ({HCA_STATES.get(hca_status, 'UNKNOWN')})   ")
        sys.stdout.flush()

        if hca_status in (3, 5, 7): # ready or active
          initialized = True
          break

        time.sleep(0.01)

      if not initialized:
        self.stop_keepalive()
        print("\n\033[91mHCA Initialization Timeout!\033[0m")
        input("\nPress Enter to return to the main menu...")
        return

      print("\n\n\033[92mHCA Initialized! Rack is ready.\033[0m")
      
      self.stop_keepalive()

      # Import raw terminal handling for wait loop
      import termios
      import tty
      import select

      fd = sys.stdin.fileno()
      
      while True:
        print("\n----------------------------------------------------")
        print(f" Ready to test step: Active {base_torque_val} cNm -> jump to {torque_val} cNm.")
        print(" [Enter] to run test | [q] to return to main menu")
        print("----------------------------------------------------")
        
        termios.tcflush(fd, termios.TCIFLUSH)
        
        old_settings = termios.tcgetattr(fd)
        ch_pressed = None
        try:
          tty.setraw(fd)
          last_send = 0
          last_render = 0
          while True:
            now = time.perf_counter()
            
            if now - last_send >= 0.01:
              self.send_hca_cmd(0, 0, 3)
              last_send = now
              
            self.update_parser()
            
            if now - last_render >= 0.1:
              raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
              sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
              angle = -raw_angle if sign_angle == 1 else raw_angle
              
              raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
              sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
              vel = -raw_vel if sign_vel == 1 else raw_vel
              
              termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
              sys.stdout.write(f"\rLive LWS: Angle={angle:7.2f}° | Velocity={vel:7.2f}°/s   ")
              sys.stdout.flush()
              tty.setraw(fd)
              last_render = now
              
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
              ch = sys.stdin.read(1)
              if ch in ('\n', '\r'):
                ch_pressed = 'enter'
                break
              elif ch.lower() == 'q':
                ch_pressed = 'q'
                break
        finally:
          termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
          
        if ch_pressed == 'q':
          break
          
        print("\n\nEngaging low baseline torque to enter active HCA state...")
        
        # Engage HCA Active and apply low baseline torque for 500ms
        last_send = 0
        baselines_angle = []
        baselines_vel = []
        baseline_start = time.perf_counter()
        
        while time.perf_counter() - baseline_start < 0.5:
          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          angle = -raw_angle if sign_angle == 1 else raw_angle

          raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
          sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
          vel = -raw_vel if sign_vel == 1 else raw_vel

          baselines_angle.append(angle)
          baselines_vel.append(vel)
          
          # Send HCA Active torque at 100Hz
          now = time.perf_counter()
          if now - last_send >= 0.01:
            self.send_hca_cmd(base_torque_val, direction, hca_send_status)
            last_send = now
          time.sleep(0.005)

        base_angle = sum(baselines_angle) / len(baselines_angle) if baselines_angle else 0.0
        base_vel = sum(baselines_vel) / len(baselines_vel) if baselines_vel else 0.0

        print(f"Jumping torque to high step command ({torque_val} cNm)...")

        # Execute high step torque jump
        t_cmd = time.perf_counter()
        data_log = []
        step_duration = 1.0
        t_resp_angle = None
        t_resp_vel = None
        t_resp_vel_05 = None
        t_resp_vel_15 = None
        t_resp_vel_min = None
        t_resp_angle_min = None

        while time.perf_counter() - t_cmd < step_duration:
          now = time.perf_counter()
          
          # Send High Torque command at 100Hz
          if now - last_send >= 0.01:
            self.send_hca_cmd(torque_val, direction, hca_send_status)
            last_send = now

          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          angle = -raw_angle if sign_angle == 1 else raw_angle

          raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
          sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
          vel = -raw_vel if sign_vel == 1 else raw_vel

          rel_time = now - t_cmd
          angle_delta = angle - base_angle
          vel_delta = vel - base_vel

          data_log.append((rel_time, angle_delta, vel_delta))

          if t_resp_vel_min is None and abs(vel_delta) >= 0.04375:
            t_resp_vel_min = rel_time
          if t_resp_angle_min is None and abs(angle_delta) >= 0.04375:
            t_resp_angle_min = rel_time
          if t_resp_vel_05 is None and abs(vel_delta) >= 0.5:
            t_resp_vel_05 = rel_time
          if t_resp_vel_15 is None and abs(vel_delta) >= 1.5:
            t_resp_vel_15 = rel_time
            t_resp_vel = rel_time
          if t_resp_angle is None and abs(angle_delta) >= 0.1:
            t_resp_angle = rel_time

          time.sleep(0.002)

        self.send_hca_cmd(0, 0, 3)

        print("\nStep complete! HCA returned to Ready.")
        print("\n====================================================")
        print("                JUMP TEST RESULTS                   ")
        print("====================================================")
        print(f" Baseline Active Torque: {base_torque_val} cNm")
        print(f" High Jump Command Sent: {torque_val} cNm")
        print(f" LWS Angle Baseline    : {base_angle:.3f}°")
        print(f" LWS Velocity Baseline : {base_vel:.3f}°/s")
        
        if t_resp_vel_min is not None:
          print(f"\033[92m Velocity Delay (min LSB): {t_resp_vel_min * 1000:.2f} ms\033[0m (threshold: 0.04375°/s change)")
        else:
          print("\033[91m Velocity Delay (min LSB): No Response Detected\033[0m")

        if t_resp_vel_05 is not None:
          print(f"\033[92m Velocity Delay (0.5°/s) : {t_resp_vel_05 * 1000:.2f} ms\033[0m")
        else:
          print("\033[91m Velocity Delay (0.5°/s) : No Response Detected\033[0m")

        if t_resp_vel_15 is not None:
          print(f"\033[92m Velocity Delay (1.5°/s) : {t_resp_vel_15 * 1000:.2f} ms\033[0m")
        else:
          print("\033[91m Velocity Delay (1.5°/s) : No Response Detected\033[0m")

        if t_resp_angle_min is not None:
          print(f"\033[92m Angle Delay (min LSB)   : {t_resp_angle_min * 1000:.2f} ms\033[0m (threshold: 0.04375° change)")
        else:
          print("\033[91m Angle Delay (min LSB)   : No Response Detected\033[0m")

        if t_resp_angle is not None:
          print(f"\033[92m Angle Delay (0.1°)      : {t_resp_angle * 1000:.2f} ms\033[0m")
        else:
          print("\033[91m Angle Delay (0.1°)      : No Response Detected\033[0m")
        
        self.render_ascii_plot(data_log, t_resp_vel)

    except KeyboardInterrupt:
      print("\nInterrupted! Resetting steering control.")
    finally:
      self.stop_keepalive()
      try:
        addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
          "LM_Offset": 0,
          "LM_OffSign": 0,
          "HCA_Status": 3,
          "Vib_Freq": 16,
          "Vib_Amp": 0,
        })
        self.panda.can_send(addr, dat, bus)
      except:
        pass

    input("\nPress Enter to return to the main menu...")

  def render_ascii_plot(self, log_data, response_time):
    """ Renders a high-quality ASCII-art timeline plot of velocity response """
    if not log_data:
      return

    print("\n====================================================")
    print("       STEP VELOCITY DELTA TIMELINE (ASCII PLOT)    ")
    print("====================================================")
    
    # Bucket data into 25 time increments (up to 500ms)
    max_plot_time = 0.5 # 500ms
    num_buckets = 25
    buckets = [[] for _ in range(num_buckets)]
    
    for rel_time, _, vel_delta in log_data:
      if rel_time <= max_plot_time:
        idx = int((rel_time / max_plot_time) * num_buckets)
        if idx < num_buckets:
          buckets[idx].append(abs(vel_delta))

    # Calculate average absolute velocity change for each bucket
    plot_vals = []
    for b in buckets:
      if b:
        plot_vals.append(sum(b) / len(b))
      else:
        plot_vals.append(0.0)

    max_val = max(plot_vals) if plot_vals else 1.0
    if max_val < 1.0:
      max_val = 1.0

    # Draw ASCII graph
    for y_level in range(8, -1, -1):
      threshold = (y_level / 8) * max_val
      line = f" {threshold:5.1f}°/s | "
      for val in plot_vals:
        if val >= threshold and val > 0.05:
          line += "█"
        elif val >= threshold - (max_val / 16) and val > 0.05:
          line += "▄"
        else:
          line += " "
      print(line)

    print("        " + "-" * (num_buckets + 3))
    
    # X axis labels
    labels_line = "  Time:  "
    for i in range(num_buckets):
      t_ms = int((i / num_buckets) * max_plot_time * 1000)
      if i == 0:
        labels_line += "0ms"
      elif i == int(num_buckets / 2):
        labels_line += f"  {t_ms}ms"
      elif i == num_buckets - 1:
        labels_line += f"   {t_ms}ms"
      else:
        # Add spaces based on lengths
        if i not in (1, 2, int(num_buckets/2)-1, int(num_buckets/2), int(num_buckets/2)+1, num_buckets-2):
          labels_line += " "
    print(labels_line)
    
    if response_time is not None:
      print(f"\n * Torque Command triggered at 0ms.")
      print(f" * Rack motion detected velocity deviation at \033[92m{response_time * 1000:.1f} ms\033[0m.")

  def auto_center_wheel(self, target_max_torque=120):
    print("Auto-centering steering wheel...")
    
    # Safely stop and join background daemon thread to completely eradicate pyusb thread-unsafe collisions
    self.stop_keepalive()
    self.daemon_running = False
    if self.daemon_thread:
      self.daemon_thread.join(timeout=0.3)
      self.daemon_thread = None
    
    # Phase 1: Wake up and wait for HCA to enter Ready/Active state (synchronously in main thread)
    init_start = time.perf_counter()
    last_send = 0
    initialized = False
    
    while time.perf_counter() - init_start < 4.0:
      now = time.perf_counter()
      
      # Keep parser updated and read feedback in main thread
      msgs = self.panda.can_recv() or []
      if msgs:
        frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
        self.parser.update([(time.time_ns(), frames)])
        
      hca_status_feedback = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
      sys.stdout.write(f"\r[Centering Wakeup] Waiting for HCA Ready... Current EPS Status: {hca_status_feedback} ({HCA_STATES.get(hca_status_feedback, 'UNKNOWN')})   ")
      sys.stdout.flush()
      
      if hca_status_feedback in (3, 5, 7):
        initialized = True
        break
        
      # Send HCA Status 3 keepalive in main thread
      if now - last_send >= 0.01:
        self.send_hca_cmd(0, 0, 3)
        last_send = now
        
      time.sleep(0.005)
    print() # Newline
      
    if not initialized:
      print("[Auto-Centering] \033[91mHCA failed to wake up. Centering aborted.\033[0m")
      # Restart background daemon thread for standby keepalive
      self.daemon_running = True
      self.daemon_thread = threading.Thread(target=self._bg_daemon_loop)
      self.daemon_thread.daemon = True
      self.daemon_thread.start()
      self.start_keepalive("HCA")
      return
      
    # Phase 2: Active PI + Damping control loop to center the wheel precisely
    t_start = time.perf_counter()
    last_send = 0
    last_print = 0
    last_time = time.perf_counter()
    success = False
    integral = 0.0
    
    while time.perf_counter() - t_start < 5.0: # 5s safety timeout
      now = time.perf_counter()
      dt = max(0.001, now - last_time)
      last_time = now
      
      # Read CAN messages in main thread
      msgs = self.panda.can_recv() or []
      if msgs:
        frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
        self.parser.update([(time.time_ns(), frames)])
        
      raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
      sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
      angle = -raw_angle if sign_angle == 1 else raw_angle
      
      raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
      sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
      velocity = -raw_vel if sign_vel == 1 else raw_vel
      
      if abs(angle) <= 0.6 and abs(velocity) <= 2.0:
        success = True
        break
        
      # Determine return direction: steer opposite to current angle sign
      dir_sign = 1 if angle > 0 else 0
      
      if abs(angle) > 20.0:
        torque = 500
      else:
        # Linear ramp down from 500 at 20.0 deg to 130 at 0.6 deg
        ratio = (abs(angle) - 0.6) / (20.0 - 0.6)
        ratio = max(0.0, min(1.0, ratio))
        torque = int(130 + (500 - 130) * ratio)
        
        # Apply velocity damping to prevent overshoot
        torque = max(100, torque - int(1.5 * abs(velocity)))
        
      # Apply target maximum torque limit
      target_max_torque = max(target_max_torque, 500)
      if torque > target_max_torque:
        torque = target_max_torque
        
      # Scale down torque smoothly very close to center to prevent micro-oscillations
      if abs(angle) < 0.6:
        torque = int(torque * (abs(angle) / 0.6))
        
      # Send active torque command in main thread (Status 7)
      if now - last_send >= 0.01:
        self.send_hca_cmd(torque, dir_sign, 7) # Command Status 7
        last_send = now
        
      if now - last_print >= 0.1:
        hca_status_feedback = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        sys.stdout.write(f"\r[Centering] Angle: {angle:6.2f}° | Vel: {velocity:6.2f}°/s | Cmd Torque: {torque:3d} cNm | EPS HCA Status: {hca_status_feedback} ({HCA_STATES.get(hca_status_feedback, 'UNKNOWN')})   ")
        sys.stdout.flush()
        last_print = now
        
      time.sleep(0.002)
      
    # Phase 3: Hold centered position quietly for 1.0 second to completely settle (synchronously in main thread)
    stabilize_start = time.perf_counter()
    while time.perf_counter() - stabilize_start < 1.0:
      now = time.perf_counter()
      
      msgs = self.panda.can_recv() or []
      if msgs:
        frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
        self.parser.update([(time.time_ns(), frames)])
        
      if now - last_send >= 0.01:
        self.send_hca_cmd(0, 0, 3)
        last_send = now
      time.sleep(0.005)

    if success:
      print("\n[Auto-Centering] Wheel stabilized at center.")
    else:
      print("\n[Auto-Centering] \033[93mWarning: Centering timed out before reaching target.\033[0m")

    # Restart background daemon thread for standby keepalive
    self.daemon_running = True
    self.daemon_thread = threading.Thread(target=self._bg_daemon_loop)
    self.daemon_thread.daemon = True
    self.daemon_thread.start()
    self.start_keepalive("HCA")

  def run_hca_auto_delay_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("      AUTOMATED HCA STEP RESPONSE DELAY TEST        ")
    print("====================================================")
    print("This utility will automatically run 10 step tests:")
    print(" 1. Applies torque to run step response test (L/R).")
    print(" 2. Profiles LWS motion delay and velocity feedback.")
    print(" 3. Automatically return-centers the wheel using HCA.")
    print(" 4. Settle, then loop completely hands-free.")
    print("====================================================")
    
    # Configure parameters
    try:
      torque_input = input("Enter test torque to apply (cNm, default: 150): ")
      torque_val = int(torque_input) if torque_input.strip() else 150
      
      status_input = input("Enter HCA_Status to send (default: 5 [active]): ")
      hca_send_status = int(status_input) if status_input.strip() else 5

      dir_input = input("Enter test direction (L/R, default: L): ").strip().upper()
      direction = 1 if dir_input == 'R' else 0
      
      runs_input = input("Enter number of runs to execute (default: 10): ")
      total_runs = int(runs_input) if runs_input.strip() else 10
    except ValueError:
      print("Invalid inputs. Using defaults: Torque=150 cNm, HCA_Status=5, Direction=Left, Runs=10")
      torque_val = 150
      hca_send_status = 5
      direction = 0
      total_runs = 10

    print("\n--- Phase 1: Initializing HCA ---")
    self.panda.can_recv()
    
    initialized = False
    init_start = time.perf_counter()
    self.start_keepalive("HCA")
    
    try:
      while time.perf_counter() - init_start < 4.0:
        self.update_parser()
        hca_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        if hca_status in (3, 5, 7):
          initialized = True
          break
        time.sleep(0.01)

      if not initialized:
        self.stop_keepalive()
        print("\n\033[91mHCA Initialization Timeout!\033[0m")
        input("\nPress Enter to return...")
        return

      print("\n\033[92mHCA Initialized! Rack is ready.\033[0m")
      input("Press Enter to start the hands-free auto-test loop...")
      
      # We stop the background keepalive thread because we will manually manage HCA frames at 100Hz in our loops.
      self.stop_keepalive()
      
      runs_data = [] # stores dicts with: {run_id, vel_delay, angle_delay, success}
      
      for run_idx in range(1, total_runs + 1):
        print(f"\n====================================================")
        print(f"               RUN {run_idx:2d} OF {total_runs:2d}                    ")
        print(f"====================================================")
        
        # Ensure wheel is centered before starting
        self.auto_center_wheel(target_max_torque=min(120, torque_val))
        
        print(f"Starting Run {run_idx} step response test...")
        
        # Collect baseline for 200ms
        last_send = 0
        baselines_angle = []
        baselines_vel = []
        baseline_start = time.perf_counter()
        while time.perf_counter() - baseline_start < 0.2:
          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          angle = -raw_angle if sign_angle == 1 else raw_angle

          raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
          sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
          vel = -raw_vel if sign_vel == 1 else raw_vel

          baselines_angle.append(angle)
          baselines_vel.append(vel)
          
          # Keep HCA alive with 0 torque
          now = time.perf_counter()
          if now - last_send >= 0.01:
            addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
              "LM_Offset": 0,
              "LM_OffSign": 0,
              "HCA_Status": 3,
              "Vib_Freq": 16,
              "Vib_Amp": 0,
            })
            try:
              self.panda.can_send(addr, dat, bus)
            except Exception:
              pass
            last_send = now
          time.sleep(0.005)

        base_angle = sum(baselines_angle) / len(baselines_angle) if baselines_angle else 0.0
        base_vel = sum(baselines_vel) / len(baselines_vel) if baselines_vel else 0.0

        # Execute test step command
        t_cmd = time.perf_counter()
        t_resp_angle = None
        t_resp_vel = None
        step_duration = 0.8 # apply test torque for 800ms
        
        while time.perf_counter() - t_cmd < step_duration:
          now = time.perf_counter()
          
          # Send active test command
          if now - last_send >= 0.01:
            addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
              "LM_Offset": torque_val,
              "LM_OffSign": direction,
              "HCA_Status": hca_send_status,
              "Vib_Freq": 16,
              "Vib_Amp": 0,
            })
            try:
              self.panda.can_send(addr, dat, bus)
            except Exception:
              pass
            last_send = now
            
          # Read feedback
          self.update_parser()
          raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
          sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
          angle = -raw_angle if sign_angle == 1 else raw_angle

          raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
          sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
          vel = -raw_vel if sign_vel == 1 else raw_vel

          rel_time = now - t_cmd
          angle_delta = angle - base_angle
          vel_delta = vel - base_vel

          # Detect motion responses
          if t_resp_vel is None and abs(vel_delta) >= 1.5:
            t_resp_vel = rel_time
          if t_resp_angle is None and abs(angle_delta) >= 0.1:
            t_resp_angle = rel_time
            
          time.sleep(0.002)
          
        # Disengage HCA torque immediately after step duration
        addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
          "LM_Offset": 0,
          "LM_OffSign": 0,
          "HCA_Status": 3,
          "Vib_Freq": 16,
          "Vib_Amp": 0,
        })
        try:
          self.panda.can_send(addr, dat, bus)
        except Exception:
          pass
          
        # Log results
        runs_data.append({
          "run_id": run_idx,
          "vel_delay": t_resp_vel,
          "angle_delay": t_resp_angle,
          "success": t_resp_vel is not None or t_resp_angle is not None
        })
        
        # Output immediate run stats
        vel_str = f"{t_resp_vel*1000:6.1f} ms" if t_resp_vel else "No Resp"
        ang_str = f"{t_resp_angle*1000:6.1f} ms" if t_resp_angle else "No Resp"
        print(f"\033[92mRun {run_idx} results:\033[0m")
        print(f" -> Velocity Response Delay: {vel_str}")
        print(f" -> Angle Motion Delay     : {ang_str}")
        
        # Sleep brief moment before centering
        time.sleep(0.2)
        
      # Perform final automated return-center so wheel is centered at the end
      self.auto_center_wheel(target_max_torque=min(120, torque_val))

      # Print comprehensive test summary table!
      print("\n\n====================================================")
      print("             AUTOMATED DELAY TEST SUMMARY REPORT    ")
      print("====================================================")
      print(" RUN | STATUS | VELOCITY RESPONSE DELAY | ANGLE MOTION DELAY")
      print("----------------------------------------------------")
      
      total_vd = 0.0
      total_ad = 0.0
      successful_runs = 0
      
      for r in runs_data:
        status_str = "\033[92m PASS \033[0m" if r["success"] else "\033[91m FAIL \033[0m"
        vel_str = f"{r['vel_delay']*1000:7.1f} ms" if r["vel_delay"] else "No Response"
        ang_str = f"{r['angle_delay']*1000:7.1f} ms" if r["angle_delay"] else "No Response"
        print(f"  {r['run_id']:2d} | {status_str} | {vel_str:23s} | {ang_str}")
        
        if r["success"]:
          if r["vel_delay"]:
            total_vd += r["vel_delay"]
          if r["angle_delay"]:
            total_ad += r["angle_delay"]
          successful_runs += 1
          
      print("----------------------------------------------------")
      if successful_runs > 0:
        avg_vd = (total_vd / successful_runs) * 1000
        avg_ad = (total_ad / successful_runs) * 1000
        print(f" AVG |        | {avg_vd:7.1f} ms             | {avg_ad:7.1f} ms")
      else:
        print(" AVG |        | N/A                     | N/A")
      print("====================================================")
      
    except KeyboardInterrupt:
      print("\n\n\033[91mAuto Test Interrupted!\033[0m")
    finally:
      self.stop_keepalive()
      # Clean up HCA control safely
      for _ in range(10):
        addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
          "LM_Offset": 0,
          "LM_OffSign": 0,
          "HCA_Status": 3,
          "Vib_Freq": 16,
          "Vib_Amp": 0,
        })
        try:
          self.panda.can_send(addr, dat, bus)
        except:
          pass
        time.sleep(0.01)
      input("\nPress Enter to return to main menu...")

  def run_can_dashboard(self):
    print("Starting Live CAN Monitor...")
    try:
      while True:
        self.update_parser()
        
        # Read LWS Angle
        lrw = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
        lrw_sign = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
        lrw_deg = -lrw if lrw_sign == 1 else lrw
        
        # Read LWS velocity
        lrw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
        vel_sign = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
        lrw_vel_deg = -lrw_vel if vel_sign == 1 else lrw_vel
        
        # Lenkhilfe 1 (Motor feedback)
        lh1_current = self.parser.vl["Lenkhilfe_1"]["LH1_Lastinfo"]
        lh1_power = self.parser.vl["Lenkhilfe_1"]["LH1_ULeistung"]
        
        # Lenkhilfe 2 (HCA/PLA statuses)
        lh2_hca = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        lh2_pla = int(self.parser.vl["Lenkhilfe_2"]["LH2_StatEPS_PLA"])
        lh2_err = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Err"])
        lh2_abbr = int(self.parser.vl["Lenkhilfe_2"]["LH2_PLA_Abbr"])
        # Lenkhilfe 3 (Torque sensor)
        lh3_lm = self.parser.vl["Lenkhilfe_3"]["LH3_LM"]
        lh3_lmsign = self.parser.vl["Lenkhilfe_3"]["LH3_LMSign"]
        driver_torque = -lh3_lm if lh3_lmsign == 1 else lh3_lm

        # Clean terminal render
        sys.stdout.write("\033[H\033[J")
        sys.stdout.write("====================================================\n")
        sys.stdout.write("      AUDI TT MK2 (PQ35/PQ46) STEERING MONITOR      \n")
        sys.stdout.write("====================================================\n")
        sys.stdout.write(f" Steering Wheel Angle (LWS) : \033[96m{lrw_deg:7.2f}°\033[0m\n")
        sys.stdout.write(f" LWS Angular Velocity       : \033[96m{lrw_vel_deg:7.2f}°/s\033[0m\n")
        sys.stdout.write(f" Driver Steering Torque     : \033[96m{driver_torque:7.1f} cNm\033[0m\n")
        sys.stdout.write(f" HCA Status State           : \033[95m{lh2_hca:2d} ({HCA_STATES.get(lh2_hca, 'UNKNOWN')})\033[0m\n")
        sys.stdout.write(f" PLA Status State           : \033[95m{lh2_pla:2d}\033[0m\n")
        sys.stdout.write(f" PLA Error State (Err)      : \033[91m{lh2_err:2d}\033[0m\n")
        sys.stdout.write(f" PLA Abort State (Abbr)     : \033[91m{lh2_abbr:2d}\033[0m\n")
        sys.stdout.write(f" EPS Motor Current (LH1)    : {lh1_current:7.1f} A\n")
        sys.stdout.write(f" EPS Motor Load Power (LH1) : {lh1_power:7.1f} %\n")
        sys.stdout.write("====================================================\n")
        sys.stdout.write("Press Ctrl+C to exit and return to main menu...\n")
        sys.stdout.flush()
        
        time.sleep(0.1)
 
    except KeyboardInterrupt:
      pass

  def run_interactive_control(self):
    print("\033[H\033[J")
    print("====================================================")
    print("       INTERACTIVE KEYBOARD STEERING CONTROL        ")
    print("====================================================")
    print("Initializing HCA...")
    
    # Pre-declare variables for safety in finally block
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    self.panda.can_recv()
    self.start_keepalive("HCA")
    
    # Wait for HCA ready
    init_start = time.perf_counter()
    initialized = False
    while time.perf_counter() - init_start < 4.0:
      self.update_parser()
      hca_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
      if hca_status in (3, 5, 7):
        initialized = True
        break
      time.sleep(0.01)
      
    if not initialized:
      self.stop_keepalive()
      print("\033[91mHCA failed to initialize.\033[0m")
      input("Press Enter to return...")
      return
      
    self.stop_keepalive()
    
    # Terminal raw mode setup
    import tty
    import select
    
    target_torque = 150
    hca_send_status = 5 # Default HCA Status (can toggle between 5 and 7)
    active_direction = None # 0 = Left, 1 = Right
    last_arrow_time = 0
    last_send = 0
    last_render = 0
    
    # State variables for change-detection based rendering
    last_target_torque = -1
    last_hca_send_status = -1
    last_cmd_torque = -1
    last_cmd_direction = -1
    last_cmd_hca_status = -1
    last_lrw_deg = -999.0
    last_lrw_vel_deg = -999.0
    last_raw_speed = -1.0
    last_driver_torque = -999.0
    last_lh2_hca = -1
    
    should_exit = False
    
    try:
      tty.setraw(fd)
      
      while True:
        now = time.perf_counter()
        
        # Drain all pending keys non-blockingly to be ultra responsive
        key_pressed = False
        while True:
          rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
          if not rlist:
            break
            
          ch = sys.stdin.read(1)
          key_pressed = True
          if ch == '\x1b': # Escape sequence (likely arrow keys)
            # Arrow keys are usually 3 bytes: \x1b[D (Left), \x1b[C (Right), etc.
            r2, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r2:
              seq = sys.stdin.read(2)
              if seq == '[D': # Left Arrow
                active_direction = 0
                last_arrow_time = now
              elif seq == '[C': # Right Arrow
                active_direction = 1
                last_arrow_time = now
          elif ch.isdigit():
            # Number keys: '1' -> 50, ..., '0' -> 500
            val = int(ch)
            val = 10 if val == 0 else val
            target_torque = val * 50
          elif ch.lower() == 's':
            # Toggle HCA Send Status between 5 and 7
            hca_send_status = 7 if hca_send_status == 5 else 5
          elif ch.lower() == 't':
            # Switch momentarily to cooked mode to get clean user input
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            sys.stdout.write("\r\n\r\n\033[93mEnter custom torque (cNm, 0-500): \033[0m")
            sys.stdout.flush()
            try:
              val_str = sys.stdin.readline().strip()
              if val_str:
                val = int(val_str)
                if 0 <= val <= 500:
                  target_torque = val
                  sys.stdout.write(f"\033[92mTorque set to {target_torque} cNm\033[0m\r\n")
                else:
                  sys.stdout.write("\033[91mError: Value must be between 0 and 500 cNm.\033[0m\r\n")
              else:
                sys.stdout.write("\033[91mCancelled.\033[0m\r\n")
              time.sleep(1)
            except ValueError:
              sys.stdout.write("\033[91mError: Invalid number.\033[0m\r\n")
              time.sleep(1)
            finally:
              tty.setraw(fd)
              last_render = 0
          elif ch.lower() in ('q', '\x03'): # 'q' or Ctrl+C
            should_exit = True
            break
            
        if should_exit:
          break
            
        # Determine current command
        is_arrow_active = (now - last_arrow_time < 0.5)
        if is_arrow_active and active_direction is not None:
          cmd_torque = target_torque
          cmd_direction = active_direction
          cmd_hca_status = hca_send_status
        else:
          cmd_torque = 0
          cmd_direction = 0
          cmd_hca_status = 3 # Ready
          
        # Send HCA_1 at 100Hz
        if now - last_send >= 0.01:
          self.send_hca_cmd(cmd_torque, cmd_direction, cmd_hca_status)
          last_send = now
          
        # Read vehicle status feedback at high frequency
        self.update_parser()
        
        # Read feedback signals
        raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
        sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
        lrw_deg = -raw_angle if sign_angle == 1 else raw_angle

        raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
        sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
        lrw_vel_deg = -raw_vel if sign_vel == 1 else raw_vel
        
        lh2_hca = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        
        lh3_lm = self.parser.vl["Lenkhilfe_3"]["LH3_LM"]
        lh3_lmsign = self.parser.vl["Lenkhilfe_3"]["LH3_LMSign"]
        driver_torque = -lh3_lm if lh3_lmsign == 1 else lh3_lm

        # Parse Kombi speed
        raw_speed = self.parser.vl["Kombi_1"]["Geschwindigkeit__Kombi_1_"]
        speed_mph = raw_speed * 0.621371
        
        # Change detection for screen rendering to prevent SSH queue clogging and lag
        cmd_state_changed = (target_torque != last_target_torque or
                             hca_send_status != last_hca_send_status or
                             cmd_torque != last_cmd_torque or
                             cmd_direction != last_cmd_direction or
                             cmd_hca_status != last_cmd_hca_status)
                             
        telemetry_changed = (abs(lrw_deg - last_lrw_deg) > 0.4 or
                             abs(lrw_vel_deg - last_lrw_vel_deg) > 3.0 or
                             abs(driver_torque - last_driver_torque) > 5.0 or
                             lh2_hca != last_lh2_hca or
                             abs(raw_speed - last_raw_speed) > 1.0)
                             
        time_to_refresh = (now - last_render >= 0.25) # Max 4Hz update rate for active telemetry
        force_refresh = (now - last_render >= 1.0) # Force update at least once a second
        
        should_redraw = (cmd_state_changed or 
                         key_pressed or
                         (telemetry_changed and time_to_refresh) or 
                         force_refresh or 
                         last_render == 0)
                         
        if should_redraw:
          # Render using cursor home sequence (avoids full screen flickering)
          sys.stdout.write("\033[H")
          sys.stdout.write("====================================================\r\n")
          sys.stdout.write("      INTERACTIVE KEYBOARD STEERING CONTROL         \r\n")
          sys.stdout.write("====================================================\r\n")
          sys.stdout.write(f" Target Torque Preset  : \033[93m{target_torque:3d} cNm\033[0m (Press 1-0 for preset, 't' for custom)\r\n")
          sys.stdout.write(f" HCA Active Status     : \033[95m{hca_send_status}\033[0m (Press 's' to toggle 5 vs 7)\r\n")
          sys.stdout.write(f" Command State         : " + (f"\033[92mSTEERING {'LEFT' if cmd_direction == 0 else 'RIGHT'} ({cmd_torque} cNm)\033[0m" if cmd_torque > 0 else "\033[90mIDLE / READY (0 cNm)\033[0m") + "             \r\n")
          sys.stdout.write("----------------------------------------------------\r\n")
          sys.stdout.write(f" Steering Wheel Angle  : \033[96m{lrw_deg:7.2f}°\033[0m\r\n")
          sys.stdout.write(f" Steering Ang. Velocity: \033[96m{lrw_vel_deg:7.2f}°/s\033[0m\r\n")
          sys.stdout.write(f" Vehicle Velocity      : \033[96m{raw_speed:7.2f} km/h ({speed_mph:.1f} mph)\033[0m\r\n")
          sys.stdout.write(f" Driver Input Torque   : \033[96m{driver_torque:7.1f} cNm\033[0m\r\n")
          sys.stdout.write(f" EPS HCA State         : \033[95m{lh2_hca:2d} ({HCA_STATES.get(lh2_hca, 'UNKNOWN')})\r\n")
          sys.stdout.write("====================================================\r\n")
          sys.stdout.write(" CONTROLS:\r\n")
          sys.stdout.write("   - Press [Left Arrow] / [Right Arrow] to steer for 0.5 sec\r\n")
          sys.stdout.write("   - Press keys [1] through [0] to set torque (50 to 500 cNm)\r\n")
          sys.stdout.write("   - Press [t] to type a custom torque value (0 to 500 cNm)\r\n")
          sys.stdout.write("   - Press [s] to toggle active HCA status between 5 and 7\r\n")
          sys.stdout.write("   - Press [q] or [Ctrl+C] to exit\r\n")
          sys.stdout.flush()
          
          # Store last rendered state
          last_target_torque = target_torque
          last_hca_send_status = hca_send_status
          last_cmd_torque = cmd_torque
          last_cmd_direction = cmd_direction
          last_cmd_hca_status = cmd_hca_status
          last_lrw_deg = lrw_deg
          last_lrw_vel_deg = lrw_vel_deg
          last_raw_speed = raw_speed
          last_driver_torque = driver_torque
          last_lh2_hca = lh2_hca
          last_render = now
          
        time.sleep(0.01)
        
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
      
      # Clean up HCA control safely
      print("\n\nDisengaging HCA...")
      for _ in range(10):
        addr, dat, bus = self.packer.make_can_msg("HCA_1", self.bus, {
          "LM_Offset": 0,
          "LM_OffSign": 0,
          "HCA_Status": 0,
          "Vib_Freq": 16,
          "Vib_Amp": 0,
        })
        try:
          self.panda.can_send(addr, dat, bus)
        except Exception:
          pass
        time.sleep(0.01)
      print("\033[92mDisengaged HCA successfully!\033[0m")
      input("\nPress Enter to return to main menu...")

  def render_large_state(self, cmd_torque, cmd_direction, current_angle, current_velocity):
    # Clear screen using terminal code
    sys.stdout.write("\033[H\033[J")
    
    dir_str = " L" if cmd_direction == 0 and cmd_torque > 0 else " R" if cmd_direction == 1 and cmd_torque > 0 else ""
    cmd_str = f"CMD: {cmd_torque}{dir_str} cNm"
    ang_str = f"ANG: {current_angle:+.1f} o"
    vel_str = f"VEL: {current_velocity:+.1f} o/s"
    
    def print_big(text):
      lines = ["", "", "", "", ""]
      for char in text:
        char_lines = BIG_FONT.get(char, BIG_FONT[' '])
        for i in range(5):
          lines[i] += char_lines[i] + " "
      for line in lines:
        print(line)
        
    print("==================================================================================")
    print_big(cmd_str)
    print("----------------------------------------------------------------------------------")
    print_big(ang_str)
    print("----------------------------------------------------------------------------------")
    print_big(vel_str)
    print("==================================================================================")

  def auto_center_wheel_strong(self, target_max_torque=200):
    print("\n[Auto-Centering] Returning wheel to center...")
    
    # Safely stop and join background daemon thread to completely eradicate pyusb thread-unsafe collisions
    self.stop_keepalive()
    self.daemon_running = False
    if self.daemon_thread:
      self.daemon_thread.join(timeout=0.3)
      self.daemon_thread = None
      
    # Phase 1: Wake up and wait for HCA to enter Ready/Active state (synchronously in main thread)
    init_start = time.perf_counter()
    last_send = 0
    initialized = False
    
    while time.perf_counter() - init_start < 4.0:
      now = time.perf_counter()
      
      # Keep parser updated and read feedback in main thread
      msgs = self.panda.can_recv() or []
      if msgs:
        frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
        self.parser.update([(time.time_ns(), frames)])
        
      hca_status_feedback = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
      sys.stdout.write(f"\r[Centering Wakeup] Waiting for HCA Ready... Current EPS Status: {hca_status_feedback} ({HCA_STATES.get(hca_status_feedback, 'UNKNOWN')})   ")
      sys.stdout.flush()
      
      if hca_status_feedback in (3, 5, 7):
        initialized = True
        break
        
      # Send HCA Status 3 keepalive in main thread
      if now - last_send >= 0.01:
        self.send_hca_cmd(0, 0, 3)
        last_send = now
        
      time.sleep(0.005)
    print() # Newline
      
    if not initialized:
      print("[Auto-Centering] \033[91mHCA failed to wake up. Centering aborted.\033[0m")
      # Restart background daemon thread for standby keepalive
      self.daemon_running = True
      self.daemon_thread = threading.Thread(target=self._bg_daemon_loop)
      self.daemon_thread.daemon = True
      self.daemon_thread.start()
      self.start_keepalive("HCA")
      return
      
    # Phase 2: Active PI + Damping control loop to center the wheel precisely
    t_start = time.perf_counter()
    last_send = 0
    last_print = 0
    last_time = time.perf_counter()
    success = False
    integral = 0.0
    
    while time.perf_counter() - t_start < 6.0: # 6s safety timeout
      now = time.perf_counter()
      dt = max(0.001, now - last_time)
      last_time = now
      
      # Read CAN messages in main thread
      msgs = self.panda.can_recv() or []
      if msgs:
        frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
        self.parser.update([(time.time_ns(), frames)])
      
      raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
      sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
      angle = -raw_angle if sign_angle == 1 else raw_angle
      
      raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
      sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
      velocity = -raw_vel if sign_vel == 1 else raw_vel
      
      # Stop if we are within 0.5 degrees of center and moving very slowly
      if abs(angle) <= 0.5 and abs(velocity) <= 2.0:
        success = True
        break
        
      # Determine return direction: steer opposite to current angle sign
      dir_sign = 1 if angle > 0 else 0
      
      if abs(angle) > 20.0:
        torque = 500
      else:
        # Linear ramp down from 500 at 20.0 deg to 130 at 0.5 deg
        ratio = (abs(angle) - 0.5) / (20.0 - 0.5)
        ratio = max(0.0, min(1.0, ratio))
        torque = int(130 + (500 - 130) * ratio)
        
        # Apply velocity damping to prevent overshoot
        torque = max(100, torque - int(1.5 * abs(velocity)))
        
      # Apply target maximum torque limit
      target_max_torque = max(target_max_torque, 500)
      if torque > target_max_torque:
        torque = target_max_torque
        
      # Scale down torque smoothly very close to center to prevent micro-oscillations
      if abs(angle) < 0.5:
        torque = int(torque * (abs(angle) / 0.5))
        
      # Send active torque command in main thread (Status 7)
      if now - last_send >= 0.01:
        self.send_hca_cmd(torque, dir_sign, 7) # Command Status 7
        last_send = now
        
      if now - last_print >= 0.1:
        hca_status_feedback = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        sys.stdout.write(f"\r[Centering] Angle: {angle:6.2f}° | Vel: {velocity:6.2f}°/s | Cmd Torque: {torque:3d} cNm | EPS HCA Status: {hca_status_feedback} ({HCA_STATES.get(hca_status_feedback, 'UNKNOWN')})   ")
        sys.stdout.flush()
        last_print = now
        
      time.sleep(0.002)
      
    # Phase 3: Hold center position with 0 torque and Ready status to completely stabilize (synchronously in main thread)
    stabilize_start = time.perf_counter()
    while time.perf_counter() - stabilize_start < 1.2:
      now = time.perf_counter()
      
      msgs = self.panda.can_recv() or []
      if msgs:
        frames = [(rx_addr, rx_data, rx_bus) for rx_addr, rx_data, rx_bus in msgs]
        self.parser.update([(time.time_ns(), frames)])
        
      if now - last_send >= 0.01:
        self.send_hca_cmd(0, 0, 3)
        last_send = now
      time.sleep(0.005)

    if success:
      print("\n[Auto-Centering] Wheel stabilized at center.")
    else:
      print("\n[Auto-Centering] \033[93mWarning: Centering timed out before reaching target.\033[0m")

    # Restart background daemon thread for standby keepalive
    self.daemon_running = True
    self.daemon_thread = threading.Thread(target=self._bg_daemon_loop)
    self.daemon_thread.daemon = True
    self.daemon_thread.start()
    self.start_keepalive("HCA")

  def save_test_data(self, test_name, target_torque, data_log, label=None):
    log_dir = "steering_test_logs"
    if not os.path.exists(log_dir):
      os.makedirs(log_dir)
      
    timestamp = int(time.time())
    suffix = f"_{label}" if label else ""
    filename = os.path.join(log_dir, f"torque_{test_name}_{target_torque}cNm_{timestamp}{suffix}.csv")
    
    try:
      with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_sec", "commanded_torque", "wheel_angle", "angular_velocity", "driver_torque", "lh3_lm"])
        for row in data_log:
          writer.writerow(row)
      print(f"\n[Data Saved] Saved test log to {filename}")
    except Exception as e:
      print(f"\n\033[91mError saving log file: {e}\033[0m")

  def render_velocity_timeline(self, data_log, duration=5.0):
    if not data_log:
      return
    print("\n====================================================")
    print(f"          ANGULAR VELOCITY TIMELINE ({duration:.1f}s FULL RUN)  ")
    print("====================================================")
    
    num_buckets = 50
    buckets = [[] for _ in range(num_buckets)]
    
    for elapsed, _, _, velocity, _, _ in data_log:
      if elapsed <= duration:
        idx = int((elapsed / duration) * num_buckets)
        if idx < num_buckets:
          buckets[idx].append(abs(velocity))
          
    plot_vals = []
    for b in buckets:
      if b:
        plot_vals.append(sum(b) / len(b))
      else:
        plot_vals.append(0.0)
        
    max_val = max(plot_vals) if plot_vals else 1.0
    if max_val < 5.0:
      max_val = 5.0
      
    # Draw ASCII graph
    for y_level in range(10, -1, -1):
      threshold = (y_level / 10) * max_val
      line = f" {threshold:5.1f}°/s | "
      for val in plot_vals:
        if val >= threshold and val > 0.5:
          line += "█"
        elif val >= threshold - (max_val / 20) and val > 0.5:
          line += "▄"
        else:
          line += " "
      print(line)
      
    print("        " + "-" * (num_buckets + 3))
    
    # Dynamic X axis labels
    labels_line = "  Time:  0.0s"
    step = duration / 4.0
    labels_line += f" ... {step:.1f}s ... {2*step:.1f}s ... {3*step:.1f}s ... {duration:.1f}s"
    print(labels_line)

  def render_angle_timeline(self, data_log, duration=5.0):
    if not data_log:
      return
    print("\n====================================================")
    print(f"          STEERING WHEEL ANGLE TIMELINE ({duration:.1f}s RUN)   ")
    print("====================================================")
    
    num_buckets = 50
    buckets = [[] for _ in range(num_buckets)]
    
    for elapsed, _, angle, _, _, _ in data_log:
      if elapsed <= duration:
        idx = int((elapsed / duration) * num_buckets)
        if idx < num_buckets:
          buckets[idx].append(abs(angle))
          
    plot_vals = []
    for b in buckets:
      if b:
        plot_vals.append(sum(b) / len(b))
      else:
        plot_vals.append(0.0)
        
    max_val = max(plot_vals) if plot_vals else 1.0
    if max_val < 5.0:
      max_val = 5.0
      
    # Draw ASCII graph
    for y_level in range(10, -1, -1):
      threshold = (y_level / 10) * max_val
      line = f" {threshold:5.1f}°   | "
      for val in plot_vals:
        if val >= threshold and val > 0.1:
          line += "█"
        elif val >= threshold - (max_val / 20) and val > 0.1:
          line += "▄"
        else:
          line += " "
      print(line)
      
    print("        " + "-" * (num_buckets + 3))
    
    # Dynamic X axis labels
    labels_line = "  Time:  0.0s"
    step = duration / 4.0
    labels_line += f" ... {step:.1f}s ... {2*step:.1f}s ... {3*step:.1f}s ... {duration:.1f}s"
    print(labels_line)

  def run_ramped_torque_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("         RAMPED TORQUE VELOCITY RESPONSE TEST       ")
    print("====================================================")
    
    try:
      torque_input = input("Enter target torque to apply (cNm, default: 150): ")
      target_torque = int(torque_input) if torque_input.strip() else 150
      
      ramp_input = input("Enter ramp duration (seconds, default: 1.5): ")
      ramp_dur = float(ramp_input) if ramp_input.strip() else 1.5
      
      duration_input = input("Enter total test duration (seconds, default: 5.0): ")
      duration = float(duration_input) if duration_input.strip() else 5.0
      
      dir_input = input("Enter direction (L/R, default: L): ").strip().upper()
      direction = 1 if dir_input == 'R' else 0
      
      label_input = input("Enter custom name/label to append to filename (optional): ").strip()
      label = label_input if label_input else None
    except ValueError:
      print("Invalid inputs. Using defaults: Torque=150 cNm, Ramp=1.5s, Duration=5.0s, Direction=Left")
      target_torque = 150
      ramp_dur = 1.5
      duration = 5.0
      direction = 0
      label = None

    print("\nInitializing HCA...")
    self.panda.can_recv()
    
    # Center wheel first
    self.auto_center_wheel_strong()
    
    self.start_keepalive("HCA")
    init_start = time.perf_counter()
    initialized = False
    while time.perf_counter() - init_start < 4.0:
      self.update_parser()
      hca_status = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
      if hca_status in (3, 5, 7):
        initialized = True
        break
      time.sleep(0.01)
      
    if not initialized:
      self.stop_keepalive()
      print("\033[91mHCA failed to initialize.\033[0m")
      input("Press Enter to return...")
      return
      
    self.stop_keepalive()
    print("\nStarting ramped torque test...")
    
    t_start = time.perf_counter()
    last_send = 0
    last_render = 0
    
    data_log = []
    peak_vel = 0.0
    peak_angle = 0.0
    
    try:
      while True:
        now = time.perf_counter()
        elapsed = now - t_start
        if elapsed >= duration:
          break
          
        if elapsed < ramp_dur:
          cmd_torque = int((elapsed / ramp_dur) * target_torque)
        else:
          cmd_torque = target_torque
          
        self.update_parser()
        
        raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
        sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
        angle = -raw_angle if sign_angle == 1 else raw_angle

        raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
        sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
        velocity = -raw_vel if sign_vel == 1 else raw_vel
        
        lh3_lm = self.parser.vl["Lenkhilfe_3"]["LH3_LM"]
        lh3_lmsign = self.parser.vl["Lenkhilfe_3"]["LH3_LMSign"]
        driver_torque = -lh3_lm if lh3_lmsign == 1 else lh3_lm
        
        data_log.append((elapsed, cmd_torque, angle, velocity, driver_torque, lh3_lm))
        
        if abs(velocity) > peak_vel:
          peak_vel = abs(velocity)
        if abs(angle) > peak_angle:
          peak_angle = abs(angle)
          
        if now - last_send >= 0.01:
          self.send_hca_cmd(cmd_torque, direction, 5) # status 5
          last_send = now
          
        if now - last_render >= 0.1:
          self.render_large_state(cmd_torque, direction, angle, velocity)
          last_render = now
          
        time.sleep(0.002)
        
    finally:
      for _ in range(10):
        self.send_hca_cmd(0, 0, 3)
        time.sleep(0.01)
        
    print("\n====================================================")
    print("                 TEST RAMP COMPLETED                ")
    print("====================================================")
    print(f" Target Torque       : {target_torque} cNm")
    print(f" Ramp Duration       : {ramp_dur} seconds")
    print(f" Total Duration      : {duration} seconds")
    print(f" Direction           : {'Right' if direction else 'Left'}")
    print(f" Peak Velocity       : \033[92m{peak_vel:.2f}°/s\033[0m")
    print(f" Peak Angle          : \033[92m{peak_angle:.2f}°\033[0m")
    
    self.render_velocity_timeline(data_log, duration=duration)
    self.render_angle_timeline(data_log, duration=duration)
    self.save_test_data("ramped", target_torque, data_log, label=label)
    self.auto_center_wheel_strong()
    input("\nPress Enter to return to main menu...")

  def run_instant_torque_test(self, target_torque=None, direction=None, hca_status=5, duration=5.0, label=None, auto_run=False):
    if not auto_run:
      print("\033[H\033[J")
      print("====================================================")
      print("         INSTANT TORQUE VELOCITY RESPONSE TEST       ")
      print("====================================================")
      
      try:
        torque_input = input("Enter target torque to apply (cNm, default: 150): ")
        target_torque = int(torque_input) if torque_input.strip() else 150
        
        duration_input = input("Enter test duration (seconds, default: 5.0): ")
        duration = float(duration_input) if duration_input.strip() else 5.0
        
        dir_input = input("Enter direction (L/R, default: L): ").strip().upper()
        direction = 1 if dir_input == 'R' else 0
        
        status_input = input("Enter HCA Status (5 or 7, default: 5): ").strip()
        hca_status = int(status_input) if status_input in ('5', '7') else 5
        
        label_input = input("Enter custom name/label to append to filename (optional): ").strip()
        label = label_input if label_input else None
      except ValueError:
        print("Invalid inputs. Using defaults: Torque=150 cNm, Duration=5.0s, Direction=Left, HCA_Status=5")
        target_torque = 150
        duration = 5.0
        direction = 0
        hca_status = 5
        label = None

    if not auto_run:
      print("\nInitializing HCA...")
      self.panda.can_recv()
      self.auto_center_wheel_strong()
      
      self.start_keepalive("HCA")
      init_start = time.perf_counter()
      initialized = False
      while time.perf_counter() - init_start < 4.0:
        self.update_parser()
        hca_status_feedback = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
        if hca_status_feedback in (3, 5, 7):
          initialized = True
          break
        time.sleep(0.01)
        
      if not initialized:
        self.stop_keepalive()
        print("\033[91mHCA failed to initialize.\033[0m")
        input("Press Enter to return...")
        return
        
      self.stop_keepalive()
      print("\nStarting instant torque test...")
      
    t_start = time.perf_counter()
    last_send = 0
    last_render = 0
    
    data_log = []
    peak_vel = 0.0
    peak_angle = 0.0
    
    try:
      while True:
        now = time.perf_counter()
        elapsed = now - t_start
        if elapsed >= duration:
          break
          
        self.update_parser()
        
        raw_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW"]
        sign_angle = self.parser.vl["Lenkwinkel_1"]["LW1_LRW_Sign"]
        angle = -raw_angle if sign_angle == 1 else raw_angle

        raw_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"]
        sign_vel = self.parser.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"]
        velocity = -raw_vel if sign_vel == 1 else raw_vel
        
        lh3_lm = self.parser.vl["Lenkhilfe_3"]["LH3_LM"]
        lh3_lmsign = self.parser.vl["Lenkhilfe_3"]["LH3_LMSign"]
        driver_torque = -lh3_lm if lh3_lmsign == 1 else lh3_lm
        
        data_log.append((elapsed, target_torque, angle, velocity, driver_torque, lh3_lm))
        
        if abs(velocity) > peak_vel:
          peak_vel = abs(velocity)
        if abs(angle) > peak_angle:
          peak_angle = abs(angle)
          
        if now - last_send >= 0.01:
          self.send_hca_cmd(target_torque, direction, hca_status)
          last_send = now
          
        if now - last_render >= 0.1:
          self.render_large_state(target_torque, direction, angle, velocity)
          last_render = now
          
        time.sleep(0.002)
        
    finally:
      for _ in range(10):
        self.send_hca_cmd(0, 0, 3)
        time.sleep(0.01)
        
    if not auto_run:
      print("\n====================================================")
      print("                 TEST COMPLETED                      ")
      print("====================================================")
      print(f" Target Torque       : {target_torque} cNm")
      print(f" Total Duration      : {duration} seconds")
      print(f" Direction           : {'Right' if direction else 'Left'}")
      print(f" Peak Velocity       : \033[92m{peak_vel:.2f}°/s\033[0m")
      print(f" Peak Angle          : \033[92m{peak_angle:.2f}°\033[0m")
      
      self.render_velocity_timeline(data_log, duration=duration)
      self.render_angle_timeline(data_log, duration=duration)
      self.save_test_data("instant", target_torque, data_log, label=label)
      self.auto_center_wheel_strong()
      input("\nPress Enter to return to main menu...")
      
    return data_log, peak_vel, peak_angle

  def run_automated_sweep_test(self):
    print("\033[H\033[J")
    print("====================================================")
    print("         AUTOMATED TORQUE SWEEP TEST SUITE          ")
    print("====================================================")
    
    try:
      dir_input = input("Enter sweep direction (L/R, default: L): ").strip().upper()
      direction = 1 if dir_input == 'R' else 0
      
      status_input = input("Enter HCA Active Status (5 or 7, default: 5): ").strip()
      hca_status = int(status_input) if status_input in ('5', '7') else 5
      
      duration_input = input("Enter test duration for each run (seconds, default: 5.0): ")
      duration = float(duration_input) if duration_input.strip() else 5.0
      
      print("\nSelect torque sweep configuration mode:")
      print("  [1] Define by Range (min, max, step/increment)")
      print("  [2] Define by Explicit List (comma-separated)")
      mode_input = input("Choose mode (1 or 2, default: 1): ").strip()
      
      if mode_input == "2":
        sweep_input = input("Enter torque levels to sweep (comma-separated, default: 50,100,150,200,250,300): ")
        if sweep_input.strip():
          sweep_torques = [int(t.strip()) for t in sweep_input.split(",")]
        else:
          sweep_torques = [50, 100, 150, 200, 250, 300]
      else:
        min_input = input("Enter minimum torque (cNm, default: 50): ").strip()
        min_t = int(min_input) if min_input else 50
        
        max_input = input("Enter maximum torque (cNm, default: 300): ").strip()
        max_t = int(max_input) if max_input else 300
        
        step_input = input("Enter step/increment increment (cNm, default: 50): ").strip()
        step_t = int(step_input) if step_input else 50
        
        sweep_torques = list(range(min_t, max_t + 1, step_t))
        
      label_input = input("Enter custom name/label to append to sweep filenames (optional): ").strip()
      label = label_input if label_input else None
        
    except ValueError:
      print("Invalid inputs. Using defaults: Sweep=[50, 100, 150, 200, 250, 300] cNm, Direction=Left, HCA_Status=5, Duration=5.0s")
      sweep_torques = [50, 100, 150, 200, 250, 300]
      direction = 0
      hca_status = 5
      duration = 5.0
      label = None

    print(f"\nSweep plan: {sweep_torques} cNm in {'Right' if direction else 'Left'} direction (HCA Status: {hca_status}, Duration: {duration}s).")
    print("Please clear hands/obstacles from the steering wheel.")
    input("Press Enter to begin the automated sweep...")
    
    print("\nInitializing HCA...")
    self.panda.can_recv()
    self.start_keepalive("HCA")
    init_start = time.perf_counter()
    initialized = False
    while time.perf_counter() - init_start < 4.0:
      self.update_parser()
      hca_status_feedback = int(self.parser.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
      if hca_status_feedback in (3, 5, 7):
        initialized = True
        break
      time.sleep(0.01)
      
    if not initialized:
      self.stop_keepalive()
      print("\033[91mHCA failed to initialize.\033[0m")
      input("Press Enter to return...")
      return
      
    self.stop_keepalive()
    summary_data = []
    
    try:
      for idx, torque in enumerate(sweep_torques):
        print(f"\n====================================================")
        print(f"          SWEEP RUN {idx+1}/{len(sweep_torques)}: {torque} cNm          ")
        print("====================================================")
        
        self.auto_center_wheel_strong()
        print("Settling wheel position...")
        time.sleep(1.0)
        
        print(f"Applying instant {torque} cNm command...")
        data_log, peak_vel, peak_angle = self.run_instant_torque_test(torque, direction, hca_status=hca_status, duration=duration, label=label, auto_run=True)
        
        self.save_test_data("sweep_run", torque, data_log, label=label)
        
        time_points = [0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0]
        vel_at_times = {}
        for tp in time_points:
          closest_entry = min(data_log, key=lambda x: abs(x[0] - tp))
          vel_at_times[tp] = abs(closest_entry[3])
          
        summary_data.append({
          "torque": torque,
          "peak": peak_vel,
          "peak_ang": peak_angle,
          "v_0_1": vel_at_times[0.1],
          "v_0_2": vel_at_times[0.2],
          "v_0_5": vel_at_times[0.5],
          "v_1_0": vel_at_times[1.0],
          "v_2_0": vel_at_times[2.0],
          "v_3_0": vel_at_times[3.0],
          "v_5_0": vel_at_times[5.0],
        })
        
        print(f"Peak velocity: {peak_vel:.2f}°/s | Peak angle: {peak_angle:.2f}°")
        time.sleep(0.5)
        
      self.auto_center_wheel_strong()
      
      print("\n\n=================================================================================================")
      print("                                  AUTOMATED SWEEP SUMMARY REPORT                                 ")
      print("=================================================================================================")
      print(" TORQUE | PEAK VEL | PEAK ANG | VEL@0.1s | VEL@0.2s | VEL@0.5s | VEL@1.0s | VEL@2.0s | VEL@3.0s | VEL@5.0s ")
      print(" (cNm)  |  (deg/s) |   (deg)  |  (deg/s) |  (deg/s) |  (deg/s) |  (deg/s) |  (deg/s) |  (deg/s) |  (deg/s) ")
      print("-------------------------------------------------------------------------------------------------")
      for row in summary_data:
        print(f"  {row['torque']:5d} |  {row['peak']:7.2f} |  {row['peak_ang']:7.2f} |  {row['v_0_1']:7.2f} |  {row['v_0_2']:7.2f} |  {row['v_0_5']:7.2f} |  {row['v_1_0']:7.2f} |  {row['v_2_0']:7.2f} |  {row['v_3_0']:7.2f} |  {row['v_5_0']:7.2f} ")
      print("================================================================================================-")
      
      log_dir = "steering_test_logs"
      if not os.path.exists(log_dir):
        os.makedirs(log_dir)
      timestamp = int(time.time())
      suffix = f"_{label}" if label else ""
      summary_filename = os.path.join(log_dir, f"torque_sweep_summary_{timestamp}{suffix}.csv")
      with open(summary_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["torque_cNm", "peak_vel_degs", "peak_angle_deg", "vel_0.1s", "vel_0.2s", "vel_0.5s", "vel_1.0s", "vel_2.0s", "vel_3.0s", "vel_5.0s"])
        for row in summary_data:
          writer.writerow([row["torque"], row["peak"], row["peak_ang"], row["v_0_1"], row["v_0_2"], row["v_0_5"], row["v_1_0"], row["v_2_0"], row["v_3_0"], row["v_5_0"]])
      print(f"\n[Summary Saved] Saved sweep summary to {summary_filename}")
      
    except KeyboardInterrupt:
      print("\n\n\033[91mSweep Interrupted! Safely centering wheel...\033[0m")
      self.auto_center_wheel_strong()
      
    input("\nPress Enter to return to main menu...")

  def run_monitor_0x200_0x201(self):
    print("\033[H\033[J")
    print("====================================================")
    print("        MONITOR & LOG 0x200 / 0x201 CAN MESSAGES    ")
    print("====================================================")
    print("Stopping background daemon thread...")
    
    # Safely stop and join background daemon thread
    self.stop_keepalive()
    self.daemon_running = False
    if self.daemon_thread:
      self.daemon_thread.join(timeout=0.3)
      self.daemon_thread = None

    log_dir = "steering_test_logs"
    if not os.path.exists(log_dir):
      os.makedirs(log_dir)

    timestamp = int(time.time())
    filename = os.path.join(log_dir, f"monitor_0x200_0x201_{timestamp}.csv")
    
    print(f"Logging matched CAN frames to: {filename}")
    print("Press Ctrl+C to stop monitoring and return to menu.")
    print("--------------------------------------------------------------------------------")
    print("   Time (s)   | Bus | Message ID | Data (Hex)")
    print("--------------------------------------------------------------------------------")

    try:
      self.panda.can_recv() # Clear buffer
      
      with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_sec", "bus", "can_id_hex", "can_id_dec", "data_hex"])
        
        t_start = time.perf_counter()
        
        while True:
          msgs = self.panda.can_recv() or []
          for addr, dat, bus in msgs:
            if addr in (0x200, 0x201):
              elapsed = time.perf_counter() - t_start
              data_hex = dat.hex()
              id_hex = f"0x{addr:03X}"
              
              writer.writerow([f"{elapsed:.6f}", bus, id_hex, addr, data_hex])
              csvfile.flush()
              
              print(f" {elapsed:12.6f} |  {bus:1d}  |    {id_hex}   | {data_hex}")
              
          time.sleep(0.005)
          
    except KeyboardInterrupt:
      print("\nMonitoring stopped.")
    finally:
      # Restart background daemon thread for standby keepalive
      self.daemon_running = True
      self.daemon_thread = threading.Thread(target=self._bg_daemon_loop)
      self.daemon_thread.daemon = True
      self.daemon_thread.start()
      self.start_keepalive("HCA")
      input("\nPress Enter to return to the main menu...")

  def run_raw_can_send(self):
    print("\033[H\033[J")
    print("====================================================")
    print("            RAW CAN ID SEND TOOL                    ")
    print("====================================================")
    print("Send arbitrary CAN frames by specifying a hex ID and")
    print("hex payload bytes. Payload is zero-padded to 8 bytes.")
    print("====================================================")
    print("\nExamples:")
    print("  ID: 7e0   Payload: 02 01 00")
    print("  ID: 200   Payload: ff,01,aa     (commas ok)")
    print("  ID: 300   Payload:              (sends 8x 00)")
    print("")

    last_arb_id = None
    last_payload = None

    try:
      while True:
        # --- Get CAN ID ---
        if last_arb_id is not None:
          id_prompt = f"CAN ID hex [Enter=0x{last_arb_id:03X}] (q to quit): "
        else:
          id_prompt = "CAN ID hex (q to quit): "
        id_input = input(id_prompt).strip()

        if id_input.lower() == 'q':
          break

        if id_input == '' and last_arb_id is not None:
          arb_id = last_arb_id
        else:
          try:
            arb_id = int(id_input, 16)
          except ValueError:
            print("\033[91mInvalid hex ID. Try again.\033[0m")
            continue

        # --- Get payload ---
        if last_payload is not None:
          pay_prompt = f"Payload hex bytes [Enter={' '.join(f'{b:02X}' for b in last_payload)}]: "
        else:
          pay_prompt = "Payload hex bytes (space/comma separated, default all 00): "
        pay_input = input(pay_prompt).strip()

        if pay_input == '' and last_payload is not None:
          payload = list(last_payload)
        elif pay_input == '':
          payload = []
        else:
          # Accept space or comma separated hex bytes
          pay_input = pay_input.replace(',', ' ')
          tokens = pay_input.split()
          try:
            payload = [int(t, 16) & 0xFF for t in tokens]
          except ValueError:
            print("\033[91mInvalid hex payload bytes. Try again.\033[0m")
            continue

        # Pad or truncate to 8 bytes
        if len(payload) > 8:
          print(f"\033[93mWarning: payload truncated from {len(payload)} to 8 bytes.\033[0m")
          payload = payload[:8]
        while len(payload) < 8:
          payload.append(0x00)

        last_arb_id = arb_id
        last_payload = list(payload)

        dat = bytes(payload)

        # --- Send ---
        try:
          self.panda.can_send(arb_id, dat, self.bus)
          print(f"\033[92mSENT\033[0m  0x{arb_id:03X}  [{' '.join(f'{b:02X}' for b in dat)}]  bus={self.bus}")
        except Exception as e:
          print(f"\033[91mSend failed: {e}\033[0m")
          continue

        # --- Listen for responses briefly (500ms) ---
        print("Listening for responses (500ms)...")
        listen_start = time.perf_counter()
        rx_count = 0
        while time.perf_counter() - listen_start < 0.5:
          try:
            msgs = self.panda.can_recv() or []
          except Exception:
            msgs = []
          for rx_addr, rx_data, rx_bus in msgs:
            # Show all traffic, but highlight responses to our ID
            tag = "\033[96m>>\033[0m" if rx_addr == arb_id else "  "
            hex_dat = ' '.join(f'{b:02X}' for b in rx_data)
            print(f"  {tag} 0x{rx_addr:03X}  [{hex_dat}]  bus={rx_bus}")
            rx_count += 1
          time.sleep(0.01)

        if rx_count == 0:
          print("  (no CAN traffic received)")
        print("")

    except KeyboardInterrupt:
      print("\n")

    input("Press Enter to return to the main menu...")

  def main_menu(self):
    while True:
      print("\033[H\033[J")
      print("====================================================")
      print("      AUDI TT MK2 STEERING DIAGNOSTIC TEST UTILITY  ")
      print("====================================================")
      print("  [1] Validate PLA Capability (Status 8 Test)")
      print("  [2] Profile HCA Step Response Delay (Ready -> Active)")
      print("  [3] Profile HCA Step Response Delay (Active Low -> High)")
      print("  [4] Execute PLA Controlled Angle Sweep Test")
      print("  [5] Launch Real-Time CAN Signal Dashboard")
      print("  [6] Interactive Keyboard Steering Control")
      print("  [7] Automated Step Response Delay Test (10 Runs)")
      print("  [8] Profile Ramped Torque Velocity Response (5s)")
      print("  [9] Profile Instant Full Torque Velocity Response (5s with Timeline Plot)")
      print("  [10] Execute Automated Torque Sweep Test Suite")
      print("  [11] Monitor and Log 0x200 & 0x201 CAN Messages")
      print("  [12] Send Raw CAN ID (arbitrary frame sender)")
      print("  [13] Exit")
      print("====================================================")
      
      choice = input("Enter choice (1-13): ").strip()
      if choice == "1":
        self.run_pla_test()
      elif choice == "2":
        self.run_hca_step_test()
      elif choice == "3":
        self.run_hca_active_step_test()
      elif choice == "4":
        self.run_pla_sweep_test()
      elif choice == "5":
        self.run_can_dashboard()
      elif choice == "6":
        self.run_interactive_control()
      elif choice == "7":
        self.run_hca_auto_delay_test()
      elif choice == "8":
        self.run_ramped_torque_test()
      elif choice == "9":
        self.run_instant_torque_test()
      elif choice == "10":
        self.run_automated_sweep_test()
      elif choice == "11":
        self.run_monitor_0x200_0x201()
      elif choice == "12":
        self.run_raw_can_send()
      elif choice == "13":
        print("\nExiting. Safe travels!")
        break
      else:
        print("\033[91mInvalid choice. Please try again.\033[0m")
        time.sleep(1)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Test Audi TT MK2 steering capabilities (PLA and HCA)")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus to use (default: 0)")
  args = parser.parse_args()

  check_pandad()
  
  tester = AudiSteeringTester(bus=args.bus)
  tester.connect()
  tester.main_menu()
