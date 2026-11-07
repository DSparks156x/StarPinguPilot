#pragma once

// StarPinguPilot: always-on lateral for Volkswagen PQ with stock cruise (e.g. Audi TT Mk2, plain cruise control).
// Same conditions as StarPilot 6: lateral with the cruise main switch on. PQ stalks latch, so a main switch already on at
// startup needs an observed off->on cycle or a stock cruise engagement first. The arm is physical state: it never depends
// on the heartbeat, because the host only starts its heartbeat after it sees the same edge. Permission still needs a fresh
// host request. Stops, reverse and the EPS are the host's to pause.
// Must match VolkswagenSafetyFlags.PQ_AOL in opendbc/car/volkswagen/values.py
#define FLAG_VOLKSWAGEN_PQ_AOL 1024U
#define VW_PQ_AOL_ALT_EXP 32U
#define VW_PQ_AOL_FRESH_US 300000U


static bool vw_pq_aol_enabled = false;
static bool vw_pq_aol_token = false;
static bool vw_pq_aol_main_neutral = false;
static bool vw_pq_aol_main = false;
static bool vw_pq_aol_requested = false;
static bool vw_pq_aol_main_seen = false;
static uint32_t vw_pq_aol_main_ts = 0U;

static void vw_pq_aol_clear(void) {
  vw_pq_aol_token = false;
}

static void vw_pq_aol_reset(void) {
  vw_pq_aol_clear();
  vw_pq_aol_enabled = false;
  vw_pq_aol_main_neutral = false;
  vw_pq_aol_main = false;
  vw_pq_aol_requested = false;
  vw_pq_aol_main_seen = false;
  vw_pq_aol_main_ts = 0U;
}

static bool vw_pq_aol_current(void) {
  return vw_pq_aol_main_seen &&
    (safety_get_ts_elapsed(microsecond_timer_get(), vw_pq_aol_main_ts) <= VW_PQ_AOL_FRESH_US);
}

static void vw_pq_aol_request(uint8_t mask) {
  vw_pq_aol_requested |= mask != 0U;
}

static uint8_t vw_pq_aol_request_mask(void) {
  const bool expired = vw_pq_aol_requested &&
    (safety_get_ts_elapsed(microsecond_timer_get(), aol_host_request_ts) > AOL_HOST_REQUEST_TIMEOUT_US);
  uint8_t request = 0U;
  if (!vw_pq_aol_enabled || relay_malfunction || safety_rx_checks_invalid) {
    vw_pq_aol_clear();
  } else if (!expired && heartbeat_engaged && aol_rx_healthy() && vw_pq_aol_current()) {
    request = aol_host_axis_mask;
  } else {
  }
  return request;
}

static uint8_t vw_pq_aol_permission(void) {
  const uint8_t request = vw_pq_aol_request_mask();
  uint8_t permission = 0U;
  if (vw_pq_aol_main && vw_pq_aol_token) {
    permission = request & 1U;  // lateral only, never longitudinal
  }
  return permission;
}

static void vw_pq_aol_rx(const CANPacket_t *msg, uint8_t bus) {
  if (vw_pq_aol_enabled) {
    if (msg_matches(msg, MSG_MOTOR_2, bus)) {
      // Signal: Motor_2.MO2_Sta_GRA
      const int acc_status = (msg->data[2] & 0xC0U) >> 6;
      if (((acc_status == 1) || (acc_status == 2)) && vw_pq_aol_main && controls_allowed) {
        vw_pq_aol_token = true;  // a stock engagement arms lateral
      }
    } else if (msg_matches(msg, MSG_MOTOR_5, bus)) {
      // Signal: Motor_5.MO5_GRA_Hauptsch
      vw_pq_aol_main = GET_BIT(msg, 50U);
      vw_pq_aol_main_seen = true;
      vw_pq_aol_main_ts = microsecond_timer_get();
      if (!vw_pq_aol_main) {
        vw_pq_aol_clear();
        vw_pq_aol_main_neutral = true;  // only an observed OFF may prepare the next main-ON edge
      } else if (vw_pq_aol_main_neutral && !safety_rx_checks_invalid) {
        vw_pq_aol_token = true;
        vw_pq_aol_main_neutral = false;
      } else {
      }
    } else {
    }
  }
}

static void vw_pq_aol_init(uint16_t param) {
  vw_pq_aol_reset();
  vw_pq_aol_enabled = ((unsigned int)alternative_experience == VW_PQ_AOL_ALT_EXP) && GET_FLAG(param, FLAG_VOLKSWAGEN_PQ_AOL);
  if (vw_pq_aol_enabled) {
    static const AolSafetyPolicy policy = {
      .reset = vw_pq_aol_reset,
      .host_request = vw_pq_aol_request,
      .request_mask = vw_pq_aol_request_mask,
      .permission_mask = vw_pq_aol_permission,
      .rx_invalid = vw_pq_aol_clear,
    };
    aol_policy = &policy;
  }
}

// The AOL alternative experience without the AOL word (or the word with openpilot longitudinal) is a mismatched
// host configuration: send nothing rather than guess.
static bool vw_pq_aol_config_invalid(uint16_t param) {
  const bool aol_exp = (unsigned int)alternative_experience == VW_PQ_AOL_ALT_EXP;
  const bool aol_word = GET_FLAG(param, FLAG_VOLKSWAGEN_PQ_AOL);
  return (aol_exp != aol_word) || (aol_word && GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL));
}
