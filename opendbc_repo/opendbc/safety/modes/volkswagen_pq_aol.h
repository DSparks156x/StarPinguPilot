#pragma once

// StarPinguPilot: always-on lateral for Volkswagen PQ with stock cruise (e.g. Audi TT Mk2, plain cruise control).
// Modeled on volkswagen_stock_aol.h. Lenkhilfe_2 and Gate_Komf_1 are not rx checks, so they arrive via optional_rx. The lateral arm (token) comes from an observed cruise main switch off->on cycle
// or a stock cruise engagement; a main switch already on at startup never arms, since PQ stalks latch. Permission also
// needs the main switch on, motion, not in reverse, a ready EPS and a fresh host request.
// Must match VolkswagenSafetyFlags.PQ_AOL in opendbc/car/volkswagen/values.py
#define FLAG_VOLKSWAGEN_PQ_AOL 1024U
#define VW_PQ_AOL_ALT_EXP 32U
#define VW_PQ_AOL_FRESH_US 300000U

#define MSG_LENKHILFE_2         0x3D2U   // RX from EPS, HCA status
#define MSG_GATE_KOMF_1         0x390U   // RX from J519, reverse light

static bool vw_pq_aol_enabled = false;
static bool vw_pq_aol_token = false;
static bool vw_pq_aol_main_neutral = false;
static bool vw_pq_aol_main = false;
static bool vw_pq_aol_cruise_engaged = false;
static bool vw_pq_aol_requested = false;
static bool vw_pq_aol_not_reverse = false;
static bool vw_pq_aol_eps_ready = false;
static uint8_t vw_pq_aol_seen = 0U;  // 1 = Motor_5, 2 = Gate_Komf_1, 4 = Lenkhilfe_2
static uint32_t vw_pq_aol_main_ts = 0U;
static uint32_t vw_pq_aol_gear_ts = 0U;
static uint32_t vw_pq_aol_eps_ts = 0U;

static void vw_pq_aol_clear(void) {
  vw_pq_aol_token = false;
}

static void vw_pq_aol_reset(void) {
  vw_pq_aol_clear();
  vw_pq_aol_enabled = false;
  vw_pq_aol_main_neutral = false;
  vw_pq_aol_main = false;
  vw_pq_aol_cruise_engaged = false;
  vw_pq_aol_requested = false;
  vw_pq_aol_not_reverse = false;
  vw_pq_aol_eps_ready = false;
  vw_pq_aol_seen = 0U;
  vw_pq_aol_main_ts = 0U;
  vw_pq_aol_gear_ts = 0U;
  vw_pq_aol_eps_ts = 0U;
}

static bool vw_pq_aol_current(void) {
  const uint32_t now = microsecond_timer_get();
  return (vw_pq_aol_seen == 7U) &&
    (safety_get_ts_elapsed(now, vw_pq_aol_main_ts) <= VW_PQ_AOL_FRESH_US) &&
    (safety_get_ts_elapsed(now, vw_pq_aol_gear_ts) <= VW_PQ_AOL_FRESH_US) &&
    (safety_get_ts_elapsed(now, vw_pq_aol_eps_ts) <= VW_PQ_AOL_FRESH_US);
}

static void vw_pq_aol_request(uint8_t mask) {
  vw_pq_aol_requested |= mask != 0U;
}

static uint8_t vw_pq_aol_request_mask(void) {
  const bool expired = vw_pq_aol_requested &&
    (safety_get_ts_elapsed(microsecond_timer_get(), aol_host_request_ts) > AOL_HOST_REQUEST_TIMEOUT_US);
  uint8_t request = 0U;
  if (!vw_pq_aol_enabled || relay_malfunction || expired || safety_rx_checks_invalid ||
      !heartbeat_engaged || !aol_rx_healthy() || !vw_pq_aol_current()) {
    vw_pq_aol_clear();
  } else {
    request = aol_host_axis_mask;
  }
  return request;
}

static uint8_t vw_pq_aol_permission(void) {
  const uint8_t request = vw_pq_aol_request_mask();
  uint8_t permission = 0U;
  if (vw_pq_aol_main && vehicle_moving && vw_pq_aol_not_reverse && vw_pq_aol_eps_ready && vw_pq_aol_token) {
    permission = request & 1U;  // lateral only, never longitudinal
  }
  return permission;
}

static void vw_pq_aol_rx(const CANPacket_t *msg, uint8_t bus) {
  if (vw_pq_aol_enabled) {
    if (msg_matches(msg, MSG_LENKHILFE_2, bus)) {
      // Signal: Lenkhilfe_2.LH2_Sta_HCA (0 disabled, 1 init, 2 fault, 3 ready, 4 rejected, 5/7 active)
      const unsigned int eps = msg->data[2] & 15U;
      vw_pq_aol_seen |= 4U;
      vw_pq_aol_eps_ts = microsecond_timer_get();
      vw_pq_aol_eps_ready = (eps == 3U) || (eps == 5U) || (eps == 7U);
      if ((eps != 1U) && (eps != 3U) && (eps != 4U) && (eps != 5U) && (eps != 7U)) {
        vw_pq_aol_clear();
      }
    } else if (msg_matches(msg, MSG_GATE_KOMF_1, bus)) {
      // Signal: Gate_Komf_1.GK1_Rueckfahr (reverse light)
      vw_pq_aol_seen |= 2U;
      vw_pq_aol_gear_ts = microsecond_timer_get();
      vw_pq_aol_not_reverse = !GET_BIT(msg, 28U);
    } else if (msg_matches(msg, MSG_MOTOR_2, bus)) {
      // Signal: Motor_2.MO2_Sta_GRA
      const int acc_status = (msg->data[2] & 0xC0U) >> 6;
      const bool engaged = (acc_status == 1) || (acc_status == 2);
      if (engaged && !vw_pq_aol_cruise_engaged && vw_pq_aol_main && controls_allowed) {
        vw_pq_aol_token = true;  // a fresh stock engagement arms lateral
      }
      vw_pq_aol_cruise_engaged = engaged;
    } else if (msg_matches(msg, MSG_MOTOR_5, bus)) {
      // Signal: Motor_5.MO5_GRA_Hauptsch
      vw_pq_aol_main = GET_BIT(msg, 50U);
      vw_pq_aol_seen |= 1U;
      vw_pq_aol_main_ts = microsecond_timer_get();
      if (!heartbeat_engaged || !aol_rx_healthy() || safety_rx_checks_invalid) {
        vw_pq_aol_clear();
      } else if (!vw_pq_aol_main) {
        vw_pq_aol_clear();
        vw_pq_aol_main_neutral = true;  // only an observed OFF may prepare the next main-ON edge
      } else if (vw_pq_aol_main_neutral) {
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
