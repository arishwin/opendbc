#pragma once

#include "opendbc/safety/declarations.h"

static void perodua_rx_hook(const CANPacket_t *msg) {
  // Minimal state updates to align with openpilot's CarState

  // Brake pressed from BRAKE (161) bit 5
  if ((msg->addr == 161U) && (msg->bus == 0U)) {
    brake_pressed = GET_BIT(msg, 5U);
  }

  // Gas pressed from GAS_PEDAL_2 (399) bit 1, pressed when 0
  if ((msg->addr == 399U) && (msg->bus == 0U)) {
    gas_pressed = !GET_BIT(msg, 1U);
  }

  // Fallback gas from PCM_BUTTONS (520) PEDAL_DEPRESSED at bit 12
  if ((msg->addr == 520U) && (msg->bus == 0U)) {
    gas_pressed = GET_BIT(msg, 12U);
  }

  // ACC availability and engagement from ACC_CMD_HUD (627) on cam bus 2
  if ((msg->addr == 627U) && (msg->bus == 2U)) {
    acc_main_on = GET_BIT(msg, 9U);  // SET_ME_1_2
    // consider any non-zero ACC_CMD as engaged (allow controls)
    uint16_t acc_cmd = (uint16_t)(GET_BYTES(msg, 2, 2) & 0xFFFFU);
    if (acc_cmd > 0U) {
      pcm_cruise_check(true);
    }
  }
  vehicle_moving = true;
  controls_allowed = true;
  SAFETY_UNUSED(msg);
}

static bool perodua_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  int len = GET_LEN(msg);
  SAFETY_UNUSED(msg->addr);
  SAFETY_UNUSED(len);

  return tx;
}

static safety_config perodua_init(uint16_t param) {
  SAFETY_UNUSED(param);
  static const CanMsg PERODUA_TX_MSGS[] = {
    {464, 0, 8, .check_relay = true},
    {628, 0, 8, .check_relay = true},
    {625, 0, 8, .check_relay = true},
    {627, 0, 8, .check_relay = true},
    {520, 0, 6, .check_relay = false},  // PCM_BUTTONS
    {624, 0, 8, .check_relay = false},  // ADAS_AEB (brake cmd frame)
    {0x750, 2, 8, .check_relay = false}, // Tester Present keepalive for camera
    // Allow OBD/UDS queries on common buses
    {2015, 0, 8, .check_relay = false}, // 0x7DF functional
    {2015, 1, 8, .check_relay = false}, // 0x7DF functional on OBD mux
    {417018865, 0, 8, .check_relay = false}, // 0x18DB33F1 broadcast
    {417018865, 1, 8, .check_relay = false},
    {416940273, 0, 8, .check_relay = false}, // 0x18DA00F1 tester->ECU
    {416944369, 0, 8, .check_relay = false}, // 0x18DA10F1
    {416948465, 0, 8, .check_relay = false}, // 0x18DA20F1
    {416952561, 0, 8, .check_relay = false}, // 0x18DA30F1
    {416956657, 0, 8, .check_relay = false}, // 0x18DA40F1
    {416960753, 0, 8, .check_relay = false}, // 0x18DA50F1
    {416964849, 0, 8, .check_relay = false}, // 0x18DA60F1
    {416968945, 0, 8, .check_relay = false}, // 0x18DA70F1
    {416973041, 0, 8, .check_relay = false}, // 0x18DA80F1
    {416977137, 0, 8, .check_relay = false}, // 0x18DA90F1
    {416981233, 0, 8, .check_relay = false}, // 0x18DAA0F1
    {416985329, 0, 8, .check_relay = false}, // 0x18DAB0F1
    {416989425, 0, 8, .check_relay = false}, // 0x18DAC0F1
    {416993521, 0, 8, .check_relay = false}, // 0x18DAD0F1
    {416997617, 0, 8, .check_relay = false}, // 0x18DAE0F1
    {417001713, 0, 8, .check_relay = false}, // 0x18DAF0F1
    // Commonly on OBD mux bus too
    {416940273, 1, 8, .check_relay = false},
    {416944369, 1, 8, .check_relay = false},
    {416948465, 1, 8, .check_relay = false},
    {416952561, 1, 8, .check_relay = false},
    {416956657, 1, 8, .check_relay = false},
    {416960753, 1, 8, .check_relay = false},
    {416964849, 1, 8, .check_relay = false},
    {416968945, 1, 8, .check_relay = false},
    {416973041, 1, 8, .check_relay = false},
    {416977137, 1, 8, .check_relay = false},
    {416981233, 1, 8, .check_relay = false},
    {416985329, 1, 8, .check_relay = false},
    {416989425, 1, 8, .check_relay = false},
    {416993521, 1, 8, .check_relay = false},
    {416997617, 1, 8, .check_relay = false},
    {417001713, 1, 8, .check_relay = false},
  };

  static RxCheck perodua_rx_checks[] = {
    // Minimal RX checks to ensure safety_config_valid()
    {.msg = {{608, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // WHEEL_SPEED_CLEAN
    {.msg = {{520, 0, 6, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // PCM_BUTTONS
    {.msg = {{627, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // ACC_CMD_HUD
    {.msg = {{628, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // LKAS_HUD
  };

  return BUILD_SAFETY_CFG(perodua_rx_checks, PERODUA_TX_MSGS);
}


const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx   = perodua_rx_hook,
  .tx   = perodua_tx_hook,
};
