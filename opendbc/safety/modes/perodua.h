#pragma once

#include "opendbc/safety/safety_declarations.h"

static void perodua_rx_hook(const CANPacket_t *msg) {
  // Minimal state updates to align with openpilot's CarState

  // Brake pressed from BRAKE (161) bit 5
  if ((msg->addr == 161) && (msg->bus == 0)) {
    brake_pressed = GET_BIT(msg, 5U);
  }

  // Gas pressed from GAS_PEDAL_2 (399) bit 1, pressed when 0
  if ((msg->addr == 399) && (msg->bus == 0)) {
    gas_pressed = !GET_BIT(msg, 1U);
  }

  // ACC availability and engagement from ACC_CMD_HUD (627) on cam bus 2
  if ((msg->addr == 627) && (msg->bus == 2)) {
    acc_main_on = GET_BIT(msg, 9U);  // SET_ME_1_2
    // consider any non-zero ACC_CMD as engaged (allow controls)
    uint16_t acc_cmd = (uint16_t)(GET_BYTES(msg, 2, 2) & 0xFFFFU);
    if (acc_cmd > 0U) {
      pcm_cruise_check(true);
    }
  }

  UNUSED(msg);
}

static bool perodua_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  int len = GET_LEN(msg);
  UNUSED(msg->addr);
  UNUSED(len);

  return tx;
}

static safety_config perodua_init(uint16_t param) {
  UNUSED(param);
  static const CanMsg PERODUA_TX_MSGS[] = {
    {464, 0, 8, .check_relay = false},
    {628, 0, 8, .check_relay = false},
    {625, 0, 8, .check_relay = false},
    {627, 0, 8, .check_relay = false},
    {2015, 0, 8, .check_relay = false}, // 0x7DF
  };

  static RxCheck perodua_rx_checks[] = {
    // Minimal RX checks to ensure safety_config_valid()
    {.msg = {{608, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // WHEEL_SPEED_CLEAN
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
