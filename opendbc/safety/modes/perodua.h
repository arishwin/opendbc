#pragma once

#include "opendbc/safety/safety_declarations.h"

static void perodua_rx_hook(const CANPacket_t *to_push) {
  // perodua is never at standstill
  vehicle_moving = true;
  controls_allowed = true;
  UNUSED(to_push);
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
//    {.msg = {{0x35F, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(perodua_rx_checks, PERODUA_TX_MSGS);
}


const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx   = perodua_rx_hook,
  .tx   = perodua_tx_hook,
};
