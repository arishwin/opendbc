const CanMsg PERODUA_TX_MSGS[] = {{464, 0, 8}, {628, 0, 8}, {625, 0, 8}, {627, 0, 8}};

RxCheck perodua_rx_checks[] = {
  //{.msg = {{0x35F, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},
};

static void perodua_rx_hook(const CANPacket_t *to_push) {
  // perodua is never at standstill
  vehicle_moving = true;
  controls_allowed = true;
  UNUSED(to_push);
}

static bool perodua_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);
  UNUSED(addr);
  UNUSED(len);

  return tx;
}

static int perodua_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  if (bus_num == 0) {
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    bool is_lkas_msg = ((addr == 464) || (addr == 628));
    bool is_acc_msg = ((addr == 625) || (addr == 627));
    bool block_msg = is_lkas_msg || is_acc_msg;
    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config perodua_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(perodua_rx_checks, PERODUA_TX_MSGS);
}


const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx = perodua_rx_hook,
  .tx = perodua_tx_hook,
  .fwd = perodua_fwd_hook,
};
