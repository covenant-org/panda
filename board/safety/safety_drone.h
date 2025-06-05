const CanMsg DRONE_TX_MSGS[] = {{0x265, 0, 6}, {0x266, 0, 8}, {0x267, 0, 2}};

RxCheck drone_rx_checks[] = {
//  {.msg = {{0x201, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
};

static void drone_rx_hook(const CANPacket_t *to_push) {
  // body is never at standstill
  UNUSED(to_push);
  vehicle_moving = true;

  controls_allowed = true;
}

static bool drone_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
//  return tx;
  return true;
}

static safety_config drone_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(drone_rx_checks, DRONE_TX_MSGS);
}

const safety_hooks drone_hooks = {
  .init = drone_init,
  .rx = drone_rx_hook,
  .tx = drone_tx_hook,
  .fwd = default_fwd_hook,
};
