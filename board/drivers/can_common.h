#include "board/drivers/drivers.h"

uint32_t safety_tx_blocked = 0;
uint32_t safety_rx_invalid = 0;
uint32_t tx_buffer_overflow = 0;
uint32_t rx_buffer_overflow = 0;

can_health_t can_health[PANDA_CAN_CNT] = {{0}, {0}, {0}};

// Ignition detected from CAN meessages
bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;

bool can_silent = true;
bool can_loopback = false;

// ********************* instantiate queues *********************
#define can_buffer(x, size) \
  static CANPacket_t elems_##x[size]; \
  extern can_ring can_##x; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = (size), .elems = (CANPacket_t *)&(elems_##x) };

#define CAN_RX_BUFFER_SIZE 4096U
#define CAN_TX_BUFFER_SIZE 416U

#ifdef STM32H7
// ITCM RAM and DTCM RAM are the fastest for Cortex-M7 core access
__attribute__((section(".axisram"))) can_buffer(rx_q, CAN_RX_BUFFER_SIZE)
__attribute__((section(".itcmram"))) can_buffer(tx1_q, CAN_TX_BUFFER_SIZE)
__attribute__((section(".itcmram"))) can_buffer(tx2_q, CAN_TX_BUFFER_SIZE)
#else  // kept for PC
can_buffer(rx_q, CAN_RX_BUFFER_SIZE)
can_buffer(tx1_q, CAN_TX_BUFFER_SIZE)
can_buffer(tx2_q, CAN_TX_BUFFER_SIZE)
#endif
can_buffer(tx3_q, CAN_TX_BUFFER_SIZE)

// FIXME:
// cppcheck-suppress misra-c2012-9.3
can_ring *can_queues[PANDA_CAN_CNT] = {&can_tx1_q, &can_tx2_q, &can_tx3_q};

// ********************* interrupt safe queue *********************
bool can_pop(can_ring *q, CANPacket_t *elem) {
  bool ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1U) == q->fifo_size) {
      q->r_ptr = 0;
    } else {
      q->r_ptr += 1U;
    }
    ret = 1;
  }
  EXIT_CRITICAL();

  return ret;
}

bool can_push(can_ring *q, const CANPacket_t *elem) {
  bool ret = false;
  uint32_t next_w_ptr;

  ENTER_CRITICAL();
  if ((q->w_ptr + 1U) == q->fifo_size) {
    next_w_ptr = 0;
  } else {
    next_w_ptr = q->w_ptr + 1U;
  }
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();
  if (!ret) {
    #ifdef DEBUG
      print("can_push to ");
      if (q == &can_rx_q) {
        print("can_rx_q");
      } else if (q == &can_tx1_q) {
        print("can_tx1_q");
      } else if (q == &can_tx2_q) {
        print("can_tx2_q");
      } else if (q == &can_tx3_q) {
        print("can_tx3_q");
      } else {
        print("unknown");
      }
      print(" failed!\n");
    #endif
  }
  return ret;
}

uint32_t can_slots_empty(const can_ring *q) {
  uint32_t ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr >= q->r_ptr) {
    ret = q->fifo_size - 1U - q->w_ptr + q->r_ptr;
  } else {
    ret = q->r_ptr - q->w_ptr - 1U;
  }
  EXIT_CRITICAL();

  return ret;
}

void can_clear(can_ring *q) {
  ENTER_CRITICAL();
  q->w_ptr = 0;
  q->r_ptr = 0;
  EXIT_CRITICAL();
  // handle TX buffer full with zero ECUs awake on the bus
  refresh_can_tx_slots_available();
}

// assign CAN numbering
// bus num: CAN Bus numbers in panda, sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// forwarding bus: If >= 0, forward all messages from this bus to the specified bus.

// Helpers
// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
bus_config_t bus_config[PANDA_CAN_CNT] = {
  { .bus_lookup = 0U, .can_num_lookup = 0U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 1U, .can_num_lookup = 1U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 2U, .can_num_lookup = 2U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
};

void can_init_all(void) {
  for (uint8_t i=0U; i < PANDA_CAN_CNT; i++) {
    bus_config[i].canfd_enabled = false;
    can_clear(can_queues[i]);
    (void)can_init(i);
  }
}

void can_set_orientation(bool flipped) {
  bus_config[0].bus_lookup = flipped ? 2U : 0U;
  bus_config[0].can_num_lookup = flipped ? 2U : 0U;
  bus_config[2].bus_lookup = flipped ? 0U : 2U;
  bus_config[2].can_num_lookup = flipped ? 0U : 2U;
}

#ifdef PANDA_JUNGLE
void can_set_forwarding(uint8_t from, uint8_t to) {
  bus_config[from].forwarding_bus = to;
}
#endif

// ── J1939 / heavy truck ignition ────────────────────────────────────────────
//
// WHY THIS BLOCK EXISTS. Neither of openpilot's two ignition sources works on a
// Class 8 truck. `ignition_line` is harness_check_ignition(), a GPIO on the
// OBD-C SBU pins, and the OBD-C spec signals ignition by tying SBU2 to GND
// through 1k -- which nothing in a Deutsch-9 -> OBD-II -> comma power chain
// does, so it reads false for the life of the vehicle. `ignition_can` below is
// four hardcoded per-brand exceptions (GM/Rivian/Tesla/Mazda), none of them
// J1939. commaai/hardware's BUILD_HARNESS.md points at exactly this hook as the
// supported workaround when a vehicle provides no ignition signal.
//
// The cost of that gap, measured on the fleet: `ignition` pinned false meant a
// video-recording gate that trusted it recorded NOTHING for weeks while CAN
// recording ran fine, and the daemon has carried an RPM-plus-bus-activity
// fallback ever since. This puts the answer back where openpilot expects it.
//
// TWO THINGS HAD TO CHANGE, not one. The brand exceptions are guarded by
// `msg->bus == 0`, and the truck's J1939 reaches the panda OBD-multiplexed onto
// BUS 1 -- verified against a real capture, where 400,000 consecutive frames are
// 100% bus 1. So a J1939 case added inside the old guard would have matched
// nothing, silently. This block sits outside it and is scoped to the OBD bus.
//
// MATCHED ON EEC1 FROM THE ENGINE, SA 0x00 -- deliberately not "any EEC1".
// PGN 0xFEF1 on this same truck is transmitted by TWO ECUs, one of which sends
// N/A: 49.8% of its frames read N/A at 88 km/h, and treating those as real cost
// us a shipped-and-reverted regression. Pinning the source address means a
// second transmitter cannot answer for the first. In the capture EEC1 appears
// as exactly one id, 0x0CF00400.
//
// RPM, NOT PRESENCE. The engine ECU transmits EEC1 whenever it is POWERED, so
// presence means key-on, not running -- 8.26% of EEC1 frames in the capture
// carry rpm 0. Gating on presence would assert ignition on a parked truck with
// the key on and defeat power save. The threshold sits in a genuinely empty gap:
// rpm < 0.5 and rpm < 50 select the identical 19,643 samples, and rpm < 300 adds
// only 106 more (crank transients). 50 also mirrors the daemon's ENGINE_RPM_ON,
// so firmware and userspace agree on what "running" means.
//
// Staleness is the existing ignition_can_cnt expiry: key off, the bus dies, and
// ignition_can clears after ~2 s. That is tighter than the daemon's 12 s stale
// window and cannot be starved by a busy userspace.
#define J1939_EEC1_ADDR       0x0CF00400U   // prio 3, PGN 0xF004, SA 0x00 (engine)
#define J1939_EEC1_ADDR_MASK  0x03FFFFFFU   // ignore the 3 priority bits
#define J1939_OBD_BUS         1U
#define J1939_ENGINE_RPM_ON   400U          // 0.125 rpm/bit -> 50 rpm

void ignition_can_hook(CANPacket_t *msg) {
  // Heavy-truck J1939, on the OBD-multiplexed bus. Extended IDs only, so a
  // standard-ID vehicle can never reach this.
  if ((msg->bus == J1939_OBD_BUS) && (msg->extended != 0U) &&
      ((msg->addr & J1939_EEC1_ADDR_MASK) == (J1939_EEC1_ADDR & J1939_EEC1_ADDR_MASK)) &&
      (GET_LEN(msg) >= 5)) {
    // SPN 190 Engine Speed: bytes 4-5, little endian, 0.125 rpm/bit.
    // Cast the whole expression: C promotes the operands to int before the
    // OR, so assigning straight to uint16_t is a narrowing conversion that
    // MISRA (and -Wconversion) reject.
    uint16_t rpm_raw = (uint16_t)(((uint16_t)msg->data[3]) | (((uint16_t)msg->data[4]) << 8U));
    // 0xFB00 and above is the J1939 error/not-available range. Not running --
    // it is the ECU declining to answer, so it must neither assert ignition nor
    // refresh the expiry, exactly as the daemon's decoders treat it.
    if (rpm_raw < 0xFB00U) {
      ignition_can = rpm_raw > J1939_ENGINE_RPM_ON;
      ignition_can_cnt = 0U;
    }
  }

  if (msg->bus == 0U) {
    int len = GET_LEN(msg);

    // GM exception
    if ((msg->addr == 0x1F1U) && (len == 8)) {
      // SystemPowerMode (2=Run, 3=Crank Request)
      ignition_can = (msg->data[0] & 0x2U) != 0U;
      ignition_can_cnt = 0U;
    }

    // Rivian R1S/T GEN1 exception
    if ((msg->addr == 0x152U) && (len == 8)) {
      // 0x152 overlaps with Subaru pre-global which has this bit as the high beam
      int counter = msg->data[1] & 0xFU;  // max is only 14

      static int prev_counter_rivian = -1;
      if ((counter == ((prev_counter_rivian + 1) % 15)) && (prev_counter_rivian != -1)) {
        // VDM_OutputSignals->VDM_EpasPowerMode
        ignition_can = ((msg->data[7] >> 4U) & 0x3U) == 1U;  // VDM_EpasPowerMode_Drive_On=1
        ignition_can_cnt = 0U;
      }
      prev_counter_rivian = counter;
    }

    // Tesla Model 3/Y exception
    if ((msg->addr == 0x221U) && (len == 8)) {
      // 0x221 overlaps with Rivian which has random data on byte 0
      int counter = msg->data[6] >> 4;

      static int prev_counter_tesla = -1;
      if ((counter == ((prev_counter_tesla + 1) % 16)) && (prev_counter_tesla != -1)) {
        // VCFRONT_LVPowerState->VCFRONT_vehiclePowerState
        int power_state = (msg->data[0] >> 5U) & 0x3U;
        ignition_can = power_state == 0x3;  // VEHICLE_POWER_STATE_DRIVE=3
        ignition_can_cnt = 0U;
      }
      prev_counter_tesla = counter;
    }

    // Mazda exception
    if ((msg->addr == 0x9EU) && (len == 8)) {
      ignition_can = (msg->data[0] >> 5) == 0x6U;
      ignition_can_cnt = 0U;
    }

  }
}

bool can_tx_check_min_slots_free(uint32_t min) {
  return
    (can_slots_empty(&can_tx1_q) >= min) &&
    (can_slots_empty(&can_tx2_q) >= min) &&
    (can_slots_empty(&can_tx3_q) >= min);
}

uint8_t calculate_checksum(const uint8_t *dat, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= dat[i];
  }
  return checksum;
}

void can_set_checksum(CANPacket_t *packet) {
  packet->checksum = 0U;
  packet->checksum = calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet));
}

bool can_check_checksum(CANPacket_t *packet) {
  return (calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet)) == 0U);
}

void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook) {
  if (skip_tx_hook || safety_tx_hook(to_push) != 0) {
    if (bus_number < PANDA_CAN_CNT) {
      // add CAN packet to send queue
      tx_buffer_overflow += can_push(can_queues[bus_number], to_push) ? 0U : 1U;
      process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
    }
  } else {
    safety_tx_blocked += 1U;
    to_push->returned = 0U;
    to_push->rejected = 1U;

    // data changed
    can_set_checksum(to_push);
    rx_buffer_overflow += can_push(&can_rx_q, to_push) ? 0U : 1U;
  }
}

bool is_speed_valid(uint32_t speed, const uint32_t *all_speeds, uint8_t len) {
  bool ret = false;
  for (uint8_t i = 0U; i < len; i++) {
    if (all_speeds[i] == speed) {
      ret = true;
    }
  }
  return ret;
}
