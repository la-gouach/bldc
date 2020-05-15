#include <string.h>
#include "ch.h"
#include "app.h"
#include "can_dict.h"
#include "comm_can.h"

static can_dict_variable_metadata dictionary[64];

static THD_WORKING_AREA(can_dict_thread_wa, 512);
static THD_FUNCTION(can_dict_thread, arg);

bool can_dict_init() {
  chThdCreateStatic(can_dict_thread_wa, sizeof(can_dict_thread_wa), NORMALPRIO + 1,
    can_dict_thread, NULL);
  return true;
}

bool can_dict_add_variable(can_dict_type id, uint8_t length, can_dict_variable default_value, bool readable, bool writable, int send_interval_ms) {
  if ((int8_t)id < 0 || (int)id >= CAN_DICT_VARIABLES) return false;
  if (length > sizeof(can_dict_variable)) return false;
  if (dictionary[(int)id].active) return false;
  dictionary[(int)id].length = length;
  dictionary[(int)id].variable = default_value;
  dictionary[(int)id].readable = readable;
  dictionary[(int)id].writable = writable;
  dictionary[(int)id].active = true;
  dictionary[(int)id].send_interval_ms = send_interval_ms;
  return true;
}
bool can_dict_add_variable_int(can_dict_type id, uint8_t length, int64_t default_value, bool readable, bool writable, int send_interval_ms) {
  can_dict_variable v = {.i64 = default_value};
  return can_dict_add_variable(id, length, v, readable, writable, send_interval_ms);
}
bool can_dict_add_variable_float(can_dict_type id, float default_value, bool readable, bool writable, int send_interval_ms) {
  can_dict_variable v = {.f = default_value};
  return can_dict_add_variable(id, 4, v, readable, writable, send_interval_ms);
}
can_dict_variable *can_dict_get_variable(can_dict_type id) {
  if ((int8_t)id < 0 || (int)id >= CAN_DICT_VARIABLES) return 0;
  if (!dictionary[(int)id].active) return 0;
  return &(dictionary[(int)id].variable);
}

bool can_dict_handle_write_request(can_dict_type id, uint8_t *payload, uint8_t payload_length) {
  if ((int8_t)id < 0 || (int)id >= CAN_DICT_VARIABLES) return false;
  if (!dictionary[(int)id].active || !dictionary[(int)id].writable) return false;

  memcpy(&dictionary[(int)id].variable, payload, payload_length <= dictionary[(int)id].length ? payload_length : dictionary[(int)id].length);
  for (int i = 0; i < CAN_DICT_MAX_WRITE_CALLBACKS; ++i) {
    if (dictionary[(int)id].write_callbacks[i] != 0) {
      dictionary[(int)id].write_callbacks[i](id, dictionary[(int)id].variable); // pass by value (copy)
    }
  }
  return true;
}

uint8_t can_dict_handle_read_request(can_dict_type id, uint8_t *result, uint8_t result_length) {
  if ((int8_t)id < 0 || (int)id >= CAN_DICT_VARIABLES) return 0;
  if (!dictionary[(int)id].active || !dictionary[(int)id].readable) return 0;

  uint8_t len = result_length <= dictionary[(int)id].length ? result_length : dictionary[(int)id].length;
  memcpy(result, &dictionary[(int)id].variable, len);
  dictionary[id]._last_value_send_ms = ST2MS(chVTGetSystemTime());
  return len;
}

bool can_dict_on_write(can_dict_type id, can_dict_write_callback cb) {
  if ((int8_t)id < 0 || (int)id >= CAN_DICT_VARIABLES) return false;
  if (!dictionary[(int)id].active) return false;
  for (int i = 0; i < CAN_DICT_MAX_WRITE_CALLBACKS; ++i) {
    if (dictionary[(int)id].write_callbacks[i] == 0) {
      dictionary[(int)id].write_callbacks[i] = cb;
      return true;
    }
  }
  return false;
}

static THD_FUNCTION(can_dict_thread, arg) {
  (void)arg;
  while (!chThdShouldTerminateX()) {
    uint32_t now = ST2MS(chVTGetSystemTime());
    for (int id = 0; id < CAN_DICT_VARIABLES; ++id) {
      if (!dictionary[id].active) continue;
      if (dictionary[id].send_interval_ms <= 0) continue;
      if (now - dictionary[id]._last_value_send_ms >= dictionary[id].send_interval_ms) {
        uint8_t response[8] = { id };
        int written_length = can_dict_handle_read_request(id, &(response[1]), 7);
        if (written_length > 0) {
          comm_can_transmit_eid((CAN_PACKET_DICTIONARY_VALUE << 8) | app_get_configuration()->controller_id, response, written_length + 1);
        }
      }
    }
    chThdSleepMilliseconds(10);
  }
}