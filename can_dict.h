#ifndef CAN_DICT_H_
#define CAN_DICT_H_
#include <stdint.h>
#include <stdbool.h>

#define CAN_DICT_MAX_WRITE_CALLBACKS 4
#define CAN_DICT_VARIABLES 64

#define CAN_DICT_NO_SEND_INTERVAL 0

typedef enum {
  CAN_DICT_NOTHING = 0x00,
  // LIGHTS
  CAN_DICT_LIGHTS = 0x01,
  // PEDELEC
  CAN_DICT_PEDELEC_MAGNETS = 0x10,
  CAN_DICT_PEDELEC_CURVE_ALPHA = 0x11,
  CAN_DICT_PEDELEC_STOP_TIMEOUT = 0x12,
  // APP CUSTOM
  CAN_DICT_CONTROL_MODE = 0x21,
  // BIKE CONFIG
  CAN_DICT_WHEEL_DIAM = 0x30,
  CAN_DICT_MAX_POWER = 0x31,
  CAN_DICT_MAX_SPEED = 0x32,
  // REAL TIME INFO
  CAN_DICT_POWER = 0x80, // In watts
  CAN_DICT_SPEED = 0x81, // In km/h
} can_dict_type;

typedef union {
  uint64_t u64;
  int64_t  i64;
  uint32_t u32;
  int32_t  i32;
  uint16_t u16;
  int16_t  i16;
  uint8_t  u8;
  int8_t   i8;
  float    f;
  int8_t   bytes[6];
  bool     b;
} can_dict_variable;

typedef void    (*can_dict_write_callback)(can_dict_type, can_dict_variable);
typedef void    (*can_dict_setter)(can_dict_variable *dest, uint8_t variable_length, void *buffer, uint8_t buffer_length);
typedef uint8_t (*can_dict_getter)(can_dict_variable *src, uint8_t variable_length,  void *buffer, uint8_t buffer_length);

typedef struct {
  can_dict_write_callback  write_callbacks[CAN_DICT_MAX_WRITE_CALLBACKS];
  can_dict_variable       *variable;
  uint8_t                  length;
  bool                     active;
  bool                     readable;
  bool                     writable;
  can_dict_setter          setter;
  can_dict_getter          getter;
  uint32_t                 send_interval_ms; // If set to a positive value, the value of the variable will be sent on the CAN bus every send_interval_ms milliseconds
  uint32_t                _last_value_send_ms;
} can_dict_variable_metadata;


bool can_dict_init(void);

bool can_dict_add_variable(can_dict_type id, uint8_t length, can_dict_variable default_value, bool readable, bool writable, int send_interval_ms);
bool can_dict_add_variable_int(can_dict_type id, uint8_t length, int64_t default_value, bool readable, bool writable, int send_interval_ms);
bool can_dict_add_variable_float(can_dict_type id, float default_value, bool readable, bool writable, int send_interval_ms);

bool can_dict_bind_variable(can_dict_type id, can_dict_variable *memory, uint8_t length, bool readable, bool writable, int send_interval_ms);
can_dict_variable *can_dict_get_variable(can_dict_type id);

bool can_dict_set_setter(can_dict_type id, can_dict_setter setter);
bool can_dict_set_getter(can_dict_type id, can_dict_getter getter);

bool can_dict_handle_write_request(can_dict_type id, uint8_t *payload, uint8_t payload_length);
uint8_t can_dict_handle_read_request(can_dict_type id, uint8_t *result, uint8_t result_length);

bool can_dict_on_write(can_dict_type id, can_dict_write_callback cb);
#endif
