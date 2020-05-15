#ifndef CAN_DICT_H_
#define CAN_DICT_H_
#include <stdint.h>
#include <stdbool.h>

#define CAN_DICT_MAX_WRITE_CALLBACKS 4
#define CAN_DICT_VARIABLES 64

typedef enum {
  CAN_DICT_NOTHING = 0,
  CAN_DICT_TEST = 1
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
} can_dict_variable;

typedef void (*can_dict_write_callback)(can_dict_type, can_dict_variable);

typedef struct {
  // public:
  can_dict_write_callback write_callbacks[CAN_DICT_MAX_WRITE_CALLBACKS];
  can_dict_variable       variable;
  uint8_t                 length;
  bool                    active;
  bool                    readable;
  bool                    writable;
  uint32_t                send_interval_ms; // If set to a positive value, the value of the variable will be sent on the CAN bus every send_interval_ms milliseconds
  // private:
  uint32_t                _last_value_send_ms;
} can_dict_variable_metadata;

bool can_dict_init(void);
bool can_dict_add_variable(can_dict_type id, uint8_t length, bool readable, bool writable, int send_interval_ms);
can_dict_variable *can_dict_get_variable(can_dict_type id);
bool can_dict_handle_write_request(can_dict_type id, uint8_t *payload, uint8_t payload_length);
uint8_t can_dict_handle_read_request(can_dict_type id, uint8_t *result, uint8_t result_length);
bool can_dict_on_write(can_dict_type id, can_dict_write_callback cb);
#endif