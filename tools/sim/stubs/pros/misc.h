// Testbench stub of PROS misc.h: controller output is discarded.
#pragma once
#include <cstdint>

namespace pros {
typedef enum { E_CONTROLLER_MASTER = 0, E_CONTROLLER_PARTNER } controller_id_e_t;

namespace c {
inline int32_t controller_print(controller_id_e_t, uint8_t, uint8_t, const char*, ...) { return 1; }
inline int32_t controller_rumble(controller_id_e_t, const char*) { return 1; }
inline int32_t controller_set_text(controller_id_e_t, uint8_t, uint8_t, const char*) { return 1; }
inline int32_t controller_clear(controller_id_e_t) { return 1; }
inline uint8_t competition_get_status(void) { return 0; }
} // namespace c
} // namespace pros
