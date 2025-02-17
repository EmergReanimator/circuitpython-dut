#include "py/objtuple.h"
#include "shared-bindings/board/__init__.h"

// This mapping only includes functional names because pins broken
// out on connectors are labeled with their MCU name available from
// microcontroller.pin.
STATIC const mp_rom_map_elem_t board_global_dict_table[] = {
    // FC1 I2C
    { MP_OBJ_NEW_QSTR(MP_QSTR_SDA),     MP_ROM_PTR(&pin_P0_13) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCL),     MP_ROM_PTR(&pin_P0_14) },

    // FC3 SPI
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCK),     MP_ROM_PTR(&pin_P0_6) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MISO),    MP_ROM_PTR(&pin_P0_2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MOSI),    MP_ROM_PTR(&pin_P0_3) },
    #if (0)
    { MP_OBJ_NEW_QSTR(MP_QSTR_SSEL),    MP_ROM_PTR(&pin_P0_4) },
    #endif

    { MP_OBJ_NEW_QSTR(MP_QSTR_LED_B),   MP_ROM_PTR(&pin_P1_4) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LED_R),   MP_ROM_PTR(&pin_P1_6) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LED_G),   MP_ROM_PTR(&pin_P1_7) },

    // FC0 UART
    { MP_OBJ_NEW_QSTR(MP_QSTR_RX),      MP_ROM_PTR(&pin_P0_29) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX),      MP_ROM_PTR(&pin_P0_30) },

    // ADC
    { MP_OBJ_NEW_QSTR(MP_QSTR_ADC0_N),  MP_ROM_PTR(&pin_P0_16) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ADC0_P),  MP_ROM_PTR(&pin_P0_17) },

    // GPIO0
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_0),    MP_ROM_PTR(&pin_P0_0) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_1),    MP_ROM_PTR(&pin_P0_1) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_2),    MP_ROM_PTR(&pin_P0_2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_3),    MP_ROM_PTR(&pin_P0_3) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_4),    MP_ROM_PTR(&pin_P0_4) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_5),    MP_ROM_PTR(&pin_P0_5) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_6),    MP_ROM_PTR(&pin_P0_6) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_7),    MP_ROM_PTR(&pin_P0_7) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_8),    MP_ROM_PTR(&pin_P0_8) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_9),    MP_ROM_PTR(&pin_P0_9) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_10),   MP_ROM_PTR(&pin_P0_10) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_11),   MP_ROM_PTR(&pin_P0_11) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_12),   MP_ROM_PTR(&pin_P0_12) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_13),   MP_ROM_PTR(&pin_P0_13) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_14),   MP_ROM_PTR(&pin_P0_14) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_15),   MP_ROM_PTR(&pin_P0_15) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_16),   MP_ROM_PTR(&pin_P0_16) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_17),   MP_ROM_PTR(&pin_P0_17) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_18),   MP_ROM_PTR(&pin_P0_18) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_19),   MP_ROM_PTR(&pin_P0_19) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_20),   MP_ROM_PTR(&pin_P0_20) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_21),   MP_ROM_PTR(&pin_P0_21) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_22),   MP_ROM_PTR(&pin_P0_22) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_23),   MP_ROM_PTR(&pin_P0_23) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_24),   MP_ROM_PTR(&pin_P0_24) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_25),   MP_ROM_PTR(&pin_P0_25) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_26),   MP_ROM_PTR(&pin_P0_26) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_27),   MP_ROM_PTR(&pin_P0_27) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_28),   MP_ROM_PTR(&pin_P0_28) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_29),   MP_ROM_PTR(&pin_P0_29) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_30),   MP_ROM_PTR(&pin_P0_30) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P0_31),   MP_ROM_PTR(&pin_P0_31) },

    // GPIO1
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_0),    MP_ROM_PTR(&pin_P1_0) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_1),    MP_ROM_PTR(&pin_P1_1) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_2),    MP_ROM_PTR(&pin_P1_2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_3),    MP_ROM_PTR(&pin_P1_3) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_4),    MP_ROM_PTR(&pin_P1_4) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_5),    MP_ROM_PTR(&pin_P1_5) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_6),    MP_ROM_PTR(&pin_P1_6) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_7),    MP_ROM_PTR(&pin_P1_7) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_8),    MP_ROM_PTR(&pin_P1_8) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_9),    MP_ROM_PTR(&pin_P1_9) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_10),   MP_ROM_PTR(&pin_P1_10) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_11),   MP_ROM_PTR(&pin_P1_11) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_12),   MP_ROM_PTR(&pin_P1_12) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_13),   MP_ROM_PTR(&pin_P1_13) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_14),   MP_ROM_PTR(&pin_P1_14) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_15),   MP_ROM_PTR(&pin_P1_15) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_16),   MP_ROM_PTR(&pin_P1_16) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_17),   MP_ROM_PTR(&pin_P1_17) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_18),   MP_ROM_PTR(&pin_P1_18) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_19),   MP_ROM_PTR(&pin_P1_19) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_20),   MP_ROM_PTR(&pin_P1_20) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_21),   MP_ROM_PTR(&pin_P1_21) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_22),   MP_ROM_PTR(&pin_P1_22) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_23),   MP_ROM_PTR(&pin_P1_23) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_24),   MP_ROM_PTR(&pin_P1_24) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_25),   MP_ROM_PTR(&pin_P1_25) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_26),   MP_ROM_PTR(&pin_P1_26) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_27),   MP_ROM_PTR(&pin_P1_27) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_28),   MP_ROM_PTR(&pin_P1_28) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_29),   MP_ROM_PTR(&pin_P1_29) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_30),   MP_ROM_PTR(&pin_P1_30) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_P1_31),   MP_ROM_PTR(&pin_P1_31) },

    { MP_ROM_QSTR(MP_QSTR_UART),        MP_ROM_PTR(&board_uart_obj) },
    { MP_ROM_QSTR(MP_QSTR_I2C),         MP_ROM_PTR(&board_i2c_obj) },
    { MP_ROM_QSTR(MP_QSTR_SPI),         MP_ROM_PTR(&board_spi_obj) },

};
MP_DEFINE_CONST_DICT(board_module_globals, board_global_dict_table);
