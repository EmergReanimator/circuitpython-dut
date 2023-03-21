#include "py/objtuple.h"
#include "shared-bindings/board/__init__.h"

// This mapping only includes functional names because pins broken
// out on connectors are labelled with their MCU name available from
// microcontroller.pin.
STATIC const mp_rom_map_elem_t board_global_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_LED1),    MP_ROM_PTR(&pin_P1_28) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LED2),    MP_ROM_PTR(&pin_P1_29) },

    // CAN1
    { MP_OBJ_NEW_QSTR(MP_QSTR_CAN1_RX), MP_ROM_PTR(&pin_P0_0) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CAN1_TX), MP_ROM_PTR(&pin_P0_1) },

    // UART0
    { MP_OBJ_NEW_QSTR(MP_QSTR_RX),      MP_ROM_PTR(&pin_P0_2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX),      MP_ROM_PTR(&pin_P0_3) },

    // I2C0
    { MP_OBJ_NEW_QSTR(MP_QSTR_SDA),     MP_ROM_PTR(&pin_P0_27) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCL),     MP_ROM_PTR(&pin_P0_28) },

    // SPI0
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCK),     MP_ROM_PTR(&pin_P1_20) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MISO),    MP_ROM_PTR(&pin_P0_17) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MOSI),    MP_ROM_PTR(&pin_P0_18) },

    // COM1
    { MP_ROM_QSTR(MP_QSTR_UART),        MP_ROM_PTR(&board_uart_obj) },
    { MP_ROM_QSTR(MP_QSTR_I2C),         MP_ROM_PTR(&board_i2c_obj) },
    { MP_ROM_QSTR(MP_QSTR_SPI),         MP_ROM_PTR(&board_spi_obj) },

};

MP_DEFINE_CONST_DICT(board_module_globals, board_global_dict_table);
