/* 
 * File:   main.c
 * Author: Arturo Gasca
 *
 * Created on 20 de noviembre de 2025, 01:57 PM
 */

#include "v2.h"

// ========== CONFIG SIMPLE-MASTER ==========
#define SMODBUS_BAUD          115200
#define SMODBUS_INT_SOURCE    MODBUS_INT_RDA      // o MODBUS_INT_RDA2 / MODBUS_INT_EXT

#define SMODBUS_RX_PIN        PIN_C7              // UART RX
#define SMODBUS_TX_PIN        PIN_C6              // UART TX

#define SMODBUS_DE_PIN        PIN_E2              // DE RS485
#define SMODBUS_RE_PIN        0                   // 0 si está amarrado a GND o unido a DE

#define SMODBUS_TIMEOUT_US    200000              // 200 ms de timeout
#define SMODBUS_RX_BUFFER     64                  // opcional

#include "../simple_master.c"
#include <bootloader.h>
#include <stdio.h>
#include <stdlib.h>
#define FW_VERSION "1.0.0"

/*
 * 
 */
void main(void) {

    protolink_io_init();
    output_low(LED1);
    output_low(LED2);
    int16 regs[4];
    smodbus_status_t st;
    int16 value;

   smodbus_init();   // Inicializa Modbus
    while (true) {
        
        output_toggle(LED1);
        delay_ms(1000);
        protolink_debug_data("FW Version %s\r\n",FW_VERSION);
        // Leer 4 holding registers desde el 0x0000 del esclavo 1
        st = smodbus_read_holding(1, 0x0000, 4, regs);
        if(st == SMODBUS_OK)
        {
           // consumir regs[0..3]
        }
        // Leer un solo registro
        st = smodbus_read_holding_u16(1, 0x0010, &value);

        // Escribir un registro
        st = smodbus_write_u16(1, 0x0020, 1234);
    }

    
}