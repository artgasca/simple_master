/* 
 * File:   main.c
 * Author: Arturo Gasca
 *
 * Created on 20 de noviembre de 2025, 01:57 PM
 */

#include "v1.h"

// ========== CONFIG SIMPLE-MASTER ==========
#define SMODBUS_DEBUG           1
#define SMODBUS_BAUD          115200
#define SMODBUS_INT_SOURCE    MODBUS_INT_RDA      // o MODBUS_INT_RDA2 / MODBUS_INT_EXT

#define SMODBUS_RX_PIN        RX_4              // UART RX
#define SMODBUS_TX_PIN        TX_4              // UART TX

#define SMODBUS_DE_PIN        TX_ENABLE              // DE RS485
#define SMODBUS_RE_PIN        RX_ENABLE                   // 0 si está amarrado a GND o unido a DE


#define SMODBUS_TIMEOUT_MS   1000   // 1 segundo
#define SMODBUS_GAP_MS       500     // 50 ms de silencio para ?fin de trama?
#define SMODBUS_RX_BUFFER     64                  // opcional
#include "../simple_master.h"
#include "../simple_master.c"
//#include <bootloader.h>
#include <stdio.h>
#include <stdlib.h>
#define FW_VERSION "1.0.0"


/*Level RS485 sensor*/

#define ID_SLAVE_ADDR   0x01
#define CODE_SLAVE_ADDRESS  0x0000
#define CODE_BAUD_READING   0x0001
#define CODE_PREASURE_UNIT  0x00002
#define CODE_DECIMAL_PLACES 0x00003
#define CODE_OUTPUT_VALUE   0X0004
#define CODE_TRANSMITER_RANGE_ZERO 0X0005
#define CODE_TRANSMITER_RANGE_FULL 0x0006
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
        //protolink_debug_data("FW Version %s\r\n",FW_VERSION);
        // Leer 4 holding registers desde el 0x0000 del esclavo 1
        st = smodbus_read_holding(ID_SLAVE_ADDR, CODE_OUTPUT_VALUE, 1, regs);
        if(st == SMODBUS_OK)
        {
           // consumir regs[0..3]
            protolink_debug_msg("Response OK\r\n");
        }
        else{
            protolink_debug_msg("Fail\r\n");
        }
        
     
        // Leer un solo registro
        //st = smodbus_read_holding_u16(1, 0x0010, &value);

        // Escribir un registro
        //st = smodbus_write_u16(1, 0x0020, 1234);
    }

    
}