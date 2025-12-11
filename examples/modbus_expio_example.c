/* 
 * File:   main.c
 * Author: Arturo Gasca
 *
 * Created on 10 de diciembre de 2025, 01:57 PM
 * Ejemplo para manipular tarjetas de expansion modbus
 */

#include "v1.h"

// ========== CONFIG SIMPLE-MASTER ==========
#define SMODBUS_DEBUG           1
#define SMODBUS_BAUD          9600
#define SMODBUS_INT_SOURCE    MODBUS_INT_RDA      // o MODBUS_INT_RDA2 / MODBUS_INT_EXT

#define SMODBUS_RX_PIN        RX_4              // UART RX
#define SMODBUS_TX_PIN        TX_4              // UART TX

#define SMODBUS_DE_PIN        TX_ENABLE              // DE RS485
#define SMODBUS_RE_PIN        RX_ENABLE                   // 0 si est� amarrado a GND o unido a DE


#define SMODBUS_TIMEOUT_MS   1000   // 1 segundo
#define SMODBUS_GAP_MS       500     // 50 ms de silencio para ?fin de trama?
#define SMODBUS_RX_BUFFER     64                  // opcional
#include "../simple_master.h"
#include "../simple_master.c"
#include <bootloader.h>
#include <stdio.h>
#include <stdlib.h>
#define FW_VERSION "1.0.0"


/*IO Expansor RS485 */

#define ID_SLAVE_ADDR   0x01
#define OUTPUT_BIT_PORT_ADDRESS     0x0070  //One bit for channel
#define INPUT_PORT_ADDRESS          0x0080  //0x0080-0x00AF One register for one channel
#define INPUT_BIT_PORT_ADDRESS      0X00C0 //One bit for channel



/*
 * 
 */
void main(void) {

    protolink_io_init();
    output_low(LED1);
    output_low(LED2);

    smodbus_status_t st;
    int16 value;
    int8 di[16];

    smodbus_init();   // Inicializa Modbus
    while (true) {
        
        output_toggle(LED1);
        delay_ms(1000);
        st = smodbus_read_holding_u16(ID_SLAVE_ADDR, INPUT_BIT_PORT_ADDRESS,&value);
        if(st == SMODBUS_OK)
        {
           // consumir 1 regs value
            protolink_debug_msg("Response OK\r\n");
            protolink_debug_data("data: %ld\r\n",value);
            int i;
            for (i = 0; i < 16; i++) {
                di[i] = bit_test(value,i);
                fprintf(DEBUG,"IN%d: %d\r\n",i,di[i]);
            }

        }
        else{
            protolink_debug_msg("Fail\r\n");
        }
        st = smodbus_write_u16(ID_SLAVE_ADDR,OUTPUT_BIT_PORT_ADDRESS,value);
        if(st == SMODBUS_OK){
            protolink_debug_msg("Write OK\r\n");
        }
        else{
            protolink_debug_msg("Write File\r\n");
        }
     
        // Leer un solo registro
        //st = smodbus_read_holding_u16(1, 0x0010, &value);

        // Escribir un registro
        //st = smodbus_write_u16(1, 0x0020, 1234);
    }

    
}