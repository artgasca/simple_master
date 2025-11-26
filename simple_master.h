#ifndef SIMPLE_MASTER_H
#define SIMPLE_MASTER_H

// ===========================================================
// CONFIGURACIÓN DEL DRIVER (puede redefinirse desde main)
// ===========================================================

// Si no se define en main, se aplican defaults
#ifndef SMODBUS_BAUD
 #define SMODBUS_BAUD            9600
#endif

#ifndef SMODBUS_TX_PIN
 #define SMODBUS_TX_PIN          PIN_C6
#endif

#ifndef SMODBUS_RX_PIN
 #define SMODBUS_RX_PIN          PIN_C7
#endif

#ifndef SMODBUS_DE_PIN
 #define SMODBUS_DE_PIN          0
#endif

#ifndef SMODBUS_RE_PIN
 #define SMODBUS_RE_PIN          0
#endif

#ifndef SMODBUS_RING_SIZE
 #define SMODBUS_RING_SIZE       128
#endif

#ifndef SMODBUS_TIMEOUT_MS
 #define SMODBUS_TIMEOUT_MS      200
#endif

#ifndef SMODBUS_GAP_MS
 #define SMODBUS_GAP_MS          5
#endif


#ifndef SMODBUS_DEBUG
 #define SMODBUS_DEBUG          0
#endif

// ===========================================================
// ENUM DE ESTADOS DEL DRIVER
// ===========================================================
typedef enum {
    SMODBUS_OK = 0,
    SMODBUS_ERR_TIMEOUT,
    SMODBUS_ERR_CRC,
    SMODBUS_ERR_FRAME,
    SMODBUS_ERR_EXCEPTION
} smodbus_status_t;


// =============================================
//  DEBUG (activar con #define SMODBUS_DEBUG 1)
// =============================================
#ifndef SMODBUS_DEBUG
 #define SMODBUS_DEBUG 0
#endif

// Salida de debug (usa printf de CCS)
void smodbus_debug_hex(char *label, unsigned int8 *data, unsigned int8 len);
void smodbus_debug_tx(unsigned int8 *frame, unsigned int8 len);
void smodbus_debug_rx(unsigned int8 *frame, unsigned int8 len);


// ===========================================================
// API PÚBLICA DEL SIMPLE MASTER
// ===========================================================

// ---- Inicialización general del driver ----
void smodbus_init(void);


// ---- Buffer y flags del receptor ----
int1 smodbus_rx_available(void);
unsigned int8 smodbus_rx_get(void);
void smodbus_rx_flush(void);
int1 smodbus_rx_overflowed(void);


// ---- Primitivas Modbus RTU ----
smodbus_status_t smodbus_read_holding(
    unsigned int8 slave,
    unsigned int16 start_address,
    unsigned int16 quantity,
    unsigned int16 *dest
);

smodbus_status_t smodbus_read_holding_u16(
    unsigned int8 slave,
    unsigned int16 reg_address,
    unsigned int16 *value
);

smodbus_status_t smodbus_write_u16(
    unsigned int8 slave,
    unsigned int16 reg_address,
    unsigned int16 value
);


// ===========================================================
// NOTA IMPORTANTE
//
// El usuario debe incluir master_simple.c en su proyecto
//
// Ejemplo:
//     #define SMODBUS_BAUD 115200
//     #include "master_simple.c"
//
// ===========================================================

#endif  // SIMPLE_MASTER_H
