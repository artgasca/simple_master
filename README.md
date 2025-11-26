# simple-master — Modbus RTU Master Driver (PIC CCS)

El proyecto simple-master es un driver minimalista de Modbus RTU Master escrito desde cero para el compilador CCS PIC C, inspirado en la librería original modbus.c pero:

- Más simple
- Menos RAM
- Sin dependencias externas
- Totalmente configurado por #define desde el main
- Basado en interrupción RDA + ring buffer
- Con helpers de debug para inspeccionar TX/RX

Este driver está diseñado para aplicaciones embebidas tipo Protolink y plataformas Modbus industriales.

## 📦 Características principales

- Implementación 100% propia, sin incluir modbus.c de CCS.
- Funciona exclusivamente como Maestro Modbus RTU.
- Usa INT_RDA + ring buffer para recepción robusta.
- Control automático de DE/RE para RS-485 half-duplex.
- CRC16 Modbus integrado.
- Transacciones bloqueantes con:
  - Timeout global de respuesta
  - Gap de silencio para fin de trama
- Debug integrado para inspección de tráfico Modbus.
- API sencilla y directa:

   - smodbus_read_holding()

  - smodbus_read_holding_u16()

  - smodbus_write_u16()
 

## 📁 Estructura del driver

El driver se divide en:
```C
master_simple.h    → Definiciones, configuración y prototipos de API
master_simple.c    → Implementación completa del driver
```



El main debe incluir ambos:
```C
#define SMODBUS_BAUD     9600
#define SMODBUS_TX_PIN   PIN_C6
#define SMODBUS_RX_PIN   PIN_C7
#define SMODBUS_DE_PIN   PIN_E2
#define SMODBUS_RE_PIN   PIN_E2
#define SMODBUS_DEBUG    1

#include "master_simple.h"
#include "master_simple.c"

```

## ⚙️ Configuración vía #define

Todos los parámetros pueden redefinirse desde el main:

| Macro                | Descripción             | Default |
| -------------------- | ----------------------- | ------- |
| `SMODBUS_BAUD`       | Baud rate UART          | 9600    |
| `SMODBUS_TX_PIN`     | Pin TX                  | PIN_C6  |
| `SMODBUS_RX_PIN`     | Pin RX                  | PIN_C7  |
| `SMODBUS_DE_PIN`     | Enable TX RS485         | 0       |
| `SMODBUS_RE_PIN`     | Enable RX RS485         | 0       |
| `SMODBUS_RING_SIZE`  | Tamaño ring RX          | 128     |
| `SMODBUS_TIMEOUT_MS` | Tiempo total de espera  | 200     |
| `SMODBUS_GAP_MS`     | Silencio = fin de trama | 5       |
| `SMODBUS_DEBUG`      | Habilitar debug         | 0       |

## 🧠 Arquitectura interna
### 1. Ring buffer RX (circular)

Implementado con:
```C
volatile uint8 smodbus_ring[SMODBUS_RING_SIZE];
volatile uint8 smodbus_ring_head;
volatile uint8 smodbus_ring_tail;
```
### 2. Interrupción INT_RDA

Lee bytes vía:
```C
#use rs232(..., stream=MODBUS_UART)

#INT_RDA
void smodbus_isr_rda(void)
{
    uint8 c = fgetc(MODBUS_UART);
    // Inserta en ring buffer
}
```
### 3. Transacción Modbus

Flujo:

- Limpia RX (smodbus_rx_flush)
- Envía trama TX
- Espera primer byte (timeout total)
- Recibe hasta detectar gap de silencio
- Valida CRC
- Retorna SMODBUS_OK o error


## 🔍 Funciones de debug

Activar en el main:
```C
#define SMODBUS_DEBUG 1

```
Helpers:
```C
smodbus_debug_tx(frame, len);
smodbus_debug_rx(frame, len);
smodbus_debug_hex("TAG", data, len);
```

Salida típica:
```C
[TX] (8 bytes): 01 03 00 04 00 01 C5 CB
[RX] (7 bytes): 01 03 02 00 7B B9 88

```



## 🧪 API Pública
Inicialización
```C
void smodbus_init(void);
```

Habilita UART, DE/RE, INT_RDA y buffer RX.

### Lecturas Holding Registers (0x03)
```C
smodbus_status_t smodbus_read_holding(
    uint8 slave,
    uint16 start_address,
    uint16 quantity,
    uint16 *dest
);
```


Leer un solo registro:
```C
smodbus_status_t smodbus_read_holding_u16(
    uint8 slave,
    uint16 reg_address,
    uint16 *value
);
```

### Escritura Holding Register (0x06)
```C
smodbus_status_t smodbus_write_u16(
    uint8 slave,
    uint16 reg_address,
    uint16 value
);
```

### Estados / Errores
```C
typedef enum {
    SMODBUS_OK = 0,
    SMODBUS_ERR_TIMEOUT,
    SMODBUS_ERR_CRC,
    SMODBUS_ERR_FRAME,
    SMODBUS_ERR_EXCEPTION
} smodbus_status_t;

```


## ⚠️ Errores comunes encontrados y corregidos
### 1. Problema con tipos mezclados (uint8 vs uint16)
```C
max_len era uint16 y len uint8 → CCS optimizaba la comparación y nunca guardaba bytes → len = 0.
```

Fix: ambos uint8.

### 2. Textos constantes en debug

CCS no acepta const char*.
Solución:
```C
smodbus_debug_hex((char*)"TX", frame, len);
```

### 3. Salida prematura del frame por gap

El driver medía “silencio” antes del primer byte.

Fix: esperar primer byte antes de iniciar conteo de gap.


## 📘 Ejemplo completo
```C
#define SMODBUS_BAUD 9600
#define SMODBUS_TX_PIN PIN_C6
#define SMODBUS_RX_PIN PIN_C7
#define SMODBUS_DE_PIN PIN_E2
#define SMODBUS_RE_PIN PIN_E2
#define SMODBUS_DEBUG 1

#include "master_simple.h"
#include "master_simple.c"

void main(void)
{
    uint16 val;
    smodbus_status_t st;

    smodbus_init();

    while(TRUE)
    {
        st = smodbus_read_holding_u16(1, 0x0004, &val);

        if(st == SMODBUS_OK)
            printf("VALUE: %Lu\r\n", val);
        else
            printf("ERROR: %u\r\n", st);

        delay_ms(500);
    }
}
```

## 🛠️ Pendientes / Extensiones futuras

Funciones Modbus adicionales:

- 0x04 Input Registers

- 0x01 Coils

- 0x02 DI Status

- 0x10 Write Multiple

- Versión no bloqueante con máquina de estados.

- Scheduler maestro multi-esclavo.

- Timeout dinámico por baud rate (t3.5 / t1.5).

- Integración directa a Protolink como módulo.
