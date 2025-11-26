#ifndef SIMPLE_MODBUS_MASTER_C
#define SIMPLE_MODBUS_MASTER_C

// ===========================================================
//  CONFIG POR DEFECTO (puedes sobrescribir en el main)
// ===========================================================
#ifndef SMODBUS_BAUD
 #define SMODBUS_BAUD         9600
#endif

#ifndef SMODBUS_TX_PIN
 #define SMODBUS_TX_PIN       PIN_C6
#endif

#ifndef SMODBUS_RX_PIN
 #define SMODBUS_RX_PIN       PIN_C7
#endif

#ifndef SMODBUS_DE_PIN
 #define SMODBUS_DE_PIN       0          // 0 = no se usa DE
#endif

#ifndef SMODBUS_RE_PIN
 #define SMODBUS_RE_PIN       0          // 0 = no se usa RE
#endif

#ifndef SMODBUS_RING_SIZE
 #define SMODBUS_RING_SIZE    128
#endif

#ifndef SMODBUS_TIMEOUT_MS
 #define SMODBUS_TIMEOUT_MS   200        // Timeout total de respuesta
#endif

#ifndef SMODBUS_GAP_MS
 #define SMODBUS_GAP_MS       5          // Silencio para considerar fin de trama
#endif

#include <stdint.h>
#include <stdio.h>
// UART por hardware
#use rs232(baud=SMODBUS_BAUD, xmit=SMODBUS_TX_PIN, rcv=SMODBUS_RX_PIN, ERRORS)

// ===========================================================
//  ESTADOS / ERRORES
// ===========================================================
typedef enum {
   SMODBUS_OK = 0,
   SMODBUS_ERR_TIMEOUT,
   SMODBUS_ERR_CRC,
   SMODBUS_ERR_FRAME,
   SMODBUS_ERR_EXCEPTION
} smodbus_status_t;

// ===========================================================
//  BUFFER CIRCULAR RX (INT_RDA)
// ===========================================================
volatile unsigned int8  smodbus_ring[SMODBUS_RING_SIZE];
volatile unsigned int8  smodbus_ring_head = 0;
volatile unsigned int8  smodbus_ring_tail = 0;
volatile int1           smodbus_ring_overflow = FALSE;

// Flag simple de “hay datos”
int1 smodbus_rx_available(void)
{
   return (smodbus_ring_head != smodbus_ring_tail);
}

int1 smodbus_rx_overflowed(void)
{
   return smodbus_ring_overflow;
}

void smodbus_rx_flush(void)
{
   disable_interrupts(GLOBAL);
   smodbus_ring_head = 0;
   smodbus_ring_tail = 0;
   smodbus_ring_overflow = FALSE;
   enable_interrupts(GLOBAL);
}

unsigned int8 smodbus_rx_get(void)
{
   unsigned int8 c;

   while(smodbus_ring_head == smodbus_ring_tail)
   {
      // Bloqueante hasta que llegue algo
   }

   c = smodbus_ring[smodbus_ring_tail];
   smodbus_ring_tail++;
   if(smodbus_ring_tail >= SMODBUS_RING_SIZE)
      smodbus_ring_tail = 0;

   return c;
}

// ISR de recepción
#INT_RDA
void smodbus_isr_rda(void)
{
   unsigned int8 c = getc();
   unsigned int8 next = smodbus_ring_head + 1;

   if(next >= SMODBUS_RING_SIZE)
      next = 0;

   if(next != smodbus_ring_tail)
   {
      smodbus_ring[smodbus_ring_head] = c;
      smodbus_ring_head = next;
   }
   else
   {
      // Overflow: perdimos bytes
      smodbus_ring_overflow = TRUE;
   }
}

// ===========================================================
//  RS485 HELPERS (DE / RE)
// ===========================================================
static void smodbus_set_tx_mode(int1 enable)
{
#if (SMODBUS_DE_PIN != 0)
   if(enable)
   {
      output_high(SMODBUS_DE_PIN);
   }
   else
   {
      output_low(SMODBUS_DE_PIN);
   }
#endif

#if (SMODBUS_RE_PIN != 0)
   if(enable)
   {
      // RE normalmente activo alto para deshabilitar RX (dependiendo del chip)
      // ajusta si tu transceiver es al revés
      output_high(SMODBUS_RE_PIN);
   }
   else
   {
      output_low(SMODBUS_RE_PIN);
   }
#endif
}

 void smodbus_send_bytes(unsigned int8 *data, unsigned int8 len)
{
   unsigned int8 i;

   smodbus_set_tx_mode(TRUE);

   for(i = 0; i < len; i++)
   {
      putc(data[i]);
   }

//   // Espera a que se vacíe el shift register
//   while(!tx_done())
//   {
//      // no-op
//   }

   smodbus_set_tx_mode(FALSE);
}

// ===========================================================
//  CRC16 MODBUS (polinomio 0xA001, LSB primero)
// ===========================================================
static unsigned int16 smodbus_crc16(unsigned int8 *data, unsigned int8 len)
{
   unsigned int16 crc = 0xFFFF;
   unsigned int8  i, j;

   for(i = 0; i < len; i++)
   {
      crc ^= data[i];
      for(j = 0; j < 8; j++)
      {
         if(crc & 0x0001)
         {
            crc >>= 1;
            crc ^= 0xA001;
         }
         else
         {
            crc >>= 1;
         }
      }
   }

   return crc;
}

// ===========================================================
//  LECTURA DE TRAMA COMPLETA DESDE EL BUFFER RING
//  - Lee hasta timeout total
//  - Termina si hay "gap" (silencio) >= SMODBUS_GAP_MS
// ===========================================================
static unsigned int8 smodbus_read_frame(unsigned int8 *buf,
                                        unsigned int8 max_len,
                                        unsigned int16 timeout_ms,
                                        unsigned int16 gap_ms)
{
   unsigned int16 t = 0;
   unsigned int16 gap = 0;
   unsigned int8  len = 0;

   while(t < timeout_ms)
   {
      if(smodbus_rx_available())
      {
         gap = 0;

         while(smodbus_rx_available())
         {
            unsigned int8 c = smodbus_rx_get();
            if(len < max_len)
            {
               buf[len++] = c;
            }
            else
            {
               // descartar extra
            }
         }
      }
      else
      {
         gap++;
         if(gap >= gap_ms)
         {
            // Tiempo de silencio suficiente: consideramos que terminó la trama
            break;
         }
      }

      delay_ms(1);
      t++;
   }

   return len;
}

// ===========================================================
//  TRANSACCIÓN GENÉRICA MODBUS RTU MAESTRO
// ===========================================================
#define SMODBUS_MAX_FRAME   256

static smodbus_status_t smodbus_transaction(unsigned int8 *req,
                                            unsigned int8 req_len,
                                            unsigned int8 *resp,
                                            unsigned int8 *resp_len)
{
   unsigned int16 crc_calc, crc_rx;
   unsigned int8  len;

   // Limpiar el buffer antes de iniciar
   smodbus_rx_flush();

   // Enviar petición
   smodbus_send_bytes(req, req_len);

   // Leer respuesta
   len = smodbus_read_frame(resp,
                            SMODBUS_MAX_FRAME,
                            SMODBUS_TIMEOUT_MS,
                            SMODBUS_GAP_MS);

   *resp_len = len;

   if(len < 5)
   {
      // Min: addr, func, data(>=1), CRC(2)
      return SMODBUS_ERR_TIMEOUT;
   }

   // Validar CRC
   crc_calc = smodbus_crc16(resp, len - 2);
   crc_rx   = make16(resp[len-1], resp[len-2]); // LSB primero

   if(crc_calc != crc_rx)
   {
      return SMODBUS_ERR_CRC;
   }

   // Revisar si es excepción (bit 7 del código de función)
   if(resp[1] & 0x80)
   {
      return SMODBUS_ERR_EXCEPTION;
   }

   return SMODBUS_OK;
}

// ===========================================================
//  API SIMPLE: READ HOLDING (0x03) Y WRITE SINGLE (0x06)
// ===========================================================

// 0x03: Leer N holding registers
static smodbus_status_t smodbus_read_holding(int8 slave,
                                      unsigned int16 start_address,
                                      unsigned int16 quantity,
                                      unsigned int16 *dest)
{
   unsigned int8  req[8];
   unsigned int8  resp[SMODBUS_MAX_FRAME];
   unsigned int8  len, byte_count;
   unsigned int16 crc;
   unsigned int8  i;

   // Armar petición
   req[0] = slave;
   req[1] = 0x03;                      // Function code
   req[2] = make8(start_address, 1);   // Hi
   req[3] = make8(start_address, 0);   // Lo
   req[4] = make8(quantity, 1);
   req[5] = make8(quantity, 0);
   crc = smodbus_crc16(req, 6);
   req[6] = make8(crc, 0);             // CRC Lo
   req[7] = make8(crc, 1);             // CRC Hi

   smodbus_status_t st = smodbus_transaction(req, 8, resp, &len);
   if(st != SMODBUS_OK)
      return st;

   // Validar slave y función
   if(resp[0] != slave || resp[1] != 0x03)
      return SMODBUS_ERR_FRAME;

   byte_count = resp[2];
   if(byte_count != (quantity * 2))
      return SMODBUS_ERR_FRAME;

   if(len < (3 + byte_count + 2))
      return SMODBUS_ERR_FRAME;

   for(i = 0; i < quantity; i++)
   {
      unsigned int8 hi = resp[3 + (2*i)];
      unsigned int8 lo = resp[4 + (2*i)];
      dest[i] = make16(hi, lo);
   }

   return SMODBUS_OK;
}

// Azúcar para leer 1 registro
static smodbus_status_t smodbus_read_holding_u16(int8 slave,
                                          unsigned int16 reg_address,
                                          unsigned int16 *value)
{
   return smodbus_read_holding(slave, reg_address, 1, value);
}

// 0x06: Escribir un solo holding register
static smodbus_status_t smodbus_write_u16(int8 slave,
                                   unsigned int16 reg_address,
                                   unsigned int16 value)
{
   int8  req[8];
   int8  resp[SMODBUS_MAX_FRAME];
   int8  len;
   int16 crc;

   // Petición
   req[0] = slave;
   req[1] = 0x06;
   req[2] = make8(reg_address, 1);
   req[3] = make8(reg_address, 0);
   req[4] = make8(value, 1);
   req[5] = make8(value, 0);
   crc = smodbus_crc16(req, 6);
   req[6] = make8(crc, 0);
   req[7] = make8(crc, 1);

   smodbus_status_t st = smodbus_transaction(req, 8, resp, &len);
   if(st != SMODBUS_OK)
      return st;

   // Respuesta de 0x06 es eco de la petición
   if(len < 8)
      return SMODBUS_ERR_FRAME;

   if(resp[0] != slave || resp[1] != 0x06)
      return SMODBUS_ERR_FRAME;

   return SMODBUS_OK;
}

// ===========================================================
//  INIT PÚBLICO
// ===========================================================
void smodbus_init(void)
{
   // Configura DE/RE en modo receive por default
#if (SMODBUS_DE_PIN != 0)
   output_low(SMODBUS_DE_PIN);
#endif
#if (SMODBUS_RE_PIN != 0)
   output_low(SMODBUS_RE_PIN);   // Ajusta según tu transceiver
#endif

   smodbus_rx_flush();

   enable_interrupts(INT_RDA);
   enable_interrupts(GLOBAL);
}

#endif // SIMPLE_MODBUS_MASTER_C
