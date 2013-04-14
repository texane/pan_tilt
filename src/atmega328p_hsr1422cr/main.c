#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define CONFIG_UART 1
#if CONFIG_UART /* uart */

static inline void set_baud_rate(long baud)
{
  uint16_t UBRR0_value = ((F_CPU / 16 + baud / 2) / baud - 1);
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
}

static void uart_init(void)
{
  /* #define CONFIG_FOSC (F_CPU * 2) */
  /* const uint16_t x = CONFIG_FOSC / (16 * BAUDS) - 1; */
#if 0 /* (bauds == 9600) */
  const uint16_t x = 206;
#elif 0 /* (bauds == 115200) */
  const uint16_t x = 16;
#elif 0 /* (bauds == 500000) */
  const uint16_t x = 3;
#elif 0 /* (bauds == 1000000) */
  const uint16_t x = 1;
#endif

  set_baud_rate(9600);

  /* baud doubler off  - Only needed on Uno XXX */
  UCSR0A &= ~(1 << U2X0);

  UCSR0B = (1 << TXEN0) | (1 << RXEN0);

  /* default to 8n1 framing */
  UCSR0C = (3 << 1);
}

static void uart_write(uint8_t* s, uint8_t n)
{
  for (; n; --n, ++s)
  {
    /* wait for transmit buffer to be empty */
    while (!(UCSR0A & (1 << UDRE0))) ;
    UDR0 = *s;
  }

  /* wait for last byte to be sent */
  while ((UCSR0A & (1 << 6)) == 0) ;
}

static inline uint8_t uart_read_byte(void)
{
  while ((UCSR0A & (1 << 7)) == 0) ;
  return UDR0;
}

__attribute__((unused))
static inline void uart_read(uint8_t* s, uint8_t n)
{
  uint8_t i;
  for (i = 0; i < n; ++i) s[i] = uart_read_byte();
}

static inline uint8_t nibble(uint32_t x, uint8_t i)
{
  return (x >> (i * 4)) & 0xf;
}

static inline uint8_t hex(uint8_t x)
{
  return (x >= 0xa) ? 'a' + x - 0xa : '0' + x;
}

__attribute__((unused))
static uint8_t* uint32_to_string(uint32_t x)
{
  static uint8_t buf[8];

  buf[7] = hex(nibble(x, 0));
  buf[6] = hex(nibble(x, 1));
  buf[5] = hex(nibble(x, 2));
  buf[4] = hex(nibble(x, 3));
  buf[3] = hex(nibble(x, 4));
  buf[2] = hex(nibble(x, 5));
  buf[1] = hex(nibble(x, 6));
  buf[0] = hex(nibble(x, 7));

  return buf;
}

#endif /* CONFIG_UART */


/* continuous servo rotation */
/* http://www.education.rec.ri.cmu.edu/content/electronics/boe/robot_motion/1.html */
/* note: at 16 mhz, prescaler of 128, 0xff = 2.04 ms */

/* incremented every interrupt */
static volatile uint8_t tcnt2_inc = 0;

ISR(TIMER2_OVF_vect)
{
  /* note: overflow flag automatically cleared */
  ++tcnt2_inc;
}

static void hsr_init(void)
{
  /* timer disabled */
  TCCR2B = 0;

  /* enable interrupt on timer overflow */
  TIMSK2 = (1 << 0);

  /* fast non inverting pwm mode */
  TCCR2B = 0;
  TCCR2A = 3 << 0;

  /* output mode */
  DDRB |= 1 << 3;
  PORTB &= ~(1 << 3);
  DDRD |= 1 << 3;
  PORTD &= ~(1 << 3);
}

static void hsr_step_common(uint8_t i, uint8_t x)
{
  /* i the pwm index, in {0, 1} */
  /* x the pwm counter */

  tcnt2_inc = 0;
  TCNT2 = 0;

  /* set compare on change counter */
  OCR2A = 0;
  OCR2B = 0;

  /* disable oc2a and oc2b */
  TCCR2A &= ~(0xf << 4);

  if (i == 0)
  {
    PORTD &= ~(1 << 3);
    OCR2A = x;
    /* oc2a set at bottom, clear on compare and match */
    TCCR2A |= 2 << 6;
  }
  else
  {
    PORTB &= ~(1 << 3);
    OCR2B = x;
    /* oc2b set at bottom, clear on compare and match */
    TCCR2A |= 2 << 4;
  }

  /* enable timer, 128 prescaler */
  TCCR2B = 5 << 0;

  /* 25 ms (2ms = 1) */
  while (tcnt2_inc <= 12) ;

  /* disable timer */
  TCCR2B = 0;
}

static inline void hsr_step_cw(uint8_t i)
{
  /* step clockwise */
  /* 1ms, 1ms */
  hsr_step_common(i, 0xff / 2);
}

static inline void hsr_step_ccw(uint8_t i)
{
  /* step counter clockwise */
  /* 2ms, 0ms */
  hsr_step_common(i, 0xff - 1);
}

__attribute__((unused)) static void hsr_stop(void)
{
  /* disable timer */
  TCCR2B = 0;
  TCCR2A = 0;
}

int main(void)
{
  hsr_init();

#if CONFIG_UART
  uart_init();
#endif

  sei();

  while (1)
  {
    uint8_t c = uart_read_byte();

    if (c == 'b') hsr_step_cw(0);
    else if (c == 'f') hsr_step_ccw(0);
    else if (c == 'p') hsr_step_cw(1);
    else if (c == 'n') hsr_step_ccw(1);
    else c = '?';

    uart_write((uint8_t*)&c, 1);
  }

  return 0;
}
