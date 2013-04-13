#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "bcm2835.h"

/* notes */
/* http://elinux.org/Rpi_Low-level_peripherals */
/* p1-2, pin 12, gpio 18, alt5 = pwm0 */


/* bcm2835 guard */

static unsigned int bcm2835_refn = 0;

static int init_bcm2835(void)
{
  if (bcm2835_refn == 0)
  {
    if (!bcm2835_init())
    {
      perror("bcm2835_init()\n");
      return -1;
    }
  }

  ++bcm2835_refn;

  return 0;
}

static void close_bcm2835(void)
{
  if ((--bcm2835_refn) == 0)
    bcm2835_close();
}


/* pwm module */

/* gpio18 alt5, pin p1_12 */
/* only channel 1 pin is available on the rpib */
/* pwm_config -> pwm_sync_1 -> pwm_chn_1 */

/* serial mode (msen1) is chosen: */
/* pwm clock is given by 19200000 / CLK_REG_PWM_DIV */
/* pwm frequency cycle count is given by PWM_REG_RNG1 */
/* pwm duty cycle count is given by PWM_REG_DAT1 */

#define CONFIG_PWM_PIN RPI_V2_GPIO_P1_12

/* module base address */
#define PWM_MAP_OFFSET ((uintptr_t)(0x20000000 + 0x20c000))
#define PWM_MAP_SIZE (32 * sizeof(uint32_t))

/* register offsets */
#define PWM_REG_CTL 0x00
#define PWM_REG_STA 0x04
#define PWM_REG_DMAC 0x08
#define PWM_REG_RNG1 0x10
#define PWM_REG_DAT1 0x14
#define PWM_REG_FIF1 0x18
#define PWM_REG_RNG2 0x20
#define PWM_REG_DAT2 0x24

/* ctl register bits */
#define PWM_CTL_BIT_MSEN1 (1 << 7)
#define PWM_CTL_BIT_CLRF1 (1 << 6)
#define PWM_CTL_BIT_USEF1 (1 << 5)
#define PWM_CTL_BIT_POLA1 (1 << 4)
#define PWM_CTL_BIT_SBIT1 (1 << 3)
#define PWM_CTL_BIT_RPTL1 (1 << 2)
#define PWM_CTL_BIT_MODE1 (1 << 1)
#define PWM_CTL_BIT_PWEN1 (1 << 0)

typedef struct pwm
{
  uintptr_t addr;
  size_t size;
} pwm_t;

static inline void pwm_write_uint32(pwm_t* pwm, uintptr_t off, uint32_t x)
{
  *(volatile uint32_t*)(pwm->addr + off) = x;
}

static inline uint32_t pwm_read_uint32(pwm_t* pwm, uintptr_t off)
{
  return *(volatile uint32_t*)(pwm->addr + off);
}

static inline void pwm_or_uint32(pwm_t* pwm, uintptr_t off, uint32_t mask)
{
  const uint32_t x = pwm_read_uint32(pwm, off);
  pwm_write_uint32(pwm, off, x | mask);
}

static inline void pwm_and_uint32(pwm_t* pwm, uintptr_t off, uint32_t mask)
{
  const uint32_t x = pwm_read_uint32(pwm, off);
  pwm_write_uint32(pwm, off, x & mask);
}

static int pwm_init(pwm_t* pwm)
{
  /* TODO: configure pwm_config (clock manager) */

  size_t off;
  int fd;

  if (init_bcm2835() == -1) return -1;

  bcm2835_gpio_fsel(CONFIG_PWM_PIN, BCM2835_GPIO_FSEL_ALT5);
  close_bcm2835();

  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd == -1)
  {
    perror("open");
    return -1;
  }

  pwm->size = PWM_MAP_SIZE;
  pwm->addr = (uintptr_t)mmap
    (NULL, PWM_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PWM_MAP_OFFSET);

  close(fd);

  if ((void*)pwm->addr == MAP_FAILED)
  {
    perror("mmap");
    return -1;
  }

  return 0;
}

static void pwm_fini(pwm_t* pwm)
{
  munmap((void*)pwm->addr, pwm->size);
}

static int pwm_set_clk_div(pwm_t* pwm, unsigned int div)
{
  /* configure the pwm clock */
  /* http://www.raspberrypi.org/phpBB3/viewtopic.php?t=8467&p=124620 */

#define CLK_MAP_SIZE (42 * sizeof(uint32_t))
#define CLK_MAP_OFFSET ((uintptr_t)(0x20000000 + 0x101000))
#define CLK_REG_PWM_CTL (40 * sizeof(uint32_t))
#define CLK_REG_PWM_DIV (41 * sizeof(uint32_t))

  int fd;
  size_t size;
  uintptr_t addr;

  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd == -1)
  {
    perror("open");
    return -1;
  }

  size = CLK_MAP_SIZE;
  addr = (uintptr_t)mmap
    (NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, CLK_MAP_OFFSET);

  close(fd);

  if (addr == (uintptr_t)MAP_FAILED)
  {
    perror("mmap");
    return -1;
  }

  /* stop the clock and wait for busy flag */
  *(volatile uint32_t*)(addr + CLK_REG_PWM_CTL) = 0x5a000000 | (1 << 5);
  usleep(10);

  /* osc clk divisor */
  *(volatile uint32_t*)(addr + CLK_REG_PWM_DIV) = 0x5a000000 | (div << 12);

  /* source = osc and enable */
  *(volatile uint32_t*)(addr + CLK_REG_PWM_CTL) = 0x5a000011;

  munmap((void*)addr, size);

  return 0;
}

static inline void pwm_set_duty(pwm_t* pwm, unsigned int d)
{
  /* data1 register */
  /* the value of this register defines the number of pulses */
  /* which is sent within the period defined by range1 */

  pwm_write_uint32(pwm, PWM_REG_DAT1, (uint32_t)d);
}

static inline void pwm_set_freq(pwm_t* pwm, unsigned int f)
{
  /* range1 register */
  /* in (this) pwm mode, evenly distributed pulses are sent */
  /* within a period of length defined by this register */

  pwm_write_uint32(pwm, PWM_REG_RNG1, (uint32_t)f);
}

static void pwm_enable(pwm_t* pwm)
{
  /* enable chan1 */

  /* control register */
  /* chan2 disabled */
  /* chan1 enabled */
  /* data reg transmitted */
  /* polarity normal */
  /* output 0 when no tranmission */
  /* pwm mode */

  pwm_write_uint32(pwm, PWM_REG_CTL, PWM_CTL_BIT_MSEN1 | PWM_CTL_BIT_PWEN1);
  usleep(10);
}

static void pwm_disable(pwm_t* pwm)
{
  /* disable chan1 */

  pwm_and_uint32(pwm, PWM_REG_CTL, ~PWM_CTL_BIT_PWEN1);
  usleep(10);
}


/* pwm multiplexer */
/* gpio23, pin p1_16 */

#define CONFIG_PWM_MUX_PIN RPI_V2_GPIO_P1_16
/* #define CONFIG_PWM_MUX_PIN RPI_V2_GPIO_P1_22 */

static int pwm_mux_init(void)
{
  if (init_bcm2835() == -1) return -1;
  bcm2835_gpio_fsel(CONFIG_PWM_MUX_PIN, BCM2835_GPIO_FSEL_OUTP);
  return 0;
}

static void pwm_mux_fini(void)
{
  bcm2835_close();
}

static void pwm_mux_sel(unsigned int i)
{
  /* i in {0,1}, the selected output */
  static const unsigned int lohi[] = { LOW, HIGH };
  bcm2835_gpio_write(CONFIG_PWM_MUX_PIN, lohi[i]);
}


/* main */

int main(int ac, char** av)
{
  unsigned int i = 0;
  pwm_t pwm;

  if (pwm_init(&pwm) == -1)
  {
    return -1;
  }

  if (pwm_mux_init() == -1)
  {
    pwm_fini(&pwm);
    return -1;
  }

  pwm_disable(&pwm);
  pwm_set_clk_div(&pwm, (unsigned int)(19200000.0 / 19200.0));
  pwm_set_freq(&pwm, 100);
  pwm_set_duty(&pwm, 50);
  pwm_enable(&pwm);

  while (1)
  {
    usleep(1000000);
    pwm_mux_sel((++i) & 1);
  }

  pwm_mux_fini();
  pwm_fini(&pwm);

  return 0;
}
