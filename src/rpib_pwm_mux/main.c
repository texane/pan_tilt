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
    if (!bcm2835_init())
      return -1;

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

#define CONFIG_PWM_PIN RPI_V2_GPIO_P1_12

/* module base address */
#define PWM_MAP_OFFSET ((uintptr_t)(0x20000000 + 0x20c000))
#define PWM_MAP_SIZE (8 * sizeof(uint32_t))

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
  uintptr_t base;
  size_t size;
} pwm_t;

static inline void pwm_write_uint32(pwm_t* pwm, uintptr_t off, uint32_t x)
{
  *(volatile uint32_t*)(pwm->base + off) = x;
}

static inline uint32_t pwm_read_uint32(pwm_t* pwm, uintptr_t off)
{
  return *(volatile uint32_t*)(pwm->base + off);
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

  int fd;

  if (init_bcm2835() == -1) return -1;
  bcm2835_gpio_fsel(CONFIG_PWM_PIN, BCM2835_GPIO_FSEL_ALT5);
  close_bcm2835();

  fd = open("/dev/mem", O_RDWR);
  if (fd == -1)
  {
    perror("open");
    return -1;
  }

  pwm->size = PWM_MAP_SIZE;
  pwm->base = (uintptr_t)
    mmap(NULL, PWM_MAP_SIZE, PROT_READ | PROT_WRITE, 0, fd, pwm->size);

  close(fd);

  if ((void*)pwm->base == MAP_FAILED)
  {
    perror("mmap");
    return -1;
  }

  return 0;
}

static void pwm_fini(pwm_t* pwm)
{
  munmap((void*)pwm->base, pwm->size);
}

static void pwm_set_freq(pwm_t* pwm, unsigned int m, unsigned int n)
{
  /* range1 register */
  /* in (this) pwm mode, evenly distributed pulses are sent */
  /* within a period of length defined by this register */

  pwm_write_uint32(pwm, PWM_REG_RNG1, (uint32_t)m);

  /* data1 register */
  /* the value of this register defines the number of pulses */
  /* which is sent within the period defined by range1 */

  pwm_write_uint32(pwm, PWM_REG_DAT1, (uint32_t)n);
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

  pwm_or_uint32(pwm, PWM_REG_CTL, PWM_CTL_BIT_PWEN1);
}

static void pwm_disable(pwm_t* pwm)
{
  /* disable chan1 */

  pwm_and_uint32(pwm, PWM_REG_CTL, ~PWM_CTL_BIT_PWEN1);
}


/* pwm multiplexer */
/* gpio23, pin p1_23 */

#define CONFIG_PWM_MUX_PIN RPI_V2_GPIO_P1_16

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

  pwm_set_freq(&pwm, 8, 2);
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