#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "pt.h"
#include "bcm2835.h"


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

static void fini_bcm2835(void)
{
  if ((--bcm2835_refn) == 0)
    bcm2835_close();
}


/* low level device memory read write */

static inline void write_uint32(uintptr_t addr, uint32_t x)
{
  *(volatile uint32_t*)addr = x;
}

static inline uint32_t read_uint32(uintptr_t addr)
{
  return *(volatile uint32_t*)addr;
}

static inline void or_uint32(uintptr_t addr, uint32_t mask)
{
  const uint32_t x = read_uint32(addr);
  write_uint32(addr, x | mask);
}

static inline void and_uint32(uintptr_t addr, uint32_t mask)
{
  const uint32_t x = read_uint32(addr);
  write_uint32(addr, x & mask);
}


/* map device from /dev/mem */

static uintptr_t map_dev_mem(uintptr_t paddr, size_t size)
{
  uintptr_t vaddr;
  int fd;

  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd == -1)
  {
    perror("open");
    return (uintptr_t)0;
  }

  vaddr = (uintptr_t)mmap
    (NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, paddr);

  close(fd);

  if (vaddr == (uintptr_t)MAP_FAILED)
  {
    perror("mmap");
    return (uintptr_t)0;
  }

  return vaddr;
}

static void unmap_dev_mem(uintptr_t addr, size_t size)
{
  munmap((void*)addr, size);
}


/* pwm module */

/* http://elinux.org/Rpi_Low-level_peripherals */
/* p1-2, pin 12, gpio 18, alt5 = pwm0 */

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

static uintptr_t pwm_init(void)
{
  /* gpio alt5 function */
  bcm2835_gpio_fsel(CONFIG_PWM_PIN, BCM2835_GPIO_FSEL_ALT5);

  /* map pwm registers */
  return map_dev_mem(PWM_MAP_OFFSET, PWM_MAP_SIZE);
}

static void pwm_fini(uintptr_t addr)
{
  unmap_dev_mem(addr, PWM_MAP_SIZE);
}

static inline void pwm_set_duty(uintptr_t addr, unsigned int d)
{
  /* data1 register */
  /* the value of this register defines the number of pulses */
  /* which is sent within the period defined by range1 */

  write_uint32(addr + PWM_REG_DAT1, (uint32_t)d);
}

static inline void pwm_set_freq(uintptr_t addr, unsigned int f)
{
  /* range1 register */
  /* in (this) pwm mode, evenly distributed pulses are sent */
  /* within a period of length defined by this register */

  write_uint32(addr + PWM_REG_RNG1, (uint32_t)f);
}

static void pwm_enable(uintptr_t addr)
{
  /* enable chan1 */

  /* control register */
  /* chan2 disabled */
  /* chan1 enabled */
  /* data reg transmitted */
  /* polarity normal */
  /* output 0 when no tranmission */
  /* pwm mode */

  write_uint32(addr + PWM_REG_CTL, PWM_CTL_BIT_MSEN1 | PWM_CTL_BIT_PWEN1);
  usleep(10);
}

static void pwm_disable(uintptr_t addr)
{
  /* disable chan1 */

  and_uint32(addr + PWM_REG_CTL, ~PWM_CTL_BIT_PWEN1);
  usleep(10);
}


/* pwm multiplexer */
/* gpio23, pin p1_16 */

#define CONFIG_PWM_MUX_PIN RPI_V2_GPIO_P1_16
/* #define CONFIG_PWM_MUX_PIN RPI_V2_GPIO_P1_22 */

static int pwm_mux_init(void)
{
  if (init_bcm2835() == -1) return -1;

  /* set pwm_mux_pin output mode */
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


/* clock manager */

/* http://www.raspberrypi.org/phpBB3/viewtopic.php?t=8467&p=124620 */

#define CLK_MAP_SIZE (42 * sizeof(uint32_t))
#define CLK_MAP_OFFSET ((uintptr_t)(0x20000000 + 0x101000))
#define CLK_REG_PWM_CTL (40 * sizeof(uint32_t))
#define CLK_REG_PWM_DIV (41 * sizeof(uint32_t))

static uintptr_t clk_init(void)
{
  return map_dev_mem(CLK_MAP_OFFSET, CLK_MAP_SIZE);
}

static void clk_fini(uintptr_t addr)
{
  unmap_dev_mem(addr, CLK_MAP_SIZE);
}

static int clk_set_pwm_div(uintptr_t addr, unsigned int div)
{
  /* stop the clock and wait for busy flag */
  write_uint32(addr + CLK_REG_PWM_CTL, 0x5a000000 | (1 << 5));
  usleep(10);

  /* osc clk divisor */
  write_uint32(addr + CLK_REG_PWM_DIV, 0x5a000000 | (div << 12));

  /* source = osc and enable */
  write_uint32(addr + CLK_REG_PWM_CTL, 0x5a000011);

  return 0;
}


/* pan tilt exported api */

int pt_open(pt_dev_t* d)
{
  if (init_bcm2835() == -1) goto on_error_0;

  /* map clock registers */
  d->clk_addr = clk_init();
  if (d->clk_addr == 0) goto on_error_1;

  /* map pwm registers */
  d->pwm_addr = pwm_init();
  if (d->pwm_addr == 0) goto on_error_2;

  /* mux gpio init */
  if (pwm_mux_init() == -1) goto on_error_3;

  /* initial pwm settings */
  pwm_disable(d->pwm_addr);
  clk_set_pwm_div(d->clk_addr, (unsigned int)(19200000.0 / 19200.0));
  pwm_set_freq(d->pwm_addr, 500);

  return 0;

 on_error_3:
  pwm_fini(d->pwm_addr);
 on_error_2:
  clk_fini(d->clk_addr);
 on_error_1:
  fini_bcm2835();
 on_error_0:
  return -1;
}

int pt_close(pt_dev_t* d)
{
  pwm_mux_fini();
  pwm_fini(d->pwm_addr);
  clk_fini(d->clk_addr);
  fini_bcm2835();
  return 0;
}

int pt_control(pt_dev_t* d, pt_op_t op)
{
  const unsigned long us = (op == PT_OP_TILT_CW) ? 10000 : 20000;
  unsigned int duty = 39;
  unsigned int sel =  1;

  if ((op == PT_OP_TILT_CW) || (op == PT_OP_TILT_CCW)) sel = 0;
  if ((op == PT_OP_TILT_CW) || (op == PT_OP_PAN_CW)) duty = 10;

  pwm_mux_sel(sel);
  pwm_set_duty(d->pwm_addr, duty);

  pwm_enable(d->pwm_addr);
  usleep(us);
  pwm_disable(d->pwm_addr);

  return 0;
}
