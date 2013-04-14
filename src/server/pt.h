#ifndef PT_H_INCLUDED
# define PT_H_INCLUDED


#include <stdint.h>
#include <sys/types.h>


typedef enum
{
  PT_OP_PAN_CW = 0,
  PT_OP_PAN_CCW,
  PT_OP_TILT_CW,
  PT_OP_TILT_CCW,
  PT_OP_INVALID
} pt_op_t;


typedef struct pt_dev
{
  /* bcm2835 pwm register mapping */
  uintptr_t pwm_addr;

  /* bcm2835 clk register mapping */
  uintptr_t clk_addr;

#if 0
  /* TODO: remove bcm2835 lib dependency */
  /* bcm2835 pwm mux gpio register mapping */
  uintptr_t muxio_addr;
#endif

} pt_dev_t;


int pt_open(pt_dev_t*);
int pt_close(pt_dev_t*);
int pt_control(pt_dev_t*, pt_op_t);

static inline int pt_pan_cw(pt_dev_t* d)
{
  return pt_control(d, PT_OP_PAN_CW);
}

static inline int pt_pan_ccw(pt_dev_t* d)
{
  return pt_control(d, PT_OP_PAN_CCW);
}

static inline int pt_tilt_cw(pt_dev_t* d)
{
  return pt_control(d, PT_OP_TILT_CW);
}

static inline int pt_tilt_ccw(pt_dev_t* d)
{
  return pt_control(d, PT_OP_TILT_CCW);
}


#endif /* PT_H_INCLUDED */
