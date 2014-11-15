/* ST L3GD20 Gyroscope Driver */

#pragma once

#include <nuttx/i2c.h>
#include <stdint.h>

/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data Types
 ************************************************************************************/

enum l3gd20_fifomode_e {
  L3GD20_BYPASS_MODE           = 0,
  L3GD20_FIFO_MODE             = 1,
  L3GD20_STREAM_MODE           = 2,
  L3GD20_STREAM_TO_FIFO_MODE   = 3,
  L3GD20_BYPASS_TO_STREAM_MODE = 4
};

struct l3gd20_dev_s;

struct l3gd20_vector_s {
  uint16_t x, y, z;
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/** Initialize ST L3GD20 Chip */

EXTERN struct l3gd20_dev_s * l3gd20_init(struct i2c_dev_s * i2c, uint16_t address);

/** Deinitialize the ST L3GD20 Chip */

EXTERN int l3gd20_deinit(struct l3gd20_dev_s * dev);

/** Power up device, start conversion */

EXTERN int l3gd20_powerup(struct l3gd20_dev_s * dev);

/** Power down device, stop conversion */

EXTERN int l3gd20_powerdown(struct l3gd20_dev_s *dev);

/** Set FIFO operation mode */

EXTERN int l3gd20_setfifomode(struct l3gd20_dev_s *dev, enum l3gd20_fifomode_e mode);

/** Get FIFO operation mode */

EXTERN int l3gd20_getfifomode(struct l3gd20_dev_s *dev, enum l3gd20_fifomode_e *mode);

/** Set FIFO watermark */

EXTERN int l3gd20_setfifoths(struct l3gd20_dev_s *dev, uint8_t ths);

/** Get FIFO watermark */

EXTERN int l3gd20_getfifoths(struct l3gd20_dev_s *dev, uint8_t *ths);

/** Read thresholds */

EXTERN int l3gd20_setthresholds(struct l3gd20_vector_s * vect);

/** Write thresholds */

EXTERN int l3gd20_getthresholds(struct l3gd20_vector_s * vect);

/** Get readings */

EXTERN const struct l3gd20_vector_s * l3gd20_getreadings(struct l3gd20_dev_s *dev);

#endif /* __ASSEMBLY */
