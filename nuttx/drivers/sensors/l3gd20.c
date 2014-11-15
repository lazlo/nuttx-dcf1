/* ST L3GD20 Gyroscope Driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/l3gd20.h>

#ifdef CONFIG_L3GD20

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addreses */

#define ST_L3GD20_WHO_AM_I      0x0f
#define ST_L3GD20_CTRL_REG1     0x20
#define ST_L3GD20_CTRL_REG2     0x21
#define ST_L3GD20_CTRL_REG3     0x22
#define ST_L3GD20_CTRL_REG4     0x23
#define ST_L3GD20_CTRL_REG5     0x24
#define ST_L3GD20_REFERENCE     0x25
#define ST_L3GD20_OUT_TEMP      0x26
#define ST_L3GD20_STATUS_REG    0x27
#define ST_L3GD20_OUT_X_L       0x28
#define ST_L3GD20_OUT_X_H       0x29
#define ST_L3GD20_OUT_Y_L       0x2A
#define ST_L3GD20_OUT_Y_H       0x2B
#define ST_L3GD20_OUT_Z_L       0x2C
#define ST_L3GD20_OUT_Z_H       0x2D
#define ST_L3GD20_FIFO_CTRL_REG 0x2E
#define ST_L3GD20_FIFO_SRC_REG  0x2F
#define ST_L3GD20_INT1_CFG      0x30
#define ST_L3GD20_INT1_SRC      0x31
#define ST_L3GD20_INT1_THS_XH   0x32
#define ST_L3GD20_INT1_THS_XL   0x33
#define ST_L3GD20_INT1_THS_YH   0x34
#define ST_L3GD20_INT1_THS_YL   0x35
#define ST_L3GD20_INT1_THS_ZH   0x36
#define ST_L3GD20_INT1_THS_ZL   0x37
#define ST_L3GD20_INT1_DURATION 0x38

/* CTRL_REG1
 *_DR1  (1 << 7)
 * DR0  (1 << 6)
 * BW1  (1 << 5)
 * BW0  (1 << 4)
 * PD   (1 << 3)
 * Zen  (1 << 2)
 * Xen  (1 << 1)
 * Yen  (1 << 0)
 *
 * CTRL_REG2
 * HPM1      (1 << 5)
 * HPM0      (1 << 4)
 * HPCF3     (1 << 3)
 * HPCF2     (1 << 2)
 * HPCF1     (1 << 1)
 * HPCF1     (1 << 0)
 *
 * CTRL_REG3
 * I1_Int1   (1 << 7)
 * I1_Boot   (1 << 6)
 * H_Lactive (1 << 5)
 * PP_OD     (1 << 4)
 * I2_DRDY   (1 << 3)
 * I2_WTM    (1 << 2)
 * I2_ORun   (1 << 1)
 * I2_Empty  (1 << 0)
 *
 * CTRL_REG4
 * BDU       (1 << 7)
 * BLE       (1 << 6)
 * FS1       (1 << 5)
 * FS0       (1 << 4)
 * SIM       (1 << 0)
 *
 * CTRL_REG5
 * BOOT      (1 << 7)
 * FIFO_EN   (1 << 6)
 * HPen      (1 << 4)
 * INT1_Sel1 (1 << 3)
 * INT1_Sel0 (1 << 2)
 * Out_Sel1  (1 << 1)
 * Out_Sel0  (1 << 0)
 *
 * STATUS_REG
 * ZYXOR     (1 << 7)
 * ZOR       (1 << 6)
 * YOR       (1 << 5)
 * XOR       (1 << 4)
 * ZYXDA     (1 << 3)
 * ZDA       (1 << 2)
 * YDA       (1 << 1)
 * XDA       (1 << 0)
 *
 * FIFO_CTRL_REG
 * FM2       (1 << 7)
 * FM1       (1 << 6)
 * FM0       (1 << 5)
 * WTM4      (1 << 4)
 * WTM3      (1 << 3)
 * WTM2      (1 << 2)
 * WTM1      (1 << 1)
 * WTM0      (1 << 0)
 */

#define ST_L3GD20_FIFO_CTRL_REG_FM_MASK         0xE0
#define ST_L3GD20_FIFO_CTRL_REG_FM_SHIFT        5
#define ST_L3GD20_FIFO_CTRL_REG_WTM_MASK        0x1F
#define ST_L3GD20_FIFO_CTRL_REG_WTM_SHIFT       0

/*
 * FIFO_SRC_REG
 * WTM
 * OVRN
 * EMPTY
 * FSS4
 * FSS3
 * FSS2
 * FSS1
 * FSS0
 *
 * INT1_CFG
 * AND/OR
 * LIR
 * ZHIE
 * ZLIE
 * YHIE
 * YLIE
 * XHIE
 * XLIE
 *
 * INT1_SRC
 * IA  (1 << 6)
 * ZH  (1 << 5)
 * ZL  (1 << 4)
 * YH  (1 << 3)
 * YL  (1 << 2)
 * XH  (1 << 1)
 * XL  (1 << 0)
 *
 * INT1_DURATION
 * WAIT
 * D6
 * D6
 * D4
 * D3
 * D2
 * D1
 * D0
 */

/* Device identification register value */

#define ST_L3GD20_WHO_AM_I_VALUE 0x4D

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct l3gd20_dev_s
{
  struct i2c_dev_s       *i2c;      /* I2C bus driver instance */
  uint16_t                address;  /* I2C slave adress */
  uint8_t                 cr1;
  uint8_t                 cr2;
  uint8_t                 cr3;
  uint8_t                 cr4;
  uint8_t                 cr5;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int l3gd20_access(FAR struct l3gd20_dev_s *dev, uint8_t subaddr,
                         FAR uint8_t *buf, int length)
{
  uint16_t flags = 0;
  int retval;

  if (length > 0)
    {
      flags = I2C_M_READ;
    }
  else
    {
      flags = I2C_M_NORESTART;
      length = -length;
    }

  /* TODO Validate subaddress */

  if (subaddr < ST_L3GD20_WHO_AM_I || subaddr > ST_L3GD20_INT1_DURATION)
    {
      errno = EFAULT;
      return ERROR;
    }

  /* Create message and send */

  struct i2c_msg_s msgv[2] =
  {
    {
      .addr   = dev->address,
      .flags  = 0,
      .buffer = &subaddr,
      .length = 1,
    },
    {
      .addr   = dev->address,
      .flags  = flags,
      .buffer = buf,
      .length = length
    }
  };

  if ((retval = I2C_TRANSFER(dev->i2c, msgv, 2)) == OK)
    {
      return length;
    }

  return retval;
}

static int l3gd20_readregs(FAR struct l3gd20_dev_s *dev)
{
  if (l3gd20_access(dev, ST_L3GD20_CTRL_REG1, &dev->cr1, 5) != 5)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct l3gd20_dev_s * l3gd20_init(FAR struct i2c_dev_s * i2c,
                                      uint16_t address)
{
  FAR struct l3gd20_dev_s *dev;
  uint8_t retval;

  ASSERT(i2c);
  ASSERT(address);

  dev = kmm_malloc(sizeof(struct l3gd20_dev_s));
  if (dev == NULL)
    {
      errno = ENOMEM;
      return NULL;
    }

  memset(dev, 0, sizeof(struct l3gd20_dev_s));
  dev->i2c     = i2c;
  dev->address = address;

  /* Probe device */

  if (l3gd20_access(dev, ST_L3GD20_WHO_AM_I, &retval, 1) > 0)
    {
      if (retval == ST_L3GD20_WHO_AM_I_VALUE)
        {
          if (l3gd20_readregs(dev) == OK && l3gd20_powerup(dev) == OK)
            {
              errno = 0;
              return dev;
            }
        }
      retval = ENODEV;
    }
  else
    {
      retval = EFAULT;
    }

  kmm_free(dev);
  errno = retval;
  return NULL;
}

int l3gd20_deinit(struct l3gd20_dev_s *dev)
{
  ASSERT(dev);

  kmm_free(dev);

  return OK;
}

int l3gd20_powerup(struct l3gd20_dev_s *dev)
{
  return ERROR;
}

int l3gd20_powerdown(struct l3gd20_dev_s *dev)
{
  return ERROR;
}

int l3gd20_setfifomode(struct l3gd20_dev_s *dev, enum l3gd20_fifomode_e mode)
{
  ASSERT(dev);

  /* TODO Set mode in FIFO_CTRL_REG */

  return ERROR;
}

int l3gd20_getfifomode(struct l3gd20_dev_s *dev, enum l3gd20_fifomode_e *mode)
{
  const uint8_t regaddr = ST_L3GD20_FIFO_CTRL_REG;
  const uint8_t mask    = ST_L3GD20_FIFO_CTRL_REG_FM_MASK;
  const uint8_t shift   = ST_L3GD20_FIFO_CTRL_REG_FM_SHIFT;
  uint8_t regval;

  ASSERT(dev);

  if (l3gd20_access(dev, regaddr, &regval, 1) != 1)
    return ERROR;

  /* Extract the FIFO mode from the register value */
  *mode = (regval & mask) >> shift;

  return OK;
}

int l3gd20_setfifoths(struct l3gd20_dev_s *dev, uint8_t ths)
{
  return OK;
}

int l3gd20_getfifoths(struct l3gd20_dev_s *dev, uint8_t *ths)
{
  const uint8_t regaddr = ST_L3GD20_FIFO_CTRL_REG;
  const uint8_t mask    = ST_L3GD20_FIFO_CTRL_REG_WTM_MASK;
  const uint8_t shift   = ST_L3GD20_FIFO_CTRL_REG_WTM_SHIFT;
  uint8_t regval;

  ASSERT(dev);

  if (l3gd20_access(dev, regaddr, &regval, 1) != 1)
    return ERROR;

  *ths = (regval & mask) >> shift;

  return OK;
}

int l3gd20_setthresholds(FAR struct l3gd20_vector_s *vect)
{
  return ERROR;
}

int l3gd20_getthresholds(FAR struct l3gd20_vector_s *vect)
{
  return ERROR;
}

FAR const struct l3gd20_vector_s *
l3gd20_getreadings(FAR struct l3gd20_dev_s *dev)
{
  struct l3gd20_vector_s *vect;

  ASSERT(dev);

  /* TODO Implement read address to samples */

  return NULL;
}

#endif /* CONFIG_L3GD20 */
