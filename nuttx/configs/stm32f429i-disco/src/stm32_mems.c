
#include <nuttx/i2c.h>
#include <nuttx/sensors/l3gd20.h>

#ifdef CONFIG_L3GD20

void stm32_mems_initialize(void)
{
  const int port = 3;
  const int addr = 0x41;
  struct i2c_dev_s *i2c;
  struct l3gd20_dev_s *dev;

  i2c = up_i2cinitialize(port);
  if (!i2c)
    {
      /* TODO Set error / notify user */
      return;
    }

  dev = l3gd20_init(i2c, addr);
  if (!dev)
    {
      /* TODO Set error / notify user */
      return;
    }
}

#endif /* CONFIG_L3GD20 */
