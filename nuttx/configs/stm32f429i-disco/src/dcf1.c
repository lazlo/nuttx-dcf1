/* Driver for the DCF1 */

/***********************************************************************/
/***********************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <stm32.h>

/***********************************************************************/
/***********************************************************************/

#define GPIO_DCF1_LED	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN2)
#define GPIO_DCF1_PON	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN4)
#define GPIO_DCF1_DATA	(GPIO_INPUT|GPIO_EXTI|GPIO_OPENDRAIN|GPIO_PORTF|GPIO_PIN5)

#define dcf1dbg	printf

/***********************************************************************/
/***********************************************************************/

typedef FAR struct file file_t;

static void	dcf1_init(void);

static int	dcf1_open(file_t *filep);
static int	dcf1_close(file_t *filep);
static ssize_t	dcf1_read(file_t *filep, FAR char *buf, size_t buflen);
static ssize_t	dcf1_write(file_t *filep, FAR const char *buf, size_t buflen);

static const struct file_operations dcf1_ops = {
	dcf1_open,	/* open */
	dcf1_close,	/* close */
	dcf1_read,	/* read */
	dcf1_write,	/* write */
	0,		/* seek */
	0,		/* ioctl */
};

/***********************************************************************/
/***********************************************************************/

static void dcf1_init(void)
{
	dcf1dbg("dcf1_init\n");

	/* Setup pins */
	stm32_configgpio(GPIO_DCF1_LED);
	stm32_configgpio(GPIO_DCF1_PON);
	stm32_configgpio(GPIO_DCF1_DATA);

	/* Set default output levels */
	stm32_gpiowrite(GPIO_DCF1_LED, true);
	stm32_gpiowrite(GPIO_DCF1_PON, true);

	/* TODO register handler for ext. interrupt on DCF1_DATA */

	/* Enable by pulling PON pin low */
	stm32_gpiowrite(GPIO_DCF1_PON, false);
}

static int dcf1_open(file_t *filep)
{
	dcf1dbg("dcf1_open\n");
	return OK;
}

static int dcf1_close(file_t *filep)
{
	dcf1dbg("dcf1_close\n");
	return OK;
}

static ssize_t dcf1_read(file_t *filep, FAR char *buf, size_t buflen)
{
	dcf1dbg("dcf1_read\n");
	if (buf == NULL || buflen < 1)
		return -EINVAL;

	*buf = stm32_gpioread(GPIO_DCF1_DATA);

	return OK;
}

static ssize_t dcf1_write(file_t *filep, FAR const char *buf, size_t buflen)
{
	dcf1dbg("dcf1_write\n");
	if (buf == NULL || buflen < 1)
		return -EINVAL;

	return OK;
}

/***********************************************************************/
/***********************************************************************/

void up_dcf1(void)
{
	dcf1dbg("up_dcf1\n");
	dcf1_init();

	(void)register_driver("/dev/dcf1", &dcf1_ops, 0444, NULL);
}
