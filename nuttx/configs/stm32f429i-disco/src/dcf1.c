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

static void	dcf1_enable(const bool onoff);
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

static struct dcf1_dev {
	bool	led_state;
	sem_t	isr_sem;
} dev;

/***********************************************************************/
/***********************************************************************/

/* Handles interrupt */
static int dcf1_interrupt(int irq, void *context)
{
	sem_post(&dev.isr_sem);
	return OK;
}

/* Process data */
static int dcf1_procirq(int argc, char *argv[])
{
	while (1)
	{
		/* Wait for interrupt to occur */
		sem_wait(&dev.isr_sem);
		/* Read the current state of data */
		dev.led_state = stm32_gpioread(GPIO_DCF1_DATA);

		dcf1dbg("dcf1 data %d\n", dev.led_state);

		/* Make the LED mirror the current data state */
		stm32_gpiowrite(GPIO_DCF1_LED, dev.led_state);
	}
	return OK;
}

/* Turn the module on or off */
static void dcf1_enable(const bool onoff)
{
	/* Enable by pulling PON pin low */
	/* Disable receiver module by pulling pin high */
	stm32_gpiowrite(GPIO_DCF1_PON, !onoff);
}

static void dcf1_init(void)
{
	dcf1dbg("dcf1_init\n");

	/* Initialize the device state */
	dev.led_state = true;
	sem_init(&dev.isr_sem, 1, 0);

	/* Setup pins */
	stm32_configgpio(GPIO_DCF1_LED);
	stm32_configgpio(GPIO_DCF1_PON);
	stm32_configgpio(GPIO_DCF1_DATA);

	/* Set default output levels */
	stm32_gpiowrite(GPIO_DCF1_LED, true);
	stm32_gpiowrite(GPIO_DCF1_PON, true);

	/* Register handler for ext. interrupt on DCF1_DATA */
	stm32_gpiosetevent(GPIO_DCF1_DATA, true, true, false, dcf1_interrupt);

	/* Fork a process to wait for interrupts to process */
	task_create("dcf1", 100, 1024, dcf1_procirq, NULL);

	dcf1_enable(true);
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
