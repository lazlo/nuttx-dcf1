/* Driver for the DCF1 */

/* http://www.mikrocontroller.net/topic/248487 -- DCF77 Datagram Synchronization */

/***********************************************************************/
/***********************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <stm32.h>

/***********************************************************************/
/***********************************************************************/

#define GPIO_DCF1_LED	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN2)
#define GPIO_DCF1_PON	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN4)
#define GPIO_DCF1_DATA	(GPIO_INPUT|GPIO_EXTI|GPIO_OPENDRAIN|GPIO_PORTF|GPIO_PIN5)

#define DCF1_REFCLOCK	CLOCK_MONOTONIC

/* Configuration for decoding the signal provided by the DCF1 module */

#define DCF1_DATA_0_MS		100
#define DCF1_DATA_1_MS		200
#define DCF1_DATA_ERR_MS	30

#define DCF1_DATA_0_ERR_MS	DCF1_DATA_ERR_MS
#define DCF1_DATA_0_MAX_MS	(DCF1_DATA_0_MS + DCF1_DATA_0_ERR_MS)
#define DCF1_DATA_0_MIN_MS	(DCF1_DATA_0_MS - DCF1_DATA_0_ERR_MS)

#define DCF1_DATA_1_ERR_MS	DCF1_DATA_ERR_MS
#define DCF1_DATA_1_MAX_MS	(DCF1_DATA_1_MS + DCF1_DATA_1_ERR_MS)
#define DCF1_DATA_1_MIN_MS	(DCF1_DATA_1_MS - DCF1_DATA_1_ERR_MS)

#define DCF1_IS_DATA_0(dt)	(dt > DCF1_DATA_0_MIN_MS && dt < DCF1_DATA_1_MAX_MS)
#define DCF1_IS_DATA_1(dt)	(dt > DCF1_DATA_1_MIN_MS && dt < DCF1_DATA_1_MAX_MS)

/* Measure time between DATA pin level transitions */
//#define DEBUG_DCF1_MEASURE

/* Calculate bits from time delta */
#define DEBUG_DCF1_DECODE

#define dcf1dbg	printf

#ifdef DEBUG_DCF1_MEASURE
# 	define dcf1dbg_me	dcf1dbg
#else
#	define dcf1dbg_me(x...)
#endif
#ifdef DEBUG_DCF1_DECODE
#	define dcf1dbg_de	dcf1dbg
#else
#	define dcf1dbg_de(x...)
#endif

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

	bool	data;
	bool	data_last;

	struct timespec	t1; /* Time of low to high transition of data pin */
	struct timespec t2; /* Time of high to low transition of data pin */
} dev;

/***********************************************************************/
/***********************************************************************/

static void dcf1_decode(const long delta_msec)
{
	dcf1dbg_de("dcf1 RX ");

	/* Decide if the delta is a binary 1, 0 or error */
	if (DCF1_IS_DATA_0(delta_msec))
		dcf1dbg_de("0 (dt %d)", delta_msec);
	else if (DCF1_IS_DATA_1(delta_msec))
		dcf1dbg_de("1 (dt %d)", delta_msec);
	else
		dcf1dbg_de("err d %ld = (%ld - %ld) / %ld",
				delta_msec, dev.t2.tv_nsec, dev.t1.tv_nsec,
				1000000);
	dcf1dbg_de("\n");
}

/* Handles interrupt */
static int dcf1_interrupt(int irq, void *context)
{
	sem_post(&dev.isr_sem);
	return OK;
}

/* Process data */
static int dcf1_procirq(int argc, char *argv[])
{
	long delta_msec = 0;

	while (1)
	{
		/* Wait for interrupt to occur */
		sem_wait(&dev.isr_sem);

		/*
		 * Measure low/high phase duration
		 */

		/* Read the current state of data */
		dev.data = stm32_gpioread(GPIO_DCF1_DATA);
		dcf1dbg_me("dcf1  %d", dev.data);

		/* Make the LED mirror the current data state */
		dev.led_state = dev.data;
		stm32_gpiowrite(GPIO_DCF1_LED, dev.led_state);

		if (dev.data_last == 0 && dev.data == 1)
		{
			dcf1dbg_me(" t1");

			/* Save the current time as t1 or t_START */
			clock_gettime(DCF1_REFCLOCK, &dev.t1);
			dcf1dbg_me("=%ld", dev.t1.tv_nsec / 1000000);
		}
		else if (dev.data_last == 1 && dev.data == 0)
		{
			dcf1dbg_me(" t2");

			/* Save the current time as t2 or t_END */
			clock_gettime(DCF1_REFCLOCK, &dev.t2);
			dcf1dbg_me("=%ld", dev.t2.tv_sec / 1000000);

			/* Subtract t2 - t1 and display result */
			delta_msec = (dev.t2.tv_nsec - dev.t1.tv_nsec) / 1000000;

			dcf1dbg_me(" d %ld", delta_msec);
		}
		else
		{
			/* Should not happen! */
			dcf1dbg_me(" err");
		}
		dcf1dbg_me("\n");

		/*
		 * Decode duration into bits
		 */

		if (delta_msec)
		{
			dcf1_decode(delta_msec);

			delta_msec = 0;
		}

		/* Prepare for new loop iteration */
		dev.data_last = dev.data;
	}
	return OK;
}

/* Turn the module on or off */
static void dcf1_enable(const bool onoff)
{
	/* Enable by pulling PON pin low */
	/* Disable receiver module by pulling pin high */
	stm32_gpiowrite(GPIO_DCF1_PON, !onoff);
	stm32_gpiowrite(GPIO_DCF1_LED, onoff);
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

	/* Set default output levels (receiver and LED off by default) */
	dcf1_enable(false);

	/* Register handler for ext. interrupt on DCF1_DATA */
	stm32_gpiosetevent(GPIO_DCF1_DATA, true, true, false, dcf1_interrupt);

	/* Fork a process to wait for interrupts to process */
	task_create("dcf1", 100, 1024, dcf1_procirq, NULL);

	/* Enable the receiver module */
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
