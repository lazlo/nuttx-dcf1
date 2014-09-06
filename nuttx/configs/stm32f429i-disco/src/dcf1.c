/* Driver for the DCF1 by D. Laszlo Sitzer <dlsitzer@gmail.com> */

/* To activate the driver the following configuration options need
 * to be set:
 *
 * - CONFIG_CLOCK_MONOTONIC=y
 *
 *   RTOS Features ---> Clocks and Timers ---> [*] Support CLOCK_MONOTONIC
 *
 * - CONFIG_NSH_ARCHINIT=y
 *
 *   Application Configuration ---> NSH Library ---> [*] Have architecture-specific initialization
 */

/* http://www.mikrocontroller.net/topic/248487 -- DCF77 Datagram Synchronization */

/***********************************************************************/
/* Includes                                                            */
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
/* Configuration                                                       */
/***********************************************************************/

#define GPIO_DCF1_LED	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN2)
#define GPIO_DCF1_PON	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN4)
#define GPIO_DCF1_DATA	(GPIO_INPUT|GPIO_EXTI|GPIO_OPENDRAIN|GPIO_PORTF|GPIO_PIN5)

/* Specifies the clock id to use when measuring the time between DATA
 * pin level transitions.
 *
 * The default for this driver is to use the monotonic clock, which is
 * not enabled by default in NuttX.
 *
 * If you run menuconfig you can find the option at:
 *
 * RTOS Features ---> Clocks and Timers ---> [*] Support CLOCK_MONOTONIC
 */
#define DCF1_REFCLOCK	CLOCK_MONOTONIC

/* Configuration for decoding the signal provided by the DCF1 module */

#define DCF1_DATA_0_MS		100
#define DCF1_DATA_1_MS		200
#define DCF1_DATA_ERR_MS	30

/* Measure time between DATA pin level transitions */
//#define DEBUG_DCF1_MEASURE

/* Calculate bits from time delta */
#define DEBUG_DCF1_DECODE

/***********************************************************************/
/* Helpers                                                             */
/***********************************************************************/

#define DCF1_DATA_0_ERR_MS	DCF1_DATA_ERR_MS
#define DCF1_DATA_0_MAX_MS	(DCF1_DATA_0_MS + DCF1_DATA_0_ERR_MS)
#define DCF1_DATA_0_MIN_MS	(DCF1_DATA_0_MS - DCF1_DATA_0_ERR_MS)

#define DCF1_DATA_1_ERR_MS	DCF1_DATA_ERR_MS
#define DCF1_DATA_1_MAX_MS	(DCF1_DATA_1_MS + DCF1_DATA_1_ERR_MS)
#define DCF1_DATA_1_MIN_MS	(DCF1_DATA_1_MS - DCF1_DATA_1_ERR_MS)

#define DCF1_IS_DATA_0(dt)	(dt >= DCF1_DATA_0_MIN_MS && dt <= DCF1_DATA_0_MAX_MS)
#define DCF1_IS_DATA_1(dt)	(dt >= DCF1_DATA_1_MIN_MS && dt <= DCF1_DATA_1_MAX_MS)

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

static bool	dcf1_read_data_pin(void);
static void	dcf1_write_led_pin(const bool out);
static void	dcf1_getreftime(struct timespec *t);

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

	struct timespec	t_start; /* Time of low to high transition of data pin */
	struct timespec t_end; /* Time of high to low transition of data pin */

	uint64_t rxbuf;
} dev;

/***********************************************************************/
/* Private Functions                                                   */
/***********************************************************************/

static bool dcf1_read_data_pin(void)
{
	return stm32_gpioread(GPIO_DCF1_DATA);
}

static void dcf1_write_led_pin(const bool out)
{
	stm32_gpiowrite(GPIO_DCF1_LED, out);
}

static void dcf1_getreftime(struct timespec *t)
{
	clock_gettime(DCF1_REFCLOCK, t);
}

/* Measure the time difference between a low-to-high and the next
 * high-to-low transisition on the DATA pin in miliseconds. */
static long dcf1_measure(void)
{
	long delta_msec = 0;

	/* Read the current state of data */
	dev.data = dcf1_read_data_pin();
	dcf1dbg_me("dcf1  %d", dev.data);

	/* Make the LED mirror the current data state */
	dev.led_state = dev.data;
	dcf1_write_led_pin(dev.led_state);

	if (dev.data_last == 0 && dev.data == 1)
	{
		dcf1dbg_me(" t_sta");

		/* Save the current time as t1 or t_START */
		dcf1_getreftime(&dev.t_start);
		dcf1dbg_me("=%ld", dev.t_start.tv_nsec / 1000000);
	}
	else if (dev.data_last == 1 && dev.data == 0)
	{
		dcf1dbg_me(" t_end");

		/* Save the current time as t2 or t_END */
		dcf1_getreftime(&dev.t_end);
		dcf1dbg_me("=%ld", dev.t_end.tv_nsec / 1000000);

		/* Subtract t2 - t1 and display result */
		delta_msec = (dev.t_end.tv_nsec - dev.t_start.tv_nsec) / 1000000;

		dcf1dbg_me(" (dt %ld)", delta_msec);
	}
	else
	{
		/* Should not happen! */
		dcf1dbg_me(" err");
	}
	dcf1dbg_me("\n");

	return delta_msec;
}

/* Decode the time delta measured into a bit. Return -1 on error. */
static char dcf1_decode(const long delta_msec)
{
	char bit;

	dcf1dbg_de("dcf1 RX ");

	/* Decide if the delta is a binary 1, 0 or error */
	if (DCF1_IS_DATA_0(delta_msec))
	{
		bit = 0;
		dcf1dbg_de("0 (dt %d)", delta_msec);
	}
	else if (DCF1_IS_DATA_1(delta_msec))
	{
		bit = 1;
		dcf1dbg_de("1 (dt %d)", delta_msec);
	}
	else
	{
		bit = -1;
		dcf1dbg_de("er dt %ld = %ld - %ld",
				delta_msec,
				dev.t_start.tv_nsec / 1000000,
				dev.t_end.tv_nsec / 1000000);
	}
	dcf1dbg_de("\n");
	return bit;
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
	char bit;

	while (1)
	{
		/* Wait for interrupt to occur */
		sem_wait(&dev.isr_sem);

		/*
		 * Measure low/high phase duration
		 */

		delta_msec = dcf1_measure();

		/*
		 * Decode duration into bits
		 */

		if (delta_msec)
		{
			bit = dcf1_decode(delta_msec);

			/* Only modify receive buffer on successful decoding */
			if (bit != -1)
			{
				/* Make space for one bit in the receive buffer */
				dev.rxbuf <<= 1;

				if (bit)
					dev.rxbuf |= 1;
			}

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


	/* Display min/max values for decoding (only for development) */
	dcf1dbg("dcf1 0 = %d ms (max: %d min: %d) 1 = %d ms (max: %d min: %d)\n",
		DCF1_DATA_0_MS,
		DCF1_DATA_0_MAX_MS,
		DCF1_DATA_0_MIN_MS,
		DCF1_DATA_1_MS,
		DCF1_DATA_1_MAX_MS,
		DCF1_DATA_1_MIN_MS);
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

	/* TODO Check if there is a DCF77 message we can read */
	/* TODO Read a complete receieved DCF77 message */

	*buf = stm32_gpioread(GPIO_DCF1_DATA);

	return OK;
}

static ssize_t dcf1_write(file_t *filep, FAR const char *buf, size_t buflen)
{
	dcf1dbg("dcf1_write\n");
	if (buf == NULL || buflen < 1)
		return -EINVAL;

	/* TODO Write might be used to turn the receiver on or off
	 * using the PON pin. */

	return OK;
}

/***********************************************************************/
/* Public Functions                                                    */
/***********************************************************************/

void up_dcf1(void)
{
	dcf1dbg("up_dcf1\n");
	dcf1_init();

	(void)register_driver("/dev/dcf1", &dcf1_ops, 0444, NULL);
}
