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
 *
 * - CONFIG_RADIO_DCF1=y
 *
 *   Device Drivers ---> Radio Device Support ---> [*] DCF1 Time Signal Receiver
 */

/* http://www.mikrocontroller.net/topic/248487 -- DCF77 Datagram Synchronization */

/***********************************************************************/
/* Includes                                                            */
/***********************************************************************/

#include <nuttx/config.h>
#include <nuttx/radio/ioctl.h>

#include <debug.h>
#include <time.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>

#include <nuttx/radio/dcf1.h>
#include <nuttx/radio/dcf77.h>

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

/* TODO Better check for CLOCK_MONOTONIC symbol ifself. */
#ifndef CONFIG_CLOCK_MONOTONIC
#  error "CLOCK_MONOTONIC=y needs to be set in .config"
#endif

#ifndef CONFIG_NSH_ARCHINIT
#  error "CONFIG_NSH_ARCHINIT is not defined"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "CONFIG_SCHED_WORKQUEUE is not defined"
#endif

/* Configuration *******************************************************/

/* Device path to be used for the driver */

#define DCF1_DEVFILE "/dev/dcf1"

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
/* TODO Stuff that is DCF77 specific is to be moved to the dcf77.h */

#define DCF1_DATA_0_MS		100
#define DCF1_DATA_1_MS		200
#define DCF1_DATA_ERR_MS	30

#define DCF1_SYNC_MARK_MIN_MS	1800
#define DCF1_SYNC_MARK_MAX_MS	2000
#define DCF1_SYNC_MARK_ERR_MS	10

/***********************************************************************/
/* Helpers                                                             */
/***********************************************************************/

/* TODO Replace macros and instead use calculations for values in
 * 	threshold struct. */

#define DCF1_DATA_0_ERR_MS	DCF1_DATA_ERR_MS
#define DCF1_DATA_0_MAX_MS	(DCF1_DATA_0_MS + DCF1_DATA_0_ERR_MS)
#define DCF1_DATA_0_MIN_MS	(DCF1_DATA_0_MS - DCF1_DATA_0_ERR_MS)

#define DCF1_DATA_1_ERR_MS	DCF1_DATA_ERR_MS
#define DCF1_DATA_1_MAX_MS	(DCF1_DATA_1_MS + DCF1_DATA_1_ERR_MS)
#define DCF1_DATA_1_MIN_MS	(DCF1_DATA_1_MS - DCF1_DATA_1_ERR_MS)

#define DCF1_IS_DATA_0(dt)	(dt >= DCF1_DATA_0_MIN_MS && dt <= DCF1_DATA_0_MAX_MS)
#define DCF1_IS_DATA_1(dt)	(dt >= DCF1_DATA_1_MIN_MS && dt <= DCF1_DATA_1_MAX_MS)

#define TS_TO_MS(ts)		((ts.tv_sec * MSEC_PER_SEC) + (ts.tv_nsec / USEC_PER_SEC))

#define DCF1_IS_START(dt)	(TS_TO_MS(dt) >= DCF1_SYNC_MARK_MIN_MS && TS_TO_MS(dt) <= DCF1_SYNC_MARK_MAX_MS)

/* TODO Replace macros with static functions */

#ifdef CONFIG_DEBUG_DCF1_MEASUREMENT
# 	define dcf1dbg_me	dcf1dbg
#else
#	define dcf1dbg_me(x...)
#endif
#ifdef CONFIG_DEBUG_DCF1_DECODE
#	define dcf1dbg_de	dcf1dbg
#else
#	define dcf1dbg_de(x...)
#endif
#ifdef CONFIG_DEBUG_DCF1_RECEIVE
#	define dcf1dbg_rx	dcf1dbg
#else
#	define dcf1dbg_rx(x...)
#endif
#ifdef CONFIG_DEBUG_DCF1_SYNC
#	define dcf1dbg_sy	dcf1dbg
#else
#	define dcf1dbg_sy(x...)
#endif

/***********************************************************************/
/* Private Types                                                       */
/***********************************************************************/

typedef FAR struct file file_t;

struct dcf1_thr {
	uint16_t data_0_ms;
	uint16_t data_0_min_ms;
	uint16_t data_0_max_ms;
	uint16_t data_1_ms;
	uint16_t data_1_min_ms;
	uint16_t data_1_max_ms;
};

struct dcf1_dev {

	uint32_t	gpio_data;
	uint32_t	gpio_pon;
	uint32_t	gpio_led;

	struct dcf1_gpio_s *pinops;
	struct dcf1_lower_s *lower;

	bool		led_out_state;
	struct work_s	irqwork;

	bool		data_in_state;
	bool		data_in_state_last;

	struct timespec t_start; /* Time of low to high transition of data pin */
	struct timespec t_end; /* Time of high to low transition of data pin */
	struct timespec dt;

	/* Measurement thresholds */

	struct dcf1_thr thr;

	uint64_t	rxbuf;

	/* Receive Buffer Synchonization */

	struct timespec ti_last;
	struct timespec ti;
	struct timespec tid;
};

/***********************************************************************/
/* Private Function Prototypes                                         */
/***********************************************************************/

static void	dcf1dbg(const char *format, ...);

/* Functions that deal with I/O from/to GPIOs */

static bool	dcf1_read_data_pin(void);
static void	dcf1_write_pon_pin(const bool out);
static void	dcf1_write_led_pin(const bool out);
static void	dcf1_enable(const bool onoff);

/* Time Measurement */

static void	dcf1_getreftime(struct timespec *t);
static void	dcf1_timespec_sub(struct timespec min, struct timespec sub, struct timespec *dif);

/* Receive Buffer Management */

static void	dcf1_rxbuf_reset(void);
static void	dcf1_rxbuf_append(const bool bit);
static void	dcf1_rxbuf_show(const unsigned short rxbuflen, const unsigned short split_nbit);

/* Signal Processing */

static void 	dcf1_thresholds_show(struct dcf1_thr *t);
static long	dcf1_measure(void);
static char	dcf1_decode(const long delta_msec);

/* Receive Buffer Synchonization */

static bool	dcf1_synchonize(void);

/* Interrupt Handling */

static void 	dcf1_irqworker(FAR void *arg);
static int	dcf1_interrupt(int irq, void *context);

/* Device File System Interface */

static int	dcf1_open(file_t *filep);
static int	dcf1_close(file_t *filep);
static ssize_t	dcf1_read(file_t *filep, FAR char *buf, size_t buflen);
static ssize_t	dcf1_write(file_t *filep, FAR const char *buf, size_t buflen);
static int	dcf1_ioctl(file_t *filep, int cmd, unsigned long arg);

/***********************************************************************/
/* Private Variables                                                   */
/***********************************************************************/

static const struct file_operations dcf1_ops = {
	dcf1_open,	/* open */
	dcf1_close,	/* close */
	dcf1_read,	/* read */
	dcf1_write,	/* write */
	0,		/* seek */
	dcf1_ioctl,	/* ioctl */
};

static struct dcf1_dev dev;

/***********************************************************************/
/* Private Functions                                                   */
/***********************************************************************/

static void dcf1dbg(const char *format, ...)
{
	va_list args;

	va_start(args, format);
	vprintf(format, args);
	va_end(args);
}

/* GPIO Control ********************************************************/

static bool dcf1_read_data_pin(void)
{
	return (*dev.pinops->read)(dev.gpio_data);
}

static void dcf1_write_pon_pin(const bool out)
{
	(*dev.pinops->write)(dev.gpio_pon, out);
}

static void dcf1_write_led_pin(const bool out)
{
	(*dev.pinops->write)(dev.gpio_led, out);
}

/* Module Control ******************************************************/

/* Turn the module on or off */
static void dcf1_enable(const bool onoff)
{
	/* Enable by pulling PON pin low */
	/* Disable receiver module by pulling pin high */
	dcf1_write_pon_pin(!onoff);
	dcf1_write_led_pin(onoff);

	/* Enable or disable the interrupt */
	if (onoff)
		dev.lower->enable(dev.lower);
	else
		dev.lower->disable(dev.lower);
}

/* Time and Math *******************************************************/

static void dcf1_getreftime(struct timespec *t)
{
	clock_gettime(DCF1_REFCLOCK, t);
}

#if 1
/* TODO Implement subtraction for timespec structures.
 * See http://lists.gnu.org/archive/html/bug-gnulib/2011-06/msg00371.html */
static void dcf1_timespec_sub(struct timespec min, struct timespec sub,
				struct timespec *dif)
{
	if (min.tv_nsec < sub.tv_nsec)
	{
//		dcf1dbg(" -X- "); /* subtract with carry */
		min.tv_nsec += NSEC_PER_SEC;
		min.tv_sec  -= 1;
	}

	dif->tv_nsec = min.tv_nsec - sub.tv_nsec;
	dif->tv_sec = min.tv_sec - sub.tv_sec;
}
#else
/* Subtract one timespect from the other and save the result in a third timespec structure */
static void timespec_sub(struct timespec *min, struct timespec *sub, struct timespec *dif)
{
	if (min->tv_sec <= sub->tv_sec)
	{
		dcf1dbg("dcf1 err tsub\n");
		return;
	}

	/* Subtract with carry? */
	if (min->tv_nsec < sub->tv_nsec)
	{
		/* When current nanosecond value is greater than from the last
		 * measurement, we will naturally get a negative value when subtracting.
		 *
		 * To get the correct value we take the number of seconds, turn them into
		 * nanoseconds, add the original nanosecond value. Now we subtract the
		 * last nanosecond value.
		 *
		 * In the next step we calculate the seconds. Here we rely on the fact
		 * the our calculation result of nanoseconds also contains the the full
		 * seconds. These are now taken into account instead of the actual seconds
		 * value.
		 *
		 * In the final step we need to remove the full seconds from the nanoseonds.
		 */
		dif->tv_nsec = (min->tv_sec * NSEC_PER_SEC) + min->tv_nsec - sub->tv_nsec;
		dif->tv_sec = (dif->tv_nsec / NSEC_PER_SEC) - min->tv_sec;
		dif->tv_nsec = dif->tv_nsec % NSEC_PER_SEC;
	}
	else
	{
		dif->tv_nsec = min->tv_nsec - sub->tv_nsec;
		dif->tv_sec = min->tv_sec - sub->tv_sec;
	}
}
#endif

/* Receive Buffer Management *******************************************/

static void dcf1_rxbuf_reset(void)
{
	dev.rxbuf = 0;
}

static void dcf1_rxbuf_append(const bool bit)
{
#define APPEND_RIGHTSHIFT

#ifdef APPEND_RIGHTSHIFT
#	define make_space(b)	(b >>= 1)
#else
#	define make_space(b)	(b <<= 1)
#endif

#ifdef APPEND_RIGHTSHIFT
#	define append_bit(b)	(b |= ((uint64_t)1 << 63))
#else
#	define append_bit(b)	(b |= 1);
#endif

	/* Make space for one bit in the receive buffer */
	make_space(dev.rxbuf);

	if (bit)
		append_bit(dev.rxbuf);
}

static void dcf1_rxbuf_show(const unsigned short rxbuflen, const unsigned short split_nbit)
{
	unsigned short i;

	dcf1dbg_rx("dcf1 rxbuf ");
	for (i = rxbuflen; i > 0; i--)
	{
		dcf1dbg_rx("%c", (dev.rxbuf & ((uint64_t)1 << i)) ? '1' : '0');
		if (((1+i) % split_nbit) == 0)
			dcf1dbg_rx(" ");
	}
	dcf1dbg_rx("\n");
}

/* Signal Processing ***************************************************/

static void dcf1_thresholds_show(struct dcf1_thr *t)
{
	/* Display min/max values for decoding (only for development) */
	dcf1dbg("dcf1 0 = %d ms (min: %d max: %d) 1 = %d ms (min: %d max: %d)\n",
		t->data_0_ms,
		t->data_0_min_ms,
		t->data_0_max_ms,
		t->data_1_ms,
		t->data_1_min_ms,
		t->data_1_max_ms);
}

/* Measure the time difference between a low-to-high and the next
 * high-to-low transisition on the DATA pin in miliseconds. */
static long dcf1_measure(void)
{
#	define LOW2HIGH(d)	(d.data_in_state_last == 0 && d.data_in_state == 1)
#	define HIGH2LOW(d)	(d.data_in_state_last == 1 && d.data_in_state == 0)
#	define putts(ts)	dcf1dbg_me("%d.%09ld s", ts.tv_sec, ts.tv_nsec)
#	define save_t_start()	dcf1_getreftime(&dev.t_start)
#	define save_t_end()	dcf1_getreftime(&dev.t_end)
#	define calc_dt()	dcf1_timespec_sub(dev.t_end, dev.t_start, &dev.dt)

	/* Read the current state of data */
	dev.data_in_state = dcf1_read_data_pin();

	/* Make the LED mirror the current data state */
	dev.led_out_state = dev.data_in_state;
	dcf1_write_led_pin(dev.led_out_state);

	if (LOW2HIGH(dev))
	{
		/* Save the current time as t1 or t_START */
		save_t_start();
	}
	else if (HIGH2LOW(dev))
	{
		/* Save the current time as t2 or t_END */
		save_t_end();

		/* Subtract t2 - t1 and display result */
		calc_dt();
	}

#ifdef CONFIG_DEBUG_DCF1_MEASUREMENT
	dcf1dbg_me("dcf1 ME %d", dev.data_in_state);
	if (LOW2HIGH(dev))
	{
		dcf1dbg_me(" t_sta = ");
		putts(dev.t_start);
	}
	else if (HIGH2LOW(dev))
	{
		dcf1dbg_me(" t_end = ");
		putts(dev.t_end);

		dcf1dbg_me(" (dt ");
		putts(dev.dt);
		dcf1dbg_me(")");
	}
	else
	{
		/* Should not happen! */
		dcf1dbg_me(" err");
	}
	dcf1dbg_me("\n");
#endif

	return dev.dt.tv_nsec / USEC_PER_SEC;
}

/* Decode the time delta measured into a bit. Return -1 on error. */
static char dcf1_decode(const long delta_msec)
{
	char bit;

	/* Decide if the delta is a binary 1, 0 or error */
	if (DCF1_IS_DATA_0(delta_msec))
	{
		bit = 0;
	}
	else if (DCF1_IS_DATA_1(delta_msec))
	{
		bit = 1;
	}
	else
	{
		bit = -1;
	}

#ifdef CONFIG_DEBUG_DCF1_DECODE
	dcf1dbg_de("dcf1 DE ");
	if (DCF1_IS_DATA_0(delta_msec))
	{
		dcf1dbg_de("%d   (dt %3ld ms)", bit, delta_msec);
	}
	else if (DCF1_IS_DATA_1(delta_msec))
	{
		dcf1dbg_de("%d   (dt %3ld ms)", bit, delta_msec);
	}
	else
	{
		/* TODO Use dcf1_timespec_sub() */
		dcf1dbg_de("er  (dt %3ld ms) = %ld.%ld-%ld.%ld",
				delta_msec,
				dev.t_end.tv_sec, (dev.t_end.tv_nsec / USEC_PER_SEC),
				dev.t_start.tv_sec, (dev.t_start.tv_nsec / USEC_PER_SEC));
	}
	dcf1dbg_de("\n");
#endif

	return bit;
}

static bool dcf1_synchonize(void)
{
	bool rc = false;

	/* Save time to calculate delta between two received bits */
	dcf1_getreftime(&dev.ti);

	/* Now that we have decoded a valid bit, we shall calculate
	 * the delta between this and the last valid bit received. */
	dcf1_timespec_sub(dev.ti, dev.ti_last, &dev.tid);

	if (DCF1_IS_START(dev.tid))
	{
		rc = true;
	}

#ifdef CONFIG_DEBUG_DCF1_SYNC
	dcf1dbg_sy("dcf1 SY ");
	if (DCF1_IS_START(dev.tid))
		dcf1dbg_sy("found start ");
	else
		dcf1dbg_sy("?  ");
	dcf1dbg_sy(" (dt %4d ms)\n", TS_TO_MS(dev.tid));
#endif

	/* Save current time as last for next measurement */
	memcpy(&dev.ti_last, &dev.ti, sizeof(dev.ti));

	return rc;
}

/* Process data
 *
 * TODO Measure time between last and current interrupt that
 * 	produced a valid bit. This information can than be
 * 	used to find the 60th bit position (not modulated)
 * 	by looking at the difference in time.
 * 	One would expect that when an interrupt for the bit 0
 * 	is triggered, the time since the last interrupt should
 * 	be 800 ms from bit 58, 1000 ms for bit 59.
 * 	So if the difference is 1800 ms and the current bit read
 * 	is zero, we have found the beginning of the datagram.
 */
static void dcf1_irqworker(FAR void *arg)
{
	const int dcf77msg_nbits = 60;
	const int pad = 64 % dcf77msg_nbits;

	long delta_msec = 0;
	char bit;
	int i;

	/*
	 * Measure low/high phase duration
	 */

	delta_msec = dcf1_measure();
	if (delta_msec)
	{
		/*
		 * Decode duration into bits
		 */

		bit = dcf1_decode(delta_msec);

		/* Only modify receive buffer on successful decoding */
		if (bit == -1)
			goto next;

		dcf1_rxbuf_append(bit);

		/* Display contents of receive buffer (for development)
		 * Display 60 bits (from the uint64_t) in groups of 20 bits. */
		/* TODO This is problematic since we do not specify if to show
		 * the first or the last 60 bits (little or big endian with respect
		 * to the direction the bits are shifted in!).
		 * You see, this is already complicated and causes bugs in dcf1_rxbuf_show()! */
		dcf1_rxbuf_show(dcf77msg_nbits, 20);

		if (dcf1_synchonize())
		{
			/* TODO Reset the current bit position counter to ... 0? */

			/* When shifting right, we right padd the
			 * receive buffer by 4 bits */
			for (i = 0; i < pad; i++)
			{
				dcf1_rxbuf_append(0);
				dcf1_rxbuf_show(dcf77msg_nbits, 20);
			}

			/* TODO Replace with call to dcf1_rxbuf_get() */
			struct dcf77msg_s *m = (struct dcf77msg_s *)&dev.rxbuf;

			if (dcf77msg_valid(*m))
			{
				dcf77msg_dump(*m);
			}
			else
			{
				dcf1dbg("dcf1 DCF77 msg invalid\n");
			}

			/* Empty the receive buffer */
			dcf1_rxbuf_reset();
		}

next:
		delta_msec = 0;
		memset(&dev.dt, 0, sizeof(dev.dt));
	}

	/* Prepare for new loop iteration */
	dev.data_in_state_last = dev.data_in_state;

	/* Enable interrupts again */
	dev.lower->enable(dev.lower);
}

/* Handles interrupt */
static int dcf1_interrupt(int irq, void *context)
{
	DEBUGASSERT(work_available(&dev.irqwork));

	dev.lower->disable(dev.lower);

	return work_queue(HPWORK, &dev.irqwork, dcf1_irqworker, (FAR void *)&dev, 0);
}

/* File System Interface ***********************************************/

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

	*buf = (*dev.pinops->read)(dev.gpio_data);

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

static int dcf1_ioctl(file_t *filep, int cmd, unsigned long arg)
{
	enum { DCF1_CMD_ONOFF };
	struct dcf1_ioctl_args {
		bool onoff;
	} *a = (struct dcf1_ioctl_args *)&arg;

	switch (cmd)
	{
	/* TODO Turn debugging on/off for each stage */
	/* TODO Turn on/off */
	case DCF1_CMD_ONOFF:

		/* -- determin if to turn on or off */
		/* -- update internal shadow state */
		/* -- set the PON pin accoding to request */
		dcf1_enable(a->onoff);
		break;

	/* TODO Get signal quality */
	/* TODO Get raw datagram */

	default:
		return ERROR;
	}

	return OK;
}

/***********************************************************************/
/* Public Functions                                                    */
/***********************************************************************/

void dcf1_init(struct dcf1_gpio_s *pinops, uint32_t datapin,
					uint32_t ponpin, uint32_t ledpin,
		struct dcf1_lower_s *lower)
{
	struct dcf1_thr *t = &dev.thr;

	dcf1dbg("dcf1_init\n");

	/* Load measurement thresholds */
	t->data_0_ms		= DCF1_DATA_0_MS;
	t->data_0_min_ms	= DCF1_DATA_0_MIN_MS;
	t->data_0_max_ms	= DCF1_DATA_0_MAX_MS;
	t->data_1_ms		= DCF1_DATA_1_MS;
	t->data_1_min_ms	= DCF1_DATA_1_MIN_MS;
	t->data_1_max_ms	= DCF1_DATA_1_MAX_MS;

	/* Initialize the device state */
	dev.gpio_data = datapin;
	dev.gpio_pon  = ponpin;
	dev.gpio_led  = ledpin;

	dev.pinops = pinops;
	dev.lower = lower;

	dev.led_out_state = true;

	/* Setup pins */
	(*dev.pinops->config)(dev.gpio_led);
	(*dev.pinops->config)(dev.gpio_pon);
	(*dev.pinops->config)(dev.gpio_data);

	/* Set default output levels (receiver and LED off by default) */
	dcf1_enable(false);

	/* Attach the interrupt to the driver */

	if (dev.lower->attach(dev.lower, dcf1_interrupt))
	{
		return;
	}

	/* Enable the receiver module (enables the interrupt as well) */
	dcf1_enable(true);

	/* Finally register the driver */
	(void)register_driver(DCF1_DEVFILE, &dcf1_ops, 0444, NULL);

	/* Display the configuration of the measurement thresholds */
	dcf1_thresholds_show(&dev.thr);
}
