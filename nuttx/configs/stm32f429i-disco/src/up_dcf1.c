#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/radio/dcf1.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct dcf1_lower_s lower;   /* Low-level MCU interface */
  xcpt_t                    handler; /* DCF1 interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(FAR const struct dcf1_lower_s *lower, xcpt_t handler);
static void up_enable(FAR const struct dcf1_lower_s *lower);
static void up_disable(FAR const struct dcf1_lower_s *lower);

/* GPIO Abstraction */

static int configgpio(uint32_t cfgset);
static xcpt_t gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                           bool event, xcpt_t func);
static bool gpioread(uint32_t pinset);
static void gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_lower_s g_dcf1lower =
{
  .lower =
  {
    .attach  = up_attach,
    .enable  = up_enable,
    .disable = up_disable,
  },
  .handler = NULL,
};

static struct stm32_gpio_s g_dcf1gpio =
{
  .config    = configgpio,
  .setevent  = gpiosetevent,
  .read      = gpioread,
  .write     = gpiowrite,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name struct dcf1_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct dcf1_lower_s *lower, xcpt_t handler)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void up_enable(FAR const struct dcf1_lower_s *lower)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  (void)stm32_gpiosetevent(GPIO_DCF1_DATA, true, true, false, priv->handler);
}

static void up_disable(FAR const struct dcf1_lower_s *lower)
{
  (void)stm32_gpiosetevent(GPIO_DCF1_DATA, false, false, false, NULL);
}

/****************************************************************************
 * Name struct dcf1_gpio_s methods
 ****************************************************************************/

static int configgpio(uint32_t cfgset)
{
	return stm32_configgpio(cfgset);
}

static xcpt_t gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                           bool event, xcpt_t func)
{
	return stm32_gpiosetevent(pinset, risingedge, fallingedge, event, func);
}

static bool gpioread(uint32_t pinset)
{
	return stm32_gpioread(pinset);
}

static void gpiowrite(uint32_t pinset, bool value)
{
	stm32_gpiowrite(pinset, value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dcf1initialize
 ****************************************************************************/

void up_dcf1initialize(void)
{
  dcf1_init();
}
