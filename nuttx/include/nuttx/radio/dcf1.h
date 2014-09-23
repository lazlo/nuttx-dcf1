#ifndef __INCLUDE_NUTTX_RADIO_DCF1_H
#define __INCLUDE_NUTTX_RADIO_DCF1_H

#include <nuttx/irq.h>

#include <stdbool.h>
#include <stdint.h>

struct dcf1_lower_s
{
  int  (*attach)(FAR const struct dcf1_lower_s *lower, xcpt_t handler);
  void (*enable)(FAR const struct dcf1_lower_s *lower);
  void (*disable)(FAR const struct dcf1_lower_s *lower);
};

struct dcf1_gpio_s
{
  int (*config)(uint32_t cfgset);
  xcpt_t (*setevent)(uint32_t pinset, bool risingedge, bool fallingedge,
                     bool event, xcpt_t func);
  bool (*read)(uint32_t pinset);
  void (*write)(uint32_t pinset, bool value);
};

/* Initialization */

void	dcf1_init(struct dcf1_gpio_s *pinops, uint32_t datapin,
                                              uint32_t ponpin,
                                              uint32_t ledpin);

#endif /* __INCLUDE_NUTTX_RADIO_DCF1_H */
