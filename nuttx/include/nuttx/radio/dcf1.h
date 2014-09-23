#pragma once

#include <nuttx/irq.h>

struct dcf1_lower_s
{
  int  (*attach)(FAR const struct dcf1_lower_s *lower, xcpt_t handler);
  void (*enable)(FAR const struct dcf1_lower_s *lower);
  void (*disable)(FAR const struct dcf1_lower_s *lower);
};

/* Initialization */

void	dcf1_init(void);
