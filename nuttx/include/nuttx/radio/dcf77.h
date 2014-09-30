/* DCF77 message structure */

#pragma once

struct dcf77msg
{
  /*--- page 1 (0:19) ---*/

  uint64_t start        : 1; /* 0: start of minute */

  /* civil warning bits (1:14) */

  uint64_t              : 1;  /* 1 */
  uint64_t              : 1;  /* 2 */
  uint64_t              : 1;  /* 3 */
  uint64_t              : 1;  /* 4 */
  uint64_t              : 1;  /* 5 */
  uint64_t              : 1;  /* 6 */
  uint64_t              : 1;  /* 7 */
  uint64_t              : 1;  /* 8 */
  uint64_t              : 1;  /* 9 */
  uint64_t              : 1;  /* 10 */
  uint64_t              : 1;  /* 11 */
  uint64_t              : 1;  /* 12 */
  uint64_t              : 1;  /* 13 */
  uint64_t              : 1;  /* 14 */

  /* call bit/summer time/CEST/CET/leap second 15:19 */

  uint64_t              : 1;  /* 15 */
  uint64_t              : 1;  /* 16 */
  uint64_t              : 1;  /* 17 */
  uint64_t              : 1;  /* 18 */
  uint64_t              : 1;  /* 19 */

  /*--- page 2 (20:39) ---*/

  uint64_t start_time   : 1;  /* 20 - start of encoded time (always 1) */

  /* minutes + parity bit (21:28) */

  uint64_t m1           : 1;  /* 21 */
  uint64_t m2           : 1;  /* 22 */
  uint64_t m4           : 1;  /* 23 */
  uint64_t m8           : 1;  /* 24 */
  uint64_t m10          : 1;  /* 25 */
  uint64_t m20          : 1;  /* 26 */
  uint64_t m40          : 1;  /* 27 */
  uint64_t m_parity     : 1;  /* 28 - parity over minute bits 21-28 */

  /* hours + parity bit (29:35) */

  uint64_t h1           : 1;  /* 29 */
  uint64_t h2           : 1;  /* 30 */
  uint64_t h4           : 1;  /* 31 */
  uint64_t h8           : 1;  /* 32 */
  uint64_t h10          : 1;  /* 33 */
  uint64_t h20          : 1;  /* 34 */
  uint64_t              : 1;  /* 35 - parity over hour bits 29-35 */

  /* day of month (36:39) */

  uint64_t dm1          : 1; /* 36 */
  uint64_t dm2          : 1; /* 37 */
  uint64_t dm4          : 1; /* 38 */
  uint64_t dm8          : 1; /* 39 */

  /*--- page 3 (40:59) ---*/

  /* day of month (40:41) */

  uint64_t dm10         : 1; /* 40 */
  uint64_t dm20         : 1; /* 42 */

  /* day of week (42:44) */

  uint64_t dw1          : 1; /* 42 */
  uint64_t dw2          : 1; /* 43 */
  uint64_t dw4          : 1; /* 44 */

  /* month number (0-99) (45:49) */

  uint64_t mn1          : 1; /* 45 */
  uint64_t mn2          : 1; /* 46 */
  uint64_t mn4          : 1; /* 47 */
  uint64_t mn8          : 1; /* 48 */
  uint64_t mn10         : 1; /* 49 */

  /* year within century (50:57) */

  uint64_t y1           : 1; /* 50 */
  uint64_t y2           : 1; /* 51 */
  uint64_t y4           : 1; /* 52 */
  uint64_t y8           : 1; /* 53 */
  uint64_t y10          : 1; /* 54 */
  uint64_t y20          : 1; /* 55 */
  uint64_t y40          : 1; /* 56 */
  uint64_t y80          : 1; /* 57 */
  uint64_t              : 1; /* 58 - parity over bits 36-58 */
  uint64_t end          : 1; /* 59 end of minute mark */

  /* END OF DCF77 message */

  uint64_t pad          : 4; /* 60:64 */
};
