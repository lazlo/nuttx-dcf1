/****************************************************************************
 * examples/dcf1/dcf1_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/radio/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dcf1_main(int argc, char *argv[])
#endif
{
  bool onoff;
  int fd;
  int rv;

  /* Check arguments */

  if (argc != 2)
    {
      /* FIXME argv[0] contains "<noname>\n" but should contain the program name! */
      printf("Usage: %s on|off\n", "dcf1");
      return 1;
    }

  /* Parse arguments */

  if (strncmp(argv[1], "on", 2) == 0)
    {
      onoff = true;
    }
  else if (strncmp(argv[1], "off", 3) == 0)
    {
      onoff = false;
    }
  else
    {
      return 1;
    }

  /* Perform IO control */

  fd = open("/dev/dcf1", O_RDONLY);
  if (fd == -1)
    {
      printf("ERROR open %d\n", errno);
      return -1;
    }
  rv = ioctl(fd, RFIOCS_ONOFF, (unsigned long)onoff);
  if (rv == -1)
    {
      printf("ERROR ioctl %d\n", errno);
      close(fd);
      return -1;
    }
  rv = close(fd);
  if (rv == -1)
    {
      printf("ERROR close %d\n", errno);
    }

  return rv;
}
