#ifndef _GSM0710_H_
#define _GSM0710_H_

/*
 * gsm0710.h -- definitions needed by the gsm0710 protocol daemon. 
 *
 * Copyright (C) 2003 Tuukka Karvonen <tkarvone@iki.fi>
 * 
 * Version 1.0 October 2003
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

// for debugging
#ifdef DEBUG
#  define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

// basic mode flag for frame start and end
#define F_FLAG 0xF9

// bits: Poll/final, Command/Response, Extension
#define PF 16
#define CR 2
#define EA 1
// the types of the frames
#define SABM 47
#define UA 99
#define DM 15
#define DISC 67
#define UIH 239
#define UI 3
// the types of the control channel commands
#define C_CLD 193
#define C_TEST 33
#define C_MSC 225
#define C_NSC 17
// V.24 signals: flow control, ready to communicate, ring indicator, data valid
// three last ones are not supported by Siemens TC_3x
#define S_FC 2
#define S_RTC 4
#define S_RTR 8
#define S_IC 64
#define S_DV 128

#define COMMAND_IS(command, type) ((type & ~CR) == command)
#define PF_ISSET(frame) ((frame->control & PF) == PF)
#define FRAME_IS(type, frame) ((frame->control & ~PF) == type)

// Channel status tells if the DLC is open and what were the last
// v.24 signals sent
typedef struct Channel_Status {
  int opened;
  unsigned char v24_signals;
} Channel_Status;

// for debugging 
#define print_bits(n) printf("%d%d%d%d%d%d%d%d", ((n&128) == 128), \
			     ((n&64) == 64),((n&32) == 32),((n&16) == 16), \
			     ((n&8) == 8),((n&4) == 4),((n&2) == 2), \
			     ((n&1) == 1));

#endif /* _GSM0710_H_ */





