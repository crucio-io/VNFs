/*
  Copyright(c) 2010-2014 Intel Corporation.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <curses.h>
#include "display_utils.h"

#ifdef BRAS_STATS

void poll_key(struct key_val *key_val)
{
	int car = getch();

	switch (car) {
	case ERR:
		key_val->type = TYPE_NO_KEY;
		return;
	case '\n':
		key_val->type = TYPE_SPECIAL_KEY;
		key_val->val = KEY_ENTER;
		return;
	case 0x7F:
		key_val->type = TYPE_SPECIAL_KEY;
		key_val->val = KEY_BACKSPACE;
		return;
	case KEY_RESIZE:
		key_val->type = TYPE_SPECIAL_KEY;
		key_val->val = KEY_RESIZE;
		return;
	case 0x1B:
		switch (car = getch()) {
		case ERR:
			key_val->type = TYPE_SPECIAL_KEY;
			key_val->val = KEY_ESC;
			return;
		case 'O':
			key_val->type = TYPE_SPECIAL_KEY;
			switch (car = getch()) {
			case 'P':
				key_val->val = KEY_F(1);
				return;
			case 'Q':
				key_val->val = KEY_F(2);
				return;
			case 'R':
				key_val->val = KEY_F(3);
				return;
			case 'S':
				key_val->val = KEY_F(4);
				return;
			default:
				key_val->type = TYPE_NOT_SUPPORTED;
				return;
			}
			return;
		case '[':
			switch (car = getch()) {
			case 'A':
				key_val->type = TYPE_SPECIAL_KEY;
				key_val->val = KEY_UP;
				return;
			case 'B':
				key_val->type = TYPE_SPECIAL_KEY;
				key_val->val = KEY_DOWN;
				return;
			case '1':
				car = getch() - '0';
				getch(); // read the last tilde character
				key_val->type = TYPE_SPECIAL_KEY;
				switch (car) {

				case 1:
					key_val->val = KEY_F(1);
					return;
				case 2:
					key_val->val = KEY_F(2);
					return;
				};
				return;
			case '2':
				car = getch() - '0';
				getch(); // read the last tilde character
				key_val->type = TYPE_SPECIAL_KEY;
				switch (car) {
				case 0:
					key_val->val = KEY_F(9);
					return;
				case 1:
					key_val->val = KEY_F(10);
					return;
				case 3:
					key_val->val = KEY_F(11);
					return;
				case 4:
					key_val->val = KEY_F(12);
					return;
				};
				return;
			case '5':
				getch();
				key_val->type = TYPE_SPECIAL_KEY;
				key_val->val = KEY_PPAGE;
				return;
			case '6':
				getch();
				key_val->type = TYPE_SPECIAL_KEY;
				key_val->val = KEY_NPAGE;
				return;
			default:
				key_val->type = TYPE_NOT_SUPPORTED;
				return;
			}
			return;
		}
	default:
		key_val->type = TYPE_NORMAL_KEY;
		key_val->val = car;
	}
}

#else

void poll_key(__attribute__((unused)) struct key_val *key_val) {}

#endif
