//      Board.h
//      PIN control on Arduino board.
//
//      Copyright (C) 2011 Sam (Yujia Zhai) <yujia.zhai@usc.edu>
//      Aerial Robotics Team, USC Robotics Society - http://www.uscrs.org - http://uscrs.googlecode.com
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, please refer to the online version on
//      http://www.gnu.org/licenses/gpl.html, or write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

#ifndef _BOARD_
#define _BOARD_

enum PINID {red = 35, yellow = 36, green = 37, switch1 = 41, switch2 = 40};

class Board
{
  public:
  
	Board() {}
	
	void init()
	{
		pinMode(red, OUTPUT);
		pinMode(yellow, OUTPUT);
		pinMode(green, OUTPUT);
		pinMode(switch1, INPUT);
		pinMode(switch2, INPUT);  
	}

	void ledon(PINID _id) { digitalWrite(_id, HIGH); }
	void ledoff(PINID _id) { digitalWrite(_id, LOW); }
};

#endif
