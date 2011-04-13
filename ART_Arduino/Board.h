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
