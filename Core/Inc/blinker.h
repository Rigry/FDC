#pragma once

class Blinker : TickSubscriber
{
	Pin& led;
	uint16_t time{0};
	uint16_t time_blink{200};
	uint16_t time_pause{1000};
	uint8_t qty{0};
	uint8_t qty_blink{0};
	bool pause{false};

public:

	Blinker (Pin& led) : led{led}
	{
		subscribed = false;
	}

	void start_qty(uint8_t v) {
		qty_blink = v;
		if (not subscribed)
			subscribe();
	}

	void stop() {
		qty = 0;
		time = 0;
		pause = false;
		led = false;
		if(subscribed)
			unsubscribe();
	}

	void notify() {
		if(time++ >= 350 and not pause) {
			time = 0;
			led ^= true;
			if(qty++ >= qty_blink * 2) {
				pause = true;
				led = false;
			}
		}

		if( not (time % 1500) and pause and time) {
			pause = false;
			time = 0;
			qty = 0;
		}


	}
};
