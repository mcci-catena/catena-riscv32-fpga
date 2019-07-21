// simple C++ test program
#include "riscv_ino.h"
#include "log.h"
#include "debug.h"
#include "gpio.h"

class cTest
	{
public:
	cTest()
		{
		this->m_Now = millis();
		log_printfnl("cTest(): now=%lu", this->m_Now);
		};
	~cTest() {};

	uint32_t getWhen(void) const
		{ return this->m_Now; };

	void setNow(uint32_t t)
		{ this->m_Now = t; }

	bool setNeedNewline(bool v)
		{
		const bool oldv = this->m_fNeedNewline;
		this->m_fNeedNewline = v;
		return oldv;
		}

	bool getNeedNewline(void) const
		{ return this->m_fNeedNewline; }

private:
	uint32_t m_Now;
	bool m_fNeedNewline;
	};

cTest Test;

void setup(void)
	{
	gpio_write(1);
	log_printfnl("in setup()");
	Test.setNow(millis());
	}

void loop(void)
	{
	uint32_t when = Test.getWhen();
	uint32_t now = millis();
	uint32_t tick = (now >> 9) & 1;	// 1.024 second period
	uint8_t b;

	b = gpio_read();
	b = (b & ~1u) | tick;
	gpio_write(b);

	while (debug_checkbyte())
		{
		char c[2];

		c[0] = (char) debug_getbyte();
		c[1] = 0;

		Test.setNeedNewline(true);
		log_puts(c);
		}

	if ((now - when) < 2000)
		return;

	Test.setNow(now);

	if (Test.setNeedNewline(false))
		log_puts("\n");

	log_printfnl("in loop: now=%lu", now);
	}