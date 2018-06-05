#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include "../tonegen_private.h"

int main()
{
	uint32_t msg;
	int number = 123;
	int velocity = 250;
	int delay = 4090;
	{
		msg = MSG_CONTACT_ON(number, velocity, delay);
		unsigned int newM = MSG_GET_MSG(msg);
		unsigned int newN = MSG_GET_NUMBER(msg);
		unsigned int newV = MSG_GET_VELOCITY(msg);
		unsigned int newD = MSG_GET_DELAY(msg);

		printf("DELAYMASK %x\n", MSG_DELAY_MASK);
		printf("%x\n", msg);
		printf("%s, number: %d (%d), velocity: %d (%d), delay: %d (%d)\n",
			newM == MSG_MCONTACTON ? "MSG_MCONTACTON" : "ERROR",
			number, newN,
			velocity, newV,
			delay, newD
		      );
		assert(newM == MSG_MCONTACTON);
		assert(number == newN);
		assert(velocity == newV);
		assert(delay == newD);
	}
	{
		msg = MSG_CONTACT_OFF(number, velocity, delay);
		unsigned int newM = MSG_GET_MSG(msg);
		unsigned int newN = MSG_GET_NUMBER(msg);
		unsigned int newV = MSG_GET_VELOCITY(msg);
		unsigned int newD = MSG_GET_DELAY(msg);

		printf("%s, number: %d (%d), velocity: %d (%d), delay: %d (%d)\n",
			newM == MSG_MCONTACTOFF ? "MSG_MCONTACTOFF" : "ERROR",
			number, newN,
			velocity, newV,
			delay, newD
		      );
		assert(newM == MSG_MCONTACTOFF);
		assert(number == newN);
		assert(velocity == newV);
		assert(delay == newD);
	}

	unsigned int param = 1245;
	{
		msg = MSG_KEY_ON(param);
		unsigned int newM = MSG_GET_MSG(msg);
		unsigned int newP = MSG_GET_PRM(msg);
		printf("%s, param: %d (%d)\n",
			newM == MSG_MKEYON ? "MSG_MKEYON" : "ERROR",
			number, newP);
		assert(newM == MSG_MKEYON);
		assert(param == newP);
	}{
		msg = MSG_KEY_OFF(param);
		unsigned int newM = MSG_GET_MSG(msg);
		unsigned int newP = MSG_GET_PRM(msg);
		printf("%s, param: %d (%d)\n",
			newM == MSG_MKEYOFF ? "MSG_MKEYOFF" : "ERROR",
			number, newP);
		assert(newM == MSG_MKEYOFF);
		assert(param == newP);
	}
	return 0;
}
