#include <BouncingEnvelope.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* The state word must be initialized to non-zero */
uint32_t xorshift32(uint32_t state[static 1])
{
	/* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
	uint32_t x = state[0];
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state[0] = x;
	return x;
}

static float open = 0;
static float closed = 1;

// triangular oscillator. Input range: 0:1, output range: 0:1
static float tri(float phase)
{
	if(phase > 0.5)
		return phase * 2.f;
	else
		return 0.5f - (phase - 0.5f);
}

void BouncingEnvelope_init(BouncingEnvelope* be, short velocity)
{	
	be->amplitude = 3.f * velocity/64.f;
	if(be->amplitude < 0)
		be->amplitude = 0;
	be->contactPosition = be->amplitude;
	be->highThreshold = 0.2;
	be->lowThreshold = 0.015;
	static unsigned int ran = 1;
	be->phase = xorshift32(&ran) / (float)UINT_MAX * 0.7f;
	be->phaseStep = 0.0907 / (2 * M_PI);
	be->e = 0.3;
	be->samplesSinceLastTransition = 0;
	be->contactState = closed;
}

void BouncingEnvelope_step(BouncingEnvelope* be, unsigned int length, float* buffer/*, float* position*/)
{
	float amplitude = be->amplitude;
	float contactPositionPrev = be->contactPosition;
	float phase = be->phase;
	unsigned int sinceTrans = be->samplesSinceLastTransition;
	float contactState= be->contactState;
	const float phaseStep = be->phaseStep;
	const float e = be->e;
	float contactPosition;
	if(1)
	for(int n = 0; n < length; ++n)
	{
		contactPosition = amplitude * tri(phase);
		if (contactPositionPrev > 0 && contactPosition < 0)
		{
			// changed direction, start a new oscillation
			// with updated conditions
			phase = 0;//-0.5f * (float)M_PI - phaseStep;
			amplitude = amplitude * e;
		}
		contactPositionPrev = contactPosition;
		phase = phase + phaseStep;
		continue;

		/*
		if(position)
			position[n] = contactPosition;
		*/
		if(contactPosition < be->lowThreshold){
			contactState = closed;
			sinceTrans = 0;
		}
		else if (contactPosition > be->highThreshold){
			contactState = open;
			sinceTrans = 0;
		}
		else
	       	{
			static int ran = 1;
			// probability that there is a transition 
			float prob = 0.1f + ( (float)RAND_MAX * 0.4f * sinceTrans);
			ran = xorshift32(ran);
			int transition = ran < prob;
			//printf("ran: %f, prob: %f, sinceTran: %d, transition: %d\n", ran, prob, sinceTrans, transition);
			if(transition){
				contactState = !contactState;
				sinceTrans = 0;
			} else {
				sinceTrans = sinceTrans + 1;
			}
		}
		buffer[n] = contactState;
	}
	be->amplitude = amplitude;
	be->contactPosition = contactPosition;
	be->phase = phase;
	be->samplesSinceLastTransition = sinceTrans;
	be->contactState = contactState;
}


/*
int main()
{
	BouncingEnvelope be;
	short velocity = 100;
	BouncingEnvelope_init(&be, velocity);
	const unsigned int length = 64;
	float buffer[length];
	float position[length];
	printf("out = [\n");
	for(int n = 0; n < 3; ++n)
	{
		BouncingEnvelope_step(&be, length, buffer, position);
		for(int n = 0; n < length; ++n)
		{
			printf("%.5f %.5f\n", buffer[n], position[n]);
			//printf("%.5f\n", position[n]);
		}
	}
	printf("];\n");
	return 0;
}
*/
