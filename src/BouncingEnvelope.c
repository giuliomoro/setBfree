#include <BouncingEnvelope.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>

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

enum
{
	open = 0,
	closed = 1,
};

// triangular oscillator (shaped like a cos, but different phase range). Input range: -1:1, output range: -1:1 (input 0, gives 1)
static float tri(float phase)
{
	if(phase < 0)
		return 1.f + 2.f * phase;

	else
		return 1.f - phase * 2.f;
}

void BouncingEnvelope_init(BouncingEnvelope* be, short velocity, unsigned int delay)
{	
	if(delay > 1024)
		be->remaining = delay;
	else
		be->remaining = 1024;
	be->delay = delay;
	be->amplitude = velocity / 127.f;
	if(be->amplitude < 0)
		be->amplitude = 0;
	be->contactPosition = be->amplitude;
	be->highThreshold = 0.2;
	be->lowThreshold = 0.015;
	be->phase = 0;
	be->phaseStep = 1302.f / 44100; // bounce frequency / samplerate
	be->e = 0.7;
	be->contactState = open;
}

int BouncingEnvelope_step(BouncingEnvelope* be, unsigned int length, float* buffer)
{
	//rt_printf("_step %p \n", be);
	be->remaining -= length;
	int start;
	if(be->delay >= length) {
		start = length;
		be->delay -= length;
	} else {
		start = be->delay;
		be->delay = 0;
		//rt_printf("delay %d then we go\n", be->delay);
	}
	// if there is a delay still going, we fill
	// the head of env with 0
	memset(buffer, 0, sizeof(buffer[0]) * start);
	if(start == length)
	{
		//rt_printf("Returning, still %d to go\n", be->delay);
		// no bounce to be generated,
		// let's return early to avoid useless
		// copy operations
		return -1;
	}
	float amplitude = be->amplitude;
	float phase = be->phase;
	float contactState = be->contactState;
	const float phaseStep = be->phaseStep;
	const float e = be->e;
	float contactPosition = 0;
	for(unsigned int n = start; n < length; ++n)
	{
		contactPosition = amplitude * tri(phase);
		//printf("contactPosition: %.4f phase: %.4f\n", contactPosition, phase);
		if (phase > 0.5) // crossing zero
		{
			// changed direction, start a new oscillation
			// with updated conditions
			phase = -0.5;
			amplitude = amplitude * e;
		}
		phase = phase + phaseStep;

		if(contactPosition < be->lowThreshold){
			// definitely closed
			contactState = closed;
		}
		else if (contactPosition > be->highThreshold){
			// definitely open
			contactState = open;
		}
		else
	       	{
			// bouncing fast
			static unsigned int ran = 1;
			ran = xorshift32(&ran);
			contactState = ran > RAND_MAX / 2;
		}
		buffer[n] = contactState;
	}
	be->amplitude = amplitude;
	be->contactPosition = contactPosition;
	be->phase = phase;
	be->contactState = contactState;
	return be->remaining > 0 ? be->remaining : 0;
}

