#include <BouncingEnvelope.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>

int gEnvelopeFilter = 0;
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

// inverse function of triangular oscillator
static float arctri(float value)
{
	float phase;
	if(value > 0)
	{
		// assume first quadrant
		// value = 1 - phase * 2;
		phase = (1 - value) / 2.f;
	} else {
		// assume fourth quadrant
		// value = 1.f + 2.f * phase;
		phase = (value - 1.f) / 2.f;
	}
	return phase;
}
// the actual length will be defined by the amplitude: once the amplitude
// is below the lowThreshold, then we are done.
// So, there is no real good reason for having a maxEnvLength, except to avoid
// troubles in case  you set the lowThreshold too low
#define maxEnvLength 8192 // max length of the envelope (after delay)
#define contactBounceFrequency 1302.f
#define MIN_CONTACT_PERIOD 2 // has to be even

void BouncingEnvelope_init(BouncingEnvelope* be, short velocity, unsigned int delay, float scale, unsigned int rampTime)
{	
	if(delay > maxEnvLength)
		be->remaining = delay;
	else
		be->remaining = maxEnvLength;
	be->delay = delay;
	be->amplitude = velocity / 127.f;
	if(be->amplitude < 0)
		be->amplitude = 0;
	be->highThreshold = 0.05;
	be->lowThreshold = 0.015;
	be->phase = arctri(be->highThreshold); // initialize so that bouncing starts immediately ( after delay!)
	be->phaseStep = contactBounceFrequency / 44100.f; // bounce frequency / samplerate
	be->e = 0.5;
	be->contactState = open;
	be->pastContactState = open;
	be->rampValue = 0;
	be->elapsed = 0;
	//be->scale = 0.3;
	be->scale = scale;
	//be->rampTime could be estimated automatically from amplitude and e
	be->rampTime = rampTime;
	float rampTarget = 1.f - be->scale;
	be->rampStep = rampTarget / be->rampTime;
	//rt_printf("scale: %f, step: %f, rampTime: %d\n", be->scale, be->rampStep, be->rampTime);
	memset(be->pastIn, BE_FILTER_SIZE * sizeof(float), 0);
	memset(be->pastOut, BE_FILTER_SIZE * sizeof(float), 0);
}

float Bn[][3] = {
	{1.0000000000, 0.0000000000, 0.0000000000}, // [B, A] = butter(2, 1.0000)
	{0.2928932188, 0.5857864376, 0.2928932188}, // [B, A] = butter(2, 0.5000)
	{0.0200833656, 0.0401667311, 0.0200833656}, // [B, A] = butter(2, 0.1000)
	{0.0055427172, 0.0110854344, 0.0055427172}, // [B, A] = butter(2, 0.0500)
	{0.0002413590, 0.0004827181, 0.0002413590}, // [B, A] = butter(2, 0.0100)
	{0.0000610062, 0.0001220124, 0.0000610062}, // [B, A] = butter(2, 0.0050)
	{0.0000024619, 0.0000049239, 0.0000024619}, // [B, A] = butter(2, 0.0010)
};
float An[][3] = {
	{1.0000000000, 0.0000000000, 0.0000000000}, // [B, A] = butter(2, 1.0000)
	{1.0000000000, -0.0000000000, 0.1715728753}, // [B, A] = butter(2, 0.5000)
	{1.0000000000, -1.5610180758, 0.6413515381}, // [B, A] = butter(2, 0.1000)
	{1.0000000000, -1.7786317778, 0.8008026467}, // [B, A] = butter(2, 0.0500)
	{1.0000000000, -1.9555782403, 0.9565436765}, // [B, A] = butter(2, 0.0100)
	{1.0000000000, -1.9777864838, 0.9780305085}, // [B, A] = butter(2, 0.0050)
	{1.0000000000, -1.9955571243, 0.9955669721}, // [B, A] = butter(2, 0.0010)
};


int BouncingEnvelope_step(BouncingEnvelope* be, unsigned int length, float* buffer)
{
	//rt_printf("_step %p \n", be);
	int start;
	if(be->delay >= length) {
		be->delay -= length;
		// no bounce to be generated, yet.
		// let's return early to avoid useless
		// copy operations
		return -1;
	} else {
		start = (be->delay & (MIN_CONTACT_PERIOD - 1)); // make sure start is a multiple of MIN_CONTACT_PERIOD
		be->delay = 0;
		be->remaining -= length; // only decrement after delay
		be->elapsed += length - start;
	}
	// if there is a delay still going, we fill
	// the head of env with 0
	memset(buffer, 0, sizeof(buffer[0]) * start);
	float amplitude = be->amplitude;
	float phase = be->phase;
	float contactState = be->contactState;
	const float phaseStep = be->phaseStep;
	const float e = be->e;
	float contactPosition = 0;
	for(unsigned int n = start; n < length; n += MIN_CONTACT_PERIOD)
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
		phase = phase + phaseStep * MIN_CONTACT_PERIOD;

		char bouncingFast = 0;
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
			//static unsigned int ran = 1;
			//ran = xorshift32(&ran);
			//contactState = ran > RAND_MAX / 2;
			bouncingFast = 1;
			contactState = !be->pastContactState;
		}
		float contactValue;
		if(bouncingFast)
			contactValue = contactState * be->scale + 1 - be->scale;
		else
			contactValue = contactState;
		for(unsigned int j = 0; j < MIN_CONTACT_PERIOD; ++j)
		{
			buffer[n + j] = contactValue;
		}
		be->pastContactState = contactState;
	}

	if(0)
	if(be->rampTime)
	for(unsigned int n = start; n < length; ++n)
	{
		buffer[n] = buffer[n] * be->scale + be->rampValue;
		be->rampValue += be->rampStep;
		if(be->rampValue > 1 - be->scale) // limit excursion by ramping up
			be->rampValue = 1 - be->scale;
	}
	else
	for(unsigned int n = start; n < length; ++n) // limite excursion, no ramp
	{
		buffer[n] = buffer[n] * be->scale + 1 - be->scale;
	}

	// apply low pass filter
	int idx = 0;//gEnvelopeFilter / 128.f * (sizeof(Bn) / (sizeof(float) * 3));
	float* B = Bn[idx];
	float* A = An[idx];
	for(unsigned int n = start; n < length; ++n)
	{
		float* pastIn = be->pastIn;
		float* pastOut = be->pastOut;
		float in = buffer[n];
		float out = in * B[0] + pastIn[0] * B[1] + pastIn[1] * B[2]
				- pastOut[0] * A[1] - pastOut[1] * A[2];
		pastIn[1] = pastIn[0];
		pastIn[0] = in;
		pastOut[1] = pastOut[0];
		pastOut[0] = out;
		buffer[n] = out;
	}
	be->amplitude = amplitude;
	be->phase = phase;
	be->contactState = contactState;
	if(be->elapsed > be->rampTime && ( be->amplitude < be->lowThreshold || be->remaining <= 0))
	{
		return 0; // no more bouncing will happen, let's call it quits
	} else {
		return 1; // more bouncing to be expected
	}
}

