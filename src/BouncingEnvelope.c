#include <BouncingEnvelope.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static float open = 0;
static float closed = 1;
void BouncingEnvelope_init(BouncingEnvelope* be, short velocity)
{	
	be->amplitude = 1;
	be->contactPosition = be->amplitude;
	be->highThreshold = 0.2;
	be->lowThreshold = 0.015;
	be->phase = 0;
	be->phaseStep = 0.0907;
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
	for(int n = 0; n < length; ++n)
	{
		contactPosition = amplitude * cosf(phase);
		if (contactPositionPrev > 0 && contactPosition < 0)
		{
			// changed direction, start a new oscillation
			// with updated conditions
			phase = -0.5 * M_PI - phaseStep;
			amplitude = amplitude* e;
		}
		contactPositionPrev = contactPosition;
		phase = phase + phaseStep;

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
		else {
			// probability that there is a transition 
			float prob = 0.1 + (0.4*sinceTrans);
			float ran = rand() / (float)RAND_MAX;
			short transition = ran < prob;
			//printf("ran: %f, prob: %f, sinceTran: %d, transtion: %d\n", ran, prob, sinceTrans, transition);
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
