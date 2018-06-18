#ifndef BOUNCINGENVELOPE_H
#define BOUNCINGENVELOPE_H

#define BE_FILTER_SIZE 3
typedef struct _bouncingEnvelope{
	unsigned int p;
	float e; /* restitution coefficient */
	float lowThreshold;
	float highThreshold;
	float amplitude;
	float contactVelocity;
	float phase;
	float phaseStep;
	float contactState;
	int delay;
	int remaining;
	float pastIn[BE_FILTER_SIZE];
	float pastOut[BE_FILTER_SIZE];
	float pastContactState;
	float rampValue;
	float rampStep;
	float scale;
	int elapsed;
	unsigned int rampTime;
} BouncingEnvelope;


/**
 * Initialize a new envelope
 * @param be the object
 * @param velocity the velocity of the key at onset
 * @param delay how many samples to wait before starting generating bounces
 * @param scale envelope scaling width
 * @param rampTime how many samples to spend ramping up to get from 0..scale to 1-scale..scale
 */
void BouncingEnvelope_init(BouncingEnvelope* be, short velocity, unsigned int delay, float scale, unsigned int rampTime);
/**
 * Compute a step.
 * @param length the length of the step to compute. Has to be a multiple of MIN_CONTACT_PERIOD
 * @param buffer the output buffer
 * @return 1 if the envelope has produced a useful output in buffer and is not complete,
 * 0 in case it produced a useful output uin buffer and it is completed,
 * -1 if the delay has not expired yet, and the content of buffer has not been changed
 */
int BouncingEnvelope_step(BouncingEnvelope* be, unsigned int length, float* buffer);

#endif /* BOUNCINGENVELOPE_H */
