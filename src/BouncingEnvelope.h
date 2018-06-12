#ifndef BOUNCINGENVELOPE_H
#define BOUNCINGENVELOPE_H

typedef struct _bouncingEnvelope{
	unsigned int p;
	float e; /* restitution coefficient */
	float lowThreshold;
	float highThreshold;
	float contactPosition;
	float amplitude;
	float contactVelocity;
	float phase;
	float phaseStep;
	float contactState;
	unsigned int delay;
	int remaining;
} BouncingEnvelope;

/**
 * Initialize a new envelope
 * @param length length of the buffer
 */
void BouncingEnvelope_init(BouncingEnvelope* be, short velocity, unsigned int delay);
/**
 * Compute a step.
 * @param length the length of the step to compute.
 * @param buffer the output buffer
 * @return the number of samples left to be retrieved, 0 in case there are no
 * samples left, or -1 if the delay has not expired yet
 */
int BouncingEnvelope_step(BouncingEnvelope* be, unsigned int length, float* buffer);

#endif /* BOUNCINGENVELOPE_H */
