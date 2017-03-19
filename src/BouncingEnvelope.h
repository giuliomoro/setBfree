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
	unsigned int samplesSinceLastTransition;
} BouncingEnvelope;

/**
 * Initialize a new envelope
 * @param length length of the buffer
 */
void BouncingEnvelope_init(BouncingEnvelope* be, short velocity);
/**
 * Compute a step.
 * @param length the length of the step to compute.
 * @param buffer the output buffer
 */
void BouncingEnvelope_step(BouncingEnvelope* be, unsigned int length, float* buffer/*, float* position*/);












#endif /* BOUNCINGENVELOPE_H */
