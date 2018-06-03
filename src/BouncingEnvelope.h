#ifndef BOUNCINGENVELOPE_H
#define BOUNCINGENVELOPE_H

#define BE_FILTER_SIZE 3
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
	float pastIn[BE_FILTER_SIZE];
	float pastOut[BE_FILTER_SIZE];
} BouncingEnvelope;


/**
 * Initialize a new envelope
 * @param length length of the buffer
 */
void BouncingEnvelope_init(BouncingEnvelope* be, short velocity, unsigned int delay);
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
