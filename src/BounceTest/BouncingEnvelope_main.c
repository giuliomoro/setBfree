#include <BouncingEnvelope.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#ifdef __COBALT__
#include <xenomai/init.h>
#endif

void 
enable_runfast()
{
#ifdef __arm__
	static const unsigned int x = 0x04086060;
	static const unsigned int y = 0x03000000;
	int r;
	asm volatile (
		"fmrx	%0, fpscr			\n\t"	//r0 = FPSCR
		"and	%0, %0, %1			\n\t"	//r0 = r0 & 0x04086060
		"orr	%0, %0, %2			\n\t"	//r0 = r0 | 0x03000000
		"fmxr	fpscr, %0			\n\t"	//FPSCR = r0
		: "=r"(r)
		: "r"(x), "r"(y)
	);
#endif
}

int main(int argc, char* const* argv)
{
#ifdef __COBALT__
	xenomai_init(&argc, &argv);
	struct sched_param params;
	params.sched_priority = 41;
	__wrap_sched_setscheduler(getpid(), SCHED_FIFO, &params);
#endif
	enable_runfast();
	BouncingEnvelope be;
	short velocity = 100;
	const unsigned int length = 64;
	const double sampleRate = 44100; // only to provide CPU stats
	float buffer[length];
	int numEnv = 300000;
	int numBlocks = 1;
	struct timespec start;
	struct timespec stop;
	for(int n = 1; n < argc; ++n)
	{
		switch (n)
		{
		case 1:
			numEnv = atoi(argv[n]);
			break;
		case 2:
			numBlocks = atoi(argv[n]);
			break;
		default:
			fprintf(stderr, "Error: too many arguments supplied\n");
		}
	}
	clock_gettime(CLOCK_REALTIME, &start);
	for(int e = 0; e < numEnv; ++e)
	{
		BouncingEnvelope_init(&be, velocity);
		for(int n = 0; n < numBlocks; ++n)
		{
			BouncingEnvelope_step(&be, length, buffer);
		}
	}
	clock_gettime(CLOCK_REALTIME, &stop);
	double startTime = start.tv_sec + start.tv_nsec/1000000000.0;
	double stopTime =    stop.tv_sec + stop.tv_nsec/1000000000.0;
	double duration = stopTime - startTime;
	double sampleAverage = duration / (numEnv * numBlocks * length);
	printf("it took %10.5fs to do %d envelopes for %d blocks (%10.5fus sampleAverage, %.5f%%)\n", duration, numEnv, numBlocks, sampleAverage * 1000000, 100 * sampleAverage / (1/sampleRate));
	return 0;
}

