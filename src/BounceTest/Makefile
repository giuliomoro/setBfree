CC=clang
bounce: ../BouncingEnvelope.c ../BouncingEnvelope.h BouncingEnvelope_main.c
	$(CC) -g `/usr/xenomai/bin/xeno-config --no-auto-init --skin=posix --ldflags --cflags` ../BouncingEnvelope.c BouncingEnvelope_main.c -O3 -ftree-vectorize -mfpu=neon -I.. -o bounce -save-temps -ffast-math
