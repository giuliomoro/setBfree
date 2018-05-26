#include "tonegen.h"
#include "tonegen_private.h"
//#define OFFLINE

static void postCallback(void* arg, float* buffer, unsigned int length)
{
  struct b_tonegen* t = (struct b_tonegen*)arg;
  int mute = 0;
  {
	Keys* keys = t->keys;
	float* pos = t->pos;
	float* oldPos = t->oldPos[t->oldPosCurr];
	float* oldOldPos = t->oldPos[(t->oldPosCurr + 1) % NUM_OLD_POS];
	float** contactClosingDistance = t->contactClosingDistance;
	int notZero = 0;
    for(int n = 0; n < TOTAL_SCANNER_KEYS; ++n){
      pos[n] = Keys_getNoteValue(keys, n + 24);
      if(pos[n] != 0)
        ++notZero;
    }
    if(!notZero)
    {
        // no keyboard connected
        for(int n = 0; n < TOTAL_SCANNER_KEYS; ++n){
            pos[n] = 1;
        }
    }
    for(int n = 10; n < FIRST_SOUNDING_KEY; ++n){
	  // if using one of the presets,
	  // do not generate output
      if(pos[n] < 0.5)
        mute = 1;
    }
	if(t->elapsedSamples){
      for(int n = FIRST_SOUNDING_KEY; n < TOTAL_SCANNER_KEYS; ++n){
        for(int bus = 0; bus < NOF_DRAWBARS_PER_MANUAL ; ++bus)
        {
          int playingKey = n - FIRST_SOUNDING_KEY;
          float threshold = contactClosingDistance[playingKey][bus];
          if(pos[n] <= threshold && oldPos[n] > threshold)
          { // contact was inactive, we need to turn it on
            //rt_printf("making contact for key %d\n", n);
            int contact = make_contact(bus, playingKey);
            float rawVelocity = -(pos[n] - oldOldPos[n]);
            short velocity = (rawVelocity) * 170;
            oscContactOn(t, contact, velocity);
          }
          else if(pos[n] > threshold && oldPos[n] <= threshold)
          { // contact was active, we need to turn it off
            //rt_printf("breaking contact for key %d\n", n);
            int contact = make_contact(bus, playingKey);
            float rawVelocity = -(pos[n] - oldOldPos[n]);
            short velocity = (rawVelocity) * 170;
            oscContactOff(t, contact, velocity);
          }
        }
	  }
      ++t->oldPosCurr;
      if(t->oldPosCurr == NUM_OLD_POS)
      	t->oldPosCurr = 0;
      // remember current position
      for(int n = 0; n < TOTAL_SCANNER_KEYS; ++n){
        t->oldPos[t->oldPosCurr][n] = pos[n];
      }
      t->mute = mute;
    }

	static int count = 0;
	if(0)
	if(count % 300 == 0)
	{
		rt_printf("%s ", mute ? "m" : "_");
		rt_printf("%s ", notZero ? "" : "NO_SCANNER_CONNECTED");
		for(int n = 10; n < TOTAL_SCANNER_KEYS; ++n)
			rt_printf("%2d ", (int)(pos[n]*10));
		rt_printf("\n");
	}
	count++;
  }
}

void startKeyboardScanning(struct b_tonegen *t){
  if(t->bt)
    BoardsTopology_delete(t->bt);
  if(t->keys)
    Keys_delete(t->keys);
  t->bt = BoardsTopology_new();
  t->keys = Keys_new();
  BoardsTopology_setLowestNote(t->bt, 24);
  BoardsTopology_setBoard(t->bt, 0, 0, 24);
  BoardsTopology_setBoard(t->bt, 1, 0, 23);
  BoardsTopology_setBoard(t->bt, 2, 0, 23);
#ifndef OFFLINE
  int ret = Keys_start(t->keys, t->bt, NULL);
  if(ret < 0)
  {
    printf("Error starting keys: %d\n", ret);
    exit(1);
  }
  Keys_startTopCalibration(t->keys);
  Keys_loadInverseSquareCalibrationFile(t->keys, "/root/out.calib", 24);
  Keys_setPostCallback(t->keys, postCallback, t);
  { 
	  WriteFile* file = WriteFile_new();
	  WriteFile_init(file, "audio_log.bin", false);
	  WriteFile_setFileType(file, kBinary);
	  t->audioLogFile = file;
  }
#endif /* OFFLINE */
}

void autoplay(struct b_tonegen *t)
{
	static int count = 0;
#define numContacts 25
	static int contacts[numContacts] = {0};
	if(contacts[0] == 0)
	{
		for(int n = 0; n < numContacts; ++n)
			contacts[n] = n + 20;
	}

	int period = 2;
	if(count >= period - 1){
		for(int n = 0; n < numContacts; ++n)
			oscContactOff(t, contacts[n], 0);
	}
	if(count == 0){
		static int state  = 0;
		for(int n = 0; n < numContacts; ++n)
			oscContactOn(t, contacts[n], 1 + (126 * state));
		state = !state;
	}
	++count;
	if(count == period)
		count = 0;
}
