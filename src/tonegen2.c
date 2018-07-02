extern int gEnvelopeFilter;
#include "tonegen.h"
#include "tonegen_private.h"
#define NDEBUG
#include <assert.h>
#include <stdlib.h>
//#define OFFLINE
#define WRITEFILE
#ifdef OFFLINE
#undef WRITEFILE
#endif

static void postCallback(void* arg, float* buffer, unsigned int length)
{
	static unsigned int callbackCount = -1;
	++callbackCount;
	struct b_tonegen* t = (struct b_tonegen*)arg;
	if(t->spreadHandlerUpdating)
	{
		return;
	}
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
	float hyst = 0.001;
	if(t->elapsedSamples){
		for(int n = FIRST_SOUNDING_KEY; n < TOTAL_SCANNER_KEYS; ++n){
			for(int bus = 0; bus < NOF_DRAWBARS_PER_MANUAL ; ++bus) {
				int playingKey = n - FIRST_SOUNDING_KEY;
				int contact = make_contact(bus, playingKey);
				float threshold = contactClosingDistance[playingKey][bus];
				bool onset = false;
				bool offset = false;
				if(pos[n] <= threshold && t->activeContacts[contact] == 0)
				{ // contact was inactive, we need to turn it on
					onset = true;
				}
				else if(pos[n] > threshold + hyst && t->activeContacts[contact] > 0)
				{ // contact was active, we need to turn it off
					offset = true;
				}
				float rawVelocity = -(pos[n] - oldOldPos[n]);
				short velocity = (rawVelocity) * 210;

				if(t->contactSpread == kSpreadZeroNoV
					|| t->contactSpread == kSpreadZeroV)
				{
					// same triggering point for all busses:
					// KeyOn / KeyOff
					if(onset)
					{
						oscKeyOn(t, playingKey, velocity);
					}
					else if (offset)
						oscKeyOff(t, playingKey, velocity);
					break; // do not check other busses
				}
				// individual (per-contact) triggering points:
				// ContactOn / ContactOff
				if(onset)
					oscContactOn(t, contact, velocity, 0);
				else if (offset)
					oscContactOff(t, contact, velocity, 0);
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

	if(0)
	if(callbackCount % 300 == 0)
	{
		rt_printf("%s ", mute ? "m" : "_");
		rt_printf("%s ", notZero ? "" : "NO_SCANNER_CONNECTED");
		for(int n = 10; n < TOTAL_SCANNER_KEYS; ++n)
			rt_printf("%2d ", (int)(pos[n]*10));
		rt_printf("\n");
	}
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
#ifdef WRITEFILE
  {
	  WriteFile* file = WriteFile_new();
	  WriteFile_init(file, "audio_log.bin", false);
	  WriteFile_setFileType(file, kBinary);
	  t->audioLogFile = file;
  }
#endif /* WRITEFILE */
  Keys_setPostCallback(t->keys, postCallback, t);
#endif /* OFFLINE */
}

void autoplay(struct b_tonegen *t)
{
	static int count = 0;
#define numContacts 4
	static int contacts[numContacts] = {0};
	if(contacts[0] == 0)
	{
		for(int n = 0; n < numContacts; ++n)
			contacts[n] = n * 9+ 100;
	}

	int period = 2000;
	if(count == period / 2 ){
		for(int n = 0; n < numContacts; ++n)
			oscContactOff(t, contacts[n], 0, 0);
	}
	if(count == 0){
		static int vel = 1;
		for(int n = 0; n < numContacts; ++n)
		{
			unsigned short delay = 4000 / numContacts * n;
			oscContactOn(t, contacts[n], vel, delay);
			rt_printf("contact %d on with delay %d\n", contacts[n], delay);
		}
		vel += 20;
		if(vel > 127)
			vel = 1;
	}
	++count;
	if(count == period)
		count = 0;
}

/* ----------------------------------------------------------------
 * Tonegenerator version 3, 16-jul-2004
 * ----------------------------------------------------------------*/

/*
 * This routine is where the next buffer of output sound is assembled.
 * The routine goes through the following phases:
 *
 *   Process the message queue
 *   Process the activated list
 *   Process the removal list
 *   Execute the core program interpreter
 *   Mixdown
 *
 * The message queue holds the numbers of keys (MIDI playing keys)
 * that has been closed or released since the last time this function
 * was called. For each key, its contributions of how oscillators are
 * fed to buses (drawbar rails) is analysed. The changes that typically
 * occur are:
 *   (a) Oscillators that are not already sounding are activated.
 *   (b) Oscillators that are already sounding have their volumes altered.
 *   (c) Oscillators that are already sounding are deactivated.
 *
 * The list of active oscillators is processed and instructions for
 * the core interpreter are written. Each instruction refers to a single
 * oscillator and specifies how its samples should be written to the
 * three mixing buffers: swell, vibrato and percussion. Oscillators
 * that alter their volume as a result of key action picked up from the
 * message queue, are modulated by an envelope curve. Sometimes an extra
 * instruction is needed to manage a wrap of the oscillator's sample buffer.
 *
 * The removal list contains deactivated oscillators that are to be removed
 * from the list of active oscillators. This phase takes care of that.
 *
 * The core interpreter runs the core program which mixes the proper
 * number of samples from each active oscillator into the swell, vibrato
 * and percussion buffers, while applying envelope.
 *
 * The mixdown phase runs the vibrato buffer through the vibrato FX (when
 * activated), and then mixes the swell, vibrato output and percussion
 * buffers to the output buffer. The percussion buffer is enveloped by
 * the current percussion envelope and the whole mix is subject to the
 * swell pedal volume control.
 *
 * As a side note, the above sounds like a lot of work, but the most common
 * case is either complete silence (in which case the activated list is
 * empty) or no change to the activated list. Effort is only needed when
 * there are changes to be made, and human fingers are typically quite
 * slow. Sequencers, however, may put some strain on things.
 */
#define UNIQUE_ENVELOPE
void oscGenerateFragment (struct b_tonegen *t, float ** bufs, size_t lengthSamples) {
	/* auto play */
	//autoplay(t);
  // mix inputs to mono and store in the second channel
  if(bufs[1])
  {
    for(int n = 0; n < lengthSamples; ++n)
    {
      bufs[1][n] = (bufs[0][n] + bufs[1][n]) * 0.5f;
    }
  }
  float* buf = bufs[0]; // left channel is where the organ sound goes
  t->elapsedSamples += lengthSamples;
  int i;
  float * yptr = buf;
  struct _oscillator * osp;
  unsigned int copyDone = 0;
  unsigned int recomputeRouting;
  int removedEnd = 0;
  unsigned short * const removedList = t->removedList;
  float * const swlBuffer = t->swlBuffer;
  float * const vibBuffer = t->vibBuffer;
  float * const vibYBuffr = t->vibYBuffr;
  float * const prcBuffer = t->prcBuffer;

#ifdef KEYCOMPRESSION
#ifdef INDIVIDUAL_CONTACTS
  const float keyComp = t->keyCompTable[t->contactDownCount / (NOF_BUSES)];
#else
  const float keyComp = t->keyCompTable[t->keyDownCount];
#endif /* INDIVIDUAL_CONTACTS */
  const float keyCompDelta = (keyComp - t->keyCompLevel) / (float) BUFFER_SIZE_SAMPLES; 
#define KEYCOMPCHASE() {t->keyCompLevel += keyCompDelta;}
#define KEYCOMPLEVEL (t->keyCompLevel)

#else

#define KEYCOMPLEVEL (1.0)
#define KEYCOMPCHASE()

#endif /* KEYCOMPRESSION */

  /* End of declarations */

  /* Reset the core program */
  t->coreWriter = t->coreReader = t->corePgm;

  /* ================================================================
		     M E S S S A G E   Q U E U E
     ================================================================ */

  while (t->msgQueueReader != t->msgQueueWriter) {

    msg_queue_t msg = *t->msgQueueReader++; /* Read next message */
    int keyNumber;
    ListElement * lep;

    /* Check wrap on message queue */
    if (t->msgQueueReader == t->msgQueueEnd) {
      t->msgQueueReader = t->msgQueue;
    }

    if (MSG_GET_MSG(msg) == MSG_MCONTACTON) {
      int contactNumber = MSG_GET_NUMBER(msg);
      //rt_printf("contactNumber: %d. Wheels: ", contactNumber);
      short velocity = MSG_GET_VELOCITY(msg);
      short delay = MSG_GET_DELAY(msg);
      int busNumber = get_contact_bus(contactNumber);
      int keyNumber = get_contact_key(contactNumber);
      //rt_printf("bus: %d, key: %d\n", get_contact_bus(contactNumber), get_contact_key(contactNumber));
      for (lep = t->keyContrib[keyNumber]; lep != NULL; lep = lep->next) {
        if(LE_BUSNUMBER_OF(lep) != busNumber)
          continue;
        int wheelNumber = LE_WHEEL_NUMBER_OF(lep);
        osp = &(t->oscillators[wheelNumber]);
	//rt_printf("wheel: %3d %f%s \n", wheelNumber,
			//LE_LEVEL_OF(lep),
			//LE_LEVEL_OF(lep) > 0.3 ? "*" : " "
			//);
	//rt_printf("Level: %f refCount: %d\n", LE_LEVEL_OF(lep), t->aot[wheelNumber].refCount);

	osp->velocity = velocity;
        if (t->aot[wheelNumber].refCount == 0) {
		/* Flag the oscillator as added and modified */
		osp->rflags = ORF_ADDED | ORF_MODIFIED;
          /* If not already on the active list, add it */
          if (osp->aclPos == -1) {
            osp->aclPos = t->activeOscLEnd;
            t->activeOscList[t->activeOscLEnd++] = wheelNumber;
          }
        }
        else {
          osp->rflags |= ORF_MODIFIED;
	//rt_printf("%p wheel %d modified with delay %d\n", osp, wheelNumber, delay);
        }
#ifdef UNIQUE_ENVELOPE
	if(LE_LEVEL_OF(lep) > 0.3) {
		// if the contribution is strong enough,
		// let's make sure the wheel gets an envelope (with delay)
		osp->rflags |= ORF_ENV;
		osp->envDelay = delay;
		//rt_printf("wheel %d on with delay %d\n", wheelNumber, delay);
	} else {
		// the contribution of this oscillator is fairly quiet,
		// so we are not applying an envelope to it (though some other
		// contact may do so)
		//rt_printf("wheel %d too quiet (%f), delay: %d\n", wheelNumber, LE_LEVEL_OF(lep), delay);
	}
#endif /* UNIQUE_ENVELOPE */

        t->aot[wheelNumber].busLevel[LE_BUSNUMBER_OF(lep)] += LE_LEVEL_OF(lep);
        t->aot[wheelNumber].keyCount[LE_BUSNUMBER_OF(lep)] += 1;
        t->aot[wheelNumber].refCount += 1;
        //rt_printf("wheelNumber: %d, busLevel[%d]: %.8f\n", wheelNumber, LE_BUSNUMBER_OF(lep), t->aot[wheelNumber].busLevel[LE_BUSNUMBER_OF(lep)]);
      }
    }
    else if (MSG_GET_MSG(msg) == MSG_MCONTACTOFF) {
      int contactNumber = MSG_GET_NUMBER(msg);
      short velocity = MSG_GET_VELOCITY(msg);
      int busNumber = get_contact_bus(contactNumber);
      int keyNumber = get_contact_key(contactNumber);
      for (lep = t->keyContrib[keyNumber]; lep != NULL; lep = lep->next) {
        if(LE_BUSNUMBER_OF(lep) != busNumber)
          continue;
        int wheelNumber = LE_WHEEL_NUMBER_OF(lep);
        osp = &(t->oscillators[wheelNumber]);
        osp->velocity = velocity;

        t->aot[wheelNumber].busLevel[LE_BUSNUMBER_OF(lep)] -= LE_LEVEL_OF(lep);
        t->aot[wheelNumber].keyCount[LE_BUSNUMBER_OF(lep)] -= 1;
        t->aot[wheelNumber].refCount -= 1;

    if(!(0 <= t->aot[wheelNumber].refCount))
    {
        printf("++++Assert two: %d\n", t->aot[wheelNumber].refCount);
        t->aot[wheelNumber].refCount = 0;
        assert (0 <= t->aot[wheelNumber].refCount);
    }
    if(!(-1 < osp->aclPos))
    {
        printf("++++Assert three: %d\n", osp->aclPos);
        assert (-1 < osp->aclPos); /* Must be on the active osc list */
    }

        if (t->aot[wheelNumber].refCount == 0) {
          osp->rflags = OR_REM;
        }
        else {
          osp->rflags |= ORF_MODIFIED;
        }
      }
    }
    else if (MSG_GET_MSG(msg) == MSG_MKEYON) {
      keyNumber = MSG_GET_PRM(msg);
      for (lep = t->keyContrib[keyNumber]; lep != NULL; lep = lep->next) {
        int wheelNumber = LE_WHEEL_NUMBER_OF(lep);
        osp = &(t->oscillators[wheelNumber]);

        if (t->aot[wheelNumber].refCount == 0) {
          /* Flag the oscillator as added and modified */
          osp->rflags = OR_ADD;
          /* If not already on the active list, add it */
          if (osp->aclPos == -1) {
            osp->aclPos = t->activeOscLEnd;
            t->activeOscList[t->activeOscLEnd++] = wheelNumber;
          }
        }
        else {
          osp->rflags |= ORF_MODIFIED;
        }

        t->aot[wheelNumber].busLevel[LE_BUSNUMBER_OF(lep)] += LE_LEVEL_OF(lep);
        t->aot[wheelNumber].keyCount[LE_BUSNUMBER_OF(lep)] += 1;
        t->aot[wheelNumber].refCount += 1;
        //rt_printf("wheelNumber: %d, busLevel[%d]: %.8f\n", wheelNumber, LE_BUSNUMBER_OF(lep), t->aot[wheelNumber].busLevel[LE_BUSNUMBER_OF(lep)]);
      }

    }
    else if (MSG_GET_MSG(msg) == MSG_MKEYOFF) {
      keyNumber = MSG_GET_PRM(msg);
      for (lep = t->keyContrib[keyNumber]; lep != NULL; lep = lep->next) {
        int wheelNumber = LE_WHEEL_NUMBER_OF(lep);
        osp = &(t->oscillators[wheelNumber]);

        t->aot[wheelNumber].busLevel[LE_BUSNUMBER_OF(lep)] -= LE_LEVEL_OF(lep);
        t->aot[wheelNumber].keyCount[LE_BUSNUMBER_OF(lep)] -= 1;
        t->aot[wheelNumber].refCount -= 1;

	if(0 <= t->aot[wheelNumber].refCount)
		printf("wheelNumber: %d, refCount: %d\n", wheelNumber, t->aot[wheelNumber].refCount);
	if(0 <= t->aot[wheelNumber].refCount)
	{
		printf("+++assert four: refCount = %d\n", t->aot[wheelNumber].refCount);
		t->aot[wheelNumber].refCount = 0;
		assert (0 <= t->aot[wheelNumber].refCount);
	}
	if(!(-1 < osp->aclPos))
	{
		printf("++++assert five: aclPos == %d\n", osp->aclPos);
		assert (-1 < osp->aclPos); /* Must be on the active osc list */
	}

        if (t->aot[wheelNumber].refCount == 0) {
          osp->rflags = OR_REM;
        }
        else {
          osp->rflags |= ORF_MODIFIED;
        }
      }
    }
    else {
      assert (0);
    }
  } /* while message queue reader */

  /* ================================================================
		     A C T I V A T E D   L I S T
     ================================================================ */

  if ((recomputeRouting = (t->oldRouting != t->newRouting))) {
    t->oldRouting = t->newRouting;
  }


  /*
   * At this point, new oscillators has been added to the active list
   * and removed oscillators are still on the list.
   */

  /*
   * Note that if you have a contact offset but the oscillator
   * is still playing in another contact, then the oscillator will get
   * an ATTACK envelope instead of a RELEASE one
   */
  for (i = 0; i < t->activeOscLEnd; i++) {

    int oscNumber = t->activeOscList[i]; /* Get the oscillator number */
    AOTElement * aop = &(t->aot[oscNumber]); /* Get a pointer to active struct */
    osp = &(t->oscillators[oscNumber]); /* Point to the oscillator */

    if (osp->rflags & ORF_REMOVED) {/* Decay instruction for removed osc. */
      /* Put it on the removal list */
      removedList[removedEnd++] = oscNumber;
			/* All envelopes, both attack and release must traverse 0-1. */

      t->coreWriter->env = t->releaseEnv[i & 7];
      //rt_printf("Using releaseEnv %d %d, swell: %f\n", i&7, i, aop->sumSwell);

      if (copyDone) {
        t->coreWriter->opr = CR_ADDENV;
      } else {
        t->coreWriter->opr = CR_CPYENV;
        copyDone = 1;
      }

      t->coreWriter->src = osp->wave + osp->pos;
      t->coreWriter->off = 0;	/* Start at the beginning of target buffers */
      t->coreWriter->sgain = aop->sumSwell;
      t->coreWriter->pgain = aop->sumPercn;
      t->coreWriter->vgain = aop->sumScanr;
#ifdef LONG_ENVELOPES
	  osp->be = NULL;
	  aop->sumSwell = 0;
#endif /* LONG_ENVELOPES */
      /* Target gain is zero */
      t->coreWriter->nsgain = t->coreWriter->npgain = t->coreWriter->nvgain = 0.0;

      if (osp->lengthSamples < (osp->pos + BUFFER_SIZE_SAMPLES)) {
        /* Need another instruction because of wrap */
        CoreIns * prev = t->coreWriter;
        t->coreWriter->cnt = osp->lengthSamples - osp->pos;
        osp->pos = BUFFER_SIZE_SAMPLES - t->coreWriter->cnt;
        t->coreWriter += 1;
        t->coreWriter->opr = prev->opr;
        t->coreWriter->src = osp->wave;
        t->coreWriter->off = prev->cnt;
        t->coreWriter->env = prev->env + prev->cnt;

        t->coreWriter->sgain = prev->sgain;
        t->coreWriter->pgain = prev->pgain;
        t->coreWriter->vgain = prev->vgain;        

        t->coreWriter->nsgain = prev->nsgain;
        t->coreWriter->npgain = prev->npgain;
        t->coreWriter->nvgain = prev->nvgain;

        t->coreWriter->cnt = osp->pos;
      }
      else {
        t->coreWriter->cnt = BUFFER_SIZE_SAMPLES;
        osp->pos += BUFFER_SIZE_SAMPLES;
      }

      t->coreWriter += 1;
    }
    else {			/* ADD or MODIFIED */
      int reroute = 0;

      /*
       * Copy the current gains. For unmodified oscillators these will be
       * used. For modified oscillators we provide the update below.
       */
      if (osp->rflags & ORF_ADDED) {
        t->coreWriter->sgain = t->coreWriter->pgain = t->coreWriter->vgain = 0.0;
      }
      else { /* ORF_MODIFIED */
        t->coreWriter->sgain = aop->sumSwell;
        t->coreWriter->pgain = aop->sumPercn;
        t->coreWriter->vgain = aop->sumScanr;
      }

      /* Update the oscillator's contribution to each busgroup mix */

      float sumUpper = 0.0;
      float sumLower = 0.0;
      float sumPedal = 0.0;
      if ((osp->rflags & (ORF_MODIFIED | ORF_PERSISTED )) || t->drawBarChange) {
        int d;

        for (d = UPPER_BUS_LO; d < UPPER_BUS_END; d++) {
          sumUpper += aop->busLevel[d] * t->drawBarGain[d];
        }
        for (d = LOWER_BUS_LO; d < LOWER_BUS_END; d++) {
          sumLower += aop->busLevel[d] * t->drawBarGain[d];
        }
        for (d = PEDAL_BUS_LO; d < PEDAL_BUS_END; d++) {
          sumPedal += aop->busLevel[d] * t->drawBarGain[d];
        }
        reroute = 1;
      } else {
        sumUpper = aop->sumUpper;
		sumLower = aop->sumLower;
		sumPedal = aop->sumPedal;
	  }

      /* If the group mix or routing has changed */

	  float sumPercn;
	  float sumSwell;
	  float sumScanr;
	  int doRecomputeRouting = reroute || recomputeRouting;

    if (doRecomputeRouting) {
      if (t->oldRouting & RT_PERC) { /* Percussion */
        sumPercn = aop->busLevel[t->percSendBus];
      }
      else {
        sumPercn = 0.0;
      }

      sumScanr = 0.0;	/* Initialize scanner level */
      sumSwell = sumPedal; /* Initialize swell level */

      if (t->oldRouting & RT_UPPRVIB) { /* Upper manual ... */
        sumScanr += sumUpper; /* ... to vibrato */
      }
      else {
        sumSwell += sumUpper; /* ... to swell pedal */
      }

      if (t->oldRouting & RT_LOWRVIB) { /* Lower manual ... */
        sumScanr += sumLower; /* ... to vibrato */
      }
      else {
        sumSwell += sumLower; /* ... to swell pedal */
      }
    }
    else {
      sumPercn = aop->sumPercn;
      sumSwell = aop->sumSwell;
      sumScanr = aop->sumScanr;
    } /* if rerouting */

#ifdef LONG_ENVELOPES
	short envelopeCompleted = 0;
#endif /* LONG_ENVELOPES */
	/* Emit instructions for oscillator */
	int useEnv = 0;
	if (osp->rflags & OR_ADD) {
		//oscillator has been added or modified, or the envelope is
		//persisted from the previous iteration
		float* env = t->dynamicEnvelopesBuffers[oscNumber]; // may be overridden below, if noenvEnv is assigned instead
		//rt_printf("%3d_0x%03x_", (osp-t->oscillators)/sizeof(void*), osp->rflags);
		if(!(osp->rflags & ORF_PERSISTED)) {
			// the oscillator has just been added or modified
#ifdef UNIQUE_ENVELOPE
			if(osp->rflags & ORF_ENV)
			{
#endif /* UNIQUE_ENVELOPE */
				// for at least one of the new contributing contacts -
				// the oscillator has a "strong enough" amplitude: it gets a
				// fully-featured bouncing envelope (with delay)
#ifdef LONG_ENVELOPES
				if (osp->be == NULL) { // the envelope is not init'd
					// TODO: currently it uses the older envelope.
					// we could think instead of starting a new one when
					// a more recent envelope is started while the
					// previous one is still going
					osp->be = &t->bes[oscNumber];
					BouncingEnvelope_init(osp->be, osp->velocity, osp->envDelay, t->contactEnvelopeScale, t->contactEnvelopeRampTime);
					osp->rflags |= ORF_PERSISTED;
					//rt_printf("BouncingEnvelope_init with delay: %d, now: %x\n", osp->envDelay, osp->rflags);
				}
#ifdef UNIQUE_ENVELOPE
			} else {
				// the oscillator has no new strong contribution
				// so it gets a fake envelope
				// TODO: we should instead skip this altogether
				// if noenvEnv is going to be full of zeros
				env = t->noenvEnv;
				useEnv = 1;
				osp->be = NULL;
			}
		}
#endif /* UNIQUE_ENVELOPE */
		if(osp->be) { // should be equivalent to (osp->rflags & ORF_PERSISTED)
			// note ORF_PERSISTED would sometimes get set in the previous if/else
			//the BouncingEnvelope is already active (either because it has
			//just been added/modified, or because it has been persisted).
			int remaining = BouncingEnvelope_step(osp->be, BUFFER_SIZE_SAMPLES, env);
			if (remaining >= 0)
				useEnv = 1;
			if (remaining == 0) {
				// the envelope is completed
				envelopeCompleted = 1; 
				osp->be = NULL; // deactivate it
				osp->rflags &= ~ORF_PERSISTED; // forget about it
				//rt_printf("Cleared %p: %x\n", (osp - t->oscillators)/sizeof(void*), osp->rflags);
			} // if (remaining == 0)
		} // if(osp->be)
#else /* LONG_ENVELOPES */
		env = t->attackEnv[i & 7];
		useEnv = 1;
#endif /* LONG_ENVELOPES */

		/* Next gain values */
		t->coreWriter->nsgain = sumSwell;
		t->coreWriter->npgain = sumPercn;
		t->coreWriter->nvgain = sumScanr;        

		if(useEnv)
		{
			/* Envelope attack instruction */
			t->coreWriter->env = env;

			if (copyDone) {
				t->coreWriter->opr = CR_ADDENV;
			}
			else {
				t->coreWriter->opr = CR_CPYENV;
				copyDone = 1;
			}
		} else { // useEnv
			//pointless to multiply times 0: let's generate non-ENV instructions
			if (copyDone) {
				t->coreWriter->opr = CR_ADD;
			}
			else {
				t->coreWriter->opr = CR_CPY;
				copyDone = 1;
			}
		} // useEnv
	} //if (osp->rflags & OR_ADD)
	else {
//this means if osp->rflags did not contain ORF_MODIFIED, ORF_ADDED or
//ORF_PERSISTED but also not ORF_REMOVED, so this is just a plain-old boring
//oscillator that is still playing from the past with no envelopes
#ifdef LONG_ENVELOPES
		envelopeCompleted = 1; // as such, it gets an envelopeCompleted flag!
#endif /* LONG_ENVELOPES */
		if (copyDone) {
			t->coreWriter->opr = CR_ADD;
		}
		else {
			t->coreWriter->opr = CR_CPY;
			copyDone = 1;
		}
	}

#ifdef LONG_ENVELOPES
	if (envelopeCompleted || !(osp->rflags & ORF_PERSISTED)) { // the envelope is completed, store the values
#endif /* LONG_ENVELOPES */
		aop->sumUpper = sumUpper;
		aop->sumLower = sumLower;
		aop->sumPedal = sumPedal;
		aop->sumSwell = sumSwell;
		aop->sumScanr = sumScanr;
		aop->sumPercn = sumPercn;
#ifdef LONG_ENVELOPES
	}
#endif /* LONG_ENVELOPES */


      /* The source is the wave of the oscillator at its current position */
      t->coreWriter->src = osp->wave + osp->pos;
      t->coreWriter->off = 0;


      if (osp->lengthSamples < (osp->pos + BUFFER_SIZE_SAMPLES)) {
        /* Instruction wraps source buffer */
        CoreIns * prev = t->coreWriter; /* Refer to the first instruction */
        t->coreWriter->cnt = osp->lengthSamples - osp->pos; /* Set len count */
        osp->pos = BUFFER_SIZE_SAMPLES - t->coreWriter->cnt; /* Update src pos */

        t->coreWriter += 1;	/* Advance to next instruction */

        t->coreWriter->opr = prev->opr; /* Same operation */
        t->coreWriter->src = osp->wave; /* Start of wave because of wrap */
        t->coreWriter->off = prev->cnt;
        if (t->coreWriter->opr & CR_ENV_INS) { // if it has an envelope
          t->coreWriter->env = prev->env + prev->cnt; /* Continue envelope */
        }
        /* The gains are identical to the previous instruction */
        t->coreWriter->sgain = prev->sgain;
        t->coreWriter->pgain = prev->pgain;
        t->coreWriter->vgain = prev->vgain;

        t->coreWriter->nsgain = prev->nsgain;
        t->coreWriter->npgain = prev->npgain;
        t->coreWriter->nvgain = prev->nvgain;

        t->coreWriter->cnt = osp->pos; /* Up to next read position */
      } else { // instruction does not wrap the source buffer
        t->coreWriter->cnt = BUFFER_SIZE_SAMPLES;
        osp->pos += BUFFER_SIZE_SAMPLES;
      }

      t->coreWriter += 1;	/* Advance to next instruction */


    } /* else aot element not removed, ie modified or added */

    /* Clear rendering flags, excluding persistency */
    osp->rflags &= CLEAR_RFLAGS_MASK;

  } /* for the active list */

  t->drawBarChange = 0;

  /* ================================================================
		       R E M O V A L   L I S T
     ================================================================ */

  /*
   * Core instructions are now written.
   * Process the removed entries list. [Could action be merged above?]
   */
  for (i = 0; i < removedEnd; i++) {
    int vicosc = removedList[i]; /* Victim oscillator number */
    int actidx = t->oscillators[vicosc].aclPos; /* Victim's active index */
    t->oscillators[vicosc].aclPos = -1; /* Reset victim's backindex */
    t->activeOscLEnd--;

    if(!(0 <= t->activeOscLEnd))
    {
	    printf("++++Assert one: %d\n", t->activeOscLEnd);
	    assert (0 <= t->activeOscLEnd);
    }

    if (0 < t->activeOscLEnd) {	/* If list is not yet empty ... */
      int movosc = t->activeOscList[t->activeOscLEnd]; /* Fill hole w. last entry */
      if (movosc != vicosc) {
	t->activeOscList[actidx] = movosc;
	t->oscillators[movosc].aclPos = actidx;
      }
    }
  }

  /* ================================================================
   *             L E A K A G E R
     ================================================================ */

	float leakAmount = 0.0001f + gEnvelopeFilter/127.f * 0.2f * (t->contactDownCount /(float)(NOF_CONTACTS_PER_KEY * 61));
	// if the leaking is audible
	if(leakAmount > 0.0001)
	{
		CoreIns* tmpCoreReader = t->coreReader;
		CoreIns* oldCoreWriter = t->coreWriter;
		for (; tmpCoreReader < oldCoreWriter; tmpCoreReader++) {
			// go through all active oscillators and add some leakage
		    tmpCoreReader->sgain += leakAmount;
		    tmpCoreReader->vgain += leakAmount;
		    tmpCoreReader->pgain += leakAmount;
		    tmpCoreReader->nsgain += leakAmount;
		    tmpCoreReader->nvgain += leakAmount;
		    tmpCoreReader->npgain += leakAmount;
		}
		// create core instructions for non-active (leaking only) oscillators
		// starting from the first sounding wheel, that is 1
		for(int wheelNumber = 1; wheelNumber < NOF_WHEELS; ++wheelNumber)
		{
			if (t->aot[wheelNumber].refCount == 0) {
				osp = &(t->oscillators[wheelNumber]);
				int off = 0;
				int samplesUsed = 0;

				short opr;
				if(copyDone)
					opr = CR_ADD;
				else
				{
					opr = CR_CPY;
					copyDone = 1;
				}
				while(samplesUsed < BUFFER_SIZE_SAMPLES)
				{
					t->coreWriter->opr = opr;
					t->coreWriter->sgain = leakAmount;
					t->coreWriter->pgain = leakAmount;
					t->coreWriter->vgain = leakAmount;
					t->coreWriter->nsgain = leakAmount;
					t->coreWriter->npgain = leakAmount;
					t->coreWriter->nvgain = leakAmount;
					t->coreWriter->off = off;
					t->coreWriter->src = osp->wave + osp->pos;

					int samplesToEndOfSrc = osp->lengthSamples - osp->pos;
					int samplesUsedInThisIteration;
					int samplesLeft = BUFFER_SIZE_SAMPLES - samplesUsed;
					if(samplesToEndOfSrc < samplesLeft)
					{
						samplesUsedInThisIteration = samplesToEndOfSrc;
					}
					else
					{
						samplesUsedInThisIteration = samplesLeft;
					}
					t->coreWriter->cnt = samplesUsedInThisIteration;
					samplesUsed += samplesUsedInThisIteration;
					off += samplesUsedInThisIteration;
					if(samplesToEndOfSrc < samplesLeft)
						osp->pos = 0;
					else
						osp->pos += samplesLeft;
					++t->coreWriter;
				}
			}
		}
	}

  /* ================================================================
		   C O R E   I N T E R P R E T E R
     ================================================================ */

  /*
   * Special case: silence. If the vibrato scanner is used we must run zeros
   * through it because it is stateful (has a delay line).
   * We could possibly be more efficient here but for the moment we just zero
   * the input buffers to the mixing stage and reset the percussion.
   */

  if (t->coreReader == t->coreWriter) {
    float * ys = swlBuffer;
    float * yv = vibBuffer;
    float * yp = prcBuffer;

    for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) {
      *ys++ = 0.0;
      *yv++ = 0.0;
      *yp++ = 0.0;
    }

  }

  for (; t->coreReader < t->coreWriter; t->coreReader++) {
    short opr  = t->coreReader->opr;
    int     n  = t->coreReader->cnt;
    float * ys = swlBuffer + t->coreReader->off;
    float * yv = vibBuffer + t->coreReader->off;
    float * yp = prcBuffer + t->coreReader->off;
    const float   gs = t->coreReader->sgain;
    const float   gv = t->coreReader->vgain;
    const float   gp = t->coreReader->pgain;
    const float   ds = t->coreReader->nsgain - gs; /* Move these three down */
    const float   dv = t->coreReader->nvgain - gv;
    const float   dp = t->coreReader->npgain - gp;
    const float * ep  = t->coreReader->env;
    const float * xp  = t->coreReader->src;
    //printf("CR: %f %f +:%f  (ns:%f)\n", gs, ds, gs+ds, t->coreReader->nsgain );

    if (opr & CR_ADD_INS) {	/* ADD and ADDENV */
      if (opr & CR_ENV_INS) { /* ADDENV */
        for (; 0 < n; n--) {
          float x = (float) (*xp++);
          const float e = *ep++;
          *ys++ += x * (gs + (e * ds));
          *yv++ += x * (gv + (e * dv));
          *yp++ += x * (gp + (e * dp));
        }
      } else {			/* ADD */
        for (; 0 < n; n--) {
          const float x = (float) (*xp++);
          *ys++ += x * gs;
          *yv++ += x * gv;
          *yp++ += x * gp;
        }
      }

    } else {

      if (opr & CR_ENV_INS) { /* CPY and CPYENV */
        for (; 0 < n; n--) { /* CPYENV */
          const float x = (float) (*xp++);
          const float e =  *ep++;
          *ys++ = x * (gs + (e * ds));
          *yv++ = x * (gv + (e * dv));
          *yp++ = x * (gp + (e * dp));
        }

      } else {

        for (; 0 < n; n--) {	/* CPY */
          const float x = (float) (*xp++);
          *ys++ = x * gs;
          *yv++ = x * gv;
          *yp++ = x * gp;
        }

      }
    }
  } /* while there are core instructions */

  /* ================================================================
			    M I X D O W N
     ================================================================ */

  /*
   * The percussion, sweel and scanner buffers are now written.
   */

  /* If anything is routed through the scanner, apply FX and get outbuffer */

  if (t->oldRouting & RT_VIB) {
#if 1
    vibratoProc (&t->inst_vibrato, vibBuffer, vibYBuffr, BUFFER_SIZE_SAMPLES);
#else
    size_t ii;
    for (ii=0;ii< BUFFER_SIZE_SAMPLES;++ii) vibYBuffr[ii]=0.0;
#endif

  }

  /* Mix buffers, applying percussion and swell pedal. */

  if(t->mute)
  {
	for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) { /* Mute */
	  *yptr++ = 0;
    }
  } else {
    const float * xp = swlBuffer;
    const float * vp = vibYBuffr;
    const float * pp = prcBuffer;


    if (t->oldRouting & RT_PERC) {	/* If percussion is on */
#ifdef HIPASS_PERCUSSION
      float * tp = &(prcBuffer[BUFFER_SIZE_SAMPLES - 1]);
      float temp = *tp;
      pp = tp - 1;
      for (i = 1; i < BUFFER_SIZE_SAMPLES; i++) {
        *tp = *pp - *tp;
        tp--;
        pp--;
      }
      *tp = t->pz - *tp;
      t->pz = temp;
      pp = prcBuffer;
#endif /* HIPASS_PERCUSSION */
      t->outputGain = t->swellPedalGain * t->percDrawbarGain;
      if (t->oldRouting & RT_VIB) { /* If vibrato is on */
	for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) { /* Perc and vibrato */
	  *yptr++ =
	    (t->outputGain * KEYCOMPLEVEL *
	     ((*xp++) + (*vp++) + ((*pp++) * t->percEnvGain)));
	  t->percEnvGain *= t->percEnvGainDecay;
	  KEYCOMPCHASE();
	}
      } else {			/* Percussion only */
	for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) {
	  *yptr++ =
	    (t->outputGain * KEYCOMPLEVEL * ((*xp++) + ((*pp++) * t->percEnvGain)));
	  t->percEnvGain *= t->percEnvGainDecay;
	  KEYCOMPCHASE();
	}
      }

    } else if (t->oldRouting & RT_VIB) { /* No percussion and vibrato */

      for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) {
	*yptr++ =
	  (t->swellPedalGain * KEYCOMPLEVEL * ((*xp++) + (*vp++)));
	KEYCOMPCHASE();
      }
    } else {			/* No percussion and no vibrato */
      for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) {
	*yptr++ =
	  (t->swellPedalGain * KEYCOMPLEVEL * (*xp++));
	KEYCOMPCHASE();
      }
    }
  }

  if (t->upperKeyCount == 0) {
    t->percEnvGain = t->percEnvGainReset;
  }

#ifdef WRITEFILE
  // log sensors
  {
	Keys* keys = t->keys;
    float pos[TOTAL_SCANNER_KEYS];
    for(int n = 0; n < TOTAL_SCANNER_KEYS; ++n){
      pos[n] = Keys_getNoteValue(keys, n + 24);
    }
    WriteFile_logArray(t->audioLogFile, pos, TOTAL_SCANNER_KEYS);
  }
#endif /* WRITEFILE */
  // make organ sound stereo
  float* y2ptr = bufs[1];
  yptr = bufs[0];
  for(i = 0; i < BUFFER_SIZE_SAMPLES; ++i)
  {
    //*y2ptr++ = *yptr++;
  }
#ifdef WRITEFILE
  // log audio
  WriteFile_logArray(t->audioLogFile, bufs[0], BUFFER_SIZE_SAMPLES);
  WriteFile_logArray(t->audioLogFile, bufs[1], BUFFER_SIZE_SAMPLES);
#endif /* WRITEFILE */
} /* oscGenerateFragment */

