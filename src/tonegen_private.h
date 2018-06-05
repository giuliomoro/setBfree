#include <Keys_c.h>
#define INDIVIDUAL_CONTACTS
#define TOTAL_SCANNER_KEYS 73
#define FIRST_SOUNDING_KEY 13
#define NOF_DRAWBARS_PER_MANUAL 9

#define LE_HARMONIC_NUMBER_OF(LEP) ((LEP)->u.ssf.sa)
#define LE_HARMONIC_LEVEL_OF(LEP) ((LEP)->u.ssf.fc)

#define LE_WHEEL_NUMBER_OF(LEP) ((LEP)->u.ssf.sa)
#define LE_WHEEL_LEVEL_OF(LEP) ((LEP)->u.ssf.fc)

#define LE_TERMINAL_OF(LEP) ((LEP)->u.ssf.sa)
#define LE_BUSNUMBER_OF(LEP) ((LEP)->u.ssf.sb)
#define LE_TAPER_OF(LEP) ((LEP)->u.ssf.fc)
#define LE_LEVEL_OF(LEP) ((LEP)->u.ssf.fc)

/**
 * LE_BLOCKSIZE is the number ListElements we allocate in each call to
 * malloc.
 */

#define LE_BLOCKSIZE 200

/**
 * Buses are numbered like this:
 *  0-- 8, upper manual, ( 0=16',  8=1')
 *  9--17, lower manual, ( 9=16', 17=1')
 * 18--26, pedal         (18=32')
 */
#ifndef NOF_BUSES
#define NOF_BUSES 27		/* Should be in tonegen.h */
#endif /* NOF_BUSES */
#define UPPER_BUS_LO 0
#define UPPER_BUS_END 9
#define LOWER_BUS_LO 9
#define LOWER_BUS_END 18
#define PEDAL_BUS_LO 18
#define PEDAL_BUS_END 27

/*
 * The message layout is:
 *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 * [ Message  ] [                           Parameter (currently only uses 12 bit)                 ]
 * [0  0  0  0]                             [     velocity     ] [       Key on number             ]
 * [0  0  0  1]                             [     velocity     ] [       Key on number             ]
 * [0  0  1  0]                             [     velocity     ] [      Contact on number          ]
 * [0  0  1  1]                             [     velocity     ] [      Contact on number          ]
 */

/* Message field access macros */
#define MSG_MMASK 0xf0000000
#define MSG_PMASK 0x0fffffff
#define MSG_VELOCITY_SHIFT 12
#define MSG_VELOCITY_MASK (0xffff << MSG_VELOCITY_SHIFT)
#define MSG_NUMBERMASK 0xfff

/* Retrive message part from a message */
#define MSG_GET_MSG(M) ((M) & MSG_MMASK)
/* Retrieve parameter part from a message */
#define MSG_GET_PRM(M) ((M) & MSG_PMASK)
#define MSG_GET_NUMBER(M) ((M) & MSG_NUMBERMASK)
#define MSG_GET_VELOCITY(M) (((M) & MSG_VELOCITY_MASK) >> MSG_VELOCITY_SHIFT)
/* Messages */
#define MSG_MKEYOFF 0x00000000
#define MSG_MKEYON  0x10000000
/* Key released message, arg is keynumber */
#define MSG_KEY_OFF(K) (MSG_MKEYOFF | ((K) & MSG_PMASK))
/* Key depressed message, arg is keynumber */
#define MSG_KEY_ON(K)  (MSG_MKEYON  | ((K) & MSG_PMASK))

#ifdef INDIVIDUAL_CONTACTS
#define MSG_MCONTACTOFF  0x20000000
#define MSG_MCONTACTON  0x30000000
/* Contact closed message, args is contactnumber and velocity */
#define MSG_CONTACT_OFF(C, V)  (MSG_MCONTACTOFF  |\
								(((V) << MSG_VELOCITY_SHIFT) & MSG_VELOCITY_MASK) |\
								((C) & MSG_PMASK))
/* Contact open message, args are contactnumber and velocity */
#define MSG_CONTACT_ON(C, V)  (MSG_MCONTACTON  |\
								(((V) << MSG_VELOCITY_SHIFT)  & MSG_VELOCITY_MASK) |\
								((C) & MSG_PMASK))
#endif /* INDIVIDUAL_CONTACTS */


/* Core instruction codes (opr field in struct _coreins). */

#define CR_CPY    0		/* Copy instruction */
#define CR_ADD    1		/* Add instruction */
#define CR_CPYENV 2		/* Copy via envelope instruction */
#define CR_ADDENV 3		/* Add via envelope instruction */
#define CR_ADD_INS (CR_ADD & CR_ADDENV)
#define CR_ENV_INS (CR_CPYENV & CR_ADDENV)

/* Rendering flag bits */
#define ORF_MODIFIED (1 << 2)
#define ORF_ADDED    (1 << 1)
#define ORF_REMOVED  (1 << 0)

#ifdef LONG_ENVELOPES
#define ORF_PERSISTED (1 << 3)
#define ORF_ADDED_NOENV (1 << 4)
#define OR_ADD (ORF_MODIFIED | ORF_ADDED | ORF_PERSISTED)
#define CLEAR_RFLAGS_MASK (ORF_PERSISTED) // OR here those flags that should not be cleared
#else /* LONG_ENVELOPES */
#define ORF_PERSISTED 0
#define OR_ADD (ORF_MODIFIED | ORF_ADDED)
#define CLEAR_RFLAGS_MASK 0
#endif /* LONG_ENVELOPES */

/* Composite flag bits */
#define OR_REM (ORF_MODIFIED | ORF_REMOVED)


#define RT_PERC2ND 0x08
#define RT_PERC3RD 0x04
#define RT_PERC    0x0C
#define RT_UPPRVIB 0x02
#define RT_LOWRVIB 0x01
#define RT_VIB     0x03

