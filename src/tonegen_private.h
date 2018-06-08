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
 * [ msg][                                  Parameter(s)                                            ]
 * [0  0][                                                                Key on number             ] MSG_MKEYON
 * [0  1][                                                                Key off number            ] MSG_MKEYOFF
 * [1  0][                 delay             ][     velocity         ][  Contact on number          ] MSG_MCONTACTON (ifdef'ed)
 * [1  1][                 delay             ][     velocity         ][  Contact off number         ] MSG_MCONTACTOFF (ifdef'ed)
 */

/* Message field access macros */
#define MSG_MSG_SHIFT 30
#define MSG_MMASK (0x3 << MSG_MSG_SHIFT)
#define MSG_PMASK (~MSG_MMASK)

/* Retrieve message message type from a message */
#define MSG_GET_MSG(M) ((M) & MSG_MMASK)
/* Retrieve parameter part from a message */
#define MSG_GET_PRM(M) ((M) & MSG_PMASK)

/* Messages */
#define MSG_MKEYOFF (0x0 << MSG_MSG_SHIFT)
#define MSG_MKEYON  (0x1 << MSG_MSG_SHIFT)
/* Key released message, arg is keynumber */
#define MSG_KEY_OFF(K) (MSG_MKEYOFF | ((K) & MSG_PMASK))
/* Key depressed message, arg is keynumber */
#define MSG_KEY_ON(K)  (MSG_MKEYON  | ((K) & MSG_PMASK))

#ifdef INDIVIDUAL_CONTACTS
#define MSG_MCONTACTOFF (0x2 << MSG_MSG_SHIFT)
#define MSG_MCONTACTON (0x3 << MSG_MSG_SHIFT)
// MSG_MCONTACTON:
// 10 bits contact number N
// 8 bits velocity V
// 12 bits delay D . The stored value is actually (delay >> MSG_DELAY_VALUE_SHIFT)
// 2 bits message type M
// | M M D D D D D D | D D D D D D V V | V V V V V V N N | N N N N N N N N |
#define MSG_NUMBER_SHIFT 0
#define MSG_NUMBER_MASK (0x3ff << MSG_NUMBER_SHIFT)
#define MSG_VELOCITY_SHIFT 10
#define MSG_VELOCITY_MASK (0xff << MSG_VELOCITY_SHIFT)
#define MSG_DELAY_SHIFT 18
#define MSG_DELAY_VALUE_SHIFT 2
#define MSG_DELAY_MASK (0xfff << MSG_DELAY_SHIFT)

#define MSG_GET_NUMBER(M) (((M) & MSG_NUMBER_MASK) >> MSG_NUMBER_SHIFT)
#define MSG_GET_VELOCITY(M) (((M) & MSG_VELOCITY_MASK) >> MSG_VELOCITY_SHIFT)
#define MSG_GET_DELAY(M) (((M) & MSG_DELAY_MASK) >> (MSG_DELAY_SHIFT - MSG_DELAY_VALUE_SHIFT))

#define MSG_CONTACT_X(M, N, V, D) \
	(((M) & MSG_MMASK) |\
	(((D >> MSG_DELAY_VALUE_SHIFT) << MSG_DELAY_SHIFT) & MSG_DELAY_MASK) |\
	(((V) << MSG_VELOCITY_SHIFT) & MSG_VELOCITY_MASK) |\
	((N) & MSG_PMASK))
/* Contact open message, args are contactnumber, velocity and delay*/
#define MSG_CONTACT_OFF(N, V, D) MSG_CONTACT_X(MSG_MCONTACTOFF, N, V, D)
/* Contact closed message, args are contactnumber, velocity and delay */
#define MSG_CONTACT_ON(N, V, D) MSG_CONTACT_X(MSG_MCONTACTON, N, V, D)
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
#define ORF_ENV (1 << 4)
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

