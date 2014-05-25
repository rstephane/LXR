//---------------------------------------------------------------------------
//
// 3 Band EQ :)
//
// EQ.H - Header file for 3 band EQ
//
// (c) Neil C / Etanza Systems / 2K6
//
// Shouts / Loves / Moans = etanza at lycos dot co dot uk
//
// This work is hereby placed in the public domain for all purposes, including
// use in commercial applications.
//
// The author assumes NO RESPONSIBILITY for any problems caused by the use of
// this software.
//
//----------------------------------------------------------------------------

#ifndef __EQ3BAND__
#define __EQ3BAND__

#include "stm32f4xx.h"

// ------------
//| Structures |
// ------------

typedef struct
{
// Filter #1 (Low band)

double lf; // Frequency
double f1p0; // Poles ...
double f1p1;
double f1p2;
double f1p3;

// Filter #2 (High band)

double hf; // Frequency
double f2p0; // Poles ...
double f2p1;
double f2p2;
double f2p3;

// Sample history buffer

double sdm1; // Sample data minus 1
double sdm2; // 2
double sdm3; // 3

// Gain Controls

double lg; // low gain
double mg; // mid gain
double hg; // high gain

} EQSTATE;


// ---------
//| Exports |
// ---------

//extern void init_3band_state(EQSTATE* es, int lowfreq, int highfreq, int mixfreq);
//extern double do_3band(EQSTATE* es, double sample);

// rstephane 
//void init_3band_state(EQSTATE* eq, int lowfreq, int highfreq, int mixfreq,const uint8_t size);
EQSTATE* init_3band_state(int lowfreq, int highfreq, int mixfreq,const uint8_t size);
void calc3BandEqBlock(uint8_t lowFreq, uint8_t midFreq,uint8_t  highFreq, int16_t* buf, const uint8_t size);



#endif // #ifndef __EQ3BAND__
//---------------------------------------------------------------------------

