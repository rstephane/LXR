/*
 * distortion.c
 *
 *  Created on: 14.04.2012
 * ------------------------------------------------------------------------------------------------------------------------
 *  Copyright 2013 Julian Schmidt
 *  Julian@sonic-potions.com
 * ------------------------------------------------------------------------------------------------------------------------
 *  This file is part of the Sonic Potions LXR drumsynth firmware.
 * ------------------------------------------------------------------------------------------------------------------------
 *  Redistribution and use of the LXR code or any derivative works are permitted
 *  provided that the following conditions are met:
 *
 *       - The code may not be sold, nor may it be used in a commercial product or activity.
 *
 *       - Redistributions that are modified from the original source must include the complete
 *         source code, including the source code for all components used by a binary built
 *         from the modified sources. However, as a special exception, the source code distributed
 *         need not include anything that is normally distributed (in either source or binary form)
 *         with the major components (compiler, kernel, and so on) of the operating system on which
 *         the executable runs, unless that component itself accompanies the executable.
 *
 *       - Redistributions must reproduce the above copyright notice, this list of conditions and the
 *         following disclaimer in the documentation and/or other materials provided with the distribution.
 * ------------------------------------------------------------------------------------------------------------------------
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ------------------------------------------------------------------------------------------------------------------------
 */

#include "distortion.h"
#include "math.h"


// rstephane : declare function for bit wise manipulation
#include <complex.h>
#include <fcntl.h>
#include <sys/stat.h>

// rstephane BITS Manipulation
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))


// rstephane : Alien Wah
#define samplerate 44100     
#define lfoskipsamples 25 // How many samples are processed before compute the lfo value again

struct params
{
   float freq,startphase,fb;
   int delay;
} awparams;
//alien wah internal parameters

struct alienwahinternals
{
 _Complex *delaybuf;
 float lfoskip;
 long int t;
 _Complex c;
 int k;
} awint;

// End rstephane



//--------------------------------------------------
__inline void setDistortionShape(Distortion *dist, uint8_t shape)
{
	dist->shape = 2*(shape/128.f)/(1-(shape/128.f));
}
//--------------------------------------------------
void calcDistBlock(const Distortion *dist, int16_t* buf, const uint8_t size)
{
	uint8_t i;
	for(i=0;i<size;i++)
	{
			float x = buf[i]/32767.f;
			x = (1+dist->shape)*x/(1+dist->shape*fabsf(x));
			buf[i] = (x*32767);
	}
}
//--------------------------------------------------

float distortion_calcSampleFloat(const Distortion *dist, float x)
{
	return (1+dist->shape)*x/(1+dist->shape*fabsf(x));
}

// rstephane : Range funciton 
float calcRange(uint8_t valueAmount, float old_min ,float old_max,float new_min,float new_max )
{
	float knobValue;
	knobValue = (  ( (valueAmount - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min  );
	return knobValue;
}

// rstephane : OTO biscuit FX 
void calcOTOFxBlock(uint8_t maskType, int16_t* buf,const uint8_t size, uint8_t otoAmount)
{
	
		
	uint8_t i,j;
	uint16_t temp;
	int16_t bufTemp[size];
	
	
	// for a strange effect derived from moog filter from musicdsp.org
	float c , r ,v0,v1;
	
	// TO change the range from 0 to 127 to 0.0 to 1.0
	float old_min = 0;
	float old_max = 127;
	float new_min = 0.0;
	float new_max = 1.0;
	float knobValue, dry, wet;
	
	// dry and Wet infor
	float dryFloatTemp;
	float wetFloatTemp;
	
	// knob Value (0 to 127 <--> 0 to 1) 
	// knobValue = (  ( (otoAmount - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min  );
	knobValue = calcRange(otoAmount, old_min , old_max, new_min, new_max );
	
	dry = (1-knobValue);
	wet = fabs((1-knobValue)-1);
	
	// WE copy the Sounds before manipulating it 
	for(i=0;i<size;i++)
		bufTemp[i] = buf[i] ;
					
	switch(maskType)
	{
		case 1 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0100100F; // ou 7F avce en plus COA à 0 !!!
			break;
		case 2 :
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x00000FFF; // remove 12 TOP BIts 16 bits to 8 bitmap 
			break;
		case 3 : 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 14 ;	
			break;
			/*for(i=0;i<size;i++)
				bufTemp[i] &= 0x000007FF; // remove 8 TOP BIts 16 bits to 8 bitmap 
			break;*/ 
		case 4 : 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 1 ;	
			break;
		case 5 : 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 2 ;	
			break;
		case 6 :
			for(i=0;i<size;i++)
			{
				bufTemp[i] = bufTemp[i] << 3 ;	
			}
			break;
		/* nice effect !!!!
		case 6 :
			for(i=0;i<size;i++)
			{
				bufTemp[i] = buf[i] << 3 ;
		 	
				dryFloatTemp = (float) buf[i] * dry;
				wetFloatTemp = (float) bufTemp[i] * wet;
				buf[i] = (uint16_t) dryFloatTemp + (uint16_t) wetFloatTemp ;
		 	} 
			break;
		*/
		case 7 :
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 5 ;		
			break;
		
		
		
		case 8 : 
			for(i=0;i<size;i++)
				bufTemp[i] = bufTemp[i] << 6 ;		
			break;
		
		case 9 : // reverse all bit 1 becomes 0 :-)
			for(i=0;i<size;i++)
			{
			
				for (j=0;j<16;j++)
					temp = bit_flip(bufTemp[i],BIT(j));
				bufTemp[i] = temp;
			}			
			break;
		case 10 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= (0x0000F000);	
		 	break;
		case 11 : // bof
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0000E7FF; 
			break;
		case 12 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x00000087; 
			break;
		case 13 : 
			for(i=0;i<size;i++)
				bufTemp[i] &= 0x0000F7FF; 
			break;
		case 14 : 
			for(i=0;i<size;i++)
				//bufTemp[i] &= 0x01009009; 
				bufTemp[i] = (int16_t) BassBoost((float) bufTemp[i]);
			break;
		case 15 : 
			//Parameter calculation
			//cutoff and resonance are from 0 to 127
			//   c = pow(0.5, (128-cutOff)   / 16.0);
			// r = pow(0.5, (resonance+24) / 16.0);

			 c = pow(0.5, (128-35)   / 16.0);
			 r = pow(0.5, (40+24) / 16.0);
			
			//Loop:
			v0 = v1 = 0;
			for ( i=0; i < size; i++ ) {
			   v0 =  (1-r*c)*v0  -  (c)*v1  + (c)*bufTemp[i];
			   v1 =  (1-r*c)*v1  +  (c)*v0;

			   bufTemp[i] = v1; // Low pass
			}			
			break;
		default: 
			maskType = 0;
			break;
		break;	
	}	

// We merge the Dry and Wet Signal
if (maskType!=0)
	for(i=0;i<size;i++)
	{
		dryFloatTemp = (float) (buf[i]) * dry;
		wetFloatTemp = (float) (bufTemp[i]) * wet;
		buf[i] = wetFloatTemp + dryFloatTemp ;
	}

}


// rstephane : Alien Wah from music dsp . org
/*
 Alien-Wah by Nasca Octavian Paul from Tg. Mures, Romania
 e-mail:  <paulnasca@email.ro> or <paulnasca@yahoo.com>.
*/

/*
 The algorithm was found by me by mistake(I was looking for something else);
 I called this effect "Alien Wah" because sounds a bit like wahwah, but more strange.
 The ideea of this effect is very simple: It is a feedback delay who uses complex numbers.
 If x[] represents the input and y[] is the output, so a simple feedback delay looks like this:
 y[n]=y[n-delay]*fb+x[n]*(1-fb)
 
 'fb' is a real number between 0 and 1.
 If you change the fb with a complex number who has the MODULUS smaller than 1, it will look like this.

 fb=R*(cos(alpha)+i*sin(alpha));  i^2=-1; R<1;
 y[n]=y[n-delay]*R*(cos(alpha)+i*sin(alpha))+x[n]*(1-R);

 alpha is the phase of the number and is controlled by the LFO(Low Frequency Oscillator).
 If the 'delay' parameter is low, the effect sounds more like wah-wah,
 but if it is big, the effect will sound very interesting.
 The input x[n] has the real part of the samples from the wavefile and the imaginary part is zero.
 The output of this effect is the real part of y[n].

 Here it is a simple and unoptimised implementation of the effect. All parameters should be changed at compile time.
 It was tested only with Borland C++ 3.1.

 Please send me your opinions about this effect.
 Hope you like it (especially if you are play to guitar).
 Paul.
*/

/*
Alien Wah Parameters

 freq       - "Alien Wah" LFO frequency
 startphase - "Alien Wah" LFO startphase (radians), needed for stereo
 fb         - "Alien Wah" FeedBack (0.0 - low feedback, 1.0 = 100% high feedback)
 delay      -  delay in samples at 44100 KHz (recomanded from 5 to 50...)
*/


//effect initialisation
void init_phase(float freq,float startphase,float fb,int delay)
{  
  awparams.freq=freq;
  awparams.startphase=startphase;
  awparams.fb=fb/4+0.74;
  awparams.delay=(int)(delay/44100.0*samplerate);
  
  if (delay<1) delay=1;
  
  // This should be solved , complex doesnot exist , C++ function
  // awint.delaybuf=(int)awparams.delay; //complex[awparams.delay];
  *awint.delaybuf=creal(awparams.delay),cimag(awparams.delay); //complex[awparams.delay];
  
  int i;
  for (i=0;i<delay;i++) 
  	awint.delaybuf[i]=0,0;
  awint.lfoskip=freq*2*3.141592653589/samplerate;
  awint.t=0;
}

//-----------------
// rstephane : ALien Wah from Musicdsp.org
// working fine, but not yet shown on the menu
// we should also map four parameters of this effect e:-)


void calcAlienWahFxBlock(uint8_t freq,uint8_t startphase,uint8_t fb,int8_t delay, int16_t* buf,const uint8_t size)
{
  
	uint16_t i;
	float lfo,out;
	_Complex outc;

	// WE copy the Sounds before manipulating it 
	int16_t bufTemp[size];
	for(i=0;i<size;i++)
		bufTemp[i] = buf[i] ;
	
	
	// we transform the value into another range
	// TO change the range from 0 to 127 to 0.0 to 1.0
	float old_min = 0.0;
	float old_max = 127.0;
	float new_min = 0.0;
	float new_max = 1.0;
	
	// knob Value (0 to 127 <--> 0 to 1) 
	float knobValueFreq= calcRange(freq, old_min , old_max, new_min, new_max );
	float knobValueStartphase= calcRange(startphase, old_min , old_max, new_min, new_max );
	float knobValueFb= calcRange(fb, old_min , old_max, new_min, new_max );
	
	//set effects parameters
	init_phase( knobValueFreq, knobValueStartphase, knobValueFb, delay); 
 	
 	for(i=0;i<size;i++)
 	{
   		if (awint.t++%lfoskipsamples==0)
   		{
      			lfo=(1+cos(awint.t*awint.lfoskip+awparams.startphase));
      			awint.c= creal(cos(lfo)*awparams.fb),cimag(sin(lfo)*awparams.fb);
   		};
   	
   		outc=awint.c*awint.delaybuf[awint.k]+(1-awparams.fb)*bufTemp[i];
   		awint.delaybuf[awint.k]=outc;
	   	if ((++awint.k)>=awparams.delay)
      			awint.k=0;
   
   		out=creal(outc)*3;  //take real part of outc
   		if (out<-32768) out=-32768;
   		else if (out>32767) out=32767; //Prevents clipping
   
   		bufTemp[i]=out;
	}; 

	for(i=0;i<size;i++)
		buf[i] = bufTemp[i] ;
	
	
}

/* Params:
selectivity - frequency response of the LP (higher value gives a steeper one) [70.0 to 140.0 sounds good]
ratio - how much of the filtered signal is mixed to the original
gain2 - adjusts the final volume to handle cut-offs (might be good to set dynamically) */


 float saturate( float input ) { 
//clamp without branching
#define _limit 0.95
  float x1 = fabsf( input + _limit );
  float x2 = fabsf( input - _limit );
  return 0.5 * (x1 - x2);
}

float MAX( float a, float b) {
  a -= b;
  a += fabsf( a );
  a *= 0.5;
  a += b;
  return a;
}


float MIN( float a, float b) {
  a = b - a;
  a += fabsf( a );
  a *= 0.5;
  a = b - a;
  return a;
}


float BassBoost(float sample)
{
static float selectivity = 100.0, gain1= 50.0, gain2= 50.0, ratio = 50.0, cap;
gain1 = 1.0/(selectivity + 1.0);

cap= (sample + cap*selectivity )*gain1;
sample = saturate((sample + cap*ratio)*gain2);

return sample;
}

