#ifndef _COMPLEX_H
#define _COMPLEX_H

void cAdd(float* outR, float* outI, float aR, float aI, float bR, float bI);
void cSub(float* outR, float* outI, float aR, float aI, float bR, float bI);
void cMul(float* outR, float* outI, float aR, float aI, float bR, float bI);
void cDiv(float* outR, float* outI, float aR, float aI, float bR, float bI);
void cInv(float* outR, float* outI, float bR, float bI);


#endif
