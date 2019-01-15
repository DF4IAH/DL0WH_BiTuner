#include <math.h>

void cAdd(float* outR, float* outI, float aR, float aI, float bR, float bI)
{
  if (outR && outI) {
    *outR = aR + bR;
    *outI = aI + bI;
  }
}

void cSub(float* outR, float* outI, float aR, float aI, float bR, float bI)
{
  if (outR && outI) {
    *outR = aR - bR;
    *outI = aI - bI;
  }
}

void cMul(float* outR, float* outI, float aR, float aI, float bR, float bI)
{
  if (outR && outI) {
    *outR = aR * bR  -  aI * bI;
    *outI = aI * bI  +  bI * aI;
  }
}

void cDiv(float* outR, float* outI, float aR, float aI, float bR, float bI)
{
  if (outR && outI) {
    *outR = (aR * bR  +  aI * bI) / (bR * bR  +  bI * bI);
    *outI = (aI * bR  -  aR * bI) / (bR * bR  +  bI * bI);
  }
}

void cInv(float* outR, float* outI, float bR, float bI)
{
  if (outR && outI) {
    *outR = ( bR) / (bR * bR  +  bI * bI);
    *outI = (-bI) / (bR * bR  +  bI * bI);
  }
}
