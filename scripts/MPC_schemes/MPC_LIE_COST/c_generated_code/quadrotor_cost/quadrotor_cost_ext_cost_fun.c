/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_c2 CASADI_PREFIX(c2)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[8] = {1., 0., 0., 0., 0., 0., 0., 0.};
static const casadi_real casadi_c1[64] = {1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2.};
static const casadi_real casadi_c2[16] = {6.7114093959731547e-01, 0., 0., 0., 0., 600., 0., 0., 0., 0., 600., 0., 0., 0., 0., 600.};

/* quadrotor_cost_ext_cost_fun:(i0[14],i1[4],i2[],i3[18])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, *w3=w+18, w4, *w5=w+27, *w6=w+35, w7, w8, w9, w10, *w11=w+57, *w12=w+61, *w13=w+65, *w14=w+69, *w15=w+73, *w16=w+89, *w17=w+105, *w18=w+137, *w19=w+169, *w20=w+185, *w21=w+217, *w22=w+281, w23, w24, w25, w26, w27, w28, w29, w30, w31, w32, *w33=w+355, *w34=w+363, *w35=w+377, *w36=w+380;
  /* #0: @0 = 10 */
  w0 = 10.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #3: @3 = [1, 0, 0, 0, 0, 0, 0, 0] */
  casadi_copy(casadi_c0, 8, w3);
  /* #4: @4 = 0 */
  w4 = 0.;
  /* #5: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #6: @6 = input[3][0] */
  casadi_copy(arg[3], 18, w6);
  /* #7: @7 = @6[0] */
  for (rr=(&w7), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #8: @8 = @6[1] */
  for (rr=(&w8), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #9: @8 = (-@8) */
  w8 = (- w8 );
  /* #10: @9 = @6[2] */
  for (rr=(&w9), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #11: @9 = (-@9) */
  w9 = (- w9 );
  /* #12: @10 = @6[3] */
  for (rr=(&w10), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #13: @10 = (-@10) */
  w10 = (- w10 );
  /* #14: @11 = horzcat(@7, @8, @9, @10) */
  rr=w11;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #15: @11 = @11' */
  /* #16: @7 = @6[1] */
  for (rr=(&w7), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #17: @8 = @6[0] */
  for (rr=(&w8), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #18: @9 = @6[3] */
  for (rr=(&w9), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #19: @9 = (-@9) */
  w9 = (- w9 );
  /* #20: @10 = @6[2] */
  for (rr=(&w10), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #21: @12 = horzcat(@7, @8, @9, @10) */
  rr=w12;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #22: @12 = @12' */
  /* #23: @7 = @6[2] */
  for (rr=(&w7), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #24: @8 = @6[3] */
  for (rr=(&w8), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #25: @9 = @6[0] */
  for (rr=(&w9), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #26: @10 = @6[1] */
  for (rr=(&w10), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #27: @10 = (-@10) */
  w10 = (- w10 );
  /* #28: @13 = horzcat(@7, @8, @9, @10) */
  rr=w13;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #29: @13 = @13' */
  /* #30: @7 = @6[3] */
  for (rr=(&w7), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #31: @8 = @6[2] */
  for (rr=(&w8), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #32: @8 = (-@8) */
  w8 = (- w8 );
  /* #33: @9 = @6[1] */
  for (rr=(&w9), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #34: @10 = @6[0] */
  for (rr=(&w10), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #35: @14 = horzcat(@7, @8, @9, @10) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #36: @14 = @14' */
  /* #37: @15 = horzcat(@11, @12, @13, @14) */
  rr=w15;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  /* #38: @16 = @15' */
  for (i=0, rr=w16, cs=w15; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #39: @15 = zeros(4x4) */
  casadi_clear(w15, 16);
  /* #40: @17 = horzcat(@16, @15) */
  rr=w17;
  for (i=0, cs=w16; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<16; ++i) *rr++ = *cs++;
  /* #41: @18 = @17' */
  for (i=0, rr=w18, cs=w17; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #42: @7 = @6[4] */
  for (rr=(&w7), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #43: @8 = @6[5] */
  for (rr=(&w8), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #44: @8 = (-@8) */
  w8 = (- w8 );
  /* #45: @9 = @6[6] */
  for (rr=(&w9), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #46: @9 = (-@9) */
  w9 = (- w9 );
  /* #47: @10 = @6[7] */
  for (rr=(&w10), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #48: @10 = (-@10) */
  w10 = (- w10 );
  /* #49: @11 = horzcat(@7, @8, @9, @10) */
  rr=w11;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #50: @11 = @11' */
  /* #51: @7 = @6[5] */
  for (rr=(&w7), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #52: @8 = @6[4] */
  for (rr=(&w8), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #53: @9 = @6[7] */
  for (rr=(&w9), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #54: @9 = (-@9) */
  w9 = (- w9 );
  /* #55: @10 = @6[6] */
  for (rr=(&w10), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #56: @12 = horzcat(@7, @8, @9, @10) */
  rr=w12;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #57: @12 = @12' */
  /* #58: @7 = @6[6] */
  for (rr=(&w7), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #59: @8 = @6[7] */
  for (rr=(&w8), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #60: @9 = @6[4] */
  for (rr=(&w9), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #61: @10 = @6[5] */
  for (rr=(&w10), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #62: @10 = (-@10) */
  w10 = (- w10 );
  /* #63: @13 = horzcat(@7, @8, @9, @10) */
  rr=w13;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #64: @13 = @13' */
  /* #65: @7 = @6[7] */
  for (rr=(&w7), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #66: @8 = @6[6] */
  for (rr=(&w8), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #67: @8 = (-@8) */
  w8 = (- w8 );
  /* #68: @9 = @6[5] */
  for (rr=(&w9), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #69: @10 = @6[4] */
  for (rr=(&w10), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #70: @14 = horzcat(@7, @8, @9, @10) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #71: @14 = @14' */
  /* #72: @15 = horzcat(@11, @12, @13, @14) */
  rr=w15;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  /* #73: @19 = @15' */
  for (i=0, rr=w19, cs=w15; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #74: @17 = horzcat(@19, @16) */
  rr=w17;
  for (i=0, cs=w19; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<16; ++i) *rr++ = *cs++;
  /* #75: @20 = @17' */
  for (i=0, rr=w20, cs=w17; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #76: @21 = horzcat(@18, @20) */
  rr=w21;
  for (i=0, cs=w18; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w20; i<32; ++i) *rr++ = *cs++;
  /* #77: @22 = @21' */
  for (i=0, rr=w22, cs=w21; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #78: @7 = input[0][0] */
  w7 = arg[0] ? arg[0][0] : 0;
  /* #79: @8 = input[0][1] */
  w8 = arg[0] ? arg[0][1] : 0;
  /* #80: @9 = (-@8) */
  w9 = (- w8 );
  /* #81: @10 = input[0][2] */
  w10 = arg[0] ? arg[0][2] : 0;
  /* #82: @23 = (-@10) */
  w23 = (- w10 );
  /* #83: @24 = input[0][3] */
  w24 = arg[0] ? arg[0][3] : 0;
  /* #84: @25 = (-@24) */
  w25 = (- w24 );
  /* #85: @26 = input[0][4] */
  w26 = arg[0] ? arg[0][4] : 0;
  /* #86: @27 = input[0][5] */
  w27 = arg[0] ? arg[0][5] : 0;
  /* #87: @28 = (-@27) */
  w28 = (- w27 );
  /* #88: @29 = input[0][6] */
  w29 = arg[0] ? arg[0][6] : 0;
  /* #89: @30 = (-@29) */
  w30 = (- w29 );
  /* #90: @31 = input[0][7] */
  w31 = arg[0] ? arg[0][7] : 0;
  /* #91: @32 = (-@31) */
  w32 = (- w31 );
  /* #92: @33 = vertcat(@7, @9, @23, @25, @26, @28, @30, @32) */
  rr=w33;
  *rr++ = w7;
  *rr++ = w9;
  *rr++ = w23;
  *rr++ = w25;
  *rr++ = w26;
  *rr++ = w28;
  *rr++ = w30;
  *rr++ = w32;
  /* #93: @5 = mac(@22,@33,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w22+j, tt=w33+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #94: @9 = @5[0] */
  for (rr=(&w9), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #95: @4 = (@4<@9) */
  w4  = (w4<w9);
  /* #96: @33 = (@4?@5:0) */
  for (i=0, rr=w33, cs=w5; i<8; ++i) (*rr++)  = (w4?(*cs++):0);
  /* #97: @4 = (!@4) */
  w4 = (! w4 );
  /* #98: @5 = (-@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) *rr++ = (- *cs++ );
  /* #99: @5 = (@4?@5:0) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w4?(*cs++):0);
  /* #100: @33 = (@33+@5) */
  for (i=0, rr=w33, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #101: @3 = (@3-@33) */
  for (i=0, rr=w3, cs=w33; i<8; ++i) (*rr++) -= (*cs++);
  /* #102: @33 = @3' */
  casadi_copy(w3, 8, w33);
  /* #103: @22 = 
  [[1, 0, 0, 0, 0, 0, 0, 0], 
   [0, 1, 0, 0, 0, 0, 0, 0], 
   [0, 0, 1, 0, 0, 0, 0, 0], 
   [0, 0, 0, 1, 0, 0, 0, 0], 
   [0, 0, 0, 0, 2, 0, 0, 0], 
   [0, 0, 0, 0, 0, 2, 0, 0], 
   [0, 0, 0, 0, 0, 0, 2, 0], 
   [0, 0, 0, 0, 0, 0, 0, 2]] */
  casadi_copy(casadi_c1, 64, w22);
  /* #104: @2 = mac(@33,@22,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w33+j, tt=w22+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #105: @1 = mac(@2,@3,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w3+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #106: @0 = (@0*@1) */
  w0 *= w1;
  /* #107: @1 = 0 */
  w1 = 0.;
  /* #108: @11 = zeros(1x4) */
  casadi_clear(w11, 4);
  /* #109: @12 = @6[14:18] */
  for (rr=w12, ss=w6+14; ss!=w6+18; ss+=1) *rr++ = *ss;
  /* #110: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #111: @9 = input[1][1] */
  w9 = arg[1] ? arg[1][1] : 0;
  /* #112: @23 = input[1][2] */
  w23 = arg[1] ? arg[1][2] : 0;
  /* #113: @25 = input[1][3] */
  w25 = arg[1] ? arg[1][3] : 0;
  /* #114: @13 = vertcat(@4, @9, @23, @25) */
  rr=w13;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w23;
  *rr++ = w25;
  /* #115: @12 = (@12-@13) */
  for (i=0, rr=w12, cs=w13; i<4; ++i) (*rr++) -= (*cs++);
  /* #116: @13 = @12' */
  casadi_copy(w12, 4, w13);
  /* #117: @19 = 
  [[0.671141, 0, 0, 0], 
   [0, 600, 0, 0], 
   [0, 0, 600, 0], 
   [0, 0, 0, 600]] */
  casadi_copy(casadi_c2, 16, w19);
  /* #118: @11 = mac(@13,@19,@11) */
  for (i=0, rr=w11; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w13+j, tt=w19+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #119: @1 = mac(@11,@12,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w11+j, tt=w12+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #120: @0 = (@0+@1) */
  w0 += w1;
  /* #121: @1 = 0 */
  w1 = 0.;
  /* #122: @4 = input[0][8] */
  w4 = arg[0] ? arg[0][8] : 0;
  /* #123: @9 = input[0][9] */
  w9 = arg[0] ? arg[0][9] : 0;
  /* #124: @23 = input[0][10] */
  w23 = arg[0] ? arg[0][10] : 0;
  /* #125: @25 = input[0][11] */
  w25 = arg[0] ? arg[0][11] : 0;
  /* #126: @28 = input[0][12] */
  w28 = arg[0] ? arg[0][12] : 0;
  /* #127: @30 = input[0][13] */
  w30 = arg[0] ? arg[0][13] : 0;
  /* #128: @34 = vertcat(@7, @8, @10, @24, @26, @27, @29, @31, @4, @9, @23, @25, @28, @30) */
  rr=w34;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w24;
  *rr++ = w26;
  *rr++ = w27;
  *rr++ = w29;
  *rr++ = w31;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w23;
  *rr++ = w25;
  *rr++ = w28;
  *rr++ = w30;
  /* #129: @35 = @34[8:11] */
  for (rr=w35, ss=w34+8; ss!=w34+11; ss+=1) *rr++ = *ss;
  /* #130: @36 = @35' */
  casadi_copy(w35, 3, w36);
  /* #131: @1 = mac(@36,@35,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w36+j, tt=w35+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #132: @0 = (@0+@1) */
  w0 += w1;
  /* #133: @1 = 0 */
  w1 = 0.;
  /* #134: @36 = @34[11:14] */
  for (rr=w36, ss=w34+11; ss!=w34+14; ss+=1) *rr++ = *ss;
  /* #135: @35 = @36' */
  casadi_copy(w36, 3, w35);
  /* #136: @1 = mac(@35,@36,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w35+j, tt=w36+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #137: @0 = (@0+@1) */
  w0 += w1;
  /* #138: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 383;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
