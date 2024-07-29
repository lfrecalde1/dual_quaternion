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
  #define CASADI_PREFIX(ID) quadrotor_ocp_cost_ext_cost_e_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
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

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* quadrotor_ocp_cost_ext_cost_e_fun:(i0[14],i1[],i2[],i3[18])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, *w4=w+17, *w5=w+20, *w6=w+28, w7, w8, w9, w10, *w11=w+50, w12, w13, *w14=w+56, *w15=w+60, *w16=w+64, *w17=w+68, *w18=w+84, *w19=w+100, *w20=w+132, *w21=w+164, *w22=w+180, *w23=w+212, *w24=w+276, w25, w26, w27, w28, w29, w30, w31, w32, *w33=w+348, *w34=w+362, *w35=w+370, *w36=w+373, *w37=w+377, *w38=w+383, *w39=w+389;
  /* #0: @0 = 10 */
  w0 = 10.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x6) */
  casadi_clear(w2, 6);
  /* #3: @3 = 0.5 */
  w3 = 5.0000000000000000e-01;
  /* #4: @4 = all_2.22045e-16(3x1) */
  casadi_fill(w4, 3, 2.2204460492503131e-16);
  /* #5: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #6: @6 = input[3][0] */
  casadi_copy(arg[3], 18, w6);
  /* #7: @7 = @6[0] */
  for (rr=(&w7), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #8: @8 = @6[1] */
  for (rr=(&w8), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #9: @9 = @6[2] */
  for (rr=(&w9), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #10: @10 = @6[3] */
  for (rr=(&w10), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #11: @11 = horzcat(@7, @8, @9, @10) */
  rr=w11;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #12: @11 = @11' */
  /* #13: @7 = (-@8) */
  w7 = (- w8 );
  /* #14: @12 = @6[0] */
  for (rr=(&w12), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #15: @13 = (-@9) */
  w13 = (- w9 );
  /* #16: @14 = horzcat(@7, @12, @10, @13) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w10;
  *rr++ = w13;
  /* #17: @14 = @14' */
  /* #18: @10 = (-@10) */
  w10 = (- w10 );
  /* #19: @12 = @6[0] */
  for (rr=(&w12), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #20: @15 = horzcat(@13, @10, @12, @8) */
  rr=w15;
  *rr++ = w13;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w8;
  /* #21: @15 = @15' */
  /* #22: @13 = @6[0] */
  for (rr=(&w13), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #23: @16 = horzcat(@10, @9, @7, @13) */
  rr=w16;
  *rr++ = w10;
  *rr++ = w9;
  *rr++ = w7;
  *rr++ = w13;
  /* #24: @16 = @16' */
  /* #25: @17 = horzcat(@11, @14, @15, @16) */
  rr=w17;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #26: @18 = @17' */
  for (i=0, rr=w18, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #27: @17 = zeros(4x4) */
  casadi_clear(w17, 16);
  /* #28: @19 = horzcat(@18, @17) */
  rr=w19;
  for (i=0, cs=w18; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<16; ++i) *rr++ = *cs++;
  /* #29: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #30: @10 = @6[4] */
  for (rr=(&w10), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #31: @9 = @6[5] */
  for (rr=(&w9), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #32: @7 = @6[6] */
  for (rr=(&w7), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #33: @13 = @6[7] */
  for (rr=(&w13), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #34: @11 = horzcat(@10, @9, @7, @13) */
  rr=w11;
  *rr++ = w10;
  *rr++ = w9;
  *rr++ = w7;
  *rr++ = w13;
  /* #35: @11 = @11' */
  /* #36: @10 = (-@9) */
  w10 = (- w9 );
  /* #37: @12 = @6[4] */
  for (rr=(&w12), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #38: @8 = (-@7) */
  w8 = (- w7 );
  /* #39: @14 = horzcat(@10, @12, @13, @8) */
  rr=w14;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w8;
  /* #40: @14 = @14' */
  /* #41: @13 = (-@13) */
  w13 = (- w13 );
  /* #42: @12 = @6[4] */
  for (rr=(&w12), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #43: @15 = horzcat(@8, @13, @12, @9) */
  rr=w15;
  *rr++ = w8;
  *rr++ = w13;
  *rr++ = w12;
  *rr++ = w9;
  /* #44: @15 = @15' */
  /* #45: @8 = @6[4] */
  for (rr=(&w8), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #46: @16 = horzcat(@13, @7, @10, @8) */
  rr=w16;
  *rr++ = w13;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w8;
  /* #47: @16 = @16' */
  /* #48: @17 = horzcat(@11, @14, @15, @16) */
  rr=w17;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #49: @21 = @17' */
  for (i=0, rr=w21, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #50: @19 = horzcat(@21, @18) */
  rr=w19;
  for (i=0, cs=w21; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<16; ++i) *rr++ = *cs++;
  /* #51: @22 = @19' */
  for (i=0, rr=w22, cs=w19; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #52: @23 = horzcat(@20, @22) */
  rr=w23;
  for (i=0, cs=w20; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<32; ++i) *rr++ = *cs++;
  /* #53: @24 = @23' */
  for (i=0, rr=w24, cs=w23; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #54: @13 = input[0][0] */
  w13 = arg[0] ? arg[0][0] : 0;
  /* #55: @7 = input[0][1] */
  w7 = arg[0] ? arg[0][1] : 0;
  /* #56: @10 = input[0][2] */
  w10 = arg[0] ? arg[0][2] : 0;
  /* #57: @8 = input[0][3] */
  w8 = arg[0] ? arg[0][3] : 0;
  /* #58: @12 = input[0][4] */
  w12 = arg[0] ? arg[0][4] : 0;
  /* #59: @9 = input[0][5] */
  w9 = arg[0] ? arg[0][5] : 0;
  /* #60: @25 = input[0][6] */
  w25 = arg[0] ? arg[0][6] : 0;
  /* #61: @26 = input[0][7] */
  w26 = arg[0] ? arg[0][7] : 0;
  /* #62: @27 = input[0][8] */
  w27 = arg[0] ? arg[0][8] : 0;
  /* #63: @28 = input[0][9] */
  w28 = arg[0] ? arg[0][9] : 0;
  /* #64: @29 = input[0][10] */
  w29 = arg[0] ? arg[0][10] : 0;
  /* #65: @30 = input[0][11] */
  w30 = arg[0] ? arg[0][11] : 0;
  /* #66: @31 = input[0][12] */
  w31 = arg[0] ? arg[0][12] : 0;
  /* #67: @32 = input[0][13] */
  w32 = arg[0] ? arg[0][13] : 0;
  /* #68: @33 = vertcat(@13, @7, @10, @8, @12, @9, @25, @26, @27, @28, @29, @30, @31, @32) */
  rr=w33;
  *rr++ = w13;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  *rr++ = w26;
  *rr++ = w27;
  *rr++ = w28;
  *rr++ = w29;
  *rr++ = w30;
  *rr++ = w31;
  *rr++ = w32;
  /* #69: @34 = @33[:8] */
  for (rr=w34, ss=w33+0; ss!=w33+8; ss+=1) *rr++ = *ss;
  /* #70: @5 = mac(@24,@34,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w24+j, tt=w34+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #71: @35 = @5[1:4] */
  for (rr=w35, ss=w5+1; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #72: @4 = (@4+@35) */
  for (i=0, rr=w4, cs=w35; i<3; ++i) (*rr++) += (*cs++);
  /* #73: @13 = ||@4||_F */
  w13 = sqrt(casadi_dot(3, w4, w4));
  /* #74: @7 = @5[0] */
  for (rr=(&w7), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #75: @7 = atan2(@13,@7) */
  w7  = atan2(w13,w7);
  /* #76: @7 = (2.*@7) */
  w7 = (2.* w7 );
  /* #77: @3 = (@3*@7) */
  w3 *= w7;
  /* #78: @10 = @5[1] */
  for (rr=(&w10), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #79: @3 = (@3*@10) */
  w3 *= w10;
  /* #80: @3 = (@3/@13) */
  w3 /= w13;
  /* #81: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #82: @10 = (@10*@7) */
  w10 *= w7;
  /* #83: @8 = @5[2] */
  for (rr=(&w8), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #84: @10 = (@10*@8) */
  w10 *= w8;
  /* #85: @10 = (@10/@13) */
  w10 /= w13;
  /* #86: @8 = 0.5 */
  w8 = 5.0000000000000000e-01;
  /* #87: @8 = (@8*@7) */
  w8 *= w7;
  /* #88: @7 = @5[3] */
  for (rr=(&w7), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #89: @8 = (@8*@7) */
  w8 *= w7;
  /* #90: @8 = (@8/@13) */
  w8 /= w13;
  /* #91: @13 = 0.5 */
  w13 = 5.0000000000000000e-01;
  /* #92: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #93: @7 = @5[4] */
  for (rr=(&w7), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #94: @12 = @5[5] */
  for (rr=(&w12), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #95: @12 = (-@12) */
  w12 = (- w12 );
  /* #96: @9 = @5[6] */
  for (rr=(&w9), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #97: @9 = (-@9) */
  w9 = (- w9 );
  /* #98: @25 = @5[7] */
  for (rr=(&w25), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #99: @25 = (-@25) */
  w25 = (- w25 );
  /* #100: @14 = horzcat(@7, @12, @9, @25) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  /* #101: @14 = @14' */
  /* #102: @7 = @5[5] */
  for (rr=(&w7), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #103: @12 = @5[4] */
  for (rr=(&w12), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #104: @9 = @5[7] */
  for (rr=(&w9), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #105: @9 = (-@9) */
  w9 = (- w9 );
  /* #106: @25 = @5[6] */
  for (rr=(&w25), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #107: @15 = horzcat(@7, @12, @9, @25) */
  rr=w15;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  /* #108: @15 = @15' */
  /* #109: @7 = @5[6] */
  for (rr=(&w7), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #110: @12 = @5[7] */
  for (rr=(&w12), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #111: @9 = @5[4] */
  for (rr=(&w9), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #112: @25 = @5[5] */
  for (rr=(&w25), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #113: @25 = (-@25) */
  w25 = (- w25 );
  /* #114: @16 = horzcat(@7, @12, @9, @25) */
  rr=w16;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  /* #115: @16 = @16' */
  /* #116: @7 = @5[7] */
  for (rr=(&w7), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #117: @12 = @5[6] */
  for (rr=(&w12), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #118: @12 = (-@12) */
  w12 = (- w12 );
  /* #119: @9 = @5[5] */
  for (rr=(&w9), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #120: @25 = @5[4] */
  for (rr=(&w25), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #121: @36 = horzcat(@7, @12, @9, @25) */
  rr=w36;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  /* #122: @36 = @36' */
  /* #123: @21 = horzcat(@14, @15, @16, @36) */
  rr=w21;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w36; i<4; ++i) *rr++ = *cs++;
  /* #124: @18 = @21' */
  for (i=0, rr=w18, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #125: @18 = (2.*@18) */
  for (i=0, rr=w18, cs=w18; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #126: @7 = @5[0] */
  for (rr=(&w7), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #127: @12 = @5[1] */
  for (rr=(&w12), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #128: @12 = (-@12) */
  w12 = (- w12 );
  /* #129: @9 = @5[2] */
  for (rr=(&w9), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #130: @9 = (-@9) */
  w9 = (- w9 );
  /* #131: @25 = @5[3] */
  for (rr=(&w25), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #132: @25 = (-@25) */
  w25 = (- w25 );
  /* #133: @14 = vertcat(@7, @12, @9, @25) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  /* #134: @11 = mac(@18,@14,@11) */
  for (i=0, rr=w11; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w14+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #135: @7 = @11[1] */
  for (rr=(&w7), ss=w11+1; ss!=w11+2; ss+=1) *rr++ = *ss;
  /* #136: @13 = (@13*@7) */
  w13 *= w7;
  /* #137: @7 = 0.5 */
  w7 = 5.0000000000000000e-01;
  /* #138: @12 = @11[2] */
  for (rr=(&w12), ss=w11+2; ss!=w11+3; ss+=1) *rr++ = *ss;
  /* #139: @7 = (@7*@12) */
  w7 *= w12;
  /* #140: @12 = 0.5 */
  w12 = 5.0000000000000000e-01;
  /* #141: @9 = @11[3] */
  for (rr=(&w9), ss=w11+3; ss!=w11+4; ss+=1) *rr++ = *ss;
  /* #142: @12 = (@12*@9) */
  w12 *= w9;
  /* #143: @37 = vertcat(@3, @10, @8, @13, @7, @12) */
  rr=w37;
  *rr++ = w3;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w13;
  *rr++ = w7;
  *rr++ = w12;
  /* #144: @38 = @37' */
  casadi_copy(w37, 6, w38);
  /* #145: @39 = zeros(6x6) */
  casadi_clear(w39, 36);
  /* #146: @3 = 2 */
  w3 = 2.;
  /* #147: (@39[0] = @3) */
  for (rr=w39+0, ss=(&w3); rr!=w39+1; rr+=1) *rr = *ss++;
  /* #148: @3 = 2 */
  w3 = 2.;
  /* #149: (@39[7] = @3) */
  for (rr=w39+7, ss=(&w3); rr!=w39+8; rr+=1) *rr = *ss++;
  /* #150: @3 = 2 */
  w3 = 2.;
  /* #151: (@39[14] = @3) */
  for (rr=w39+14, ss=(&w3); rr!=w39+15; rr+=1) *rr = *ss++;
  /* #152: @3 = 1.6 */
  w3 = 1.6000000000000001e+00;
  /* #153: (@39[21] = @3) */
  for (rr=w39+21, ss=(&w3); rr!=w39+22; rr+=1) *rr = *ss++;
  /* #154: @3 = 1.6 */
  w3 = 1.6000000000000001e+00;
  /* #155: (@39[28] = @3) */
  for (rr=w39+28, ss=(&w3); rr!=w39+29; rr+=1) *rr = *ss++;
  /* #156: @3 = 1.6 */
  w3 = 1.6000000000000001e+00;
  /* #157: (@39[35] = @3) */
  for (rr=w39+35, ss=(&w3); rr!=w39+36; rr+=1) *rr = *ss++;
  /* #158: @2 = mac(@38,@39,@2) */
  for (i=0, rr=w2; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w38+j, tt=w39+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #159: @1 = mac(@2,@37,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w37+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #160: @0 = (@0*@1) */
  w0 *= w1;
  /* #161: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_e_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_e_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_e_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_e_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_e_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_e_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_e_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_e_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_e_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_e_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_ocp_cost_ext_cost_e_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_e_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_e_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_e_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_e_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_e_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 425;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
