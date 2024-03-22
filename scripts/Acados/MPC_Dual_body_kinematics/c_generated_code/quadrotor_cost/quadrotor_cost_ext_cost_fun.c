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
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
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

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[64] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 5.0000000000000000e-01, 0., 0., 0., 0., 0., 0., 0., 0., 5.0000000000000000e-01, 0., 0., 0., 0., 0., 0., 0., 0., 5.0000000000000000e-01};
static const casadi_real casadi_c1[36] = {5., 0., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 5., 0., 0., 0., 0., 0., 0., 5.};

/* quadrotor_cost_ext_cost_fun:(i0[8],i1[6],i2[],i3[14])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, w4, *w5=w+20, w6, *w7=w+24, *w8=w+32, w9, w10, w11, w12, *w13=w+50, w14, w15, *w16=w+56, *w17=w+60, *w18=w+64, *w19=w+68, *w20=w+84, *w21=w+100, *w22=w+132, *w23=w+164, *w24=w+180, *w25=w+212, *w26=w+276, w27, w28, *w29=w+342, *w30=w+350, *w31=w+353, *w32=w+357, *w33=w+363, *w34=w+369, *w35=w+375;
  /* #0: @0 = 10 */
  w0 = 10.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #3: @3 = 0 */
  w3 = 0.;
  /* #4: @4 = 0.5 */
  w4 = 5.0000000000000000e-01;
  /* #5: @5 = all_2.22045e-16(3x1) */
  casadi_fill(w5, 3, 2.2204460492503131e-16);
  /* #6: @6 = 0 */
  w6 = 0.;
  /* #7: @7 = zeros(8x1) */
  casadi_clear(w7, 8);
  /* #8: @8 = input[3][0] */
  casadi_copy(arg[3], 14, w8);
  /* #9: @9 = @8[0] */
  for (rr=(&w9), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #10: @10 = @8[1] */
  for (rr=(&w10), ss=w8+1; ss!=w8+2; ss+=1) *rr++ = *ss;
  /* #11: @11 = @8[2] */
  for (rr=(&w11), ss=w8+2; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #12: @12 = @8[3] */
  for (rr=(&w12), ss=w8+3; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #13: @13 = horzcat(@9, @10, @11, @12) */
  rr=w13;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  /* #14: @13 = @13' */
  /* #15: @9 = (-@10) */
  w9 = (- w10 );
  /* #16: @14 = @8[0] */
  for (rr=(&w14), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #17: @15 = (-@11) */
  w15 = (- w11 );
  /* #18: @16 = horzcat(@9, @14, @12, @15) */
  rr=w16;
  *rr++ = w9;
  *rr++ = w14;
  *rr++ = w12;
  *rr++ = w15;
  /* #19: @16 = @16' */
  /* #20: @12 = (-@12) */
  w12 = (- w12 );
  /* #21: @14 = @8[0] */
  for (rr=(&w14), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #22: @17 = horzcat(@15, @12, @14, @10) */
  rr=w17;
  *rr++ = w15;
  *rr++ = w12;
  *rr++ = w14;
  *rr++ = w10;
  /* #23: @17 = @17' */
  /* #24: @15 = @8[0] */
  for (rr=(&w15), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #25: @18 = horzcat(@12, @11, @9, @15) */
  rr=w18;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w9;
  *rr++ = w15;
  /* #26: @18 = @18' */
  /* #27: @19 = horzcat(@13, @16, @17, @18) */
  rr=w19;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  /* #28: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #29: @19 = zeros(4x4) */
  casadi_clear(w19, 16);
  /* #30: @21 = horzcat(@20, @19) */
  rr=w21;
  for (i=0, cs=w20; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w19; i<16; ++i) *rr++ = *cs++;
  /* #31: @22 = @21' */
  for (i=0, rr=w22, cs=w21; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #32: @12 = @8[4] */
  for (rr=(&w12), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #33: @11 = @8[5] */
  for (rr=(&w11), ss=w8+5; ss!=w8+6; ss+=1) *rr++ = *ss;
  /* #34: @9 = @8[6] */
  for (rr=(&w9), ss=w8+6; ss!=w8+7; ss+=1) *rr++ = *ss;
  /* #35: @15 = @8[7] */
  for (rr=(&w15), ss=w8+7; ss!=w8+8; ss+=1) *rr++ = *ss;
  /* #36: @13 = horzcat(@12, @11, @9, @15) */
  rr=w13;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w9;
  *rr++ = w15;
  /* #37: @13 = @13' */
  /* #38: @12 = (-@11) */
  w12 = (- w11 );
  /* #39: @14 = @8[4] */
  for (rr=(&w14), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #40: @10 = (-@9) */
  w10 = (- w9 );
  /* #41: @16 = horzcat(@12, @14, @15, @10) */
  rr=w16;
  *rr++ = w12;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w10;
  /* #42: @16 = @16' */
  /* #43: @15 = (-@15) */
  w15 = (- w15 );
  /* #44: @14 = @8[4] */
  for (rr=(&w14), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #45: @17 = horzcat(@10, @15, @14, @11) */
  rr=w17;
  *rr++ = w10;
  *rr++ = w15;
  *rr++ = w14;
  *rr++ = w11;
  /* #46: @17 = @17' */
  /* #47: @10 = @8[4] */
  for (rr=(&w10), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #48: @18 = horzcat(@15, @9, @12, @10) */
  rr=w18;
  *rr++ = w15;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w10;
  /* #49: @18 = @18' */
  /* #50: @19 = horzcat(@13, @16, @17, @18) */
  rr=w19;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  /* #51: @23 = @19' */
  for (i=0, rr=w23, cs=w19; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #52: @21 = horzcat(@23, @20) */
  rr=w21;
  for (i=0, cs=w23; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w20; i<16; ++i) *rr++ = *cs++;
  /* #53: @24 = @21' */
  for (i=0, rr=w24, cs=w21; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #54: @25 = horzcat(@22, @24) */
  rr=w25;
  for (i=0, cs=w22; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w24; i<32; ++i) *rr++ = *cs++;
  /* #55: @26 = @25' */
  for (i=0, rr=w26, cs=w25; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #56: @15 = input[0][0] */
  w15 = arg[0] ? arg[0][0] : 0;
  /* #57: @9 = input[0][1] */
  w9 = arg[0] ? arg[0][1] : 0;
  /* #58: @12 = input[0][2] */
  w12 = arg[0] ? arg[0][2] : 0;
  /* #59: @10 = input[0][3] */
  w10 = arg[0] ? arg[0][3] : 0;
  /* #60: @14 = input[0][4] */
  w14 = arg[0] ? arg[0][4] : 0;
  /* #61: @11 = input[0][5] */
  w11 = arg[0] ? arg[0][5] : 0;
  /* #62: @27 = input[0][6] */
  w27 = arg[0] ? arg[0][6] : 0;
  /* #63: @28 = input[0][7] */
  w28 = arg[0] ? arg[0][7] : 0;
  /* #64: @29 = vertcat(@15, @9, @12, @10, @14, @11, @27, @28) */
  rr=w29;
  *rr++ = w15;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  *rr++ = w28;
  /* #65: @7 = mac(@26,@29,@7) */
  for (i=0, rr=w7; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w26+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #66: @15 = @7[0] */
  for (rr=(&w15), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #67: @6 = (@6<@15) */
  w6  = (w6<w15);
  /* #68: @29 = (@6?@7:0) */
  for (i=0, rr=w29, cs=w7; i<8; ++i) (*rr++)  = (w6?(*cs++):0);
  /* #69: @6 = (!@6) */
  w6 = (! w6 );
  /* #70: @7 = (-@7) */
  for (i=0, rr=w7, cs=w7; i<8; ++i) *rr++ = (- *cs++ );
  /* #71: @7 = (@6?@7:0) */
  for (i=0, rr=w7, cs=w7; i<8; ++i) (*rr++)  = (w6?(*cs++):0);
  /* #72: @29 = (@29+@7) */
  for (i=0, rr=w29, cs=w7; i<8; ++i) (*rr++) += (*cs++);
  /* #73: @30 = @29[1:4] */
  for (rr=w30, ss=w29+1; ss!=w29+4; ss+=1) *rr++ = *ss;
  /* #74: @5 = (@5+@30) */
  for (i=0, rr=w5, cs=w30; i<3; ++i) (*rr++) += (*cs++);
  /* #75: @6 = ||@5||_F */
  w6 = sqrt(casadi_dot(3, w5, w5));
  /* #76: @15 = @29[0] */
  for (rr=(&w15), ss=w29+0; ss!=w29+1; ss+=1) *rr++ = *ss;
  /* #77: @15 = atan2(@6,@15) */
  w15  = atan2(w6,w15);
  /* #78: @4 = (@4*@15) */
  w4 *= w15;
  /* #79: @9 = @29[1] */
  for (rr=(&w9), ss=w29+1; ss!=w29+2; ss+=1) *rr++ = *ss;
  /* #80: @4 = (@4*@9) */
  w4 *= w9;
  /* #81: @4 = (@4/@6) */
  w4 /= w6;
  /* #82: @9 = 0.5 */
  w9 = 5.0000000000000000e-01;
  /* #83: @9 = (@9*@15) */
  w9 *= w15;
  /* #84: @12 = @29[2] */
  for (rr=(&w12), ss=w29+2; ss!=w29+3; ss+=1) *rr++ = *ss;
  /* #85: @9 = (@9*@12) */
  w9 *= w12;
  /* #86: @9 = (@9/@6) */
  w9 /= w6;
  /* #87: @12 = 0.5 */
  w12 = 5.0000000000000000e-01;
  /* #88: @12 = (@12*@15) */
  w12 *= w15;
  /* #89: @15 = @29[3] */
  for (rr=(&w15), ss=w29+3; ss!=w29+4; ss+=1) *rr++ = *ss;
  /* #90: @12 = (@12*@15) */
  w12 *= w15;
  /* #91: @12 = (@12/@6) */
  w12 /= w6;
  /* #92: @6 = 0 */
  w6 = 0.;
  /* #93: @15 = 0.5 */
  w15 = 5.0000000000000000e-01;
  /* #94: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #95: @10 = @29[4] */
  for (rr=(&w10), ss=w29+4; ss!=w29+5; ss+=1) *rr++ = *ss;
  /* #96: @14 = @29[5] */
  for (rr=(&w14), ss=w29+5; ss!=w29+6; ss+=1) *rr++ = *ss;
  /* #97: @14 = (-@14) */
  w14 = (- w14 );
  /* #98: @11 = @29[6] */
  for (rr=(&w11), ss=w29+6; ss!=w29+7; ss+=1) *rr++ = *ss;
  /* #99: @11 = (-@11) */
  w11 = (- w11 );
  /* #100: @27 = @29[7] */
  for (rr=(&w27), ss=w29+7; ss!=w29+8; ss+=1) *rr++ = *ss;
  /* #101: @27 = (-@27) */
  w27 = (- w27 );
  /* #102: @16 = horzcat(@10, @14, @11, @27) */
  rr=w16;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  /* #103: @16 = @16' */
  /* #104: @10 = @29[5] */
  for (rr=(&w10), ss=w29+5; ss!=w29+6; ss+=1) *rr++ = *ss;
  /* #105: @14 = @29[4] */
  for (rr=(&w14), ss=w29+4; ss!=w29+5; ss+=1) *rr++ = *ss;
  /* #106: @11 = @29[7] */
  for (rr=(&w11), ss=w29+7; ss!=w29+8; ss+=1) *rr++ = *ss;
  /* #107: @11 = (-@11) */
  w11 = (- w11 );
  /* #108: @27 = @29[6] */
  for (rr=(&w27), ss=w29+6; ss!=w29+7; ss+=1) *rr++ = *ss;
  /* #109: @17 = horzcat(@10, @14, @11, @27) */
  rr=w17;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  /* #110: @17 = @17' */
  /* #111: @10 = @29[6] */
  for (rr=(&w10), ss=w29+6; ss!=w29+7; ss+=1) *rr++ = *ss;
  /* #112: @14 = @29[7] */
  for (rr=(&w14), ss=w29+7; ss!=w29+8; ss+=1) *rr++ = *ss;
  /* #113: @11 = @29[4] */
  for (rr=(&w11), ss=w29+4; ss!=w29+5; ss+=1) *rr++ = *ss;
  /* #114: @27 = @29[5] */
  for (rr=(&w27), ss=w29+5; ss!=w29+6; ss+=1) *rr++ = *ss;
  /* #115: @27 = (-@27) */
  w27 = (- w27 );
  /* #116: @18 = horzcat(@10, @14, @11, @27) */
  rr=w18;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  /* #117: @18 = @18' */
  /* #118: @10 = @29[7] */
  for (rr=(&w10), ss=w29+7; ss!=w29+8; ss+=1) *rr++ = *ss;
  /* #119: @14 = @29[6] */
  for (rr=(&w14), ss=w29+6; ss!=w29+7; ss+=1) *rr++ = *ss;
  /* #120: @14 = (-@14) */
  w14 = (- w14 );
  /* #121: @11 = @29[5] */
  for (rr=(&w11), ss=w29+5; ss!=w29+6; ss+=1) *rr++ = *ss;
  /* #122: @27 = @29[4] */
  for (rr=(&w27), ss=w29+4; ss!=w29+5; ss+=1) *rr++ = *ss;
  /* #123: @31 = horzcat(@10, @14, @11, @27) */
  rr=w31;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  /* #124: @31 = @31' */
  /* #125: @23 = horzcat(@16, @17, @18, @31) */
  rr=w23;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w31; i<4; ++i) *rr++ = *cs++;
  /* #126: @20 = @23' */
  for (i=0, rr=w20, cs=w23; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #127: @20 = (2.*@20) */
  for (i=0, rr=w20, cs=w20; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #128: @10 = @29[0] */
  for (rr=(&w10), ss=w29+0; ss!=w29+1; ss+=1) *rr++ = *ss;
  /* #129: @14 = @29[1] */
  for (rr=(&w14), ss=w29+1; ss!=w29+2; ss+=1) *rr++ = *ss;
  /* #130: @14 = (-@14) */
  w14 = (- w14 );
  /* #131: @11 = @29[2] */
  for (rr=(&w11), ss=w29+2; ss!=w29+3; ss+=1) *rr++ = *ss;
  /* #132: @11 = (-@11) */
  w11 = (- w11 );
  /* #133: @27 = @29[3] */
  for (rr=(&w27), ss=w29+3; ss!=w29+4; ss+=1) *rr++ = *ss;
  /* #134: @27 = (-@27) */
  w27 = (- w27 );
  /* #135: @16 = vertcat(@10, @14, @11, @27) */
  rr=w16;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  /* #136: @13 = mac(@20,@16,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w20+j, tt=w16+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #137: @10 = @13[1] */
  for (rr=(&w10), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #138: @15 = (@15*@10) */
  w15 *= w10;
  /* #139: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #140: @14 = @13[2] */
  for (rr=(&w14), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #141: @10 = (@10*@14) */
  w10 *= w14;
  /* #142: @14 = 0.5 */
  w14 = 5.0000000000000000e-01;
  /* #143: @11 = @13[3] */
  for (rr=(&w11), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #144: @14 = (@14*@11) */
  w14 *= w11;
  /* #145: @29 = vertcat(@3, @4, @9, @12, @6, @15, @10, @14) */
  rr=w29;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w6;
  *rr++ = w15;
  *rr++ = w10;
  *rr++ = w14;
  /* #146: @7 = @29' */
  casadi_copy(w29, 8, w7);
  /* #147: @26 = 
  [[0, 0, 0, 0, 0, 0, 0, 0], 
   [0, 2, 0, 0, 0, 0, 0, 0], 
   [0, 0, 2, 0, 0, 0, 0, 0], 
   [0, 0, 0, 2, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0, 0.5, 0, 0], 
   [0, 0, 0, 0, 0, 0, 0.5, 0], 
   [0, 0, 0, 0, 0, 0, 0, 0.5]] */
  casadi_copy(casadi_c0, 64, w26);
  /* #148: @2 = mac(@7,@26,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w26+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #149: @1 = mac(@2,@29,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #150: @0 = (@0*@1) */
  w0 *= w1;
  /* #151: @1 = 0 */
  w1 = 0.;
  /* #152: @32 = zeros(1x6) */
  casadi_clear(w32, 6);
  /* #153: @33 = @8[8:14] */
  for (rr=w33, ss=w8+8; ss!=w8+14; ss+=1) *rr++ = *ss;
  /* #154: @3 = input[1][0] */
  w3 = arg[1] ? arg[1][0] : 0;
  /* #155: @4 = input[1][1] */
  w4 = arg[1] ? arg[1][1] : 0;
  /* #156: @9 = input[1][2] */
  w9 = arg[1] ? arg[1][2] : 0;
  /* #157: @12 = input[1][3] */
  w12 = arg[1] ? arg[1][3] : 0;
  /* #158: @6 = input[1][4] */
  w6 = arg[1] ? arg[1][4] : 0;
  /* #159: @15 = input[1][5] */
  w15 = arg[1] ? arg[1][5] : 0;
  /* #160: @34 = vertcat(@3, @4, @9, @12, @6, @15) */
  rr=w34;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w6;
  *rr++ = w15;
  /* #161: @33 = (@33-@34) */
  for (i=0, rr=w33, cs=w34; i<6; ++i) (*rr++) -= (*cs++);
  /* #162: @34 = @33' */
  casadi_copy(w33, 6, w34);
  /* #163: @35 = 
  [[5, 0, 0, 0, 0, 0], 
   [0, 5, 0, 0, 0, 0], 
   [0, 0, 5, 0, 0, 0], 
   [0, 0, 0, 5, 0, 0], 
   [0, 0, 0, 0, 5, 0], 
   [0, 0, 0, 0, 0, 5]] */
  casadi_copy(casadi_c1, 36, w35);
  /* #164: @32 = mac(@34,@35,@32) */
  for (i=0, rr=w32; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w34+j, tt=w35+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #165: @1 = mac(@32,@33,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w32+j, tt=w33+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #166: @0 = (@0+@1) */
  w0 += w1;
  /* #167: output[0][0] = @0 */
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
  if (sz_arg) *sz_arg = 12;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 411;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
