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
  #define CASADI_PREFIX(ID) quadrotor_ocp_cost_ext_cost_0_fun_ ## ID
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

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[36] = {2., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00, 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00, 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00};
static const casadi_real casadi_c1[16] = {6.7114093959731547e-01, 0., 0., 0., 0., 600., 0., 0., 0., 0., 600., 0., 0., 0., 0., 600.};

/* quadrotor_ocp_cost_ext_cost_0_fun:(i0[14],i1[4],i2[],i3[18])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, *w4=w+17, *w5=w+20, w6, w7, w8, w9, w10, w11, w12, *w13=w+35, *w14=w+39, *w15=w+43, *w16=w+47, *w17=w+51, *w18=w+67, *w19=w+83, *w20=w+115, *w21=w+147, *w22=w+163, *w23=w+195, *w24=w+259, *w25=w+323, w26, *w27=w+342, *w28=w+350, *w29=w+353, *w30=w+357, *w31=w+363, *w32=w+369;
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
  /* #6: @6 = input[0][0] */
  w6 = arg[0] ? arg[0][0] : 0;
  /* #7: @7 = input[0][1] */
  w7 = arg[0] ? arg[0][1] : 0;
  /* #8: @8 = (-@7) */
  w8 = (- w7 );
  /* #9: @9 = input[0][2] */
  w9 = arg[0] ? arg[0][2] : 0;
  /* #10: @10 = (-@9) */
  w10 = (- w9 );
  /* #11: @11 = input[0][3] */
  w11 = arg[0] ? arg[0][3] : 0;
  /* #12: @12 = (-@11) */
  w12 = (- w11 );
  /* #13: @13 = horzcat(@6, @8, @10, @12) */
  rr=w13;
  *rr++ = w6;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w12;
  /* #14: @13 = @13' */
  /* #15: @8 = (-@11) */
  w8 = (- w11 );
  /* #16: @14 = horzcat(@7, @6, @8, @9) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w6;
  *rr++ = w8;
  *rr++ = w9;
  /* #17: @14 = @14' */
  /* #18: @8 = (-@7) */
  w8 = (- w7 );
  /* #19: @15 = horzcat(@9, @11, @6, @8) */
  rr=w15;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w6;
  *rr++ = w8;
  /* #20: @15 = @15' */
  /* #21: @9 = (-@9) */
  w9 = (- w9 );
  /* #22: @16 = horzcat(@11, @9, @7, @6) */
  rr=w16;
  *rr++ = w11;
  *rr++ = w9;
  *rr++ = w7;
  *rr++ = w6;
  /* #23: @16 = @16' */
  /* #24: @17 = horzcat(@13, @14, @15, @16) */
  rr=w17;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #25: @18 = @17' */
  for (i=0, rr=w18, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #26: @17 = zeros(4x4) */
  casadi_clear(w17, 16);
  /* #27: @19 = horzcat(@18, @17) */
  rr=w19;
  for (i=0, cs=w18; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<16; ++i) *rr++ = *cs++;
  /* #28: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #29: @11 = input[0][4] */
  w11 = arg[0] ? arg[0][4] : 0;
  /* #30: @9 = input[0][5] */
  w9 = arg[0] ? arg[0][5] : 0;
  /* #31: @7 = (-@9) */
  w7 = (- w9 );
  /* #32: @6 = input[0][6] */
  w6 = arg[0] ? arg[0][6] : 0;
  /* #33: @8 = (-@6) */
  w8 = (- w6 );
  /* #34: @10 = input[0][7] */
  w10 = arg[0] ? arg[0][7] : 0;
  /* #35: @12 = (-@10) */
  w12 = (- w10 );
  /* #36: @13 = horzcat(@11, @7, @8, @12) */
  rr=w13;
  *rr++ = w11;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  /* #37: @13 = @13' */
  /* #38: @7 = (-@10) */
  w7 = (- w10 );
  /* #39: @14 = horzcat(@9, @11, @7, @6) */
  rr=w14;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w7;
  *rr++ = w6;
  /* #40: @14 = @14' */
  /* #41: @7 = (-@9) */
  w7 = (- w9 );
  /* #42: @15 = horzcat(@6, @10, @11, @7) */
  rr=w15;
  *rr++ = w6;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w7;
  /* #43: @15 = @15' */
  /* #44: @6 = (-@6) */
  w6 = (- w6 );
  /* #45: @16 = horzcat(@10, @6, @9, @11) */
  rr=w16;
  *rr++ = w10;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w11;
  /* #46: @16 = @16' */
  /* #47: @17 = horzcat(@13, @14, @15, @16) */
  rr=w17;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #48: @21 = @17' */
  for (i=0, rr=w21, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #49: @19 = horzcat(@21, @18) */
  rr=w19;
  for (i=0, cs=w21; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<16; ++i) *rr++ = *cs++;
  /* #50: @22 = @19' */
  for (i=0, rr=w22, cs=w19; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #51: @23 = horzcat(@20, @22) */
  rr=w23;
  for (i=0, cs=w20; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<32; ++i) *rr++ = *cs++;
  /* #52: @24 = @23' */
  for (i=0, rr=w24, cs=w23; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #53: @25 = input[3][0] */
  casadi_copy(arg[3], 18, w25);
  /* #54: @10 = @25[0] */
  for (rr=(&w10), ss=w25+0; ss!=w25+1; ss+=1) *rr++ = *ss;
  /* #55: @6 = @25[1] */
  for (rr=(&w6), ss=w25+1; ss!=w25+2; ss+=1) *rr++ = *ss;
  /* #56: @6 = (-@6) */
  w6 = (- w6 );
  /* #57: @9 = @25[2] */
  for (rr=(&w9), ss=w25+2; ss!=w25+3; ss+=1) *rr++ = *ss;
  /* #58: @9 = (-@9) */
  w9 = (- w9 );
  /* #59: @11 = @25[3] */
  for (rr=(&w11), ss=w25+3; ss!=w25+4; ss+=1) *rr++ = *ss;
  /* #60: @11 = (-@11) */
  w11 = (- w11 );
  /* #61: @7 = @25[4] */
  for (rr=(&w7), ss=w25+4; ss!=w25+5; ss+=1) *rr++ = *ss;
  /* #62: @8 = @25[5] */
  for (rr=(&w8), ss=w25+5; ss!=w25+6; ss+=1) *rr++ = *ss;
  /* #63: @8 = (-@8) */
  w8 = (- w8 );
  /* #64: @12 = @25[6] */
  for (rr=(&w12), ss=w25+6; ss!=w25+7; ss+=1) *rr++ = *ss;
  /* #65: @12 = (-@12) */
  w12 = (- w12 );
  /* #66: @26 = @25[7] */
  for (rr=(&w26), ss=w25+7; ss!=w25+8; ss+=1) *rr++ = *ss;
  /* #67: @26 = (-@26) */
  w26 = (- w26 );
  /* #68: @27 = vertcat(@10, @6, @9, @11, @7, @8, @12, @26) */
  rr=w27;
  *rr++ = w10;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  *rr++ = w26;
  /* #69: @5 = mac(@24,@27,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w24+j, tt=w27+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #70: @28 = @5[1:4] */
  for (rr=w28, ss=w5+1; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #71: @4 = (@4+@28) */
  for (i=0, rr=w4, cs=w28; i<3; ++i) (*rr++) += (*cs++);
  /* #72: @10 = ||@4||_F */
  w10 = sqrt(casadi_dot(3, w4, w4));
  /* #73: @6 = @5[0] */
  for (rr=(&w6), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #74: @6 = atan2(@10,@6) */
  w6  = atan2(w10,w6);
  /* #75: @3 = (@3*@6) */
  w3 *= w6;
  /* #76: @9 = @5[1] */
  for (rr=(&w9), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #77: @3 = (@3*@9) */
  w3 *= w9;
  /* #78: @3 = (@3/@10) */
  w3 /= w10;
  /* #79: @9 = 0.5 */
  w9 = 5.0000000000000000e-01;
  /* #80: @9 = (@9*@6) */
  w9 *= w6;
  /* #81: @11 = @5[2] */
  for (rr=(&w11), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #82: @9 = (@9*@11) */
  w9 *= w11;
  /* #83: @9 = (@9/@10) */
  w9 /= w10;
  /* #84: @11 = 0.5 */
  w11 = 5.0000000000000000e-01;
  /* #85: @11 = (@11*@6) */
  w11 *= w6;
  /* #86: @6 = @5[3] */
  for (rr=(&w6), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #87: @11 = (@11*@6) */
  w11 *= w6;
  /* #88: @11 = (@11/@10) */
  w11 /= w10;
  /* #89: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #90: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #91: @6 = @5[4] */
  for (rr=(&w6), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #92: @7 = @5[5] */
  for (rr=(&w7), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #93: @7 = (-@7) */
  w7 = (- w7 );
  /* #94: @8 = @5[6] */
  for (rr=(&w8), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #95: @8 = (-@8) */
  w8 = (- w8 );
  /* #96: @12 = @5[7] */
  for (rr=(&w12), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #97: @12 = (-@12) */
  w12 = (- w12 );
  /* #98: @14 = horzcat(@6, @7, @8, @12) */
  rr=w14;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  /* #99: @14 = @14' */
  /* #100: @6 = @5[5] */
  for (rr=(&w6), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #101: @7 = @5[4] */
  for (rr=(&w7), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #102: @8 = @5[7] */
  for (rr=(&w8), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #103: @8 = (-@8) */
  w8 = (- w8 );
  /* #104: @12 = @5[6] */
  for (rr=(&w12), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #105: @15 = horzcat(@6, @7, @8, @12) */
  rr=w15;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  /* #106: @15 = @15' */
  /* #107: @6 = @5[6] */
  for (rr=(&w6), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #108: @7 = @5[7] */
  for (rr=(&w7), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #109: @8 = @5[4] */
  for (rr=(&w8), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #110: @12 = @5[5] */
  for (rr=(&w12), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #111: @12 = (-@12) */
  w12 = (- w12 );
  /* #112: @16 = horzcat(@6, @7, @8, @12) */
  rr=w16;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  /* #113: @16 = @16' */
  /* #114: @6 = @5[7] */
  for (rr=(&w6), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #115: @7 = @5[6] */
  for (rr=(&w7), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #116: @7 = (-@7) */
  w7 = (- w7 );
  /* #117: @8 = @5[5] */
  for (rr=(&w8), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #118: @12 = @5[4] */
  for (rr=(&w12), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #119: @29 = horzcat(@6, @7, @8, @12) */
  rr=w29;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  /* #120: @29 = @29' */
  /* #121: @21 = horzcat(@14, @15, @16, @29) */
  rr=w21;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w29; i<4; ++i) *rr++ = *cs++;
  /* #122: @18 = @21' */
  for (i=0, rr=w18, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #123: @18 = (2.*@18) */
  for (i=0, rr=w18, cs=w18; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #124: @6 = @5[0] */
  for (rr=(&w6), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #125: @7 = @5[1] */
  for (rr=(&w7), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #126: @7 = (-@7) */
  w7 = (- w7 );
  /* #127: @8 = @5[2] */
  for (rr=(&w8), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #128: @8 = (-@8) */
  w8 = (- w8 );
  /* #129: @12 = @5[3] */
  for (rr=(&w12), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #130: @12 = (-@12) */
  w12 = (- w12 );
  /* #131: @14 = vertcat(@6, @7, @8, @12) */
  rr=w14;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w12;
  /* #132: @13 = mac(@18,@14,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w14+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #133: @6 = @13[1] */
  for (rr=(&w6), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #134: @10 = (@10*@6) */
  w10 *= w6;
  /* #135: @6 = 0.5 */
  w6 = 5.0000000000000000e-01;
  /* #136: @7 = @13[2] */
  for (rr=(&w7), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #137: @6 = (@6*@7) */
  w6 *= w7;
  /* #138: @7 = 0.5 */
  w7 = 5.0000000000000000e-01;
  /* #139: @8 = @13[3] */
  for (rr=(&w8), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #140: @7 = (@7*@8) */
  w7 *= w8;
  /* #141: @30 = vertcat(@3, @9, @11, @10, @6, @7) */
  rr=w30;
  *rr++ = w3;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w6;
  *rr++ = w7;
  /* #142: @31 = @30' */
  casadi_copy(w30, 6, w31);
  /* #143: @32 = 
  [[2, 0, 0, 0, 0, 0], 
   [0, 2, 0, 0, 0, 0], 
   [0, 0, 2, 0, 0, 0], 
   [0, 0, 0, 1.6, 0, 0], 
   [0, 0, 0, 0, 1.6, 0], 
   [0, 0, 0, 0, 0, 1.6]] */
  casadi_copy(casadi_c0, 36, w32);
  /* #144: @2 = mac(@31,@32,@2) */
  for (i=0, rr=w2; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w31+j, tt=w32+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #145: @1 = mac(@2,@30,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w30+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #146: @0 = (@0*@1) */
  w0 *= w1;
  /* #147: @1 = 0 */
  w1 = 0.;
  /* #148: @13 = zeros(1x4) */
  casadi_clear(w13, 4);
  /* #149: @14 = @25[14:18] */
  for (rr=w14, ss=w25+14; ss!=w25+18; ss+=1) *rr++ = *ss;
  /* #150: @3 = input[1][0] */
  w3 = arg[1] ? arg[1][0] : 0;
  /* #151: @9 = input[1][1] */
  w9 = arg[1] ? arg[1][1] : 0;
  /* #152: @11 = input[1][2] */
  w11 = arg[1] ? arg[1][2] : 0;
  /* #153: @10 = input[1][3] */
  w10 = arg[1] ? arg[1][3] : 0;
  /* #154: @15 = vertcat(@3, @9, @11, @10) */
  rr=w15;
  *rr++ = w3;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w10;
  /* #155: @14 = (@14-@15) */
  for (i=0, rr=w14, cs=w15; i<4; ++i) (*rr++) -= (*cs++);
  /* #156: @15 = @14' */
  casadi_copy(w14, 4, w15);
  /* #157: @18 = 
  [[0.671141, 0, 0, 0], 
   [0, 600, 0, 0], 
   [0, 0, 600, 0], 
   [0, 0, 0, 600]] */
  casadi_copy(casadi_c1, 16, w18);
  /* #158: @13 = mac(@15,@18,@13) */
  for (i=0, rr=w13; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w15+j, tt=w18+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #159: @1 = mac(@13,@14,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w13+j, tt=w14+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #160: @0 = (@0+@1) */
  w0 += w1;
  /* #161: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_0_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_0_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_0_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_0_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_0_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_0_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_0_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_0_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_0_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_0_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_ocp_cost_ext_cost_0_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_0_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_0_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_0_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_0_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_0_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 12;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 405;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
