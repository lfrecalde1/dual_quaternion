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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_0_fun_jac_ ## ID
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

/* quadrotor_cost_ext_cost_0_fun_jac:(i0[14],i1[4],i2[],i3[18])->(o0,o1[18]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, *w3=w+18, w4, *w5=w+27, *w6=w+35, w7, w8, w9, w10, *w11=w+57, *w12=w+61, *w13=w+65, *w14=w+69, *w15=w+73, *w16=w+89, *w17=w+105, *w18=w+137, *w19=w+169, *w20=w+185, *w21=w+217, *w22=w+281, w23, w24, w25, w26, w27, w28, w29, w30, w31, w32, *w33=w+355, w34, *w35=w+364, *w36=w+378, *w37=w+381, *w38=w+384, *w39=w+387, w40, w41, w42, w43;
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
  /* #97: @9 = (!@4) */
  w9 = (! w4 );
  /* #98: @5 = (-@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) *rr++ = (- *cs++ );
  /* #99: @5 = (@9?@5:0) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w9?(*cs++):0);
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
  /* #110: @23 = input[1][0] */
  w23 = arg[1] ? arg[1][0] : 0;
  /* #111: @25 = input[1][1] */
  w25 = arg[1] ? arg[1][1] : 0;
  /* #112: @28 = input[1][2] */
  w28 = arg[1] ? arg[1][2] : 0;
  /* #113: @30 = input[1][3] */
  w30 = arg[1] ? arg[1][3] : 0;
  /* #114: @13 = vertcat(@23, @25, @28, @30) */
  rr=w13;
  *rr++ = w23;
  *rr++ = w25;
  *rr++ = w28;
  *rr++ = w30;
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
  /* #122: @23 = input[0][8] */
  w23 = arg[0] ? arg[0][8] : 0;
  /* #123: @25 = input[0][9] */
  w25 = arg[0] ? arg[0][9] : 0;
  /* #124: @28 = input[0][10] */
  w28 = arg[0] ? arg[0][10] : 0;
  /* #125: @30 = input[0][11] */
  w30 = arg[0] ? arg[0][11] : 0;
  /* #126: @32 = input[0][12] */
  w32 = arg[0] ? arg[0][12] : 0;
  /* #127: @34 = input[0][13] */
  w34 = arg[0] ? arg[0][13] : 0;
  /* #128: @35 = vertcat(@7, @8, @10, @24, @26, @27, @29, @31, @23, @25, @28, @30, @32, @34) */
  rr=w35;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w24;
  *rr++ = w26;
  *rr++ = w27;
  *rr++ = w29;
  *rr++ = w31;
  *rr++ = w23;
  *rr++ = w25;
  *rr++ = w28;
  *rr++ = w30;
  *rr++ = w32;
  *rr++ = w34;
  /* #129: @36 = @35[8:11] */
  for (rr=w36, ss=w35+8; ss!=w35+11; ss+=1) *rr++ = *ss;
  /* #130: @37 = @36' */
  casadi_copy(w36, 3, w37);
  /* #131: @1 = mac(@37,@36,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w37+j, tt=w36+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #132: @0 = (@0+@1) */
  w0 += w1;
  /* #133: @1 = 0 */
  w1 = 0.;
  /* #134: @37 = @35[11:14] */
  for (rr=w37, ss=w35+11; ss!=w35+14; ss+=1) *rr++ = *ss;
  /* #135: @38 = @37' */
  casadi_copy(w37, 3, w38);
  /* #136: @1 = mac(@38,@37,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w38+j, tt=w37+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #137: @0 = (@0+@1) */
  w0 += w1;
  /* #138: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #139: @11 = @11' */
  /* #140: @13 = zeros(1x4) */
  casadi_clear(w13, 4);
  /* #141: @12 = @12' */
  /* #142: @16 = @19' */
  for (i=0, rr=w16, cs=w19; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #143: @13 = mac(@12,@16,@13) */
  for (i=0, rr=w13; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w12+j, tt=w16+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #144: @13 = @13' */
  /* #145: @11 = (@11+@13) */
  for (i=0, rr=w11, cs=w13; i<4; ++i) (*rr++) += (*cs++);
  /* #146: @11 = (-@11) */
  for (i=0, rr=w11, cs=w11; i<4; ++i) *rr++ = (- *cs++ );
  /* #147: {@0, @1, @7, @8} = vertsplit(@11) */
  w0 = w11[0];
  w1 = w11[1];
  w7 = w11[2];
  w8 = w11[3];
  /* #148: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #149: output[1][1] = @1 */
  if (res[1]) res[1][1] = w1;
  /* #150: output[1][2] = @7 */
  if (res[1]) res[1][2] = w7;
  /* #151: output[1][3] = @8 */
  if (res[1]) res[1][3] = w8;
  /* #152: @35 = zeros(14x1) */
  casadi_clear(w35, 14);
  /* #153: @37 = (2.*@37) */
  for (i=0, rr=w37, cs=w37; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #154: (@35[11:14] += @37) */
  for (rr=w35+11, ss=w37; rr!=w35+14; rr+=1) *rr += *ss++;
  /* #155: @36 = (2.*@36) */
  for (i=0, rr=w36, cs=w36; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #156: (@35[8:11] += @36) */
  for (rr=w35+8, ss=w36; rr!=w35+11; rr+=1) *rr += *ss++;
  /* #157: {@8, @7, @1, @0, @10, @24, @26, @27, @29, @31, @23, @25, @28, @30} = vertsplit(@35) */
  w8 = w35[0];
  w7 = w35[1];
  w1 = w35[2];
  w0 = w35[3];
  w10 = w35[4];
  w24 = w35[5];
  w26 = w35[6];
  w27 = w35[7];
  w29 = w35[8];
  w31 = w35[9];
  w23 = w35[10];
  w25 = w35[11];
  w28 = w35[12];
  w30 = w35[13];
  /* #158: @33 = zeros(8x1) */
  casadi_clear(w33, 8);
  /* #159: @32 = 1 */
  w32 = 1.;
  /* #160: @9 = (@9?@32:0) */
  w9  = (w9?w32:0);
  /* #161: @32 = 10 */
  w32 = 10.;
  /* #162: @2 = @2' */
  /* #163: @2 = (@32*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w32*(*cs++));
  /* #164: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #165: @3 = @3' */
  /* #166: @3 = (@32*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w32*(*cs++));
  /* #167: @39 = @22' */
  for (i=0, rr=w39, cs=w22; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #168: @5 = mac(@3,@39,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w39+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #169: @5 = @5' */
  /* #170: @2 = (@2+@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #171: @5 = (@9*@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++)  = (w9*(*cs++));
  /* #172: @9 = 1 */
  w9 = 1.;
  /* #173: @4 = (@4?@9:0) */
  w4  = (w4?w9:0);
  /* #174: @2 = (@4*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #175: @5 = (@5-@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #176: @33 = mac(@21,@5,@33) */
  for (i=0, rr=w33; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w21+j, tt=w5+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #177: {@4, @9, @32, @34, @40, @41, @42, @43} = vertsplit(@33) */
  w4 = w33[0];
  w9 = w33[1];
  w32 = w33[2];
  w34 = w33[3];
  w40 = w33[4];
  w41 = w33[5];
  w42 = w33[6];
  w43 = w33[7];
  /* #178: @8 = (@8+@4) */
  w8 += w4;
  /* #179: output[1][4] = @8 */
  if (res[1]) res[1][4] = w8;
  /* #180: @7 = (@7-@9) */
  w7 -= w9;
  /* #181: output[1][5] = @7 */
  if (res[1]) res[1][5] = w7;
  /* #182: @1 = (@1-@32) */
  w1 -= w32;
  /* #183: output[1][6] = @1 */
  if (res[1]) res[1][6] = w1;
  /* #184: @0 = (@0-@34) */
  w0 -= w34;
  /* #185: output[1][7] = @0 */
  if (res[1]) res[1][7] = w0;
  /* #186: @10 = (@10+@40) */
  w10 += w40;
  /* #187: output[1][8] = @10 */
  if (res[1]) res[1][8] = w10;
  /* #188: @24 = (@24-@41) */
  w24 -= w41;
  /* #189: output[1][9] = @24 */
  if (res[1]) res[1][9] = w24;
  /* #190: @26 = (@26-@42) */
  w26 -= w42;
  /* #191: output[1][10] = @26 */
  if (res[1]) res[1][10] = w26;
  /* #192: @27 = (@27-@43) */
  w27 -= w43;
  /* #193: output[1][11] = @27 */
  if (res[1]) res[1][11] = w27;
  /* #194: output[1][12] = @29 */
  if (res[1]) res[1][12] = w29;
  /* #195: output[1][13] = @31 */
  if (res[1]) res[1][13] = w31;
  /* #196: output[1][14] = @23 */
  if (res[1]) res[1][14] = w23;
  /* #197: output[1][15] = @25 */
  if (res[1]) res[1][15] = w25;
  /* #198: output[1][16] = @28 */
  if (res[1]) res[1][16] = w28;
  /* #199: output[1][17] = @30 */
  if (res[1]) res[1][17] = w30;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_0_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_0_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_0_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_0_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_0_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_0_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_0_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 16;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 455;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
