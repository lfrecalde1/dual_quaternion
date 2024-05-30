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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_fun_jac_ ## ID
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
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_fun_jac:(i0[7],i1[3],i2[],i3[10])->(o0,o1[10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+6, w3, *w4=w+10, *w5=w+13, w6, w7, w8, w9, *w10=w+21, w11, w12, *w13=w+27, w14, *w15=w+32, *w16=w+36, *w17=w+40, *w18=w+56, w19, w20, w21, w22, *w23=w+76, *w24=w+83, w25, w26, w27, w28, w29, w30, *w31=w+92, *w32=w+95, w33, w34, w35, w36, w37, w38, w39, *w40=w+111, *w41=w+115, *w42=w+119, w43, w44, w45, *w46=w+138, *w47=w+148, *w48=w+151, *w49=w+154, *w50=w+157, *w51=w+166, *w52=w+175, *w53=w+191;
  /* #0: @0 = 1000 */
  w0 = 1000.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x3) */
  casadi_clear(w2, 3);
  /* #3: @3 = 0.5 */
  w3 = 5.0000000000000000e-01;
  /* #4: @4 = all_2.22045e-16(3x1) */
  casadi_fill(w4, 3, 2.2204460492503131e-16);
  /* #5: @5 = zeros(4x1) */
  casadi_clear(w5, 4);
  /* #6: @6 = input[3][0] */
  w6 = arg[3] ? arg[3][0] : 0;
  /* #7: @7 = input[3][1] */
  w7 = arg[3] ? arg[3][1] : 0;
  /* #8: @8 = input[3][2] */
  w8 = arg[3] ? arg[3][2] : 0;
  /* #9: @9 = input[3][3] */
  w9 = arg[3] ? arg[3][3] : 0;
  /* #10: @10 = horzcat(@6, @7, @8, @9) */
  rr=w10;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  /* #11: @10 = @10' */
  /* #12: @11 = (-@7) */
  w11 = (- w7 );
  /* #13: @12 = (-@8) */
  w12 = (- w8 );
  /* #14: @13 = horzcat(@11, @6, @9, @12) */
  rr=w13;
  *rr++ = w11;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w12;
  /* #15: @13 = @13' */
  /* #16: @14 = (-@9) */
  w14 = (- w9 );
  /* #17: @15 = horzcat(@12, @14, @6, @7) */
  rr=w15;
  *rr++ = w12;
  *rr++ = w14;
  *rr++ = w6;
  *rr++ = w7;
  /* #18: @15 = @15' */
  /* #19: @16 = horzcat(@14, @8, @11, @6) */
  rr=w16;
  *rr++ = w14;
  *rr++ = w8;
  *rr++ = w11;
  *rr++ = w6;
  /* #20: @16 = @16' */
  /* #21: @17 = horzcat(@10, @13, @15, @16) */
  rr=w17;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #22: @18 = @17' */
  for (i=0, rr=w18, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #23: @14 = input[0][0] */
  w14 = arg[0] ? arg[0][0] : 0;
  /* #24: @11 = input[0][1] */
  w11 = arg[0] ? arg[0][1] : 0;
  /* #25: @12 = input[0][2] */
  w12 = arg[0] ? arg[0][2] : 0;
  /* #26: @19 = input[0][3] */
  w19 = arg[0] ? arg[0][3] : 0;
  /* #27: @20 = input[0][4] */
  w20 = arg[0] ? arg[0][4] : 0;
  /* #28: @21 = input[0][5] */
  w21 = arg[0] ? arg[0][5] : 0;
  /* #29: @22 = input[0][6] */
  w22 = arg[0] ? arg[0][6] : 0;
  /* #30: @23 = vertcat(@14, @11, @12, @19, @20, @21, @22) */
  rr=w23;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w19;
  *rr++ = w20;
  *rr++ = w21;
  *rr++ = w22;
  /* #31: @10 = @23[:4] */
  for (rr=w10, ss=w23+0; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #32: @5 = mac(@18,@10,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w10+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #33: @24 = @5[1:4] */
  for (rr=w24, ss=w5+1; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #34: @4 = (@4+@24) */
  for (i=0, rr=w4, cs=w24; i<3; ++i) (*rr++) += (*cs++);
  /* #35: @14 = ||@4||_F */
  w14 = sqrt(casadi_dot(3, w4, w4));
  /* #36: @11 = @5[0] */
  for (rr=(&w11), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #37: @12 = atan2(@14,@11) */
  w12  = atan2(w14,w11);
  /* #38: @19 = (@3*@12) */
  w19  = (w3*w12);
  /* #39: @20 = @5[1] */
  for (rr=(&w20), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #40: @21 = (@19*@20) */
  w21  = (w19*w20);
  /* #41: @21 = (@21/@14) */
  w21 /= w14;
  /* #42: @22 = 0.5 */
  w22 = 5.0000000000000000e-01;
  /* #43: @25 = (@22*@12) */
  w25  = (w22*w12);
  /* #44: @26 = @5[2] */
  for (rr=(&w26), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #45: @27 = (@25*@26) */
  w27  = (w25*w26);
  /* #46: @27 = (@27/@14) */
  w27 /= w14;
  /* #47: @28 = 0.5 */
  w28 = 5.0000000000000000e-01;
  /* #48: @12 = (@28*@12) */
  w12  = (w28*w12);
  /* #49: @29 = @5[3] */
  for (rr=(&w29), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #50: @30 = (@12*@29) */
  w30  = (w12*w29);
  /* #51: @30 = (@30/@14) */
  w30 /= w14;
  /* #52: @24 = vertcat(@21, @27, @30) */
  rr=w24;
  *rr++ = w21;
  *rr++ = w27;
  *rr++ = w30;
  /* #53: @31 = @24' */
  casadi_copy(w24, 3, w31);
  /* #54: @32 = zeros(3x3) */
  casadi_clear(w32, 9);
  /* #55: @33 = 2 */
  w33 = 2.;
  /* #56: (@32[0] = @33) */
  for (rr=w32+0, ss=(&w33); rr!=w32+1; rr+=1) *rr = *ss++;
  /* #57: @33 = 2 */
  w33 = 2.;
  /* #58: (@32[8] = @33) */
  for (rr=w32+8, ss=(&w33); rr!=w32+9; rr+=1) *rr = *ss++;
  /* #59: @33 = 2 */
  w33 = 2.;
  /* #60: (@32[4] = @33) */
  for (rr=w32+4, ss=(&w33); rr!=w32+5; rr+=1) *rr = *ss++;
  /* #61: @2 = mac(@31,@32,@2) */
  for (i=0, rr=w2; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w31+j, tt=w32+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #62: @1 = mac(@2,@24,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w24+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #63: @0 = (@0*@1) */
  w0 *= w1;
  /* #64: @1 = 0 */
  w1 = 0.;
  /* #65: @31 = @23[4:7] */
  for (rr=w31, ss=w23+4; ss!=w23+7; ss+=1) *rr++ = *ss;
  /* #66: @10 = zeros(4x1) */
  casadi_clear(w10, 4);
  /* #67: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #68: @33 = @5[0] */
  for (rr=(&w33), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #69: @34 = @5[1] */
  for (rr=(&w34), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #70: @35 = @5[2] */
  for (rr=(&w35), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #71: @36 = @5[3] */
  for (rr=(&w36), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #72: @15 = horzcat(@33, @34, @35, @36) */
  rr=w15;
  *rr++ = w33;
  *rr++ = w34;
  *rr++ = w35;
  *rr++ = w36;
  /* #73: @15 = @15' */
  /* #74: @33 = (-@34) */
  w33 = (- w34 );
  /* #75: @37 = @5[0] */
  for (rr=(&w37), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #76: @38 = (-@35) */
  w38 = (- w35 );
  /* #77: @16 = horzcat(@33, @37, @36, @38) */
  rr=w16;
  *rr++ = w33;
  *rr++ = w37;
  *rr++ = w36;
  *rr++ = w38;
  /* #78: @16 = @16' */
  /* #79: @37 = (-@36) */
  w37 = (- w36 );
  /* #80: @39 = @5[0] */
  for (rr=(&w39), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #81: @40 = horzcat(@38, @37, @39, @34) */
  rr=w40;
  *rr++ = w38;
  *rr++ = w37;
  *rr++ = w39;
  *rr++ = w34;
  /* #82: @40 = @40' */
  /* #83: @38 = @5[0] */
  for (rr=(&w38), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #84: @41 = horzcat(@37, @35, @33, @38) */
  rr=w41;
  *rr++ = w37;
  *rr++ = w35;
  *rr++ = w33;
  *rr++ = w38;
  /* #85: @41 = @41' */
  /* #86: @18 = horzcat(@15, @16, @40, @41) */
  rr=w18;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w40; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w41; i<4; ++i) *rr++ = *cs++;
  /* #87: @42 = @18' */
  for (i=0, rr=w42, cs=w18; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #88: @37 = 0 */
  w37 = 0.;
  /* #89: @33 = input[3][4] */
  w33 = arg[3] ? arg[3][4] : 0;
  /* #90: @38 = input[3][5] */
  w38 = arg[3] ? arg[3][5] : 0;
  /* #91: @39 = input[3][6] */
  w39 = arg[3] ? arg[3][6] : 0;
  /* #92: @43 = input[3][7] */
  w43 = arg[3] ? arg[3][7] : 0;
  /* #93: @44 = input[3][8] */
  w44 = arg[3] ? arg[3][8] : 0;
  /* #94: @45 = input[3][9] */
  w45 = arg[3] ? arg[3][9] : 0;
  /* #95: @46 = vertcat(@6, @7, @8, @9, @33, @38, @39, @43, @44, @45) */
  rr=w46;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w33;
  *rr++ = w38;
  *rr++ = w39;
  *rr++ = w43;
  *rr++ = w44;
  *rr++ = w45;
  /* #96: @47 = @46[4:7] */
  for (rr=w47, ss=w46+4; ss!=w46+7; ss+=1) *rr++ = *ss;
  /* #97: @15 = vertcat(@37, @47) */
  rr=w15;
  *rr++ = w37;
  for (i=0, cs=w47; i<3; ++i) *rr++ = *cs++;
  /* #98: @13 = mac(@42,@15,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w42+j, tt=w15+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #99: @37 = @13[0] */
  for (rr=(&w37), ss=w13+0; ss!=w13+1; ss+=1) *rr++ = *ss;
  /* #100: @6 = @13[1] */
  for (rr=(&w6), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #101: @6 = (-@6) */
  w6 = (- w6 );
  /* #102: @7 = @13[2] */
  for (rr=(&w7), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #103: @7 = (-@7) */
  w7 = (- w7 );
  /* #104: @8 = @13[3] */
  for (rr=(&w8), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #105: @8 = (-@8) */
  w8 = (- w8 );
  /* #106: @16 = horzcat(@37, @6, @7, @8) */
  rr=w16;
  *rr++ = w37;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  /* #107: @16 = @16' */
  /* #108: @37 = @13[1] */
  for (rr=(&w37), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #109: @6 = @13[0] */
  for (rr=(&w6), ss=w13+0; ss!=w13+1; ss+=1) *rr++ = *ss;
  /* #110: @7 = @13[3] */
  for (rr=(&w7), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #111: @7 = (-@7) */
  w7 = (- w7 );
  /* #112: @8 = @13[2] */
  for (rr=(&w8), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #113: @40 = horzcat(@37, @6, @7, @8) */
  rr=w40;
  *rr++ = w37;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  /* #114: @40 = @40' */
  /* #115: @37 = @13[2] */
  for (rr=(&w37), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #116: @6 = @13[3] */
  for (rr=(&w6), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #117: @7 = @13[0] */
  for (rr=(&w7), ss=w13+0; ss!=w13+1; ss+=1) *rr++ = *ss;
  /* #118: @8 = @13[1] */
  for (rr=(&w8), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #119: @8 = (-@8) */
  w8 = (- w8 );
  /* #120: @41 = horzcat(@37, @6, @7, @8) */
  rr=w41;
  *rr++ = w37;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  /* #121: @41 = @41' */
  /* #122: @37 = @13[3] */
  for (rr=(&w37), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #123: @6 = @13[2] */
  for (rr=(&w6), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #124: @6 = (-@6) */
  w6 = (- w6 );
  /* #125: @7 = @13[1] */
  for (rr=(&w7), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #126: @8 = @13[0] */
  for (rr=(&w8), ss=w13+0; ss!=w13+1; ss+=1) *rr++ = *ss;
  /* #127: @13 = horzcat(@37, @6, @7, @8) */
  rr=w13;
  *rr++ = w37;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  /* #128: @13 = @13' */
  /* #129: @42 = horzcat(@16, @40, @41, @13) */
  rr=w42;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w40; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w41; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #130: @18 = @42' */
  for (i=0, rr=w18, cs=w42; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #131: @37 = @5[0] */
  for (rr=(&w37), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #132: @5 = vertcat(@37, @34, @35, @36) */
  rr=w5;
  *rr++ = w37;
  *rr++ = w34;
  *rr++ = w35;
  *rr++ = w36;
  /* #133: @10 = mac(@18,@5,@10) */
  for (i=0, rr=w10; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w5+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #134: @47 = @10[1:4] */
  for (rr=w47, ss=w10+1; ss!=w10+4; ss+=1) *rr++ = *ss;
  /* #135: @31 = (@31-@47) */
  for (i=0, rr=w31, cs=w47; i<3; ++i) (*rr++) -= (*cs++);
  /* #136: @47 = @31' */
  casadi_copy(w31, 3, w47);
  /* #137: @1 = mac(@47,@31,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w47+j, tt=w31+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #138: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #139: @0 = (@0+@1) */
  w0 += w1;
  /* #140: @1 = 0 */
  w1 = 0.;
  /* #141: @47 = zeros(1x3) */
  casadi_clear(w47, 3);
  /* #142: @48 = @46[7:10] */
  for (rr=w48, ss=w46+7; ss!=w46+10; ss+=1) *rr++ = *ss;
  /* #143: @37 = input[1][0] */
  w37 = arg[1] ? arg[1][0] : 0;
  /* #144: @34 = input[1][1] */
  w34 = arg[1] ? arg[1][1] : 0;
  /* #145: @35 = input[1][2] */
  w35 = arg[1] ? arg[1][2] : 0;
  /* #146: @49 = vertcat(@37, @34, @35) */
  rr=w49;
  *rr++ = w37;
  *rr++ = w34;
  *rr++ = w35;
  /* #147: @48 = (@48-@49) */
  for (i=0, rr=w48, cs=w49; i<3; ++i) (*rr++) -= (*cs++);
  /* #148: @49 = @48' */
  casadi_copy(w48, 3, w49);
  /* #149: @50 = zeros(3x3) */
  casadi_clear(w50, 9);
  /* #150: @37 = 0.000166667 */
  w37 = 1.6666666666666669e-04;
  /* #151: (@50[0] = @37) */
  for (rr=w50+0, ss=(&w37); rr!=w50+1; rr+=1) *rr = *ss++;
  /* #152: @37 = 0.000166667 */
  w37 = 1.6666666666666669e-04;
  /* #153: (@50[4] = @37) */
  for (rr=w50+4, ss=(&w37); rr!=w50+5; rr+=1) *rr = *ss++;
  /* #154: @37 = 0.000166667 */
  w37 = 1.6666666666666669e-04;
  /* #155: (@50[8] = @37) */
  for (rr=w50+8, ss=(&w37); rr!=w50+9; rr+=1) *rr = *ss++;
  /* #156: @47 = mac(@49,@50,@47) */
  for (i=0, rr=w47; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w49+j, tt=w50+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #157: @1 = mac(@47,@48,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w47+j, tt=w48+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #158: @0 = (@0+@1) */
  w0 += w1;
  /* #159: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #160: @47 = @47' */
  /* #161: @49 = zeros(1x3) */
  casadi_clear(w49, 3);
  /* #162: @48 = @48' */
  /* #163: @51 = @50' */
  for (i=0, rr=w51, cs=w50; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #164: @49 = mac(@48,@51,@49) */
  for (i=0, rr=w49; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w48+j, tt=w51+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #165: @49 = @49' */
  /* #166: @47 = (@47+@49) */
  for (i=0, rr=w47, cs=w49; i<3; ++i) (*rr++) += (*cs++);
  /* #167: @47 = (-@47) */
  for (i=0, rr=w47, cs=w47; i<3; ++i) *rr++ = (- *cs++ );
  /* #168: {@0, @1, @37} = vertsplit(@47) */
  w0 = w47[0];
  w1 = w47[1];
  w37 = w47[2];
  /* #169: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #170: output[1][1] = @1 */
  if (res[1]) res[1][1] = w1;
  /* #171: output[1][2] = @37 */
  if (res[1]) res[1][2] = w37;
  /* #172: @23 = zeros(7x1) */
  casadi_clear(w23, 7);
  /* #173: @47 = (2.*@31) */
  for (i=0, rr=w47, cs=w31; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #174: @31 = @31' */
  /* #175: @31 = (2.*@31) */
  for (i=0, rr=w31, cs=w31; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #176: @31 = @31' */
  /* #177: @47 = (@47+@31) */
  for (i=0, rr=w47, cs=w31; i<3; ++i) (*rr++) += (*cs++);
  /* #178: (@23[4:7] += @47) */
  for (rr=w23+4, ss=w47; rr!=w23+7; rr+=1) *rr += *ss++;
  /* #179: @10 = zeros(4x1) */
  casadi_clear(w10, 4);
  /* #180: @16 = zeros(4x1) */
  casadi_clear(w16, 4);
  /* #181: @40 = zeros(4x1) */
  casadi_clear(w40, 4);
  /* #182: @41 = zeros(4x1) */
  casadi_clear(w41, 4);
  /* #183: @47 = (-@47) */
  for (i=0, rr=w47, cs=w47; i<3; ++i) *rr++ = (- *cs++ );
  /* #184: (@41[1:4] += @47) */
  for (rr=w41+1, ss=w47; rr!=w41+4; rr+=1) *rr += *ss++;
  /* #185: @40 = mac(@42,@41,@40) */
  for (i=0, rr=w40; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w42+j, tt=w41+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #186: {@37, @1, @0, @34} = vertsplit(@40) */
  w37 = w40[0];
  w1 = w40[1];
  w0 = w40[2];
  w34 = w40[3];
  /* #187: (@16[0] += @37) */
  for (rr=w16+0, ss=(&w37); rr!=w16+1; rr+=1) *rr += *ss++;
  /* #188: @42 = zeros(4x4) */
  casadi_clear(w42, 16);
  /* #189: @40 = zeros(4x1) */
  casadi_clear(w40, 4);
  /* #190: @18 = zeros(4x4) */
  casadi_clear(w18, 16);
  /* #191: @5 = @5' */
  /* #192: @18 = mac(@41,@5,@18) */
  for (i=0, rr=w18; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w41+j, tt=w5+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #193: @52 = @18' */
  for (i=0, rr=w52, cs=w18; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #194: {@41, @5, @13, @53} = horzsplit(@52) */
  casadi_copy(w52, 4, w41);
  casadi_copy(w52+4, 4, w5);
  casadi_copy(w52+8, 4, w13);
  casadi_copy(w52+12, 4, w53);
  /* #195: @53 = @53' */
  /* #196: {@37, @35, @36, @6} = horzsplit(@53) */
  w37 = w53[0];
  w35 = w53[1];
  w36 = w53[2];
  w6 = w53[3];
  /* #197: (@40[0] += @6) */
  for (rr=w40+0, ss=(&w6); rr!=w40+1; rr+=1) *rr += *ss++;
  /* #198: (@40[1] += @36) */
  for (rr=w40+1, ss=(&w36); rr!=w40+2; rr+=1) *rr += *ss++;
  /* #199: @35 = (-@35) */
  w35 = (- w35 );
  /* #200: (@40[2] += @35) */
  for (rr=w40+2, ss=(&w35); rr!=w40+3; rr+=1) *rr += *ss++;
  /* #201: (@40[3] += @37) */
  for (rr=w40+3, ss=(&w37); rr!=w40+4; rr+=1) *rr += *ss++;
  /* #202: @13 = @13' */
  /* #203: {@37, @35, @36, @6} = horzsplit(@13) */
  w37 = w13[0];
  w35 = w13[1];
  w36 = w13[2];
  w6 = w13[3];
  /* #204: @6 = (-@6) */
  w6 = (- w6 );
  /* #205: (@40[1] += @6) */
  for (rr=w40+1, ss=(&w6); rr!=w40+2; rr+=1) *rr += *ss++;
  /* #206: (@40[0] += @36) */
  for (rr=w40+0, ss=(&w36); rr!=w40+1; rr+=1) *rr += *ss++;
  /* #207: (@40[3] += @35) */
  for (rr=w40+3, ss=(&w35); rr!=w40+4; rr+=1) *rr += *ss++;
  /* #208: (@40[2] += @37) */
  for (rr=w40+2, ss=(&w37); rr!=w40+3; rr+=1) *rr += *ss++;
  /* #209: @5 = @5' */
  /* #210: {@37, @35, @36, @6} = horzsplit(@5) */
  w37 = w5[0];
  w35 = w5[1];
  w36 = w5[2];
  w6 = w5[3];
  /* #211: (@40[2] += @6) */
  for (rr=w40+2, ss=(&w6); rr!=w40+3; rr+=1) *rr += *ss++;
  /* #212: @36 = (-@36) */
  w36 = (- w36 );
  /* #213: (@40[3] += @36) */
  for (rr=w40+3, ss=(&w36); rr!=w40+4; rr+=1) *rr += *ss++;
  /* #214: (@40[0] += @35) */
  for (rr=w40+0, ss=(&w35); rr!=w40+1; rr+=1) *rr += *ss++;
  /* #215: (@40[1] += @37) */
  for (rr=w40+1, ss=(&w37); rr!=w40+2; rr+=1) *rr += *ss++;
  /* #216: @41 = @41' */
  /* #217: {@37, @35, @36, @6} = horzsplit(@41) */
  w37 = w41[0];
  w35 = w41[1];
  w36 = w41[2];
  w6 = w41[3];
  /* #218: @6 = (-@6) */
  w6 = (- w6 );
  /* #219: (@40[3] += @6) */
  for (rr=w40+3, ss=(&w6); rr!=w40+4; rr+=1) *rr += *ss++;
  /* #220: @36 = (-@36) */
  w36 = (- w36 );
  /* #221: (@40[2] += @36) */
  for (rr=w40+2, ss=(&w36); rr!=w40+3; rr+=1) *rr += *ss++;
  /* #222: @35 = (-@35) */
  w35 = (- w35 );
  /* #223: (@40[1] += @35) */
  for (rr=w40+1, ss=(&w35); rr!=w40+2; rr+=1) *rr += *ss++;
  /* #224: (@40[0] += @37) */
  for (rr=w40+0, ss=(&w37); rr!=w40+1; rr+=1) *rr += *ss++;
  /* #225: @15 = @15' */
  /* #226: @42 = mac(@40,@15,@42) */
  for (i=0, rr=w42; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w40+j, tt=w15+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #227: @52 = @42' */
  for (i=0, rr=w52, cs=w42; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #228: {@40, @15, @41, @5} = horzsplit(@52) */
  casadi_copy(w52, 4, w40);
  casadi_copy(w52+4, 4, w15);
  casadi_copy(w52+8, 4, w41);
  casadi_copy(w52+12, 4, w5);
  /* #229: @5 = @5' */
  /* #230: {@37, @35, @36, @6} = horzsplit(@5) */
  w37 = w5[0];
  w35 = w5[1];
  w36 = w5[2];
  w6 = w5[3];
  /* #231: (@16[0] += @6) */
  for (rr=w16+0, ss=(&w6); rr!=w16+1; rr+=1) *rr += *ss++;
  /* #232: @41 = @41' */
  /* #233: {@6, @7, @8, @9} = horzsplit(@41) */
  w6 = w41[0];
  w7 = w41[1];
  w8 = w41[2];
  w9 = w41[3];
  /* #234: (@16[0] += @8) */
  for (rr=w16+0, ss=(&w8); rr!=w16+1; rr+=1) *rr += *ss++;
  /* #235: @15 = @15' */
  /* #236: {@8, @33, @38, @39} = horzsplit(@15) */
  w8 = w15[0];
  w33 = w15[1];
  w38 = w15[2];
  w39 = w15[3];
  /* #237: (@16[0] += @33) */
  for (rr=w16+0, ss=(&w33); rr!=w16+1; rr+=1) *rr += *ss++;
  /* #238: @37 = (@37+@7) */
  w37 += w7;
  /* #239: @34 = (@34-@37) */
  w34 -= w37;
  /* #240: @34 = (@34+@38) */
  w34 += w38;
  /* #241: @40 = @40' */
  /* #242: {@38, @37, @7, @33} = horzsplit(@40) */
  w38 = w40[0];
  w37 = w40[1];
  w7 = w40[2];
  w33 = w40[3];
  /* #243: @34 = (@34+@33) */
  w34 += w33;
  /* #244: (@16[3] += @34) */
  for (rr=w16+3, ss=(&w34); rr!=w16+4; rr+=1) *rr += *ss++;
  /* #245: @0 = (@0+@35) */
  w0 += w35;
  /* #246: @6 = (@6+@39) */
  w6 += w39;
  /* #247: @0 = (@0-@6) */
  w0 -= w6;
  /* #248: @0 = (@0+@7) */
  w0 += w7;
  /* #249: (@16[2] += @0) */
  for (rr=w16+2, ss=(&w0); rr!=w16+3; rr+=1) *rr += *ss++;
  /* #250: @1 = (@1+@9) */
  w1 += w9;
  /* #251: @36 = (@36+@8) */
  w36 += w8;
  /* #252: @1 = (@1-@36) */
  w1 -= w36;
  /* #253: @1 = (@1+@37) */
  w1 += w37;
  /* #254: (@16[1] += @1) */
  for (rr=w16+1, ss=(&w1); rr!=w16+2; rr+=1) *rr += *ss++;
  /* #255: (@16[0] += @38) */
  for (rr=w16+0, ss=(&w38); rr!=w16+1; rr+=1) *rr += *ss++;
  /* #256: @38 = 1000 */
  w38 = 1000.;
  /* #257: @2 = @2' */
  /* #258: @2 = (@38*@2) */
  for (i=0, rr=w2, cs=w2; i<3; ++i) (*rr++)  = (w38*(*cs++));
  /* #259: @47 = zeros(1x3) */
  casadi_clear(w47, 3);
  /* #260: @24 = @24' */
  /* #261: @24 = (@38*@24) */
  for (i=0, rr=w24, cs=w24; i<3; ++i) (*rr++)  = (w38*(*cs++));
  /* #262: @51 = @32' */
  for (i=0, rr=w51, cs=w32; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #263: @47 = mac(@24,@51,@47) */
  for (i=0, rr=w47; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w24+j, tt=w51+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #264: @47 = @47' */
  /* #265: @2 = (@2+@47) */
  for (i=0, rr=w2, cs=w47; i<3; ++i) (*rr++) += (*cs++);
  /* #266: {@38, @1, @37} = vertsplit(@2) */
  w38 = w2[0];
  w1 = w2[1];
  w37 = w2[2];
  /* #267: @36 = (@37/@14) */
  w36  = (w37/w14);
  /* #268: @12 = (@12*@36) */
  w12 *= w36;
  /* #269: (@16[3] += @12) */
  for (rr=w16+3, ss=(&w12); rr!=w16+4; rr+=1) *rr += *ss++;
  /* #270: @12 = (@1/@14) */
  w12  = (w1/w14);
  /* #271: @25 = (@25*@12) */
  w25 *= w12;
  /* #272: (@16[2] += @25) */
  for (rr=w16+2, ss=(&w25); rr!=w16+3; rr+=1) *rr += *ss++;
  /* #273: @25 = (@38/@14) */
  w25  = (w38/w14);
  /* #274: @19 = (@19*@25) */
  w19 *= w25;
  /* #275: (@16[1] += @19) */
  for (rr=w16+1, ss=(&w19); rr!=w16+2; rr+=1) *rr += *ss++;
  /* #276: @19 = sq(@14) */
  w19 = casadi_sq( w14 );
  /* #277: @8 = sq(@11) */
  w8 = casadi_sq( w11 );
  /* #278: @19 = (@19+@8) */
  w19 += w8;
  /* #279: @8 = (@14/@19) */
  w8  = (w14/w19);
  /* #280: @29 = (@29*@36) */
  w29 *= w36;
  /* #281: @28 = (@28*@29) */
  w28 *= w29;
  /* #282: @26 = (@26*@12) */
  w26 *= w12;
  /* #283: @22 = (@22*@26) */
  w22 *= w26;
  /* #284: @28 = (@28+@22) */
  w28 += w22;
  /* #285: @20 = (@20*@25) */
  w20 *= w25;
  /* #286: @3 = (@3*@20) */
  w3 *= w20;
  /* #287: @28 = (@28+@3) */
  w28 += w3;
  /* #288: @8 = (@8*@28) */
  w8 *= w28;
  /* #289: @8 = (-@8) */
  w8 = (- w8 );
  /* #290: (@16[0] += @8) */
  for (rr=w16+0, ss=(&w8); rr!=w16+1; rr+=1) *rr += *ss++;
  /* #291: @27 = (@27/@14) */
  w27 /= w14;
  /* #292: @27 = (@27*@1) */
  w27 *= w1;
  /* #293: @27 = (-@27) */
  w27 = (- w27 );
  /* #294: @30 = (@30/@14) */
  w30 /= w14;
  /* #295: @30 = (@30*@37) */
  w30 *= w37;
  /* #296: @27 = (@27-@30) */
  w27 -= w30;
  /* #297: @21 = (@21/@14) */
  w21 /= w14;
  /* #298: @21 = (@21*@38) */
  w21 *= w38;
  /* #299: @27 = (@27-@21) */
  w27 -= w21;
  /* #300: @11 = (@11/@19) */
  w11 /= w19;
  /* #301: @11 = (@11*@28) */
  w11 *= w28;
  /* #302: @27 = (@27+@11) */
  w27 += w11;
  /* #303: @27 = (@27/@14) */
  w27 /= w14;
  /* #304: @4 = (@27*@4) */
  for (i=0, rr=w4, cs=w4; i<3; ++i) (*rr++)  = (w27*(*cs++));
  /* #305: (@16[1:4] += @4) */
  for (rr=w16+1, ss=w4; rr!=w16+4; rr+=1) *rr += *ss++;
  /* #306: @10 = mac(@17,@16,@10) */
  for (i=0, rr=w10; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w17+j, tt=w16+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #307: (@23[:4] += @10) */
  for (rr=w23+0, ss=w10; rr!=w23+4; rr+=1) *rr += *ss++;
  /* #308: {@27, @14, @11, @28, @19, @21, @38} = vertsplit(@23) */
  w27 = w23[0];
  w14 = w23[1];
  w11 = w23[2];
  w28 = w23[3];
  w19 = w23[4];
  w21 = w23[5];
  w38 = w23[6];
  /* #309: output[1][3] = @27 */
  if (res[1]) res[1][3] = w27;
  /* #310: output[1][4] = @14 */
  if (res[1]) res[1][4] = w14;
  /* #311: output[1][5] = @11 */
  if (res[1]) res[1][5] = w11;
  /* #312: output[1][6] = @28 */
  if (res[1]) res[1][6] = w28;
  /* #313: output[1][7] = @19 */
  if (res[1]) res[1][7] = w19;
  /* #314: output[1][8] = @21 */
  if (res[1]) res[1][8] = w21;
  /* #315: output[1][9] = @38 */
  if (res[1]) res[1][9] = w38;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 14;
  if (sz_res) *sz_res = 9;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 195;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
