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
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
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

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[4] = {1., 0., 0., 0.};

/* f_rot_inv:(i0[4],i1[3])->(o0[3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, *w1=w+8, *w2=w+12, w3, w4, w5, w6, *w7=w+20, w8, w9, *w10=w+26, *w11=w+30, *w12=w+34, *w13=w+38, *w14=w+54, *w15=w+70;
  /* #0: @0 = zeros(4x1) */
  casadi_clear(w0, 4);
  /* #1: @1 = zeros(4x1) */
  casadi_clear(w1, 4);
  /* #2: @2 = input[0][0] */
  casadi_copy(arg[0], 4, w2);
  /* #3: @3 = @2[0] */
  for (rr=(&w3), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #4: @4 = @2[1] */
  for (rr=(&w4), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #5: @5 = @2[2] */
  for (rr=(&w5), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #6: @6 = @2[3] */
  for (rr=(&w6), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #7: @7 = horzcat(@3, @4, @5, @6) */
  rr=w7;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #8: @7 = @7' */
  /* #9: @3 = (-@4) */
  w3 = (- w4 );
  /* #10: @8 = @2[0] */
  for (rr=(&w8), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #11: @9 = (-@5) */
  w9 = (- w5 );
  /* #12: @10 = horzcat(@3, @8, @6, @9) */
  rr=w10;
  *rr++ = w3;
  *rr++ = w8;
  *rr++ = w6;
  *rr++ = w9;
  /* #13: @10 = @10' */
  /* #14: @6 = (-@6) */
  w6 = (- w6 );
  /* #15: @8 = @2[0] */
  for (rr=(&w8), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #16: @11 = horzcat(@9, @6, @8, @4) */
  rr=w11;
  *rr++ = w9;
  *rr++ = w6;
  *rr++ = w8;
  *rr++ = w4;
  /* #17: @11 = @11' */
  /* #18: @9 = @2[0] */
  for (rr=(&w9), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #19: @12 = horzcat(@6, @5, @3, @9) */
  rr=w12;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #20: @12 = @12' */
  /* #21: @13 = horzcat(@7, @10, @11, @12) */
  rr=w13;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #22: @14 = @13' */
  for (i=0, rr=w14, cs=w13; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #23: @6 = 0 */
  w6 = 0.;
  /* #24: @15 = input[1][0] */
  casadi_copy(arg[1], 3, w15);
  /* #25: @7 = vertcat(@6, @15) */
  rr=w7;
  *rr++ = w6;
  for (i=0, cs=w15; i<3; ++i) *rr++ = *cs++;
  /* #26: @1 = mac(@14,@7,@1) */
  for (i=0, rr=w1; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w14+j, tt=w7+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #27: @6 = @1[0] */
  for (rr=(&w6), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #28: @5 = @1[1] */
  for (rr=(&w5), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #29: @5 = (-@5) */
  w5 = (- w5 );
  /* #30: @3 = @1[2] */
  for (rr=(&w3), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #31: @3 = (-@3) */
  w3 = (- w3 );
  /* #32: @9 = @1[3] */
  for (rr=(&w9), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #33: @9 = (-@9) */
  w9 = (- w9 );
  /* #34: @7 = horzcat(@6, @5, @3, @9) */
  rr=w7;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #35: @7 = @7' */
  /* #36: @6 = @1[1] */
  for (rr=(&w6), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #37: @5 = @1[0] */
  for (rr=(&w5), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #38: @3 = @1[3] */
  for (rr=(&w3), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #39: @3 = (-@3) */
  w3 = (- w3 );
  /* #40: @9 = @1[2] */
  for (rr=(&w9), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #41: @10 = horzcat(@6, @5, @3, @9) */
  rr=w10;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #42: @10 = @10' */
  /* #43: @6 = @1[2] */
  for (rr=(&w6), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #44: @5 = @1[3] */
  for (rr=(&w5), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #45: @3 = @1[0] */
  for (rr=(&w3), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #46: @9 = @1[1] */
  for (rr=(&w9), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #47: @9 = (-@9) */
  w9 = (- w9 );
  /* #48: @11 = horzcat(@6, @5, @3, @9) */
  rr=w11;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #49: @11 = @11' */
  /* #50: @6 = @1[3] */
  for (rr=(&w6), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #51: @5 = @1[2] */
  for (rr=(&w5), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #52: @5 = (-@5) */
  w5 = (- w5 );
  /* #53: @3 = @1[1] */
  for (rr=(&w3), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #54: @9 = @1[0] */
  for (rr=(&w9), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #55: @1 = horzcat(@6, @5, @3, @9) */
  rr=w1;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #56: @1 = @1' */
  /* #57: @14 = horzcat(@7, @10, @11, @1) */
  rr=w14;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w1; i<4; ++i) *rr++ = *cs++;
  /* #58: @13 = @14' */
  for (i=0, rr=w13, cs=w14; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #59: @0 = mac(@13,@2,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w13+j, tt=w2+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #60: @15 = @0[1:4] */
  for (rr=w15, ss=w0+1; ss!=w0+4; ss+=1) *rr++ = *ss;
  /* #61: output[0][0] = @15 */
  casadi_copy(w15, 3, res[0]);
  return 0;
}

/* quadrotor_cost_ext_cost_fun:(i0[13],i1[4],i2[],i3[17])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+1, *rr, *ss, *tt;
  const casadi_real **arg1=arg+4, *cs;
  casadi_real w0, *w1=w+74, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+94, *w20=w+111, *w21=w+114, *w22=w+127, *w23=w+130, *w24=w+139, *w25=w+143, *w26=w+147, *w27=w+151, *w28=w+167, *w29=w+171, *w30=w+175, *w31=w+179;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = zeros(1x3) */
  casadi_clear(w1, 3);
  /* #2: @2 = input[3][0] */
  w2 = arg[3] ? arg[3][0] : 0;
  /* #3: @3 = input[3][1] */
  w3 = arg[3] ? arg[3][1] : 0;
  /* #4: @4 = input[3][2] */
  w4 = arg[3] ? arg[3][2] : 0;
  /* #5: @5 = input[3][3] */
  w5 = arg[3] ? arg[3][3] : 0;
  /* #6: @6 = input[3][4] */
  w6 = arg[3] ? arg[3][4] : 0;
  /* #7: @7 = input[3][5] */
  w7 = arg[3] ? arg[3][5] : 0;
  /* #8: @8 = input[3][6] */
  w8 = arg[3] ? arg[3][6] : 0;
  /* #9: @9 = input[3][7] */
  w9 = arg[3] ? arg[3][7] : 0;
  /* #10: @10 = input[3][8] */
  w10 = arg[3] ? arg[3][8] : 0;
  /* #11: @11 = input[3][9] */
  w11 = arg[3] ? arg[3][9] : 0;
  /* #12: @12 = input[3][10] */
  w12 = arg[3] ? arg[3][10] : 0;
  /* #13: @13 = input[3][11] */
  w13 = arg[3] ? arg[3][11] : 0;
  /* #14: @14 = input[3][12] */
  w14 = arg[3] ? arg[3][12] : 0;
  /* #15: @15 = input[3][13] */
  w15 = arg[3] ? arg[3][13] : 0;
  /* #16: @16 = input[3][14] */
  w16 = arg[3] ? arg[3][14] : 0;
  /* #17: @17 = input[3][15] */
  w17 = arg[3] ? arg[3][15] : 0;
  /* #18: @18 = input[3][16] */
  w18 = arg[3] ? arg[3][16] : 0;
  /* #19: @19 = vertcat(@2, @3, @4, @5, @6, @7, @8, @9, @10, @11, @12, @13, @14, @15, @16, @17, @18) */
  rr=w19;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w16;
  *rr++ = w17;
  *rr++ = w18;
  /* #20: @20 = @19[:3] */
  for (rr=w20, ss=w19+0; ss!=w19+3; ss+=1) *rr++ = *ss;
  /* #21: @2 = input[0][0] */
  w2 = arg[0] ? arg[0][0] : 0;
  /* #22: @3 = input[0][1] */
  w3 = arg[0] ? arg[0][1] : 0;
  /* #23: @4 = input[0][2] */
  w4 = arg[0] ? arg[0][2] : 0;
  /* #24: @5 = input[0][3] */
  w5 = arg[0] ? arg[0][3] : 0;
  /* #25: @6 = input[0][4] */
  w6 = arg[0] ? arg[0][4] : 0;
  /* #26: @7 = input[0][5] */
  w7 = arg[0] ? arg[0][5] : 0;
  /* #27: @12 = input[0][6] */
  w12 = arg[0] ? arg[0][6] : 0;
  /* #28: @13 = input[0][7] */
  w13 = arg[0] ? arg[0][7] : 0;
  /* #29: @14 = input[0][8] */
  w14 = arg[0] ? arg[0][8] : 0;
  /* #30: @15 = input[0][9] */
  w15 = arg[0] ? arg[0][9] : 0;
  /* #31: @16 = input[0][10] */
  w16 = arg[0] ? arg[0][10] : 0;
  /* #32: @17 = input[0][11] */
  w17 = arg[0] ? arg[0][11] : 0;
  /* #33: @18 = input[0][12] */
  w18 = arg[0] ? arg[0][12] : 0;
  /* #34: @21 = vertcat(@2, @3, @4, @5, @6, @7, @12, @13, @14, @15, @16, @17, @18) */
  rr=w21;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w16;
  *rr++ = w17;
  *rr++ = w18;
  /* #35: @22 = @21[:3] */
  for (rr=w22, ss=w21+0; ss!=w21+3; ss+=1) *rr++ = *ss;
  /* #36: @20 = (@20-@22) */
  for (i=0, rr=w20, cs=w22; i<3; ++i) (*rr++) -= (*cs++);
  /* #37: @22 = @20' */
  casadi_copy(w20, 3, w22);
  /* #38: @23 = zeros(3x3) */
  casadi_clear(w23, 9);
  /* #39: @2 = 10 */
  w2 = 10.;
  /* #40: (@23[0] = @2) */
  for (rr=w23+0, ss=(&w2); rr!=w23+1; rr+=1) *rr = *ss++;
  /* #41: @2 = 8 */
  w2 = 8.;
  /* #42: (@23[4] = @2) */
  for (rr=w23+4, ss=(&w2); rr!=w23+5; rr+=1) *rr = *ss++;
  /* #43: @2 = 6 */
  w2 = 6.;
  /* #44: (@23[8] = @2) */
  for (rr=w23+8, ss=(&w2); rr!=w23+9; rr+=1) *rr = *ss++;
  /* #45: @1 = mac(@22,@23,@1) */
  for (i=0, rr=w1; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w23+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #46: @0 = mac(@1,@20,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w20+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #47: @2 = 0 */
  w2 = 0.;
  /* #48: @24 = zeros(1x4) */
  casadi_clear(w24, 4);
  /* #49: @25 = @19[13:17] */
  for (rr=w25, ss=w19+13; ss!=w19+17; ss+=1) *rr++ = *ss;
  /* #50: @3 = input[1][0] */
  w3 = arg[1] ? arg[1][0] : 0;
  /* #51: @4 = input[1][1] */
  w4 = arg[1] ? arg[1][1] : 0;
  /* #52: @5 = input[1][2] */
  w5 = arg[1] ? arg[1][2] : 0;
  /* #53: @6 = input[1][3] */
  w6 = arg[1] ? arg[1][3] : 0;
  /* #54: @26 = vertcat(@3, @4, @5, @6) */
  rr=w26;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #55: @25 = (@25-@26) */
  for (i=0, rr=w25, cs=w26; i<4; ++i) (*rr++) -= (*cs++);
  /* #56: @26 = @25' */
  casadi_copy(w25, 4, w26);
  /* #57: @27 = zeros(4x4) */
  casadi_clear(w27, 16);
  /* #58: @3 = 0.671141 */
  w3 = 6.7114093959731547e-01;
  /* #59: (@27[0] = @3) */
  for (rr=w27+0, ss=(&w3); rr!=w27+1; rr+=1) *rr = *ss++;
  /* #60: @3 = 600 */
  w3 = 600.;
  /* #61: (@27[5] = @3) */
  for (rr=w27+5, ss=(&w3); rr!=w27+6; rr+=1) *rr = *ss++;
  /* #62: @3 = 600 */
  w3 = 600.;
  /* #63: (@27[10] = @3) */
  for (rr=w27+10, ss=(&w3); rr!=w27+11; rr+=1) *rr = *ss++;
  /* #64: @3 = 600 */
  w3 = 600.;
  /* #65: (@27[15] = @3) */
  for (rr=w27+15, ss=(&w3); rr!=w27+16; rr+=1) *rr = *ss++;
  /* #66: @24 = mac(@26,@27,@24) */
  for (i=0, rr=w24; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w26+j, tt=w27+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #67: @2 = mac(@24,@25,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w24+j, tt=w25+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #68: @0 = (@0+@2) */
  w0 += w2;
  /* #69: @2 = 10 */
  w2 = 10.;
  /* #70: @3 = 0 */
  w3 = 0.;
  /* #71: @24 = [1, 0, 0, 0] */
  casadi_copy(casadi_c0, 4, w24);
  /* #72: @4 = 0 */
  w4 = 0.;
  /* #73: @25 = zeros(4x1) */
  casadi_clear(w25, 4);
  /* #74: @26 = horzcat(@8, @9, @10, @11) */
  rr=w26;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #75: @26 = @26' */
  /* #76: @5 = (-@9) */
  w5 = (- w9 );
  /* #77: @6 = (-@10) */
  w6 = (- w10 );
  /* #78: @28 = horzcat(@5, @8, @11, @6) */
  rr=w28;
  *rr++ = w5;
  *rr++ = w8;
  *rr++ = w11;
  *rr++ = w6;
  /* #79: @28 = @28' */
  /* #80: @11 = (-@11) */
  w11 = (- w11 );
  /* #81: @29 = horzcat(@6, @11, @8, @9) */
  rr=w29;
  *rr++ = w6;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w9;
  /* #82: @29 = @29' */
  /* #83: @30 = horzcat(@11, @10, @5, @8) */
  rr=w30;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w8;
  /* #84: @30 = @30' */
  /* #85: @27 = horzcat(@26, @28, @29, @30) */
  rr=w27;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w29; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w30; i<4; ++i) *rr++ = *cs++;
  /* #86: @31 = @27' */
  for (i=0, rr=w31, cs=w27; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #87: @26 = @21[6:10] */
  for (rr=w26, ss=w21+6; ss!=w21+10; ss+=1) *rr++ = *ss;
  /* #88: @25 = mac(@31,@26,@25) */
  for (i=0, rr=w25; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w31+j, tt=w26+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #89: @11 = @25[0] */
  for (rr=(&w11), ss=w25+0; ss!=w25+1; ss+=1) *rr++ = *ss;
  /* #90: @4 = (@4<@11) */
  w4  = (w4<w11);
  /* #91: @26 = (@4?@25:0) */
  for (i=0, rr=w26, cs=w25; i<4; ++i) (*rr++)  = (w4?(*cs++):0);
  /* #92: @4 = (!@4) */
  w4 = (! w4 );
  /* #93: @25 = (-@25) */
  for (i=0, rr=w25, cs=w25; i<4; ++i) *rr++ = (- *cs++ );
  /* #94: @25 = (@4?@25:0) */
  for (i=0, rr=w25, cs=w25; i<4; ++i) (*rr++)  = (w4?(*cs++):0);
  /* #95: @26 = (@26+@25) */
  for (i=0, rr=w26, cs=w25; i<4; ++i) (*rr++) += (*cs++);
  /* #96: @24 = (@24-@26) */
  for (i=0, rr=w24, cs=w26; i<4; ++i) (*rr++) -= (*cs++);
  /* #97: @26 = @24' */
  casadi_copy(w24, 4, w26);
  /* #98: @3 = mac(@26,@24,@3) */
  for (i=0, rr=(&w3); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w26+j, tt=w24+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #99: @2 = (@2*@3) */
  w2 *= w3;
  /* #100: @0 = (@0+@2) */
  w0 += w2;
  /* #101: @2 = 0 */
  w2 = 0.;
  /* #102: @1 = @21[10:13] */
  for (rr=w1, ss=w21+10; ss!=w21+13; ss+=1) *rr++ = *ss;
  /* #103: @20 = @1' */
  casadi_copy(w1, 3, w20);
  /* #104: @2 = mac(@20,@1,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w1+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #105: @0 = (@0+@2) */
  w0 += w2;
  /* #106: @2 = 0 */
  w2 = 0.;
  /* #107: @26 = @21[6:10] */
  for (rr=w26, ss=w21+6; ss!=w21+10; ss+=1) *rr++ = *ss;
  /* #108: @20 = @21[3:6] */
  for (rr=w20, ss=w21+3; ss!=w21+6; ss+=1) *rr++ = *ss;
  /* #109: @1 = f_rot_inv(@26, @20) */
  arg1[0]=w26;
  arg1[1]=w20;
  res1[0]=w1;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #110: @20 = @1' */
  casadi_copy(w1, 3, w20);
  /* #111: @2 = mac(@20,@1,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w1+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #112: @0 = (@0+@2) */
  w0 += w2;
  /* #113: output[0][0] = @0 */
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
  if (sz_arg) *sz_arg = 21;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 195;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
