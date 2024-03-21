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
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
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

/* adj1_f_rot_inv:(i0[4],i1[3],out_o0[3x1,0nz],adj_o0[3])->(adj_i0[4],adj_i1[3]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, *w1=w+8, *w2=w+12, w3, w4, w5, w6, *w7=w+20, w8, w9, *w10=w+26, *w11=w+30, *w12=w+34, *w13=w+38, *w14=w+54, *w15=w+70, *w16=w+73, *w17=w+89, w18, w19, w20, w21, w22;
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
  /* #34: @10 = horzcat(@6, @5, @3, @9) */
  rr=w10;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #35: @10 = @10' */
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
  /* #41: @11 = horzcat(@6, @5, @3, @9) */
  rr=w11;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #42: @11 = @11' */
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
  /* #48: @12 = horzcat(@6, @5, @3, @9) */
  rr=w12;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w9;
  /* #49: @12 = @12' */
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
  /* #57: @14 = horzcat(@10, @11, @12, @1) */
  rr=w14;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w1; i<4; ++i) *rr++ = *cs++;
  /* #58: @10 = zeros(4x1) */
  casadi_clear(w10, 4);
  /* #59: @15 = input[3][0] */
  casadi_copy(arg[3], 3, w15);
  /* #60: (@10[1:4] += @15) */
  for (rr=w10+1, ss=w15; rr!=w10+4; rr+=1) *rr += *ss++;
  /* #61: @0 = mac(@14,@10,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w14+j, tt=w10+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #62: @14 = zeros(4x4) */
  casadi_clear(w14, 16);
  /* #63: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #64: @16 = zeros(4x4) */
  casadi_clear(w16, 16);
  /* #65: @2 = @2' */
  /* #66: @16 = mac(@10,@2,@16) */
  for (i=0, rr=w16; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w10+j, tt=w2+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #67: @17 = @16' */
  for (i=0, rr=w17, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #68: {@10, @2, @12, @1} = horzsplit(@17) */
  casadi_copy(w17, 4, w10);
  casadi_copy(w17+4, 4, w2);
  casadi_copy(w17+8, 4, w12);
  casadi_copy(w17+12, 4, w1);
  /* #69: @1 = @1' */
  /* #70: {@6, @5, @3, @9} = horzsplit(@1) */
  w6 = w1[0];
  w5 = w1[1];
  w3 = w1[2];
  w9 = w1[3];
  /* #71: (@11[0] += @9) */
  for (rr=w11+0, ss=(&w9); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #72: (@11[1] += @3) */
  for (rr=w11+1, ss=(&w3); rr!=w11+2; rr+=1) *rr += *ss++;
  /* #73: @5 = (-@5) */
  w5 = (- w5 );
  /* #74: (@11[2] += @5) */
  for (rr=w11+2, ss=(&w5); rr!=w11+3; rr+=1) *rr += *ss++;
  /* #75: (@11[3] += @6) */
  for (rr=w11+3, ss=(&w6); rr!=w11+4; rr+=1) *rr += *ss++;
  /* #76: @12 = @12' */
  /* #77: {@6, @5, @3, @9} = horzsplit(@12) */
  w6 = w12[0];
  w5 = w12[1];
  w3 = w12[2];
  w9 = w12[3];
  /* #78: @9 = (-@9) */
  w9 = (- w9 );
  /* #79: (@11[1] += @9) */
  for (rr=w11+1, ss=(&w9); rr!=w11+2; rr+=1) *rr += *ss++;
  /* #80: (@11[0] += @3) */
  for (rr=w11+0, ss=(&w3); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #81: (@11[3] += @5) */
  for (rr=w11+3, ss=(&w5); rr!=w11+4; rr+=1) *rr += *ss++;
  /* #82: (@11[2] += @6) */
  for (rr=w11+2, ss=(&w6); rr!=w11+3; rr+=1) *rr += *ss++;
  /* #83: @2 = @2' */
  /* #84: {@6, @5, @3, @9} = horzsplit(@2) */
  w6 = w2[0];
  w5 = w2[1];
  w3 = w2[2];
  w9 = w2[3];
  /* #85: (@11[2] += @9) */
  for (rr=w11+2, ss=(&w9); rr!=w11+3; rr+=1) *rr += *ss++;
  /* #86: @3 = (-@3) */
  w3 = (- w3 );
  /* #87: (@11[3] += @3) */
  for (rr=w11+3, ss=(&w3); rr!=w11+4; rr+=1) *rr += *ss++;
  /* #88: (@11[0] += @5) */
  for (rr=w11+0, ss=(&w5); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #89: (@11[1] += @6) */
  for (rr=w11+1, ss=(&w6); rr!=w11+2; rr+=1) *rr += *ss++;
  /* #90: @10 = @10' */
  /* #91: {@6, @5, @3, @9} = horzsplit(@10) */
  w6 = w10[0];
  w5 = w10[1];
  w3 = w10[2];
  w9 = w10[3];
  /* #92: @9 = (-@9) */
  w9 = (- w9 );
  /* #93: (@11[3] += @9) */
  for (rr=w11+3, ss=(&w9); rr!=w11+4; rr+=1) *rr += *ss++;
  /* #94: @3 = (-@3) */
  w3 = (- w3 );
  /* #95: (@11[2] += @3) */
  for (rr=w11+2, ss=(&w3); rr!=w11+3; rr+=1) *rr += *ss++;
  /* #96: @5 = (-@5) */
  w5 = (- w5 );
  /* #97: (@11[1] += @5) */
  for (rr=w11+1, ss=(&w5); rr!=w11+2; rr+=1) *rr += *ss++;
  /* #98: (@11[0] += @6) */
  for (rr=w11+0, ss=(&w6); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #99: @7 = @7' */
  /* #100: @14 = mac(@11,@7,@14) */
  for (i=0, rr=w14; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w11+j, tt=w7+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #101: @17 = @14' */
  for (i=0, rr=w17, cs=w14; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #102: {@7, @10, @2, @12} = horzsplit(@17) */
  casadi_copy(w17, 4, w7);
  casadi_copy(w17+4, 4, w10);
  casadi_copy(w17+8, 4, w2);
  casadi_copy(w17+12, 4, w12);
  /* #103: @12 = @12' */
  /* #104: {@6, @5, @3, @9} = horzsplit(@12) */
  w6 = w12[0];
  w5 = w12[1];
  w3 = w12[2];
  w9 = w12[3];
  /* #105: (@0[0] += @9) */
  for (rr=w0+0, ss=(&w9); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #106: @2 = @2' */
  /* #107: {@9, @8, @4, @18} = horzsplit(@2) */
  w9 = w2[0];
  w8 = w2[1];
  w4 = w2[2];
  w18 = w2[3];
  /* #108: (@0[0] += @4) */
  for (rr=w0+0, ss=(&w4); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #109: @10 = @10' */
  /* #110: {@4, @19, @20, @21} = horzsplit(@10) */
  w4 = w10[0];
  w19 = w10[1];
  w20 = w10[2];
  w21 = w10[3];
  /* #111: (@0[0] += @19) */
  for (rr=w0+0, ss=(&w19); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #112: @6 = (@6+@8) */
  w6 += w8;
  /* #113: @20 = (@20-@6) */
  w20 -= w6;
  /* #114: @7 = @7' */
  /* #115: {@6, @8, @19, @22} = horzsplit(@7) */
  w6 = w7[0];
  w8 = w7[1];
  w19 = w7[2];
  w22 = w7[3];
  /* #116: @20 = (@20+@22) */
  w20 += w22;
  /* #117: (@0[3] += @20) */
  for (rr=w0+3, ss=(&w20); rr!=w0+4; rr+=1) *rr += *ss++;
  /* #118: @9 = (@9+@21) */
  w9 += w21;
  /* #119: @5 = (@5-@9) */
  w5 -= w9;
  /* #120: @5 = (@5+@19) */
  w5 += w19;
  /* #121: (@0[2] += @5) */
  for (rr=w0+2, ss=(&w5); rr!=w0+3; rr+=1) *rr += *ss++;
  /* #122: @3 = (@3+@4) */
  w3 += w4;
  /* #123: @18 = (@18-@3) */
  w18 -= w3;
  /* #124: @18 = (@18+@8) */
  w18 += w8;
  /* #125: (@0[1] += @18) */
  for (rr=w0+1, ss=(&w18); rr!=w0+2; rr+=1) *rr += *ss++;
  /* #126: (@0[0] += @6) */
  for (rr=w0+0, ss=(&w6); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #127: output[0][0] = @0 */
  casadi_copy(w0, 4, res[0]);
  /* #128: @0 = zeros(4x1) */
  casadi_clear(w0, 4);
  /* #129: @0 = mac(@13,@11,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w13+j, tt=w11+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #130: {NULL, @15} = vertsplit(@0) */
  casadi_copy(w0+1, 3, w15);
  /* #131: output[1][0] = @15 */
  casadi_copy(w15, 3, res[1]);
  return 0;
}

/* quadrotor_cost_ext_cost_0_fun_jac:(i0[13],i1[4],i2[],i3[17])->(o0,o1[17]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+2, *rr, *ss, *tt;
  const casadi_real **arg1=arg+4, *cs;
  casadi_real w0, *w1=w+111, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+131, *w20=w+148, *w21=w+151, *w22=w+164, *w23=w+167, *w24=w+176, *w25=w+180, *w26=w+184, *w27=w+188, *w28=w+204, *w29=w+208, *w30=w+212, *w31=w+216, *w32=w+220, *w33=w+224, *w34=w+240, *w35=w+256, *w36=w+259, *w37=w+262, *w39=w+265;
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
  /* #39: @2 = 2.5 */
  w2 = 2.5000000000000000e+00;
  /* #40: (@23[0] = @2) */
  for (rr=w23+0, ss=(&w2); rr!=w23+1; rr+=1) *rr = *ss++;
  /* #41: @2 = 4 */
  w2 = 4.;
  /* #42: (@23[4] = @2) */
  for (rr=w23+4, ss=(&w2); rr!=w23+5; rr+=1) *rr = *ss++;
  /* #43: @2 = 2 */
  w2 = 2.;
  /* #44: (@23[8] = @2) */
  for (rr=w23+8, ss=(&w2); rr!=w23+9; rr+=1) *rr = *ss++;
  /* #45: @1 = mac(@22,@23,@1) */
  for (i=0, rr=w1; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w23+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #46: @0 = mac(@1,@20,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w20+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #47: @0 = (2.*@0) */
  w0 = (2.* w0 );
  /* #48: @2 = 0 */
  w2 = 0.;
  /* #49: @24 = zeros(1x4) */
  casadi_clear(w24, 4);
  /* #50: @25 = @19[13:17] */
  for (rr=w25, ss=w19+13; ss!=w19+17; ss+=1) *rr++ = *ss;
  /* #51: @3 = input[1][0] */
  w3 = arg[1] ? arg[1][0] : 0;
  /* #52: @4 = input[1][1] */
  w4 = arg[1] ? arg[1][1] : 0;
  /* #53: @5 = input[1][2] */
  w5 = arg[1] ? arg[1][2] : 0;
  /* #54: @6 = input[1][3] */
  w6 = arg[1] ? arg[1][3] : 0;
  /* #55: @26 = vertcat(@3, @4, @5, @6) */
  rr=w26;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #56: @25 = (@25-@26) */
  for (i=0, rr=w25, cs=w26; i<4; ++i) (*rr++) -= (*cs++);
  /* #57: @26 = @25' */
  casadi_copy(w25, 4, w26);
  /* #58: @27 = zeros(4x4) */
  casadi_clear(w27, 16);
  /* #59: @3 = 0.671141 */
  w3 = 6.7114093959731547e-01;
  /* #60: (@27[0] = @3) */
  for (rr=w27+0, ss=(&w3); rr!=w27+1; rr+=1) *rr = *ss++;
  /* #61: @3 = 600 */
  w3 = 600.;
  /* #62: (@27[5] = @3) */
  for (rr=w27+5, ss=(&w3); rr!=w27+6; rr+=1) *rr = *ss++;
  /* #63: @3 = 600 */
  w3 = 600.;
  /* #64: (@27[10] = @3) */
  for (rr=w27+10, ss=(&w3); rr!=w27+11; rr+=1) *rr = *ss++;
  /* #65: @3 = 600 */
  w3 = 600.;
  /* #66: (@27[15] = @3) */
  for (rr=w27+15, ss=(&w3); rr!=w27+16; rr+=1) *rr = *ss++;
  /* #67: @24 = mac(@26,@27,@24) */
  for (i=0, rr=w24; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w26+j, tt=w27+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #68: @2 = mac(@24,@25,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w24+j, tt=w25+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #69: @0 = (@0+@2) */
  w0 += w2;
  /* #70: @2 = 10 */
  w2 = 10.;
  /* #71: @3 = 0 */
  w3 = 0.;
  /* #72: @26 = [1, 0, 0, 0] */
  casadi_copy(casadi_c0, 4, w26);
  /* #73: @4 = 0 */
  w4 = 0.;
  /* #74: @28 = zeros(4x1) */
  casadi_clear(w28, 4);
  /* #75: @29 = horzcat(@8, @9, @10, @11) */
  rr=w29;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #76: @29 = @29' */
  /* #77: @5 = (-@9) */
  w5 = (- w9 );
  /* #78: @6 = (-@10) */
  w6 = (- w10 );
  /* #79: @30 = horzcat(@5, @8, @11, @6) */
  rr=w30;
  *rr++ = w5;
  *rr++ = w8;
  *rr++ = w11;
  *rr++ = w6;
  /* #80: @30 = @30' */
  /* #81: @11 = (-@11) */
  w11 = (- w11 );
  /* #82: @31 = horzcat(@6, @11, @8, @9) */
  rr=w31;
  *rr++ = w6;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w9;
  /* #83: @31 = @31' */
  /* #84: @32 = horzcat(@11, @10, @5, @8) */
  rr=w32;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w8;
  /* #85: @32 = @32' */
  /* #86: @33 = horzcat(@29, @30, @31, @32) */
  rr=w33;
  for (i=0, cs=w29; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w30; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w31; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w32; i<4; ++i) *rr++ = *cs++;
  /* #87: @34 = @33' */
  for (i=0, rr=w34, cs=w33; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #88: @29 = @21[6:10] */
  for (rr=w29, ss=w21+6; ss!=w21+10; ss+=1) *rr++ = *ss;
  /* #89: @28 = mac(@34,@29,@28) */
  for (i=0, rr=w28; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w34+j, tt=w29+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #90: @11 = @28[0] */
  for (rr=(&w11), ss=w28+0; ss!=w28+1; ss+=1) *rr++ = *ss;
  /* #91: @4 = (@4<@11) */
  w4  = (w4<w11);
  /* #92: @29 = (@4?@28:0) */
  for (i=0, rr=w29, cs=w28; i<4; ++i) (*rr++)  = (w4?(*cs++):0);
  /* #93: @11 = (!@4) */
  w11 = (! w4 );
  /* #94: @28 = (-@28) */
  for (i=0, rr=w28, cs=w28; i<4; ++i) *rr++ = (- *cs++ );
  /* #95: @28 = (@11?@28:0) */
  for (i=0, rr=w28, cs=w28; i<4; ++i) (*rr++)  = (w11?(*cs++):0);
  /* #96: @29 = (@29+@28) */
  for (i=0, rr=w29, cs=w28; i<4; ++i) (*rr++) += (*cs++);
  /* #97: @26 = (@26-@29) */
  for (i=0, rr=w26, cs=w29; i<4; ++i) (*rr++) -= (*cs++);
  /* #98: @29 = @26' */
  casadi_copy(w26, 4, w29);
  /* #99: @3 = mac(@29,@26,@3) */
  for (i=0, rr=(&w3); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w29+j, tt=w26+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #100: @2 = (@2*@3) */
  w2 *= w3;
  /* #101: @0 = (@0+@2) */
  w0 += w2;
  /* #102: @2 = 0 */
  w2 = 0.;
  /* #103: @22 = @21[10:13] */
  for (rr=w22, ss=w21+10; ss!=w21+13; ss+=1) *rr++ = *ss;
  /* #104: @35 = @22' */
  casadi_copy(w22, 3, w35);
  /* #105: @2 = mac(@35,@22,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w35+j, tt=w22+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #106: @0 = (@0+@2) */
  w0 += w2;
  /* #107: @2 = 0 */
  w2 = 0.;
  /* #108: @29 = @21[6:10] */
  for (rr=w29, ss=w21+6; ss!=w21+10; ss+=1) *rr++ = *ss;
  /* #109: @35 = @21[3:6] */
  for (rr=w35, ss=w21+3; ss!=w21+6; ss+=1) *rr++ = *ss;
  /* #110: @36 = f_rot_inv(@29, @35) */
  arg1[0]=w29;
  arg1[1]=w35;
  res1[0]=w36;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #111: @37 = @36' */
  casadi_copy(w36, 3, w37);
  /* #112: @2 = mac(@37,@36,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w37+j, tt=w36+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #113: @0 = (@0+@2) */
  w0 += w2;
  /* #114: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #115: @24 = @24' */
  /* #116: @28 = zeros(1x4) */
  casadi_clear(w28, 4);
  /* #117: @25 = @25' */
  /* #118: @34 = @27' */
  for (i=0, rr=w34, cs=w27; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #119: @28 = mac(@25,@34,@28) */
  for (i=0, rr=w28; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w25+j, tt=w34+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #120: @28 = @28' */
  /* #121: @24 = (@24+@28) */
  for (i=0, rr=w24, cs=w28; i<4; ++i) (*rr++) += (*cs++);
  /* #122: @24 = (-@24) */
  for (i=0, rr=w24, cs=w24; i<4; ++i) *rr++ = (- *cs++ );
  /* #123: {@0, @2, @3, @10} = vertsplit(@24) */
  w0 = w24[0];
  w2 = w24[1];
  w3 = w24[2];
  w10 = w24[3];
  /* #124: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #125: output[1][1] = @2 */
  if (res[1]) res[1][1] = w2;
  /* #126: output[1][2] = @3 */
  if (res[1]) res[1][2] = w3;
  /* #127: output[1][3] = @10 */
  if (res[1]) res[1][3] = w10;
  /* #128: @21 = zeros(13x1) */
  casadi_clear(w21, 13);
  /* #129: @38 = zeros(3x1,0nz) */
  /* #130: @36 = (2.*@36) */
  for (i=0, rr=w36, cs=w36; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #131: {@24, @37} = adj1_f_rot_inv(@29, @35, @38, @36) */
  arg1[0]=w29;
  arg1[1]=w35;
  arg1[2]=0;
  arg1[3]=w36;
  res1[0]=w24;
  res1[1]=w37;
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #132: (@21[3:6] += @37) */
  for (rr=w21+3, ss=w37; rr!=w21+6; rr+=1) *rr += *ss++;
  /* #133: (@21[6:10] += @24) */
  for (rr=w21+6, ss=w24; rr!=w21+10; rr+=1) *rr += *ss++;
  /* #134: @22 = (2.*@22) */
  for (i=0, rr=w22, cs=w22; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #135: (@21[10:13] += @22) */
  for (rr=w21+10, ss=w22; rr!=w21+13; rr+=1) *rr += *ss++;
  /* #136: @24 = zeros(4x1) */
  casadi_clear(w24, 4);
  /* #137: @10 = 1 */
  w10 = 1.;
  /* #138: @11 = (@11?@10:0) */
  w11  = (w11?w10:0);
  /* #139: @10 = 10 */
  w10 = 10.;
  /* #140: @29 = (@10*@26) */
  for (i=0, rr=w29, cs=w26; i<4; ++i) (*rr++)  = (w10*(*cs++));
  /* #141: @26 = @26' */
  /* #142: @26 = (@10*@26) */
  for (i=0, rr=w26, cs=w26; i<4; ++i) (*rr++)  = (w10*(*cs++));
  /* #143: @26 = @26' */
  /* #144: @29 = (@29+@26) */
  for (i=0, rr=w29, cs=w26; i<4; ++i) (*rr++) += (*cs++);
  /* #145: @26 = (@11*@29) */
  for (i=0, rr=w26, cs=w29; i<4; ++i) (*rr++)  = (w11*(*cs++));
  /* #146: @11 = 1 */
  w11 = 1.;
  /* #147: @4 = (@4?@11:0) */
  w4  = (w4?w11:0);
  /* #148: @29 = (@4*@29) */
  for (i=0, rr=w29, cs=w29; i<4; ++i) (*rr++)  = (w4*(*cs++));
  /* #149: @26 = (@26-@29) */
  for (i=0, rr=w26, cs=w29; i<4; ++i) (*rr++) -= (*cs++);
  /* #150: @24 = mac(@33,@26,@24) */
  for (i=0, rr=w24; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w33+j, tt=w26+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #151: (@21[6:10] += @24) */
  for (rr=w21+6, ss=w24; rr!=w21+10; rr+=1) *rr += *ss++;
  /* #152: @1 = @1' */
  /* #153: @1 = (2.*@1) */
  for (i=0, rr=w1, cs=w1; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #154: @22 = zeros(1x3) */
  casadi_clear(w22, 3);
  /* #155: @20 = @20' */
  /* #156: @20 = (2.*@20) */
  for (i=0, rr=w20, cs=w20; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #157: @39 = @23' */
  for (i=0, rr=w39, cs=w23; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #158: @22 = mac(@20,@39,@22) */
  for (i=0, rr=w22; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w39+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #159: @22 = @22' */
  /* #160: @1 = (@1+@22) */
  for (i=0, rr=w1, cs=w22; i<3; ++i) (*rr++) += (*cs++);
  /* #161: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<3; ++i) *rr++ = (- *cs++ );
  /* #162: (@21[:3] += @1) */
  for (rr=w21+0, ss=w1; rr!=w21+3; rr+=1) *rr += *ss++;
  /* #163: {@4, @11, @10, @3, @2, @0, @5, @8, @6, @9, @7, @12, @13} = vertsplit(@21) */
  w4 = w21[0];
  w11 = w21[1];
  w10 = w21[2];
  w3 = w21[3];
  w2 = w21[4];
  w0 = w21[5];
  w5 = w21[6];
  w8 = w21[7];
  w6 = w21[8];
  w9 = w21[9];
  w7 = w21[10];
  w12 = w21[11];
  w13 = w21[12];
  /* #164: output[1][4] = @4 */
  if (res[1]) res[1][4] = w4;
  /* #165: output[1][5] = @11 */
  if (res[1]) res[1][5] = w11;
  /* #166: output[1][6] = @10 */
  if (res[1]) res[1][6] = w10;
  /* #167: output[1][7] = @3 */
  if (res[1]) res[1][7] = w3;
  /* #168: output[1][8] = @2 */
  if (res[1]) res[1][8] = w2;
  /* #169: output[1][9] = @0 */
  if (res[1]) res[1][9] = w0;
  /* #170: output[1][10] = @5 */
  if (res[1]) res[1][10] = w5;
  /* #171: output[1][11] = @8 */
  if (res[1]) res[1][11] = w8;
  /* #172: output[1][12] = @6 */
  if (res[1]) res[1][12] = w6;
  /* #173: output[1][13] = @9 */
  if (res[1]) res[1][13] = w9;
  /* #174: output[1][14] = @7 */
  if (res[1]) res[1][14] = w7;
  /* #175: output[1][15] = @12 */
  if (res[1]) res[1][15] = w12;
  /* #176: output[1][16] = @13 */
  if (res[1]) res[1][16] = w13;
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
  if (sz_arg) *sz_arg = 21;
  if (sz_res) *sz_res = 15;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 274;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
