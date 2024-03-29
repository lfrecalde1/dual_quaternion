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
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_c2 CASADI_PREFIX(c2)
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

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[16] = {0., 0., 0., 0., 0., 1.2500000000000000e+01, 0., 0., 0., 0., 1.2500000000000000e+01, 0., 0., 0., 0., 1.2500000000000000e+01};
static const casadi_real casadi_c1[16] = {6.7114093959731547e-01, 0., 0., 0., 0., 600., 0., 0., 0., 0., 600., 0., 0., 0., 0., 600.};
static const casadi_real casadi_c2[4] = {1., 0., 0., 0.};

/* f_trans:(i0[8])->(o0[4]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, w1, w2, w3, w4, *w5=w+12, *w6=w+16, *w7=w+20, *w8=w+24, *w9=w+28, *w10=w+44;
  /* #0: @0 = zeros(4x1) */
  casadi_clear(w0, 4);
  /* #1: @1 = input[0][0] */
  w1 = arg[0] ? arg[0][0] : 0;
  /* #2: @2 = input[0][1] */
  w2 = arg[0] ? arg[0][1] : 0;
  /* #3: @2 = (-@2) */
  w2 = (- w2 );
  /* #4: @3 = input[0][2] */
  w3 = arg[0] ? arg[0][2] : 0;
  /* #5: @3 = (-@3) */
  w3 = (- w3 );
  /* #6: @4 = input[0][3] */
  w4 = arg[0] ? arg[0][3] : 0;
  /* #7: @4 = (-@4) */
  w4 = (- w4 );
  /* #8: @5 = vertcat(@1, @2, @3, @4) */
  rr=w5;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #9: @5 = (2.*@5) */
  for (i=0, rr=w5, cs=w5; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #10: @1 = @5[0] */
  for (rr=(&w1), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #11: @2 = @5[1] */
  for (rr=(&w2), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #12: @2 = (-@2) */
  w2 = (- w2 );
  /* #13: @3 = @5[2] */
  for (rr=(&w3), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #14: @3 = (-@3) */
  w3 = (- w3 );
  /* #15: @4 = @5[3] */
  for (rr=(&w4), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #16: @4 = (-@4) */
  w4 = (- w4 );
  /* #17: @6 = horzcat(@1, @2, @3, @4) */
  rr=w6;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #18: @6 = @6' */
  /* #19: @1 = @5[1] */
  for (rr=(&w1), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #20: @2 = @5[0] */
  for (rr=(&w2), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #21: @3 = @5[3] */
  for (rr=(&w3), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #22: @3 = (-@3) */
  w3 = (- w3 );
  /* #23: @4 = @5[2] */
  for (rr=(&w4), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #24: @7 = horzcat(@1, @2, @3, @4) */
  rr=w7;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #25: @7 = @7' */
  /* #26: @1 = @5[2] */
  for (rr=(&w1), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #27: @2 = @5[3] */
  for (rr=(&w2), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #28: @3 = @5[0] */
  for (rr=(&w3), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #29: @4 = @5[1] */
  for (rr=(&w4), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #30: @4 = (-@4) */
  w4 = (- w4 );
  /* #31: @8 = horzcat(@1, @2, @3, @4) */
  rr=w8;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #32: @8 = @8' */
  /* #33: @1 = @5[3] */
  for (rr=(&w1), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #34: @2 = @5[2] */
  for (rr=(&w2), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #35: @2 = (-@2) */
  w2 = (- w2 );
  /* #36: @3 = @5[1] */
  for (rr=(&w3), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #37: @4 = @5[0] */
  for (rr=(&w4), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #38: @5 = horzcat(@1, @2, @3, @4) */
  rr=w5;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #39: @5 = @5' */
  /* #40: @9 = horzcat(@6, @7, @8, @5) */
  rr=w9;
  for (i=0, cs=w6; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w8; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w5; i<4; ++i) *rr++ = *cs++;
  /* #41: @10 = @9' */
  for (i=0, rr=w10, cs=w9; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #42: @1 = input[0][4] */
  w1 = arg[0] ? arg[0][4] : 0;
  /* #43: @2 = input[0][5] */
  w2 = arg[0] ? arg[0][5] : 0;
  /* #44: @3 = input[0][6] */
  w3 = arg[0] ? arg[0][6] : 0;
  /* #45: @4 = input[0][7] */
  w4 = arg[0] ? arg[0][7] : 0;
  /* #46: @6 = vertcat(@1, @2, @3, @4) */
  rr=w6;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  /* #47: @0 = mac(@10,@6,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w10+j, tt=w6+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #48: output[0][0] = @0 */
  casadi_copy(w0, 4, res[0]);
  return 0;
}

/* adj1_f_trans:(i0[8],out_o0[4x1,0nz],adj_o0[4])->(adj_i0[8]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, *w1=w+8, *w2=w+24, w3, w4, w5, w6, *w7=w+32, *w8=w+36, *w9=w+52, *w10=w+56, *w11=w+60;
  /* #0: @0 = zeros(4x1) */
  casadi_clear(w0, 4);
  /* #1: @1 = zeros(4x4) */
  casadi_clear(w1, 16);
  /* #2: @2 = input[2][0] */
  casadi_copy(arg[2], 4, w2);
  /* #3: @3 = input[0][4] */
  w3 = arg[0] ? arg[0][4] : 0;
  /* #4: @4 = input[0][5] */
  w4 = arg[0] ? arg[0][5] : 0;
  /* #5: @5 = input[0][6] */
  w5 = arg[0] ? arg[0][6] : 0;
  /* #6: @6 = input[0][7] */
  w6 = arg[0] ? arg[0][7] : 0;
  /* #7: @7 = vertcat(@3, @4, @5, @6) */
  rr=w7;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #8: @7 = @7' */
  /* #9: @1 = mac(@2,@7,@1) */
  for (i=0, rr=w1; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w2+j, tt=w7+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #10: @8 = @1' */
  for (i=0, rr=w8, cs=w1; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #11: {@7, @9, @10, @11} = horzsplit(@8) */
  casadi_copy(w8, 4, w7);
  casadi_copy(w8+4, 4, w9);
  casadi_copy(w8+8, 4, w10);
  casadi_copy(w8+12, 4, w11);
  /* #12: @11 = @11' */
  /* #13: {@3, @4, @5, @6} = horzsplit(@11) */
  w3 = w11[0];
  w4 = w11[1];
  w5 = w11[2];
  w6 = w11[3];
  /* #14: (@0[0] += @6) */
  for (rr=w0+0, ss=(&w6); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #15: (@0[1] += @5) */
  for (rr=w0+1, ss=(&w5); rr!=w0+2; rr+=1) *rr += *ss++;
  /* #16: @4 = (-@4) */
  w4 = (- w4 );
  /* #17: (@0[2] += @4) */
  for (rr=w0+2, ss=(&w4); rr!=w0+3; rr+=1) *rr += *ss++;
  /* #18: (@0[3] += @3) */
  for (rr=w0+3, ss=(&w3); rr!=w0+4; rr+=1) *rr += *ss++;
  /* #19: @10 = @10' */
  /* #20: {@3, @4, @5, @6} = horzsplit(@10) */
  w3 = w10[0];
  w4 = w10[1];
  w5 = w10[2];
  w6 = w10[3];
  /* #21: @6 = (-@6) */
  w6 = (- w6 );
  /* #22: (@0[1] += @6) */
  for (rr=w0+1, ss=(&w6); rr!=w0+2; rr+=1) *rr += *ss++;
  /* #23: (@0[0] += @5) */
  for (rr=w0+0, ss=(&w5); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #24: (@0[3] += @4) */
  for (rr=w0+3, ss=(&w4); rr!=w0+4; rr+=1) *rr += *ss++;
  /* #25: (@0[2] += @3) */
  for (rr=w0+2, ss=(&w3); rr!=w0+3; rr+=1) *rr += *ss++;
  /* #26: @9 = @9' */
  /* #27: {@3, @4, @5, @6} = horzsplit(@9) */
  w3 = w9[0];
  w4 = w9[1];
  w5 = w9[2];
  w6 = w9[3];
  /* #28: (@0[2] += @6) */
  for (rr=w0+2, ss=(&w6); rr!=w0+3; rr+=1) *rr += *ss++;
  /* #29: @5 = (-@5) */
  w5 = (- w5 );
  /* #30: (@0[3] += @5) */
  for (rr=w0+3, ss=(&w5); rr!=w0+4; rr+=1) *rr += *ss++;
  /* #31: (@0[0] += @4) */
  for (rr=w0+0, ss=(&w4); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #32: (@0[1] += @3) */
  for (rr=w0+1, ss=(&w3); rr!=w0+2; rr+=1) *rr += *ss++;
  /* #33: @7 = @7' */
  /* #34: {@3, @4, @5, @6} = horzsplit(@7) */
  w3 = w7[0];
  w4 = w7[1];
  w5 = w7[2];
  w6 = w7[3];
  /* #35: @6 = (-@6) */
  w6 = (- w6 );
  /* #36: (@0[3] += @6) */
  for (rr=w0+3, ss=(&w6); rr!=w0+4; rr+=1) *rr += *ss++;
  /* #37: @5 = (-@5) */
  w5 = (- w5 );
  /* #38: (@0[2] += @5) */
  for (rr=w0+2, ss=(&w5); rr!=w0+3; rr+=1) *rr += *ss++;
  /* #39: @4 = (-@4) */
  w4 = (- w4 );
  /* #40: (@0[1] += @4) */
  for (rr=w0+1, ss=(&w4); rr!=w0+2; rr+=1) *rr += *ss++;
  /* #41: (@0[0] += @3) */
  for (rr=w0+0, ss=(&w3); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #42: @0 = (2.*@0) */
  for (i=0, rr=w0, cs=w0; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #43: {@3, @4, @5, @6} = vertsplit(@0) */
  w3 = w0[0];
  w4 = w0[1];
  w5 = w0[2];
  w6 = w0[3];
  /* #44: output[0][0] = @3 */
  if (res[0]) res[0][0] = w3;
  /* #45: @4 = (-@4) */
  w4 = (- w4 );
  /* #46: output[0][1] = @4 */
  if (res[0]) res[0][1] = w4;
  /* #47: @5 = (-@5) */
  w5 = (- w5 );
  /* #48: output[0][2] = @5 */
  if (res[0]) res[0][2] = w5;
  /* #49: @6 = (-@6) */
  w6 = (- w6 );
  /* #50: output[0][3] = @6 */
  if (res[0]) res[0][3] = w6;
  /* #51: @0 = zeros(4x1) */
  casadi_clear(w0, 4);
  /* #52: @6 = input[0][0] */
  w6 = arg[0] ? arg[0][0] : 0;
  /* #53: @5 = input[0][1] */
  w5 = arg[0] ? arg[0][1] : 0;
  /* #54: @5 = (-@5) */
  w5 = (- w5 );
  /* #55: @4 = input[0][2] */
  w4 = arg[0] ? arg[0][2] : 0;
  /* #56: @4 = (-@4) */
  w4 = (- w4 );
  /* #57: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #58: @3 = (-@3) */
  w3 = (- w3 );
  /* #59: @7 = vertcat(@6, @5, @4, @3) */
  rr=w7;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w4;
  *rr++ = w3;
  /* #60: @7 = (2.*@7) */
  for (i=0, rr=w7, cs=w7; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #61: @6 = @7[0] */
  for (rr=(&w6), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #62: @5 = @7[1] */
  for (rr=(&w5), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #63: @5 = (-@5) */
  w5 = (- w5 );
  /* #64: @4 = @7[2] */
  for (rr=(&w4), ss=w7+2; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #65: @4 = (-@4) */
  w4 = (- w4 );
  /* #66: @3 = @7[3] */
  for (rr=(&w3), ss=w7+3; ss!=w7+4; ss+=1) *rr++ = *ss;
  /* #67: @3 = (-@3) */
  w3 = (- w3 );
  /* #68: @9 = horzcat(@6, @5, @4, @3) */
  rr=w9;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w4;
  *rr++ = w3;
  /* #69: @9 = @9' */
  /* #70: @6 = @7[1] */
  for (rr=(&w6), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #71: @5 = @7[0] */
  for (rr=(&w5), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #72: @4 = @7[3] */
  for (rr=(&w4), ss=w7+3; ss!=w7+4; ss+=1) *rr++ = *ss;
  /* #73: @4 = (-@4) */
  w4 = (- w4 );
  /* #74: @3 = @7[2] */
  for (rr=(&w3), ss=w7+2; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #75: @10 = horzcat(@6, @5, @4, @3) */
  rr=w10;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w4;
  *rr++ = w3;
  /* #76: @10 = @10' */
  /* #77: @6 = @7[2] */
  for (rr=(&w6), ss=w7+2; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #78: @5 = @7[3] */
  for (rr=(&w5), ss=w7+3; ss!=w7+4; ss+=1) *rr++ = *ss;
  /* #79: @4 = @7[0] */
  for (rr=(&w4), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #80: @3 = @7[1] */
  for (rr=(&w3), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #81: @3 = (-@3) */
  w3 = (- w3 );
  /* #82: @11 = horzcat(@6, @5, @4, @3) */
  rr=w11;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w4;
  *rr++ = w3;
  /* #83: @11 = @11' */
  /* #84: @6 = @7[3] */
  for (rr=(&w6), ss=w7+3; ss!=w7+4; ss+=1) *rr++ = *ss;
  /* #85: @5 = @7[2] */
  for (rr=(&w5), ss=w7+2; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #86: @5 = (-@5) */
  w5 = (- w5 );
  /* #87: @4 = @7[1] */
  for (rr=(&w4), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #88: @3 = @7[0] */
  for (rr=(&w3), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #89: @7 = horzcat(@6, @5, @4, @3) */
  rr=w7;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w4;
  *rr++ = w3;
  /* #90: @7 = @7' */
  /* #91: @8 = horzcat(@9, @10, @11, @7) */
  rr=w8;
  for (i=0, cs=w9; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  /* #92: @0 = mac(@8,@2,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w8+j, tt=w2+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #93: {@6, @5, @4, @3} = vertsplit(@0) */
  w6 = w0[0];
  w5 = w0[1];
  w4 = w0[2];
  w3 = w0[3];
  /* #94: output[0][4] = @6 */
  if (res[0]) res[0][4] = w6;
  /* #95: output[0][5] = @5 */
  if (res[0]) res[0][5] = w5;
  /* #96: output[0][6] = @4 */
  if (res[0]) res[0][6] = w4;
  /* #97: output[0][7] = @3 */
  if (res[0]) res[0][7] = w3;
  return 0;
}

/* quadrotor_cost_ext_cost_fun_jac:(i0[14],i1[4],i2[],i3[18])->(o0,o1[18]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+2, *rr, *ss, *tt;
  const casadi_real **arg1=arg+4, *cs;
  casadi_real w0, *w1=w+65, *w2=w+69, *w3=w+87, *w4=w+95, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+113, *w20=w+127, *w21=w+131, *w22=w+147, *w23=w+151, *w24=w+155, *w25=w+171, *w26=w+174, *w27=w+177, *w28=w+181, *w29=w+185, *w30=w+189, *w31=w+193, *w32=w+197, *w33=w+213, *w34=w+229, *w36=w+232;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = zeros(1x4) */
  casadi_clear(w1, 4);
  /* #2: @2 = input[3][0] */
  casadi_copy(arg[3], 18, w2);
  /* #3: @3 = @2[:8] */
  for (rr=w3, ss=w2+0; ss!=w2+8; ss+=1) *rr++ = *ss;
  /* #4: @4 = f_trans(@3) */
  arg1[0]=w3;
  res1[0]=w4;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #5: @5 = input[0][0] */
  w5 = arg[0] ? arg[0][0] : 0;
  /* #6: @6 = input[0][1] */
  w6 = arg[0] ? arg[0][1] : 0;
  /* #7: @7 = input[0][2] */
  w7 = arg[0] ? arg[0][2] : 0;
  /* #8: @8 = input[0][3] */
  w8 = arg[0] ? arg[0][3] : 0;
  /* #9: @9 = input[0][4] */
  w9 = arg[0] ? arg[0][4] : 0;
  /* #10: @10 = input[0][5] */
  w10 = arg[0] ? arg[0][5] : 0;
  /* #11: @11 = input[0][6] */
  w11 = arg[0] ? arg[0][6] : 0;
  /* #12: @12 = input[0][7] */
  w12 = arg[0] ? arg[0][7] : 0;
  /* #13: @13 = input[0][8] */
  w13 = arg[0] ? arg[0][8] : 0;
  /* #14: @14 = input[0][9] */
  w14 = arg[0] ? arg[0][9] : 0;
  /* #15: @15 = input[0][10] */
  w15 = arg[0] ? arg[0][10] : 0;
  /* #16: @16 = input[0][11] */
  w16 = arg[0] ? arg[0][11] : 0;
  /* #17: @17 = input[0][12] */
  w17 = arg[0] ? arg[0][12] : 0;
  /* #18: @18 = input[0][13] */
  w18 = arg[0] ? arg[0][13] : 0;
  /* #19: @19 = vertcat(@5, @6, @7, @8, @9, @10, @11, @12, @13, @14, @15, @16, @17, @18) */
  rr=w19;
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
  /* #20: @3 = @19[:8] */
  for (rr=w3, ss=w19+0; ss!=w19+8; ss+=1) *rr++ = *ss;
  /* #21: @20 = f_trans(@3) */
  arg1[0]=w3;
  res1[0]=w20;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #22: @4 = (@4-@20) */
  for (i=0, rr=w4, cs=w20; i<4; ++i) (*rr++) -= (*cs++);
  /* #23: @20 = @4' */
  casadi_copy(w4, 4, w20);
  /* #24: @21 = 
  [[0, 0, 0, 0], 
   [0, 12.5, 0, 0], 
   [0, 0, 12.5, 0], 
   [0, 0, 0, 12.5]] */
  casadi_copy(casadi_c0, 16, w21);
  /* #25: @1 = mac(@20,@21,@1) */
  for (i=0, rr=w1; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w21+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #26: @0 = mac(@1,@4,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w4+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #27: @5 = 0 */
  w5 = 0.;
  /* #28: @20 = zeros(1x4) */
  casadi_clear(w20, 4);
  /* #29: @22 = @2[14:18] */
  for (rr=w22, ss=w2+14; ss!=w2+18; ss+=1) *rr++ = *ss;
  /* #30: @6 = input[1][0] */
  w6 = arg[1] ? arg[1][0] : 0;
  /* #31: @7 = input[1][1] */
  w7 = arg[1] ? arg[1][1] : 0;
  /* #32: @8 = input[1][2] */
  w8 = arg[1] ? arg[1][2] : 0;
  /* #33: @9 = input[1][3] */
  w9 = arg[1] ? arg[1][3] : 0;
  /* #34: @23 = vertcat(@6, @7, @8, @9) */
  rr=w23;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  /* #35: @22 = (@22-@23) */
  for (i=0, rr=w22, cs=w23; i<4; ++i) (*rr++) -= (*cs++);
  /* #36: @23 = @22' */
  casadi_copy(w22, 4, w23);
  /* #37: @24 = 
  [[0.671141, 0, 0, 0], 
   [0, 600, 0, 0], 
   [0, 0, 600, 0], 
   [0, 0, 0, 600]] */
  casadi_copy(casadi_c1, 16, w24);
  /* #38: @20 = mac(@23,@24,@20) */
  for (i=0, rr=w20; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w23+j, tt=w24+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #39: @5 = mac(@20,@22,@5) */
  for (i=0, rr=(&w5); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w22+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #40: @0 = (@0+@5) */
  w0 += w5;
  /* #41: @5 = 0 */
  w5 = 0.;
  /* #42: @25 = @19[8:11] */
  for (rr=w25, ss=w19+8; ss!=w19+11; ss+=1) *rr++ = *ss;
  /* #43: @26 = @25' */
  casadi_copy(w25, 3, w26);
  /* #44: @5 = mac(@26,@25,@5) */
  for (i=0, rr=(&w5); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w26+j, tt=w25+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #45: @0 = (@0+@5) */
  w0 += w5;
  /* #46: @5 = 10 */
  w5 = 10.;
  /* #47: @6 = 0 */
  w6 = 0.;
  /* #48: @23 = [1, 0, 0, 0] */
  casadi_copy(casadi_c2, 4, w23);
  /* #49: @7 = 0 */
  w7 = 0.;
  /* #50: @27 = zeros(4x1) */
  casadi_clear(w27, 4);
  /* #51: @8 = @2[0] */
  for (rr=(&w8), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #52: @9 = @2[1] */
  for (rr=(&w9), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #53: @10 = @2[2] */
  for (rr=(&w10), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #54: @11 = @2[3] */
  for (rr=(&w11), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #55: @28 = horzcat(@8, @9, @10, @11) */
  rr=w28;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #56: @28 = @28' */
  /* #57: @8 = (-@9) */
  w8 = (- w9 );
  /* #58: @12 = @2[0] */
  for (rr=(&w12), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #59: @13 = (-@10) */
  w13 = (- w10 );
  /* #60: @29 = horzcat(@8, @12, @11, @13) */
  rr=w29;
  *rr++ = w8;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w13;
  /* #61: @29 = @29' */
  /* #62: @11 = (-@11) */
  w11 = (- w11 );
  /* #63: @12 = @2[0] */
  for (rr=(&w12), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #64: @30 = horzcat(@13, @11, @12, @9) */
  rr=w30;
  *rr++ = w13;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w9;
  /* #65: @30 = @30' */
  /* #66: @13 = @2[0] */
  for (rr=(&w13), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #67: @31 = horzcat(@11, @10, @8, @13) */
  rr=w31;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w13;
  /* #68: @31 = @31' */
  /* #69: @32 = horzcat(@28, @29, @30, @31) */
  rr=w32;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w29; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w30; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w31; i<4; ++i) *rr++ = *cs++;
  /* #70: @33 = @32' */
  for (i=0, rr=w33, cs=w32; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #71: @28 = @19[:4] */
  for (rr=w28, ss=w19+0; ss!=w19+4; ss+=1) *rr++ = *ss;
  /* #72: @27 = mac(@33,@28,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w33+j, tt=w28+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #73: @11 = @27[0] */
  for (rr=(&w11), ss=w27+0; ss!=w27+1; ss+=1) *rr++ = *ss;
  /* #74: @7 = (@7<@11) */
  w7  = (w7<w11);
  /* #75: @28 = (@7?@27:0) */
  for (i=0, rr=w28, cs=w27; i<4; ++i) (*rr++)  = (w7?(*cs++):0);
  /* #76: @11 = (!@7) */
  w11 = (! w7 );
  /* #77: @27 = (-@27) */
  for (i=0, rr=w27, cs=w27; i<4; ++i) *rr++ = (- *cs++ );
  /* #78: @27 = (@11?@27:0) */
  for (i=0, rr=w27, cs=w27; i<4; ++i) (*rr++)  = (w11?(*cs++):0);
  /* #79: @28 = (@28+@27) */
  for (i=0, rr=w28, cs=w27; i<4; ++i) (*rr++) += (*cs++);
  /* #80: @23 = (@23-@28) */
  for (i=0, rr=w23, cs=w28; i<4; ++i) (*rr++) -= (*cs++);
  /* #81: @28 = @23' */
  casadi_copy(w23, 4, w28);
  /* #82: @6 = mac(@28,@23,@6) */
  for (i=0, rr=(&w6); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w28+j, tt=w23+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #83: @5 = (@5*@6) */
  w5 *= w6;
  /* #84: @0 = (@0+@5) */
  w0 += w5;
  /* #85: @5 = 0.5 */
  w5 = 5.0000000000000000e-01;
  /* #86: @6 = 0 */
  w6 = 0.;
  /* #87: @26 = @19[11:14] */
  for (rr=w26, ss=w19+11; ss!=w19+14; ss+=1) *rr++ = *ss;
  /* #88: @34 = @26' */
  casadi_copy(w26, 3, w34);
  /* #89: @6 = mac(@34,@26,@6) */
  for (i=0, rr=(&w6); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w34+j, tt=w26+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #90: @5 = (@5*@6) */
  w5 *= w6;
  /* #91: @0 = (@0+@5) */
  w0 += w5;
  /* #92: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #93: @20 = @20' */
  /* #94: @28 = zeros(1x4) */
  casadi_clear(w28, 4);
  /* #95: @22 = @22' */
  /* #96: @33 = @24' */
  for (i=0, rr=w33, cs=w24; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #97: @28 = mac(@22,@33,@28) */
  for (i=0, rr=w28; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w33+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #98: @28 = @28' */
  /* #99: @20 = (@20+@28) */
  for (i=0, rr=w20, cs=w28; i<4; ++i) (*rr++) += (*cs++);
  /* #100: @20 = (-@20) */
  for (i=0, rr=w20, cs=w20; i<4; ++i) *rr++ = (- *cs++ );
  /* #101: {@0, @5, @6, @10} = vertsplit(@20) */
  w0 = w20[0];
  w5 = w20[1];
  w6 = w20[2];
  w10 = w20[3];
  /* #102: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #103: output[1][1] = @5 */
  if (res[1]) res[1][1] = w5;
  /* #104: output[1][2] = @6 */
  if (res[1]) res[1][2] = w6;
  /* #105: output[1][3] = @10 */
  if (res[1]) res[1][3] = w10;
  /* #106: @19 = zeros(14x1) */
  casadi_clear(w19, 14);
  /* #107: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #108: @34 = (@10*@26) */
  for (i=0, rr=w34, cs=w26; i<3; ++i) (*rr++)  = (w10*(*cs++));
  /* #109: @26 = @26' */
  /* #110: @26 = (@10*@26) */
  for (i=0, rr=w26, cs=w26; i<3; ++i) (*rr++)  = (w10*(*cs++));
  /* #111: @26 = @26' */
  /* #112: @34 = (@34+@26) */
  for (i=0, rr=w34, cs=w26; i<3; ++i) (*rr++) += (*cs++);
  /* #113: (@19[11:14] += @34) */
  for (rr=w19+11, ss=w34; rr!=w19+14; rr+=1) *rr += *ss++;
  /* #114: @20 = zeros(4x1) */
  casadi_clear(w20, 4);
  /* #115: @10 = 1 */
  w10 = 1.;
  /* #116: @11 = (@11?@10:0) */
  w11  = (w11?w10:0);
  /* #117: @10 = 10 */
  w10 = 10.;
  /* #118: @28 = (@10*@23) */
  for (i=0, rr=w28, cs=w23; i<4; ++i) (*rr++)  = (w10*(*cs++));
  /* #119: @23 = @23' */
  /* #120: @23 = (@10*@23) */
  for (i=0, rr=w23, cs=w23; i<4; ++i) (*rr++)  = (w10*(*cs++));
  /* #121: @23 = @23' */
  /* #122: @28 = (@28+@23) */
  for (i=0, rr=w28, cs=w23; i<4; ++i) (*rr++) += (*cs++);
  /* #123: @23 = (@11*@28) */
  for (i=0, rr=w23, cs=w28; i<4; ++i) (*rr++)  = (w11*(*cs++));
  /* #124: @11 = 1 */
  w11 = 1.;
  /* #125: @7 = (@7?@11:0) */
  w7  = (w7?w11:0);
  /* #126: @28 = (@7*@28) */
  for (i=0, rr=w28, cs=w28; i<4; ++i) (*rr++)  = (w7*(*cs++));
  /* #127: @23 = (@23-@28) */
  for (i=0, rr=w23, cs=w28; i<4; ++i) (*rr++) -= (*cs++);
  /* #128: @20 = mac(@32,@23,@20) */
  for (i=0, rr=w20; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w32+j, tt=w23+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #129: (@19[:4] += @20) */
  for (rr=w19+0, ss=w20; rr!=w19+4; rr+=1) *rr += *ss++;
  /* #130: @25 = (2.*@25) */
  for (i=0, rr=w25, cs=w25; i<3; ++i) *rr++ = (2.* *cs++ );
  /* #131: (@19[8:11] += @25) */
  for (rr=w19+8, ss=w25; rr!=w19+11; rr+=1) *rr += *ss++;
  /* #132: @35 = zeros(4x1,0nz) */
  /* #133: @1 = @1' */
  /* #134: @20 = zeros(1x4) */
  casadi_clear(w20, 4);
  /* #135: @4 = @4' */
  /* #136: @32 = @21' */
  for (i=0, rr=w32, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #137: @20 = mac(@4,@32,@20) */
  for (i=0, rr=w20; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w4+j, tt=w32+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #138: @20 = @20' */
  /* #139: @1 = (@1+@20) */
  for (i=0, rr=w1, cs=w20; i<4; ++i) (*rr++) += (*cs++);
  /* #140: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) *rr++ = (- *cs++ );
  /* #141: @36 = adj1_f_trans(@3, @35, @1) */
  arg1[0]=w3;
  arg1[1]=0;
  arg1[2]=w1;
  res1[0]=w36;
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #142: (@19[:8] += @36) */
  for (rr=w19+0, ss=w36; rr!=w19+8; rr+=1) *rr += *ss++;
  /* #143: {@7, @11, @10, @6, @5, @0, @8, @13, @12, @9, @14, @15, @16, @17} = vertsplit(@19) */
  w7 = w19[0];
  w11 = w19[1];
  w10 = w19[2];
  w6 = w19[3];
  w5 = w19[4];
  w0 = w19[5];
  w8 = w19[6];
  w13 = w19[7];
  w12 = w19[8];
  w9 = w19[9];
  w14 = w19[10];
  w15 = w19[11];
  w16 = w19[12];
  w17 = w19[13];
  /* #144: output[1][4] = @7 */
  if (res[1]) res[1][4] = w7;
  /* #145: output[1][5] = @11 */
  if (res[1]) res[1][5] = w11;
  /* #146: output[1][6] = @10 */
  if (res[1]) res[1][6] = w10;
  /* #147: output[1][7] = @6 */
  if (res[1]) res[1][7] = w6;
  /* #148: output[1][8] = @5 */
  if (res[1]) res[1][8] = w5;
  /* #149: output[1][9] = @0 */
  if (res[1]) res[1][9] = w0;
  /* #150: output[1][10] = @8 */
  if (res[1]) res[1][10] = w8;
  /* #151: output[1][11] = @13 */
  if (res[1]) res[1][11] = w13;
  /* #152: output[1][12] = @12 */
  if (res[1]) res[1][12] = w12;
  /* #153: output[1][13] = @9 */
  if (res[1]) res[1][13] = w9;
  /* #154: output[1][14] = @14 */
  if (res[1]) res[1][14] = w14;
  /* #155: output[1][15] = @15 */
  if (res[1]) res[1][15] = w15;
  /* #156: output[1][16] = @16 */
  if (res[1]) res[1][16] = w16;
  /* #157: output[1][17] = @17 */
  if (res[1]) res[1][17] = w17;
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
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 16;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 240;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
