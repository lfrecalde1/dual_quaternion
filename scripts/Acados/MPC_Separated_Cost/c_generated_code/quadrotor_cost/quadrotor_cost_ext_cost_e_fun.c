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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_e_fun_ ## ID
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
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
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
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[16] = {0., 0., 0., 0., 0., 1.2500000000000000e+01, 0., 0., 0., 0., 1.2500000000000000e+01, 0., 0., 0., 0., 1.2500000000000000e+01};
static const casadi_real casadi_c1[4] = {1., 0., 0., 0.};

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

/* quadrotor_cost_ext_cost_e_fun:(i0[14],i1[],i2[],i3[18])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+1, *rr, *ss, *tt;
  const casadi_real **arg1=arg+4, *cs;
  casadi_real w0, *w1=w+61, *w2=w+65, *w3=w+83, *w4=w+91, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+109, *w20=w+123, *w21=w+127, *w22=w+143, *w23=w+146, *w24=w+149, *w25=w+153, *w26=w+157, *w27=w+161;
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
  /* #28: @22 = @19[8:11] */
  for (rr=w22, ss=w19+8; ss!=w19+11; ss+=1) *rr++ = *ss;
  /* #29: @23 = @22' */
  casadi_copy(w22, 3, w23);
  /* #30: @5 = mac(@23,@22,@5) */
  for (i=0, rr=(&w5); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w23+j, tt=w22+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #31: @0 = (@0+@5) */
  w0 += w5;
  /* #32: @5 = 10 */
  w5 = 10.;
  /* #33: @6 = 0 */
  w6 = 0.;
  /* #34: @1 = [1, 0, 0, 0] */
  casadi_copy(casadi_c1, 4, w1);
  /* #35: @7 = 0 */
  w7 = 0.;
  /* #36: @4 = zeros(4x1) */
  casadi_clear(w4, 4);
  /* #37: @8 = @2[0] */
  for (rr=(&w8), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #38: @9 = @2[1] */
  for (rr=(&w9), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #39: @10 = @2[2] */
  for (rr=(&w10), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #40: @11 = @2[3] */
  for (rr=(&w11), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #41: @20 = horzcat(@8, @9, @10, @11) */
  rr=w20;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #42: @20 = @20' */
  /* #43: @8 = (-@9) */
  w8 = (- w9 );
  /* #44: @12 = @2[0] */
  for (rr=(&w12), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #45: @13 = (-@10) */
  w13 = (- w10 );
  /* #46: @24 = horzcat(@8, @12, @11, @13) */
  rr=w24;
  *rr++ = w8;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w13;
  /* #47: @24 = @24' */
  /* #48: @11 = (-@11) */
  w11 = (- w11 );
  /* #49: @12 = @2[0] */
  for (rr=(&w12), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #50: @25 = horzcat(@13, @11, @12, @9) */
  rr=w25;
  *rr++ = w13;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w9;
  /* #51: @25 = @25' */
  /* #52: @13 = @2[0] */
  for (rr=(&w13), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #53: @26 = horzcat(@11, @10, @8, @13) */
  rr=w26;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w13;
  /* #54: @26 = @26' */
  /* #55: @21 = horzcat(@20, @24, @25, @26) */
  rr=w21;
  for (i=0, cs=w20; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w24; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w25; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  /* #56: @27 = @21' */
  for (i=0, rr=w27, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #57: @20 = @19[:4] */
  for (rr=w20, ss=w19+0; ss!=w19+4; ss+=1) *rr++ = *ss;
  /* #58: @4 = mac(@27,@20,@4) */
  for (i=0, rr=w4; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w27+j, tt=w20+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #59: @11 = @4[0] */
  for (rr=(&w11), ss=w4+0; ss!=w4+1; ss+=1) *rr++ = *ss;
  /* #60: @7 = (@7<@11) */
  w7  = (w7<w11);
  /* #61: @20 = (@7?@4:0) */
  for (i=0, rr=w20, cs=w4; i<4; ++i) (*rr++)  = (w7?(*cs++):0);
  /* #62: @7 = (!@7) */
  w7 = (! w7 );
  /* #63: @4 = (-@4) */
  for (i=0, rr=w4, cs=w4; i<4; ++i) *rr++ = (- *cs++ );
  /* #64: @4 = (@7?@4:0) */
  for (i=0, rr=w4, cs=w4; i<4; ++i) (*rr++)  = (w7?(*cs++):0);
  /* #65: @20 = (@20+@4) */
  for (i=0, rr=w20, cs=w4; i<4; ++i) (*rr++) += (*cs++);
  /* #66: @1 = (@1-@20) */
  for (i=0, rr=w1, cs=w20; i<4; ++i) (*rr++) -= (*cs++);
  /* #67: @20 = @1' */
  casadi_copy(w1, 4, w20);
  /* #68: @6 = mac(@20,@1,@6) */
  for (i=0, rr=(&w6); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w1+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #69: @5 = (@5*@6) */
  w5 *= w6;
  /* #70: @0 = (@0+@5) */
  w0 += w5;
  /* #71: @5 = 0.5 */
  w5 = 5.0000000000000000e-01;
  /* #72: @6 = 0 */
  w6 = 0.;
  /* #73: @23 = @19[11:14] */
  for (rr=w23, ss=w19+11; ss!=w19+14; ss+=1) *rr++ = *ss;
  /* #74: @22 = @23' */
  casadi_copy(w23, 3, w22);
  /* #75: @6 = mac(@22,@23,@6) */
  for (i=0, rr=(&w6); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w23+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #76: @5 = (@5*@6) */
  w5 *= w6;
  /* #77: @0 = (@0+@5) */
  w0 += w5;
  /* #78: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_e_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 177;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
