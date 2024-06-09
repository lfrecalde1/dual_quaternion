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
  #define CASADI_PREFIX(ID) quadrotor_expl_ode_fun_ ## ID
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
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* quadrotor_expl_ode_fun:(i0[8],i1[6],i2[])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real w0, *w1=w+9, w2, w3, w4, w5, w6, w7, w8, *w9=w+24, *w10=w+28, *w11=w+32, *w12=w+36, *w13=w+40, *w14=w+56, *w15=w+72, *w16=w+104, w17, w18, w19, w20, *w21=w+140, *w22=w+156, *w23=w+188, *w24=w+252, w25, w26, w27, w28, w29, *w30=w+321;
  /* #0: @0 = 0.5 */
  w0 = 5.0000000000000000e-01;
  /* #1: @1 = zeros(8x1) */
  casadi_clear(w1, 8);
  /* #2: @2 = input[0][0] */
  w2 = arg[0] ? arg[0][0] : 0;
  /* #3: @3 = input[0][1] */
  w3 = arg[0] ? arg[0][1] : 0;
  /* #4: @4 = (-@3) */
  w4 = (- w3 );
  /* #5: @5 = input[0][2] */
  w5 = arg[0] ? arg[0][2] : 0;
  /* #6: @6 = (-@5) */
  w6 = (- w5 );
  /* #7: @7 = input[0][3] */
  w7 = arg[0] ? arg[0][3] : 0;
  /* #8: @8 = (-@7) */
  w8 = (- w7 );
  /* #9: @9 = horzcat(@2, @4, @6, @8) */
  rr=w9;
  *rr++ = w2;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w8;
  /* #10: @9 = @9' */
  /* #11: @4 = (-@7) */
  w4 = (- w7 );
  /* #12: @10 = horzcat(@3, @2, @4, @5) */
  rr=w10;
  *rr++ = w3;
  *rr++ = w2;
  *rr++ = w4;
  *rr++ = w5;
  /* #13: @10 = @10' */
  /* #14: @4 = (-@3) */
  w4 = (- w3 );
  /* #15: @11 = horzcat(@5, @7, @2, @4) */
  rr=w11;
  *rr++ = w5;
  *rr++ = w7;
  *rr++ = w2;
  *rr++ = w4;
  /* #16: @11 = @11' */
  /* #17: @4 = (-@5) */
  w4 = (- w5 );
  /* #18: @12 = horzcat(@7, @4, @3, @2) */
  rr=w12;
  *rr++ = w7;
  *rr++ = w4;
  *rr++ = w3;
  *rr++ = w2;
  /* #19: @12 = @12' */
  /* #20: @13 = horzcat(@9, @10, @11, @12) */
  rr=w13;
  for (i=0, cs=w9; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #21: @14 = @13' */
  for (i=0, rr=w14, cs=w13; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #22: @13 = zeros(4x4) */
  casadi_clear(w13, 16);
  /* #23: @15 = horzcat(@14, @13) */
  rr=w15;
  for (i=0, cs=w14; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<16; ++i) *rr++ = *cs++;
  /* #24: @16 = @15' */
  for (i=0, rr=w16, cs=w15; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #25: @4 = input[0][4] */
  w4 = arg[0] ? arg[0][4] : 0;
  /* #26: @6 = input[0][5] */
  w6 = arg[0] ? arg[0][5] : 0;
  /* #27: @8 = (-@6) */
  w8 = (- w6 );
  /* #28: @17 = input[0][6] */
  w17 = arg[0] ? arg[0][6] : 0;
  /* #29: @18 = (-@17) */
  w18 = (- w17 );
  /* #30: @19 = input[0][7] */
  w19 = arg[0] ? arg[0][7] : 0;
  /* #31: @20 = (-@19) */
  w20 = (- w19 );
  /* #32: @9 = horzcat(@4, @8, @18, @20) */
  rr=w9;
  *rr++ = w4;
  *rr++ = w8;
  *rr++ = w18;
  *rr++ = w20;
  /* #33: @9 = @9' */
  /* #34: @8 = (-@19) */
  w8 = (- w19 );
  /* #35: @10 = horzcat(@6, @4, @8, @17) */
  rr=w10;
  *rr++ = w6;
  *rr++ = w4;
  *rr++ = w8;
  *rr++ = w17;
  /* #36: @10 = @10' */
  /* #37: @8 = (-@6) */
  w8 = (- w6 );
  /* #38: @11 = horzcat(@17, @19, @4, @8) */
  rr=w11;
  *rr++ = w17;
  *rr++ = w19;
  *rr++ = w4;
  *rr++ = w8;
  /* #39: @11 = @11' */
  /* #40: @8 = (-@17) */
  w8 = (- w17 );
  /* #41: @12 = horzcat(@19, @8, @6, @4) */
  rr=w12;
  *rr++ = w19;
  *rr++ = w8;
  *rr++ = w6;
  *rr++ = w4;
  /* #42: @12 = @12' */
  /* #43: @13 = horzcat(@9, @10, @11, @12) */
  rr=w13;
  for (i=0, cs=w9; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #44: @21 = @13' */
  for (i=0, rr=w21, cs=w13; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #45: @15 = horzcat(@21, @14) */
  rr=w15;
  for (i=0, cs=w21; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<16; ++i) *rr++ = *cs++;
  /* #46: @22 = @15' */
  for (i=0, rr=w22, cs=w15; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #47: @23 = horzcat(@16, @22) */
  rr=w23;
  for (i=0, cs=w16; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<32; ++i) *rr++ = *cs++;
  /* #48: @24 = @23' */
  for (i=0, rr=w24, cs=w23; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #49: @8 = 0 */
  w8 = 0.;
  /* #50: @18 = input[1][0] */
  w18 = arg[1] ? arg[1][0] : 0;
  /* #51: @20 = input[1][1] */
  w20 = arg[1] ? arg[1][1] : 0;
  /* #52: @25 = input[1][2] */
  w25 = arg[1] ? arg[1][2] : 0;
  /* #53: @26 = 0 */
  w26 = 0.;
  /* #54: @27 = input[1][3] */
  w27 = arg[1] ? arg[1][3] : 0;
  /* #55: @28 = input[1][4] */
  w28 = arg[1] ? arg[1][4] : 0;
  /* #56: @29 = input[1][5] */
  w29 = arg[1] ? arg[1][5] : 0;
  /* #57: @30 = vertcat(@8, @18, @20, @25, @26, @27, @28, @29) */
  rr=w30;
  *rr++ = w8;
  *rr++ = w18;
  *rr++ = w20;
  *rr++ = w25;
  *rr++ = w26;
  *rr++ = w27;
  *rr++ = w28;
  *rr++ = w29;
  /* #58: @1 = mac(@24,@30,@1) */
  for (i=0, rr=w1; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w24+j, tt=w30+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #59: @1 = (@0*@1) */
  for (i=0, rr=w1, cs=w1; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #60: @30 = vertcat(@2, @3, @5, @7, @4, @6, @17, @19) */
  rr=w30;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w7;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w17;
  *rr++ = w19;
  /* #61: @9 = @30[:4] */
  for (rr=w9, ss=w30+0; ss!=w30+4; ss+=1) *rr++ = *ss;
  /* #62: @2 = 10 */
  w2 = 10.;
  /* #63: @3 = 1 */
  w3 = 1.;
  /* #64: @5 = ||@9||_F */
  w5 = sqrt(casadi_dot(4, w9, w9));
  /* #65: @3 = (@3-@5) */
  w3 -= w5;
  /* #66: @2 = (@2*@3) */
  w2 *= w3;
  /* #67: @10 = (@9*@2) */
  for (i=0, rr=w10, cr=w9; i<4; ++i) (*rr++)  = ((*cr++)*w2);
  /* #68: @11 = @30[4:8] */
  for (rr=w11, ss=w30+4; ss!=w30+8; ss+=1) *rr++ = *ss;
  /* #69: @2 = 10 */
  w2 = 10.;
  /* #70: @3 = dot(@9, @11) */
  w3 = casadi_dot(4, w9, w11);
  /* #71: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #72: @2 = (@2*@3) */
  w2 *= w3;
  /* #73: @11 = (@11*@2) */
  for (i=0, rr=w11; i<4; ++i) (*rr++) *= w2;
  /* #74: @30 = vertcat(@10, @11) */
  rr=w30;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  /* #75: @1 = (@1+@30) */
  for (i=0, rr=w1, cs=w30; i<8; ++i) (*rr++) += (*cs++);
  /* #76: output[0][0] = @1 */
  casadi_copy(w1, 8, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 329;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
