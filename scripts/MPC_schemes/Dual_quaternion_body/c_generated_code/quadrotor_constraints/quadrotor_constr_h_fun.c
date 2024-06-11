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
  #define CASADI_PREFIX(ID) quadrotor_constr_h_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_dot CASADI_PREFIX(dot)
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
static const casadi_int casadi_s4[6] = {2, 1, 0, 2, 0, 1};

/* f_quat:(i0[8])->(o0[4]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[0][1] */
  w0 = arg[0] ? arg[0][1] : 0;
  /* #3: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #4: @0 = input[0][2] */
  w0 = arg[0] ? arg[0][2] : 0;
  /* #5: output[0][2] = @0 */
  if (res[0]) res[0][2] = w0;
  /* #6: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #7: output[0][3] = @0 */
  if (res[0]) res[0][3] = w0;
  return 0;
}

/* quadrotor_constr_h_fun:(i0[8],i1[6],i2[],i3[14])->(o0[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real **res1=res+1, *rr, *ss;
  const casadi_real **arg1=arg+4;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, *w8=w+9, *w9=w+17, *w10=w+21;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = input[0][1] */
  w1 = arg[0] ? arg[0][1] : 0;
  /* #2: @2 = input[0][2] */
  w2 = arg[0] ? arg[0][2] : 0;
  /* #3: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #4: @4 = input[0][4] */
  w4 = arg[0] ? arg[0][4] : 0;
  /* #5: @5 = input[0][5] */
  w5 = arg[0] ? arg[0][5] : 0;
  /* #6: @6 = input[0][6] */
  w6 = arg[0] ? arg[0][6] : 0;
  /* #7: @7 = input[0][7] */
  w7 = arg[0] ? arg[0][7] : 0;
  /* #8: @8 = vertcat(@0, @1, @2, @3, @4, @5, @6, @7) */
  rr=w8;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  /* #9: @9 = f_quat(@8) */
  arg1[0]=w8;
  res1[0]=w9;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #10: @0 = ||@9||_F */
  w0 = sqrt(casadi_dot(4, w9, w9));
  /* #11: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #12: @9 = @8[:4] */
  for (rr=w9, ss=w8+0; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #13: @10 = @8[4:8] */
  for (rr=w10, ss=w8+4; ss!=w8+8; ss+=1) *rr++ = *ss;
  /* #14: @0 = dot(@9, @10) */
  w0 = casadi_dot(4, w9, w10);
  /* #15: @0 = (2.*@0) */
  w0 = (2.* w0 );
  /* #16: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_constr_h_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 12;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 25;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
