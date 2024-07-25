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
  #define CASADI_PREFIX(ID) quadrotor_constr_h_fun_jac_uxt_zt_ ## ID
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
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)

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

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[17] = {20, 2, 0, 4, 12, 6, 7, 8, 9, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s5[3] = {2, 0, 0};

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

/* adj1_f_quat:(i0[8],out_o0[4x1,0nz],adj_o0[4])->(adj_i0[8x1,4nz]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *w0=w+0, w1, w2, w3, w4;
  /* #0: @0 = input[2][0] */
  casadi_copy(arg[2], 4, w0);
  /* #1: {@1, @2, @3, @4} = vertsplit(@0) */
  w1 = w0[0];
  w2 = w0[1];
  w3 = w0[2];
  w4 = w0[3];
  /* #2: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #3: output[0][1] = @2 */
  if (res[0]) res[0][1] = w2;
  /* #4: output[0][2] = @3 */
  if (res[0]) res[0][2] = w3;
  /* #5: output[0][3] = @4 */
  if (res[0]) res[0][3] = w4;
  return 0;
}

/* quadrotor_constr_h_fun_jac_uxt_zt:(i0[14],i1[6],i2[],i3[14])->(o0[2],o1[20x2,12nz],o2[2x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real **res1=res+3, *rr, *ss;
  const casadi_real **arg1=arg+4, *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, *w14=w+22, *w15=w+36, *w16=w+44, *w17=w+48, *w18=w+52, *w19=w+56, *w21=w+68;
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
  /* #8: @8 = input[0][8] */
  w8 = arg[0] ? arg[0][8] : 0;
  /* #9: @9 = input[0][9] */
  w9 = arg[0] ? arg[0][9] : 0;
  /* #10: @10 = input[0][10] */
  w10 = arg[0] ? arg[0][10] : 0;
  /* #11: @11 = input[0][11] */
  w11 = arg[0] ? arg[0][11] : 0;
  /* #12: @12 = input[0][12] */
  w12 = arg[0] ? arg[0][12] : 0;
  /* #13: @13 = input[0][13] */
  w13 = arg[0] ? arg[0][13] : 0;
  /* #14: @14 = vertcat(@0, @1, @2, @3, @4, @5, @6, @7, @8, @9, @10, @11, @12, @13) */
  rr=w14;
  *rr++ = w0;
  *rr++ = w1;
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
  /* #15: @15 = @14[:8] */
  for (rr=w15, ss=w14+0; ss!=w14+8; ss+=1) *rr++ = *ss;
  /* #16: @16 = f_quat(@15) */
  arg1[0]=w15;
  res1[0]=w16;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #17: @0 = ||@16||_F */
  w0 = sqrt(casadi_dot(4, w16, w16));
  /* #18: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #19: @17 = @14[:4] */
  for (rr=w17, ss=w14+0; ss!=w14+4; ss+=1) *rr++ = *ss;
  /* #20: @18 = @14[4:8] */
  for (rr=w18, ss=w14+4; ss!=w14+8; ss+=1) *rr++ = *ss;
  /* #21: @1 = dot(@17, @18) */
  w1 = casadi_dot(4, w17, w18);
  /* #22: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #23: output[0][1] = @1 */
  if (res[0]) res[0][1] = w1;
  /* #24: @19 = zeros(20x2,12nz) */
  casadi_clear(w19, 12);
  /* #25: @14 = zeros(14x1) */
  casadi_clear(w14, 14);
  /* #26: @20 = zeros(4x1,0nz) */
  /* #27: @1 = ones(2x1,1nz) */
  w1 = 1.;
  /* #28: {@2, NULL} = vertsplit(@1) */
  w2 = w1;
  /* #29: @2 = (@2/@0) */
  w2 /= w0;
  /* #30: @16 = (@2*@16) */
  for (i=0, rr=w16, cs=w16; i<4; ++i) (*rr++)  = (w2*(*cs++));
  /* #31: @21 = adj1_f_quat(@15, @20, @16) */
  arg1[0]=w15;
  arg1[1]=0;
  arg1[2]=w16;
  res1[0]=w21;
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #32: (@14[:4] += @21) */
  for (rr=w14+0, ss=w21; rr!=w14+4; rr+=1) *rr += *ss++;
  /* #33: {@2, @0, @1, @3, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@14) */
  w2 = w14[0];
  w0 = w14[1];
  w1 = w14[2];
  w3 = w14[3];
  /* #34: (@19[0] = @2) */
  for (rr=w19+0, ss=(&w2); rr!=w19+1; rr+=1) *rr = *ss++;
  /* #35: (@19[1] = @0) */
  for (rr=w19+1, ss=(&w0); rr!=w19+2; rr+=1) *rr = *ss++;
  /* #36: (@19[2] = @1) */
  for (rr=w19+2, ss=(&w1); rr!=w19+3; rr+=1) *rr = *ss++;
  /* #37: (@19[3] = @3) */
  for (rr=w19+3, ss=(&w3); rr!=w19+4; rr+=1) *rr = *ss++;
  /* #38: @14 = zeros(14x1) */
  casadi_clear(w14, 14);
  /* #39: @3 = ones(2x1,1nz) */
  w3 = 1.;
  /* #40: {NULL, @1} = vertsplit(@3) */
  w1 = w3;
  /* #41: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #42: @17 = (@1*@17) */
  for (i=0, rr=w17, cs=w17; i<4; ++i) (*rr++)  = (w1*(*cs++));
  /* #43: (@14[4:8] += @17) */
  for (rr=w14+4, ss=w17; rr!=w14+8; rr+=1) *rr += *ss++;
  /* #44: @18 = (@1*@18) */
  for (i=0, rr=w18, cs=w18; i<4; ++i) (*rr++)  = (w1*(*cs++));
  /* #45: (@14[:4] += @18) */
  for (rr=w14+0, ss=w18; rr!=w14+4; rr+=1) *rr += *ss++;
  /* #46: {@1, @3, @0, @2, @4, @5, @6, @7, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@14) */
  w1 = w14[0];
  w3 = w14[1];
  w0 = w14[2];
  w2 = w14[3];
  w4 = w14[4];
  w5 = w14[5];
  w6 = w14[6];
  w7 = w14[7];
  /* #47: (@19[4] = @1) */
  for (rr=w19+4, ss=(&w1); rr!=w19+5; rr+=1) *rr = *ss++;
  /* #48: (@19[5] = @3) */
  for (rr=w19+5, ss=(&w3); rr!=w19+6; rr+=1) *rr = *ss++;
  /* #49: (@19[6] = @0) */
  for (rr=w19+6, ss=(&w0); rr!=w19+7; rr+=1) *rr = *ss++;
  /* #50: (@19[7] = @2) */
  for (rr=w19+7, ss=(&w2); rr!=w19+8; rr+=1) *rr = *ss++;
  /* #51: (@19[8] = @4) */
  for (rr=w19+8, ss=(&w4); rr!=w19+9; rr+=1) *rr = *ss++;
  /* #52: (@19[9] = @5) */
  for (rr=w19+9, ss=(&w5); rr!=w19+10; rr+=1) *rr = *ss++;
  /* #53: (@19[10] = @6) */
  for (rr=w19+10, ss=(&w6); rr!=w19+11; rr+=1) *rr = *ss++;
  /* #54: (@19[11] = @7) */
  for (rr=w19+11, ss=(&w7); rr!=w19+12; rr+=1) *rr = *ss++;
  /* #55: output[1][0] = @19 */
  casadi_copy(w19, 12, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_constr_h_fun_jac_uxt_zt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_jac_uxt_zt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_jac_uxt_zt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 17;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 72;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
