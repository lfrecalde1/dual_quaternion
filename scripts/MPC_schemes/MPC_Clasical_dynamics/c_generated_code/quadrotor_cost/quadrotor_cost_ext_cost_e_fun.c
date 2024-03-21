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
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
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

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_e_fun:(i0[13],i1[],i2[],i3[17])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, *w1=w+5, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+25, *w20=w+42, *w21=w+45, *w22=w+58, *w23=w+61, *w24=w+70, *w25=w+74, *w26=w+78, *w27=w+82, *w28=w+86, *w29=w+90, *w30=w+106;
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
  /* #39: @2 = 1.5 */
  w2 = 1.5000000000000000e+00;
  /* #40: (@23[0] = @2) */
  for (rr=w23+0, ss=(&w2); rr!=w23+1; rr+=1) *rr = *ss++;
  /* #41: @2 = 1.5 */
  w2 = 1.5000000000000000e+00;
  /* #42: (@23[4] = @2) */
  for (rr=w23+4, ss=(&w2); rr!=w23+5; rr+=1) *rr = *ss++;
  /* #43: @2 = 2.5 */
  w2 = 2.5000000000000000e+00;
  /* #44: (@23[8] = @2) */
  for (rr=w23+8, ss=(&w2); rr!=w23+9; rr+=1) *rr = *ss++;
  /* #45: @1 = mac(@22,@23,@1) */
  for (i=0, rr=w1; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w23+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #46: @0 = mac(@1,@20,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w20+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #47: @2 = 10 */
  w2 = 10.;
  /* #48: @3 = 0 */
  w3 = 0.;
  /* #49: @4 = 0 */
  w4 = 0.;
  /* #50: @5 = 0.5 */
  w5 = 5.0000000000000000e-01;
  /* #51: @1 = all_2.22045e-16(3x1) */
  casadi_fill(w1, 3, 2.2204460492503131e-16);
  /* #52: @6 = 0 */
  w6 = 0.;
  /* #53: @24 = zeros(4x1) */
  casadi_clear(w24, 4);
  /* #54: @25 = horzcat(@8, @9, @10, @11) */
  rr=w25;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #55: @25 = @25' */
  /* #56: @7 = (-@9) */
  w7 = (- w9 );
  /* #57: @12 = (-@10) */
  w12 = (- w10 );
  /* #58: @26 = horzcat(@7, @8, @11, @12) */
  rr=w26;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w11;
  *rr++ = w12;
  /* #59: @26 = @26' */
  /* #60: @11 = (-@11) */
  w11 = (- w11 );
  /* #61: @27 = horzcat(@12, @11, @8, @9) */
  rr=w27;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w9;
  /* #62: @27 = @27' */
  /* #63: @28 = horzcat(@11, @10, @7, @8) */
  rr=w28;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w7;
  *rr++ = w8;
  /* #64: @28 = @28' */
  /* #65: @29 = horzcat(@25, @26, @27, @28) */
  rr=w29;
  for (i=0, cs=w25; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w27; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  /* #66: @30 = @29' */
  for (i=0, rr=w30, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #67: @25 = @21[6:10] */
  for (rr=w25, ss=w21+6; ss!=w21+10; ss+=1) *rr++ = *ss;
  /* #68: @24 = mac(@30,@25,@24) */
  for (i=0, rr=w24; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w30+j, tt=w25+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #69: @11 = @24[0] */
  for (rr=(&w11), ss=w24+0; ss!=w24+1; ss+=1) *rr++ = *ss;
  /* #70: @6 = (@6<@11) */
  w6  = (w6<w11);
  /* #71: @25 = (@6?@24:0) */
  for (i=0, rr=w25, cs=w24; i<4; ++i) (*rr++)  = (w6?(*cs++):0);
  /* #72: @6 = (!@6) */
  w6 = (! w6 );
  /* #73: @24 = (-@24) */
  for (i=0, rr=w24, cs=w24; i<4; ++i) *rr++ = (- *cs++ );
  /* #74: @24 = (@6?@24:0) */
  for (i=0, rr=w24, cs=w24; i<4; ++i) (*rr++)  = (w6?(*cs++):0);
  /* #75: @25 = (@25+@24) */
  for (i=0, rr=w25, cs=w24; i<4; ++i) (*rr++) += (*cs++);
  /* #76: @20 = @25[1:4] */
  for (rr=w20, ss=w25+1; ss!=w25+4; ss+=1) *rr++ = *ss;
  /* #77: @1 = (@1+@20) */
  for (i=0, rr=w1, cs=w20; i<3; ++i) (*rr++) += (*cs++);
  /* #78: @6 = ||@1||_F */
  w6 = sqrt(casadi_dot(3, w1, w1));
  /* #79: @11 = @25[0] */
  for (rr=(&w11), ss=w25+0; ss!=w25+1; ss+=1) *rr++ = *ss;
  /* #80: @11 = atan2(@6,@11) */
  w11  = atan2(w6,w11);
  /* #81: @5 = (@5*@11) */
  w5 *= w11;
  /* #82: @10 = @25[1] */
  for (rr=(&w10), ss=w25+1; ss!=w25+2; ss+=1) *rr++ = *ss;
  /* #83: @5 = (@5*@10) */
  w5 *= w10;
  /* #84: @5 = (@5/@6) */
  w5 /= w6;
  /* #85: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #86: @10 = (@10*@11) */
  w10 *= w11;
  /* #87: @7 = @25[2] */
  for (rr=(&w7), ss=w25+2; ss!=w25+3; ss+=1) *rr++ = *ss;
  /* #88: @10 = (@10*@7) */
  w10 *= w7;
  /* #89: @10 = (@10/@6) */
  w10 /= w6;
  /* #90: @7 = 0.5 */
  w7 = 5.0000000000000000e-01;
  /* #91: @7 = (@7*@11) */
  w7 *= w11;
  /* #92: @11 = @25[3] */
  for (rr=(&w11), ss=w25+3; ss!=w25+4; ss+=1) *rr++ = *ss;
  /* #93: @7 = (@7*@11) */
  w7 *= w11;
  /* #94: @7 = (@7/@6) */
  w7 /= w6;
  /* #95: @25 = vertcat(@4, @5, @10, @7) */
  rr=w25;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w10;
  *rr++ = w7;
  /* #96: @24 = @25' */
  casadi_copy(w25, 4, w24);
  /* #97: @3 = mac(@24,@25,@3) */
  for (i=0, rr=(&w3); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w24+j, tt=w25+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #98: @2 = (@2*@3) */
  w2 *= w3;
  /* #99: @0 = (@0+@2) */
  w0 += w2;
  /* #100: @2 = 0 */
  w2 = 0.;
  /* #101: @1 = @21[10:13] */
  for (rr=w1, ss=w21+10; ss!=w21+13; ss+=1) *rr++ = *ss;
  /* #102: @20 = @1' */
  casadi_copy(w1, 3, w20);
  /* #103: @2 = mac(@20,@1,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w1+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #104: @0 = (@0+@2) */
  w0 += w2;
  /* #105: @2 = 0 */
  w2 = 0.;
  /* #106: @20 = @21[3:6] */
  for (rr=w20, ss=w21+3; ss!=w21+6; ss+=1) *rr++ = *ss;
  /* #107: @1 = @20' */
  casadi_copy(w20, 3, w1);
  /* #108: @2 = mac(@1,@20,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w20+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #109: @0 = (@0+@2) */
  w0 += w2;
  /* #110: output[0][0] = @0 */
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
  if (sz_arg) *sz_arg = 21;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 122;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
