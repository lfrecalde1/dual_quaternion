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
  #define CASADI_PREFIX(ID) quadrotor_ocp_cost_ext_cost_fun_ ## ID
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

/* quadrotor_ocp_cost_ext_cost_fun:(i0[14],i1[4],i2[],i3[18])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, *w3=w+18, *w4=w+26, *w5=w+34, w6, w7, w8, w9, *w10=w+56, w11, w12, *w13=w+62, *w14=w+66, *w15=w+70, *w16=w+74, *w17=w+90, *w18=w+106, *w19=w+138, *w20=w+170, *w21=w+186, *w22=w+218, *w23=w+282, w24, w25, w26, w27, w28, w29, w30, w31, *w32=w+354, *w33=w+368;
  /* #0: @0 = 10 */
  w0 = 10.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #3: @3 = [1, 0, 0, 0, 0, 0, 0, 0] */
  casadi_copy(casadi_c0, 8, w3);
  /* #4: @4 = zeros(8x1) */
  casadi_clear(w4, 8);
  /* #5: @5 = input[3][0] */
  casadi_copy(arg[3], 18, w5);
  /* #6: @6 = @5[0] */
  for (rr=(&w6), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #7: @7 = @5[1] */
  for (rr=(&w7), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #8: @8 = @5[2] */
  for (rr=(&w8), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #9: @9 = @5[3] */
  for (rr=(&w9), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #10: @10 = horzcat(@6, @7, @8, @9) */
  rr=w10;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  /* #11: @10 = @10' */
  /* #12: @6 = (-@7) */
  w6 = (- w7 );
  /* #13: @11 = @5[0] */
  for (rr=(&w11), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #14: @12 = (-@8) */
  w12 = (- w8 );
  /* #15: @13 = horzcat(@6, @11, @9, @12) */
  rr=w13;
  *rr++ = w6;
  *rr++ = w11;
  *rr++ = w9;
  *rr++ = w12;
  /* #16: @13 = @13' */
  /* #17: @9 = (-@9) */
  w9 = (- w9 );
  /* #18: @11 = @5[0] */
  for (rr=(&w11), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #19: @14 = horzcat(@12, @9, @11, @7) */
  rr=w14;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w7;
  /* #20: @14 = @14' */
  /* #21: @12 = @5[0] */
  for (rr=(&w12), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #22: @15 = horzcat(@9, @8, @6, @12) */
  rr=w15;
  *rr++ = w9;
  *rr++ = w8;
  *rr++ = w6;
  *rr++ = w12;
  /* #23: @15 = @15' */
  /* #24: @16 = horzcat(@10, @13, @14, @15) */
  rr=w16;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  /* #25: @17 = @16' */
  for (i=0, rr=w17, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #26: @16 = zeros(4x4) */
  casadi_clear(w16, 16);
  /* #27: @18 = horzcat(@17, @16) */
  rr=w18;
  for (i=0, cs=w17; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<16; ++i) *rr++ = *cs++;
  /* #28: @19 = @18' */
  for (i=0, rr=w19, cs=w18; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #29: @9 = @5[4] */
  for (rr=(&w9), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #30: @8 = @5[5] */
  for (rr=(&w8), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #31: @6 = @5[6] */
  for (rr=(&w6), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #32: @12 = @5[7] */
  for (rr=(&w12), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #33: @10 = horzcat(@9, @8, @6, @12) */
  rr=w10;
  *rr++ = w9;
  *rr++ = w8;
  *rr++ = w6;
  *rr++ = w12;
  /* #34: @10 = @10' */
  /* #35: @9 = (-@8) */
  w9 = (- w8 );
  /* #36: @11 = @5[4] */
  for (rr=(&w11), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #37: @7 = (-@6) */
  w7 = (- w6 );
  /* #38: @13 = horzcat(@9, @11, @12, @7) */
  rr=w13;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w7;
  /* #39: @13 = @13' */
  /* #40: @12 = (-@12) */
  w12 = (- w12 );
  /* #41: @11 = @5[4] */
  for (rr=(&w11), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #42: @14 = horzcat(@7, @12, @11, @8) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w8;
  /* #43: @14 = @14' */
  /* #44: @7 = @5[4] */
  for (rr=(&w7), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #45: @15 = horzcat(@12, @6, @9, @7) */
  rr=w15;
  *rr++ = w12;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w7;
  /* #46: @15 = @15' */
  /* #47: @16 = horzcat(@10, @13, @14, @15) */
  rr=w16;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  /* #48: @20 = @16' */
  for (i=0, rr=w20, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #49: @18 = horzcat(@20, @17) */
  rr=w18;
  for (i=0, cs=w20; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<16; ++i) *rr++ = *cs++;
  /* #50: @21 = @18' */
  for (i=0, rr=w21, cs=w18; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #51: @22 = horzcat(@19, @21) */
  rr=w22;
  for (i=0, cs=w19; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w21; i<32; ++i) *rr++ = *cs++;
  /* #52: @23 = @22' */
  for (i=0, rr=w23, cs=w22; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #53: @12 = input[0][0] */
  w12 = arg[0] ? arg[0][0] : 0;
  /* #54: @6 = input[0][1] */
  w6 = arg[0] ? arg[0][1] : 0;
  /* #55: @9 = input[0][2] */
  w9 = arg[0] ? arg[0][2] : 0;
  /* #56: @7 = input[0][3] */
  w7 = arg[0] ? arg[0][3] : 0;
  /* #57: @11 = input[0][4] */
  w11 = arg[0] ? arg[0][4] : 0;
  /* #58: @8 = input[0][5] */
  w8 = arg[0] ? arg[0][5] : 0;
  /* #59: @24 = input[0][6] */
  w24 = arg[0] ? arg[0][6] : 0;
  /* #60: @25 = input[0][7] */
  w25 = arg[0] ? arg[0][7] : 0;
  /* #61: @26 = input[0][8] */
  w26 = arg[0] ? arg[0][8] : 0;
  /* #62: @27 = input[0][9] */
  w27 = arg[0] ? arg[0][9] : 0;
  /* #63: @28 = input[0][10] */
  w28 = arg[0] ? arg[0][10] : 0;
  /* #64: @29 = input[0][11] */
  w29 = arg[0] ? arg[0][11] : 0;
  /* #65: @30 = input[0][12] */
  w30 = arg[0] ? arg[0][12] : 0;
  /* #66: @31 = input[0][13] */
  w31 = arg[0] ? arg[0][13] : 0;
  /* #67: @32 = vertcat(@12, @6, @9, @7, @11, @8, @24, @25, @26, @27, @28, @29, @30, @31) */
  rr=w32;
  *rr++ = w12;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w7;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w24;
  *rr++ = w25;
  *rr++ = w26;
  *rr++ = w27;
  *rr++ = w28;
  *rr++ = w29;
  *rr++ = w30;
  *rr++ = w31;
  /* #68: @33 = @32[:8] */
  for (rr=w33, ss=w32+0; ss!=w32+8; ss+=1) *rr++ = *ss;
  /* #69: @4 = mac(@23,@33,@4) */
  for (i=0, rr=w4; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w33+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #70: @3 = (@3-@4) */
  for (i=0, rr=w3, cs=w4; i<8; ++i) (*rr++) -= (*cs++);
  /* #71: @4 = @3' */
  casadi_copy(w3, 8, w4);
  /* #72: @23 = zeros(8x8) */
  casadi_clear(w23, 64);
  /* #73: @12 = 2 */
  w12 = 2.;
  /* #74: (@23[0] = @12) */
  for (rr=w23+0, ss=(&w12); rr!=w23+1; rr+=1) *rr = *ss++;
  /* #75: @12 = 2 */
  w12 = 2.;
  /* #76: (@23[9] = @12) */
  for (rr=w23+9, ss=(&w12); rr!=w23+10; rr+=1) *rr = *ss++;
  /* #77: @12 = 2 */
  w12 = 2.;
  /* #78: (@23[18] = @12) */
  for (rr=w23+18, ss=(&w12); rr!=w23+19; rr+=1) *rr = *ss++;
  /* #79: @12 = 2 */
  w12 = 2.;
  /* #80: (@23[27] = @12) */
  for (rr=w23+27, ss=(&w12); rr!=w23+28; rr+=1) *rr = *ss++;
  /* #81: @12 = 1.6 */
  w12 = 1.6000000000000001e+00;
  /* #82: (@23[36] = @12) */
  for (rr=w23+36, ss=(&w12); rr!=w23+37; rr+=1) *rr = *ss++;
  /* #83: @12 = 1.6 */
  w12 = 1.6000000000000001e+00;
  /* #84: (@23[45] = @12) */
  for (rr=w23+45, ss=(&w12); rr!=w23+46; rr+=1) *rr = *ss++;
  /* #85: @12 = 1.6 */
  w12 = 1.6000000000000001e+00;
  /* #86: (@23[54] = @12) */
  for (rr=w23+54, ss=(&w12); rr!=w23+55; rr+=1) *rr = *ss++;
  /* #87: @12 = 1.6 */
  w12 = 1.6000000000000001e+00;
  /* #88: (@23[63] = @12) */
  for (rr=w23+63, ss=(&w12); rr!=w23+64; rr+=1) *rr = *ss++;
  /* #89: @2 = mac(@4,@23,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w4+j, tt=w23+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #90: @1 = mac(@2,@3,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w3+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #91: @0 = (@0*@1) */
  w0 *= w1;
  /* #92: @1 = 0 */
  w1 = 0.;
  /* #93: @10 = zeros(1x4) */
  casadi_clear(w10, 4);
  /* #94: @13 = @5[14:18] */
  for (rr=w13, ss=w5+14; ss!=w5+18; ss+=1) *rr++ = *ss;
  /* #95: @12 = input[1][0] */
  w12 = arg[1] ? arg[1][0] : 0;
  /* #96: @6 = input[1][1] */
  w6 = arg[1] ? arg[1][1] : 0;
  /* #97: @9 = input[1][2] */
  w9 = arg[1] ? arg[1][2] : 0;
  /* #98: @7 = input[1][3] */
  w7 = arg[1] ? arg[1][3] : 0;
  /* #99: @14 = vertcat(@12, @6, @9, @7) */
  rr=w14;
  *rr++ = w12;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w7;
  /* #100: @13 = (@13-@14) */
  for (i=0, rr=w13, cs=w14; i<4; ++i) (*rr++) -= (*cs++);
  /* #101: @14 = @13' */
  casadi_copy(w13, 4, w14);
  /* #102: @20 = zeros(4x4) */
  casadi_clear(w20, 16);
  /* #103: @12 = 0.671141 */
  w12 = 6.7114093959731547e-01;
  /* #104: (@20[0] = @12) */
  for (rr=w20+0, ss=(&w12); rr!=w20+1; rr+=1) *rr = *ss++;
  /* #105: @12 = 600 */
  w12 = 600.;
  /* #106: (@20[5] = @12) */
  for (rr=w20+5, ss=(&w12); rr!=w20+6; rr+=1) *rr = *ss++;
  /* #107: @12 = 600 */
  w12 = 600.;
  /* #108: (@20[10] = @12) */
  for (rr=w20+10, ss=(&w12); rr!=w20+11; rr+=1) *rr = *ss++;
  /* #109: @12 = 600 */
  w12 = 600.;
  /* #110: (@20[15] = @12) */
  for (rr=w20+15, ss=(&w12); rr!=w20+16; rr+=1) *rr = *ss++;
  /* #111: @10 = mac(@14,@20,@10) */
  for (i=0, rr=w10; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w14+j, tt=w20+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #112: @1 = mac(@10,@13,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w10+j, tt=w13+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #113: @0 = (@0+@1) */
  w0 += w1;
  /* #114: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_ocp_cost_ext_cost_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 376;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
