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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_e_fun_jac_hess_ ## ID
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
#define casadi_mtimes CASADI_PREFIX(mtimes)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s12 CASADI_PREFIX(s12)
#define casadi_s13 CASADI_PREFIX(s13)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)

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

void casadi_mtimes(const casadi_real* x, const casadi_int* sp_x, const casadi_real* y, const casadi_int* sp_y, casadi_real* z, const casadi_int* sp_z, casadi_real* w, casadi_int tr) {
  casadi_int ncol_x, ncol_y, ncol_z, cc;
  const casadi_int *colind_x, *row_x, *colind_y, *row_y, *colind_z, *row_z;
  ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2; row_y = sp_y + 2 + ncol_y+1;
  ncol_z = sp_z[1];
  colind_z = sp_z+2; row_z = sp_z + 2 + ncol_z+1;
  if (tr) {
    for (cc=0; cc<ncol_z; ++cc) {
      casadi_int kk;
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        w[row_y[kk]] = y[kk];
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_z[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          z[kk] += x[kk1] * w[row_x[kk1]];
        }
      }
    }
  } else {
    for (cc=0; cc<ncol_y; ++cc) {
      casadi_int kk;
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        w[row_z[kk]] = z[kk];
      }
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_y[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          w[row_x[kk1]] += x[kk1]*y[kk];
        }
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        z[kk] = w[row_z[kk]];
      }
    }
  }
}

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[5] = {8, 1, 0, 1, 0};
static const casadi_int casadi_s2[75] = {8, 8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s3[5] = {8, 1, 0, 1, 1};
static const casadi_int casadi_s4[5] = {8, 1, 0, 1, 2};
static const casadi_int casadi_s5[5] = {8, 1, 0, 1, 3};
static const casadi_int casadi_s6[5] = {8, 1, 0, 1, 4};
static const casadi_int casadi_s7[5] = {8, 1, 0, 1, 5};
static const casadi_int casadi_s8[5] = {8, 1, 0, 1, 6};
static const casadi_int casadi_s9[5] = {8, 1, 0, 1, 7};
static const casadi_int casadi_s10[3] = {0, 0, 0};
static const casadi_int casadi_s11[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s12[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s13[11] = {0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static const casadi_real casadi_c0[8] = {1., 0., 0., 0., 0., 0., 0., 0.};
static const casadi_real casadi_c1[64] = {1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 2.0000000000000001e-01, 0., 0., 0., 0., 0., 0., 0., 0., 2.0000000000000001e-01, 0., 0., 0., 0., 0., 0., 0., 0., 2.0000000000000001e-01, 0., 0., 0., 0., 0., 0., 0., 0., 2.0000000000000001e-01};

/* quadrotor_cost_ext_cost_e_fun_jac_hess:(i0[8],i1[],i2[],i3[14])->(o0,o1[8],o2[8x8],o3[],o4[0x8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, *w3=w+18, w4, *w5=w+27, *w6=w+35, w7, w8, w9, w10, *w11=w+53, w12, w13, *w14=w+59, *w15=w+63, *w16=w+67, *w17=w+71, *w18=w+87, *w19=w+103, *w20=w+135, *w21=w+167, *w22=w+183, *w23=w+215, *w24=w+279, w25, w26, *w27=w+345, *w28=w+353, *w29=w+417, *w30=w+481;
  /* #0: @0 = 10 */
  w0 = 10.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #3: @3 = [1, 0, 0, 0, 0, 0, 0, 0] */
  casadi_copy(casadi_c0, 8, w3);
  /* #4: @4 = 0 */
  w4 = 0.;
  /* #5: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #6: @6 = input[3][0] */
  casadi_copy(arg[3], 14, w6);
  /* #7: @7 = @6[0] */
  for (rr=(&w7), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #8: @8 = @6[1] */
  for (rr=(&w8), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #9: @9 = @6[2] */
  for (rr=(&w9), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #10: @10 = @6[3] */
  for (rr=(&w10), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #11: @11 = horzcat(@7, @8, @9, @10) */
  rr=w11;
  *rr++ = w7;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #12: @11 = @11' */
  /* #13: @7 = (-@8) */
  w7 = (- w8 );
  /* #14: @12 = @6[0] */
  for (rr=(&w12), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #15: @13 = (-@9) */
  w13 = (- w9 );
  /* #16: @14 = horzcat(@7, @12, @10, @13) */
  rr=w14;
  *rr++ = w7;
  *rr++ = w12;
  *rr++ = w10;
  *rr++ = w13;
  /* #17: @14 = @14' */
  /* #18: @10 = (-@10) */
  w10 = (- w10 );
  /* #19: @12 = @6[0] */
  for (rr=(&w12), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #20: @15 = horzcat(@13, @10, @12, @8) */
  rr=w15;
  *rr++ = w13;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w8;
  /* #21: @15 = @15' */
  /* #22: @13 = @6[0] */
  for (rr=(&w13), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #23: @16 = horzcat(@10, @9, @7, @13) */
  rr=w16;
  *rr++ = w10;
  *rr++ = w9;
  *rr++ = w7;
  *rr++ = w13;
  /* #24: @16 = @16' */
  /* #25: @17 = horzcat(@11, @14, @15, @16) */
  rr=w17;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #26: @18 = @17' */
  for (i=0, rr=w18, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #27: @17 = zeros(4x4) */
  casadi_clear(w17, 16);
  /* #28: @19 = horzcat(@18, @17) */
  rr=w19;
  for (i=0, cs=w18; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<16; ++i) *rr++ = *cs++;
  /* #29: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #30: @10 = @6[4] */
  for (rr=(&w10), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #31: @9 = @6[5] */
  for (rr=(&w9), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #32: @7 = @6[6] */
  for (rr=(&w7), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #33: @13 = @6[7] */
  for (rr=(&w13), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #34: @11 = horzcat(@10, @9, @7, @13) */
  rr=w11;
  *rr++ = w10;
  *rr++ = w9;
  *rr++ = w7;
  *rr++ = w13;
  /* #35: @11 = @11' */
  /* #36: @10 = (-@9) */
  w10 = (- w9 );
  /* #37: @12 = @6[4] */
  for (rr=(&w12), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #38: @8 = (-@7) */
  w8 = (- w7 );
  /* #39: @14 = horzcat(@10, @12, @13, @8) */
  rr=w14;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w8;
  /* #40: @14 = @14' */
  /* #41: @13 = (-@13) */
  w13 = (- w13 );
  /* #42: @12 = @6[4] */
  for (rr=(&w12), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #43: @15 = horzcat(@8, @13, @12, @9) */
  rr=w15;
  *rr++ = w8;
  *rr++ = w13;
  *rr++ = w12;
  *rr++ = w9;
  /* #44: @15 = @15' */
  /* #45: @8 = @6[4] */
  for (rr=(&w8), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #46: @16 = horzcat(@13, @7, @10, @8) */
  rr=w16;
  *rr++ = w13;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w8;
  /* #47: @16 = @16' */
  /* #48: @17 = horzcat(@11, @14, @15, @16) */
  rr=w17;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  /* #49: @21 = @17' */
  for (i=0, rr=w21, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #50: @19 = horzcat(@21, @18) */
  rr=w19;
  for (i=0, cs=w21; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<16; ++i) *rr++ = *cs++;
  /* #51: @22 = @19' */
  for (i=0, rr=w22, cs=w19; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #52: @23 = horzcat(@20, @22) */
  rr=w23;
  for (i=0, cs=w20; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<32; ++i) *rr++ = *cs++;
  /* #53: @24 = @23' */
  for (i=0, rr=w24, cs=w23; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #54: @13 = input[0][0] */
  w13 = arg[0] ? arg[0][0] : 0;
  /* #55: @7 = input[0][1] */
  w7 = arg[0] ? arg[0][1] : 0;
  /* #56: @10 = input[0][2] */
  w10 = arg[0] ? arg[0][2] : 0;
  /* #57: @8 = input[0][3] */
  w8 = arg[0] ? arg[0][3] : 0;
  /* #58: @12 = input[0][4] */
  w12 = arg[0] ? arg[0][4] : 0;
  /* #59: @9 = input[0][5] */
  w9 = arg[0] ? arg[0][5] : 0;
  /* #60: @25 = input[0][6] */
  w25 = arg[0] ? arg[0][6] : 0;
  /* #61: @26 = input[0][7] */
  w26 = arg[0] ? arg[0][7] : 0;
  /* #62: @27 = vertcat(@13, @7, @10, @8, @12, @9, @25, @26) */
  rr=w27;
  *rr++ = w13;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  *rr++ = w26;
  /* #63: @5 = mac(@24,@27,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w24+j, tt=w27+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #64: @13 = @5[0] */
  for (rr=(&w13), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #65: @4 = (@4<@13) */
  w4  = (w4<w13);
  /* #66: @27 = (@4?@5:0) */
  for (i=0, rr=w27, cs=w5; i<8; ++i) (*rr++)  = (w4?(*cs++):0);
  /* #67: @13 = (!@4) */
  w13 = (! w4 );
  /* #68: @5 = (-@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) *rr++ = (- *cs++ );
  /* #69: @5 = (@13?@5:0) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w13?(*cs++):0);
  /* #70: @27 = (@27+@5) */
  for (i=0, rr=w27, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #71: @3 = (@3-@27) */
  for (i=0, rr=w3, cs=w27; i<8; ++i) (*rr++) -= (*cs++);
  /* #72: @27 = @3' */
  casadi_copy(w3, 8, w27);
  /* #73: @28 = 
  [[1, 0, 0, 0, 0, 0, 0, 0], 
   [0, 1, 0, 0, 0, 0, 0, 0], 
   [0, 0, 1, 0, 0, 0, 0, 0], 
   [0, 0, 0, 1, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0.2, 0, 0, 0], 
   [0, 0, 0, 0, 0, 0.2, 0, 0], 
   [0, 0, 0, 0, 0, 0, 0.2, 0], 
   [0, 0, 0, 0, 0, 0, 0, 0.2]] */
  casadi_copy(casadi_c1, 64, w28);
  /* #74: @2 = mac(@27,@28,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w27+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #75: @1 = mac(@2,@3,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w3+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #76: @0 = (@0*@1) */
  w0 *= w1;
  /* #77: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #78: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #79: @0 = 1 */
  w0 = 1.;
  /* #80: @0 = (@13?@0:0) */
  w0  = (w13?w0:0);
  /* #81: @1 = 10 */
  w1 = 10.;
  /* #82: @2 = @2' */
  /* #83: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #84: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #85: @3 = @3' */
  /* #86: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #87: @29 = @28' */
  for (i=0, rr=w29, cs=w28; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #88: @5 = mac(@3,@29,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #89: @5 = @5' */
  /* #90: @2 = (@2+@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #91: @5 = (@0*@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #92: @7 = 1 */
  w7 = 1.;
  /* #93: @7 = (@4?@7:0) */
  w7  = (w4?w7:0);
  /* #94: @2 = (@7*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #95: @5 = (@5-@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #96: @27 = mac(@23,@5,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w5+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #97: output[1][0] = @27 */
  casadi_copy(w27, 8, res[1]);
  /* #98: @30 = zeros(8x8) */
  casadi_clear(w30, 64);
  /* #99: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #100: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #101: @10 = 1 */
  w10 = 1.;
  /* #102: @4 = (@4?@10:0) */
  w4  = (w4?w10:0);
  /* #103: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #104: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #105: {@8, NULL, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #106: @31 = 00 */
  /* #107: @32 = 00 */
  /* #108: @33 = 00 */
  /* #109: @34 = 00 */
  /* #110: @35 = 00 */
  /* #111: @36 = 00 */
  /* #112: @37 = 00 */
  /* #113: @10 = vertcat(@8, @31, @32, @33, @34, @35, @36, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #114: @2 = mac(@24,@10,@2) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s1, w2, casadi_s0, w, 0);
  /* #115: @3 = (@4*@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #116: @10 = 1 */
  w10 = 1.;
  /* #117: @13 = (@13?@10:0) */
  w13  = (w13?w10:0);
  /* #118: @2 = (@13*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #119: @3 = (@3-@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #120: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #121: @2 = @3' */
  casadi_copy(w3, 8, w2);
  /* #122: @5 = mac(@2,@28,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #123: @5 = @5' */
  /* #124: @5 = (@1*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #125: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #126: @3 = @3' */
  /* #127: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #128: @2 = mac(@3,@29,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #129: @2 = @2' */
  /* #130: @5 = (@5+@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) += (*cs++);
  /* #131: @2 = (@0*@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #132: @5 = (@7*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #133: @2 = (@2-@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #134: @27 = mac(@23,@2,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w2+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #135: (@30[:8] = @27) */
  for (rr=w30+0, ss=w27; rr!=w30+8; rr+=1) *rr = *ss++;
  /* #136: (@30[:64:8] = @27) */
  for (rr=w30+0, ss=w27; rr!=w30+64; rr+=8) *rr = *ss++;
  /* #137: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #138: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #139: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #140: @31 = 00 */
  /* #141: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #142: {NULL, @8, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #143: @32 = 00 */
  /* #144: @33 = 00 */
  /* #145: @34 = 00 */
  /* #146: @35 = 00 */
  /* #147: @36 = 00 */
  /* #148: @37 = 00 */
  /* #149: @10 = vertcat(@31, @8, @32, @33, @34, @35, @36, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #150: @5 = mac(@24,@10,@5) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s3, w5, casadi_s0, w, 0);
  /* #151: @3 = (@4*@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #152: @5 = (@13*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #153: @3 = (@3-@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #154: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #155: @5 = @3' */
  casadi_copy(w3, 8, w5);
  /* #156: @2 = mac(@5,@28,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #157: @2 = @2' */
  /* #158: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #159: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #160: @3 = @3' */
  /* #161: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #162: @5 = mac(@3,@29,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #163: @5 = @5' */
  /* #164: @2 = (@2+@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #165: @5 = (@0*@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #166: @2 = (@7*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #167: @5 = (@5-@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #168: @27 = mac(@23,@5,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w5+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #169: (@30[8:16] = @27) */
  for (rr=w30+8, ss=w27; rr!=w30+16; rr+=1) *rr = *ss++;
  /* #170: (@30[1:65:8] = @27) */
  for (rr=w30+1, ss=w27; rr!=w30+65; rr+=8) *rr = *ss++;
  /* #171: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #172: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #173: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #174: @31 = 00 */
  /* #175: @32 = 00 */
  /* #176: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #177: {NULL, NULL, @8, NULL, NULL, NULL, NULL, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #178: @33 = 00 */
  /* #179: @34 = 00 */
  /* #180: @35 = 00 */
  /* #181: @36 = 00 */
  /* #182: @37 = 00 */
  /* #183: @10 = vertcat(@31, @32, @8, @33, @34, @35, @36, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #184: @2 = mac(@24,@10,@2) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s4, w2, casadi_s0, w, 0);
  /* #185: @3 = (@4*@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #186: @2 = (@13*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #187: @3 = (@3-@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #188: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #189: @2 = @3' */
  casadi_copy(w3, 8, w2);
  /* #190: @5 = mac(@2,@28,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #191: @5 = @5' */
  /* #192: @5 = (@1*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #193: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #194: @3 = @3' */
  /* #195: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #196: @2 = mac(@3,@29,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #197: @2 = @2' */
  /* #198: @5 = (@5+@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) += (*cs++);
  /* #199: @2 = (@0*@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #200: @5 = (@7*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #201: @2 = (@2-@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #202: @27 = mac(@23,@2,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w2+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #203: (@30[16:24] = @27) */
  for (rr=w30+16, ss=w27; rr!=w30+24; rr+=1) *rr = *ss++;
  /* #204: (@30[2:66:8] = @27) */
  for (rr=w30+2, ss=w27; rr!=w30+66; rr+=8) *rr = *ss++;
  /* #205: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #206: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #207: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #208: @31 = 00 */
  /* #209: @32 = 00 */
  /* #210: @33 = 00 */
  /* #211: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #212: {NULL, NULL, NULL, @8, NULL, NULL, NULL, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #213: @34 = 00 */
  /* #214: @35 = 00 */
  /* #215: @36 = 00 */
  /* #216: @37 = 00 */
  /* #217: @10 = vertcat(@31, @32, @33, @8, @34, @35, @36, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #218: @5 = mac(@24,@10,@5) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s5, w5, casadi_s0, w, 0);
  /* #219: @3 = (@4*@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #220: @5 = (@13*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #221: @3 = (@3-@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #222: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #223: @5 = @3' */
  casadi_copy(w3, 8, w5);
  /* #224: @2 = mac(@5,@28,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #225: @2 = @2' */
  /* #226: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #227: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #228: @3 = @3' */
  /* #229: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #230: @5 = mac(@3,@29,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #231: @5 = @5' */
  /* #232: @2 = (@2+@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #233: @5 = (@0*@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #234: @2 = (@7*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #235: @5 = (@5-@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #236: @27 = mac(@23,@5,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w5+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #237: (@30[24:32] = @27) */
  for (rr=w30+24, ss=w27; rr!=w30+32; rr+=1) *rr = *ss++;
  /* #238: (@30[3:67:8] = @27) */
  for (rr=w30+3, ss=w27; rr!=w30+67; rr+=8) *rr = *ss++;
  /* #239: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #240: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #241: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #242: @31 = 00 */
  /* #243: @32 = 00 */
  /* #244: @33 = 00 */
  /* #245: @34 = 00 */
  /* #246: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #247: {NULL, NULL, NULL, NULL, @8, NULL, NULL, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #248: @35 = 00 */
  /* #249: @36 = 00 */
  /* #250: @37 = 00 */
  /* #251: @10 = vertcat(@31, @32, @33, @34, @8, @35, @36, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #252: @2 = mac(@24,@10,@2) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s6, w2, casadi_s0, w, 0);
  /* #253: @3 = (@4*@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #254: @2 = (@13*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #255: @3 = (@3-@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #256: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #257: @2 = @3' */
  casadi_copy(w3, 8, w2);
  /* #258: @5 = mac(@2,@28,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #259: @5 = @5' */
  /* #260: @5 = (@1*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #261: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #262: @3 = @3' */
  /* #263: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #264: @2 = mac(@3,@29,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #265: @2 = @2' */
  /* #266: @5 = (@5+@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) += (*cs++);
  /* #267: @2 = (@0*@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #268: @5 = (@7*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #269: @2 = (@2-@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #270: @27 = mac(@23,@2,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w2+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #271: (@30[32:40] = @27) */
  for (rr=w30+32, ss=w27; rr!=w30+40; rr+=1) *rr = *ss++;
  /* #272: (@30[4:68:8] = @27) */
  for (rr=w30+4, ss=w27; rr!=w30+68; rr+=8) *rr = *ss++;
  /* #273: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #274: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #275: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #276: @31 = 00 */
  /* #277: @32 = 00 */
  /* #278: @33 = 00 */
  /* #279: @34 = 00 */
  /* #280: @35 = 00 */
  /* #281: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #282: {NULL, NULL, NULL, NULL, NULL, @8, NULL, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #283: @36 = 00 */
  /* #284: @37 = 00 */
  /* #285: @10 = vertcat(@31, @32, @33, @34, @35, @8, @36, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #286: @5 = mac(@24,@10,@5) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s7, w5, casadi_s0, w, 0);
  /* #287: @3 = (@4*@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #288: @5 = (@13*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #289: @3 = (@3-@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #290: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #291: @5 = @3' */
  casadi_copy(w3, 8, w5);
  /* #292: @2 = mac(@5,@28,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #293: @2 = @2' */
  /* #294: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #295: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #296: @3 = @3' */
  /* #297: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #298: @5 = mac(@3,@29,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #299: @5 = @5' */
  /* #300: @2 = (@2+@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #301: @5 = (@0*@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #302: @2 = (@7*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #303: @5 = (@5-@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #304: @27 = mac(@23,@5,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w5+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #305: (@30[40:48] = @27) */
  for (rr=w30+40, ss=w27; rr!=w30+48; rr+=1) *rr = *ss++;
  /* #306: (@30[5:69:8] = @27) */
  for (rr=w30+5, ss=w27; rr!=w30+69; rr+=8) *rr = *ss++;
  /* #307: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #308: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #309: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #310: @31 = 00 */
  /* #311: @32 = 00 */
  /* #312: @33 = 00 */
  /* #313: @34 = 00 */
  /* #314: @35 = 00 */
  /* #315: @36 = 00 */
  /* #316: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #317: {NULL, NULL, NULL, NULL, NULL, NULL, @8, NULL} = vertsplit(@10) */
  w8 = w10;
  /* #318: @37 = 00 */
  /* #319: @10 = vertcat(@31, @32, @33, @34, @35, @36, @8, @37) */
  rr=(&w10);
  *rr++ = w8;
  /* #320: @2 = mac(@24,@10,@2) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s8, w2, casadi_s0, w, 0);
  /* #321: @3 = (@4*@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #322: @2 = (@13*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #323: @3 = (@3-@2) */
  for (i=0, rr=w3, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #324: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #325: @2 = @3' */
  casadi_copy(w3, 8, w2);
  /* #326: @5 = mac(@2,@28,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #327: @5 = @5' */
  /* #328: @5 = (@1*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #329: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #330: @3 = @3' */
  /* #331: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #332: @2 = mac(@3,@29,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #333: @2 = @2' */
  /* #334: @5 = (@5+@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) += (*cs++);
  /* #335: @2 = (@0*@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #336: @5 = (@7*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #337: @2 = (@2-@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #338: @27 = mac(@23,@2,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w2+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #339: (@30[48:56] = @27) */
  for (rr=w30+48, ss=w27; rr!=w30+56; rr+=1) *rr = *ss++;
  /* #340: (@30[6:70:8] = @27) */
  for (rr=w30+6, ss=w27; rr!=w30+70; rr+=8) *rr = *ss++;
  /* #341: @27 = zeros(8x1) */
  casadi_clear(w27, 8);
  /* #342: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #343: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #344: @31 = 00 */
  /* #345: @32 = 00 */
  /* #346: @33 = 00 */
  /* #347: @34 = 00 */
  /* #348: @35 = 00 */
  /* #349: @36 = 00 */
  /* #350: @37 = 00 */
  /* #351: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #352: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, @8} = vertsplit(@10) */
  w8 = w10;
  /* #353: @10 = vertcat(@31, @32, @33, @34, @35, @36, @37, @8) */
  rr=(&w10);
  *rr++ = w8;
  /* #354: @5 = mac(@24,@10,@5) */
  casadi_mtimes(w24, casadi_s2, (&w10), casadi_s9, w5, casadi_s0, w, 0);
  /* #355: @3 = (@4*@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++)  = (w4*(*cs++));
  /* #356: @5 = (@13*@5) */
  for (i=0, rr=w5, cs=w5; i<8; ++i) (*rr++)  = (w13*(*cs++));
  /* #357: @3 = (@3-@5) */
  for (i=0, rr=w3, cs=w5; i<8; ++i) (*rr++) -= (*cs++);
  /* #358: @3 = (-@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) *rr++ = (- *cs++ );
  /* #359: @5 = @3' */
  casadi_copy(w3, 8, w5);
  /* #360: @2 = mac(@5,@28,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w5+j, tt=w28+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #361: @2 = @2' */
  /* #362: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #363: @5 = zeros(1x8) */
  casadi_clear(w5, 8);
  /* #364: @3 = @3' */
  /* #365: @3 = (@1*@3) */
  for (i=0, rr=w3, cs=w3; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #366: @5 = mac(@3,@29,@5) */
  for (i=0, rr=w5; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w3+j, tt=w29+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #367: @5 = @5' */
  /* #368: @2 = (@2+@5) */
  for (i=0, rr=w2, cs=w5; i<8; ++i) (*rr++) += (*cs++);
  /* #369: @5 = (@0*@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #370: @2 = (@7*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w7*(*cs++));
  /* #371: @5 = (@5-@2) */
  for (i=0, rr=w5, cs=w2; i<8; ++i) (*rr++) -= (*cs++);
  /* #372: @27 = mac(@23,@5,@27) */
  for (i=0, rr=w27; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w5+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #373: (@30[56:64] = @27) */
  for (rr=w30+56, ss=w27; rr!=w30+64; rr+=1) *rr = *ss++;
  /* #374: (@30[7:71:8] = @27) */
  for (rr=w30+7, ss=w27; rr!=w30+71; rr+=8) *rr = *ss++;
  /* #375: @23 = @30' */
  for (i=0, rr=w23, cs=w30; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #376: output[2][0] = @23 */
  casadi_copy(w23, 64, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_e_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s10;
    case 2: return casadi_s10;
    case 3: return casadi_s11;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s12;
    case 1: return casadi_s0;
    case 2: return casadi_s2;
    case 3: return casadi_s10;
    case 4: return casadi_s13;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 12;
  if (sz_res) *sz_res = 13;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 545;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
