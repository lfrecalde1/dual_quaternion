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
  #define CASADI_PREFIX(ID) quadrotor_impl_dae_jac_x_xdot_u_z_ ## ID
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
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_mtimes CASADI_PREFIX(mtimes)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s12 CASADI_PREFIX(s12)
#define casadi_s13 CASADI_PREFIX(s13)
#define casadi_s14 CASADI_PREFIX(s14)
#define casadi_s15 CASADI_PREFIX(s15)
#define casadi_s16 CASADI_PREFIX(s16)
#define casadi_s17 CASADI_PREFIX(s17)
#define casadi_s18 CASADI_PREFIX(s18)
#define casadi_s19 CASADI_PREFIX(s19)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s20 CASADI_PREFIX(s20)
#define casadi_s21 CASADI_PREFIX(s21)
#define casadi_s22 CASADI_PREFIX(s22)
#define casadi_s23 CASADI_PREFIX(s23)
#define casadi_s24 CASADI_PREFIX(s24)
#define casadi_s25 CASADI_PREFIX(s25)
#define casadi_s26 CASADI_PREFIX(s26)
#define casadi_s27 CASADI_PREFIX(s27)
#define casadi_s28 CASADI_PREFIX(s28)
#define casadi_s29 CASADI_PREFIX(s29)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s30 CASADI_PREFIX(s30)
#define casadi_s31 CASADI_PREFIX(s31)
#define casadi_s32 CASADI_PREFIX(s32)
#define casadi_s33 CASADI_PREFIX(s33)
#define casadi_s34 CASADI_PREFIX(s34)
#define casadi_s35 CASADI_PREFIX(s35)
#define casadi_s36 CASADI_PREFIX(s36)
#define casadi_s37 CASADI_PREFIX(s37)
#define casadi_s38 CASADI_PREFIX(s38)
#define casadi_s39 CASADI_PREFIX(s39)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s40 CASADI_PREFIX(s40)
#define casadi_s41 CASADI_PREFIX(s41)
#define casadi_s42 CASADI_PREFIX(s42)
#define casadi_s43 CASADI_PREFIX(s43)
#define casadi_s44 CASADI_PREFIX(s44)
#define casadi_s45 CASADI_PREFIX(s45)
#define casadi_s46 CASADI_PREFIX(s46)
#define casadi_s47 CASADI_PREFIX(s47)
#define casadi_s48 CASADI_PREFIX(s48)
#define casadi_s49 CASADI_PREFIX(s49)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_trans CASADI_PREFIX(trans)

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

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
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

static const casadi_int casadi_s0[11] = {4, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[11] = {8, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[15] = {4, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 0, 1, 2, 3};
static const casadi_int casadi_s3[11] = {8, 4, 0, 1, 2, 3, 4, 4, 5, 6, 7};
static const casadi_int casadi_s4[15] = {4, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 1, 2, 3};
static const casadi_int casadi_s5[19] = {8, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s6[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s7[8] = {0, 4, 8, 12, 16, 24, 32, 40};
static const casadi_int casadi_s8[11] = {4, 4, 0, 1, 2, 3, 4, 1, 0, 3, 2};
static const casadi_int casadi_s9[11] = {8, 4, 0, 1, 2, 3, 4, 1, 0, 3, 2};
static const casadi_int casadi_s10[15] = {4, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 1, 0, 3, 2};
static const casadi_int casadi_s11[11] = {8, 4, 0, 1, 2, 3, 4, 5, 4, 7, 6};
static const casadi_int casadi_s12[15] = {4, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 1, 0, 3, 2};
static const casadi_int casadi_s13[19] = {8, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 1, 0, 3, 2, 5, 4, 7, 6};
static const casadi_int casadi_s14[8] = {1, 5, 9, 13, 17, 25, 33, 41};
static const casadi_int casadi_s15[11] = {4, 4, 0, 1, 2, 3, 4, 2, 3, 0, 1};
static const casadi_int casadi_s16[11] = {8, 4, 0, 1, 2, 3, 4, 2, 3, 0, 1};
static const casadi_int casadi_s17[15] = {4, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 2, 3, 0, 1};
static const casadi_int casadi_s18[11] = {8, 4, 0, 1, 2, 3, 4, 6, 7, 4, 5};
static const casadi_int casadi_s19[15] = {4, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 2, 3, 0, 1};
static const casadi_int casadi_s20[19] = {8, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 2, 3, 0, 1, 6, 7, 4, 5};
static const casadi_int casadi_s21[8] = {2, 6, 10, 14, 18, 26, 34, 42};
static const casadi_int casadi_s22[11] = {4, 4, 0, 1, 2, 3, 4, 3, 2, 1, 0};
static const casadi_int casadi_s23[11] = {8, 4, 0, 1, 2, 3, 4, 3, 2, 1, 0};
static const casadi_int casadi_s24[15] = {4, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 3, 2, 1, 0};
static const casadi_int casadi_s25[11] = {8, 4, 0, 1, 2, 3, 4, 7, 6, 5, 4};
static const casadi_int casadi_s26[15] = {4, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 3, 2, 1, 0};
static const casadi_int casadi_s27[19] = {8, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 3, 2, 1, 0, 7, 6, 5, 4};
static const casadi_int casadi_s28[8] = {3, 7, 11, 15, 19, 27, 35, 43};
static const casadi_int casadi_s29[15] = {8, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 4, 5, 6, 7};
static const casadi_int casadi_s30[15] = {8, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 1, 2, 3};
static const casadi_int casadi_s31[15] = {8, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 5, 4, 7, 6};
static const casadi_int casadi_s32[15] = {8, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 1, 0, 3, 2};
static const casadi_int casadi_s33[15] = {8, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 6, 7, 4, 5};
static const casadi_int casadi_s34[15] = {8, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 2, 3, 0, 1};
static const casadi_int casadi_s35[15] = {8, 8, 0, 1, 2, 3, 4, 4, 4, 4, 4, 7, 6, 5, 4};
static const casadi_int casadi_s36[15] = {8, 8, 0, 0, 0, 0, 0, 1, 2, 3, 4, 3, 2, 1, 0};
static const casadi_int casadi_s37[59] = {8, 8, 0, 8, 16, 24, 32, 36, 40, 44, 48, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7};
static const casadi_int casadi_s38[59] = {8, 8, 0, 4, 8, 12, 16, 24, 32, 40, 48, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s39[5] = {8, 1, 0, 1, 1};
static const casadi_int casadi_s40[75] = {8, 8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s41[5] = {8, 1, 0, 1, 2};
static const casadi_int casadi_s42[5] = {8, 1, 0, 1, 3};
static const casadi_int casadi_s43[5] = {8, 1, 0, 1, 5};
static const casadi_int casadi_s44[5] = {8, 1, 0, 1, 6};
static const casadi_int casadi_s45[5] = {8, 1, 0, 1, 7};
static const casadi_int casadi_s46[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s47[3] = {0, 0, 0};
static const casadi_int casadi_s48[57] = {8, 6, 0, 8, 16, 24, 32, 40, 48, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s49[3] = {8, 0, 0};

/* quadrotor_impl_dae_jac_x_xdot_u_z:(i0[8],i1[8],i2[6],i3[],i4[])->(o0[8x8,48nz],o1[8x8,8nz],o2[8x6],o3[8x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j;
  casadi_real *rr, *ss;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real *w0=w+8, w1, *w2=w+57, w3, w4, w8, w10, w11, *w12=w+70, *w13=w+74, *w14=w+78, *w15=w+82, *w16=w+90, w17, w18, w19, *w20=w+101, *w22=w+109, *w23=w+157, *w24=w+161, *w25=w+177, *w26=w+193, *w27=w+225, *w28=w+257, *w29=w+273, *w30=w+305, *w31=w+369;
  /* #0: @0 = zeros(8x8,48nz) */
  casadi_clear(w0, 48);
  /* #1: @1 = 0.5 */
  w1 = 5.0000000000000000e-01;
  /* #2: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #3: @3 = ones(8x1,1nz) */
  w3 = 1.;
  /* #4: {@4, NULL, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@3) */
  w4 = w3;
  /* #5: @5 = 00 */
  /* #6: @6 = 00 */
  /* #7: @7 = 00 */
  /* #8: @3 = horzcat(@4, @5, @6, @7) */
  rr=(&w3);
  *rr++ = w4;
  /* #9: @3 = @3' */
  /* #10: @5 = 00 */
  /* #11: @6 = 00 */
  /* #12: @7 = 00 */
  /* #13: @8 = horzcat(@5, @4, @6, @7) */
  rr=(&w8);
  *rr++ = w4;
  /* #14: @8 = @8' */
  /* #15: @6 = 00 */
  /* #16: @9 = 00 */
  /* #17: @10 = horzcat(@7, @6, @4, @9) */
  rr=(&w10);
  *rr++ = w4;
  /* #18: @10 = @10' */
  /* #19: @7 = 00 */
  /* #20: @11 = horzcat(@6, @7, @5, @4) */
  rr=(&w11);
  *rr++ = w4;
  /* #21: @11 = @11' */
  /* #22: @12 = horzcat(@3, @8, @10, @11) */
  rr=w12;
  *rr++ = w3;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w11;
  /* #23: @13 = @12' */
  casadi_trans(w12,casadi_s0, w13, casadi_s0, iw);
  /* #24: @6 = zeros(4x4,0nz) */
  /* #25: @12 = horzcat(@13, @6) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #26: @14 = @12' */
  casadi_trans(w12,casadi_s2, w14, casadi_s1, iw);
  /* #27: @6 = zeros(4x4,0nz) */
  /* #28: @12 = horzcat(@6, @13) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #29: @13 = @12' */
  casadi_trans(w12,casadi_s4, w13, casadi_s3, iw);
  /* #30: @15 = horzcat(@14, @13) */
  rr=w15;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #31: @16 = @15' */
  casadi_trans(w15,casadi_s5, w16, casadi_s5, iw);
  /* #32: @3 = 0 */
  w3 = 0.;
  /* #33: @8 = input[2][0] */
  w8 = arg[2] ? arg[2][0] : 0;
  /* #34: @10 = input[2][1] */
  w10 = arg[2] ? arg[2][1] : 0;
  /* #35: @11 = input[2][2] */
  w11 = arg[2] ? arg[2][2] : 0;
  /* #36: @4 = 0 */
  w4 = 0.;
  /* #37: @17 = input[2][3] */
  w17 = arg[2] ? arg[2][3] : 0;
  /* #38: @18 = input[2][4] */
  w18 = arg[2] ? arg[2][4] : 0;
  /* #39: @19 = input[2][5] */
  w19 = arg[2] ? arg[2][5] : 0;
  /* #40: @15 = vertcat(@3, @8, @10, @11, @4, @17, @18, @19) */
  rr=w15;
  *rr++ = w3;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w4;
  *rr++ = w17;
  *rr++ = w18;
  *rr++ = w19;
  /* #41: @2 = mac(@16,@15,@2) */
  casadi_mtimes(w16, casadi_s5, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #42: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #43: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #44: (@0[0, 4, 8, 12, 16, 24, 32, 40] = @2) */
  for (cii=casadi_s7, rr=w0, ss=w2; cii!=casadi_s7+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #45: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #46: @6 = 00 */
  /* #47: @3 = ones(8x1,1nz) */
  w3 = 1.;
  /* #48: {NULL, @8, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@3) */
  w8 = w3;
  /* #49: @3 = (-@8) */
  w3 = (- w8 );
  /* #50: @7 = 00 */
  /* #51: @5 = 00 */
  /* #52: @10 = horzcat(@6, @3, @7, @5) */
  rr=(&w10);
  *rr++ = w3;
  /* #53: @10 = @10' */
  /* #54: @7 = 00 */
  /* #55: @5 = 00 */
  /* #56: @3 = horzcat(@8, @6, @7, @5) */
  rr=(&w3);
  *rr++ = w8;
  /* #57: @3 = @3' */
  /* #58: @7 = 00 */
  /* #59: @11 = (-@8) */
  w11 = (- w8 );
  /* #60: @4 = horzcat(@5, @7, @6, @11) */
  rr=(&w4);
  *rr++ = w11;
  /* #61: @4 = @4' */
  /* #62: @5 = 00 */
  /* #63: @11 = horzcat(@7, @5, @8, @6) */
  rr=(&w11);
  *rr++ = w8;
  /* #64: @11 = @11' */
  /* #65: @14 = horzcat(@10, @3, @4, @11) */
  rr=w14;
  *rr++ = w10;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w11;
  /* #66: @13 = @14' */
  casadi_trans(w14,casadi_s8, w13, casadi_s8, iw);
  /* #67: @7 = zeros(4x4,0nz) */
  /* #68: @14 = horzcat(@13, @7) */
  rr=w14;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #69: @12 = @14' */
  casadi_trans(w14,casadi_s10, w12, casadi_s9, iw);
  /* #70: @7 = zeros(4x4,0nz) */
  /* #71: @14 = horzcat(@7, @13) */
  rr=w14;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #72: @13 = @14' */
  casadi_trans(w14,casadi_s12, w13, casadi_s11, iw);
  /* #73: @16 = horzcat(@12, @13) */
  rr=w16;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #74: @20 = @16' */
  casadi_trans(w16,casadi_s13, w20, casadi_s13, iw);
  /* #75: @2 = mac(@20,@15,@2) */
  casadi_mtimes(w20, casadi_s13, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #76: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #77: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #78: (@0[1, 5, 9, 13, 17, 25, 33, 41] = @2) */
  for (cii=casadi_s14, rr=w0, ss=w2; cii!=casadi_s14+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #79: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #80: @7 = 00 */
  /* #81: @5 = 00 */
  /* #82: @10 = ones(8x1,1nz) */
  w10 = 1.;
  /* #83: {NULL, NULL, @3, NULL, NULL, NULL, NULL, NULL} = vertsplit(@10) */
  w3 = w10;
  /* #84: @10 = (-@3) */
  w10 = (- w3 );
  /* #85: @6 = 00 */
  /* #86: @4 = horzcat(@7, @5, @10, @6) */
  rr=(&w4);
  *rr++ = w10;
  /* #87: @4 = @4' */
  /* #88: @5 = 00 */
  /* #89: @6 = 00 */
  /* #90: @10 = horzcat(@5, @7, @6, @3) */
  rr=(&w10);
  *rr++ = w3;
  /* #91: @10 = @10' */
  /* #92: @6 = 00 */
  /* #93: @9 = 00 */
  /* #94: @11 = horzcat(@3, @6, @7, @9) */
  rr=(&w11);
  *rr++ = w3;
  /* #95: @11 = @11' */
  /* #96: @3 = (-@3) */
  w3 = (- w3 );
  /* #97: @8 = horzcat(@6, @3, @5, @7) */
  rr=(&w8);
  *rr++ = w3;
  /* #98: @8 = @8' */
  /* #99: @12 = horzcat(@4, @10, @11, @8) */
  rr=w12;
  *rr++ = w4;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w8;
  /* #100: @13 = @12' */
  casadi_trans(w12,casadi_s15, w13, casadi_s15, iw);
  /* #101: @6 = zeros(4x4,0nz) */
  /* #102: @12 = horzcat(@13, @6) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #103: @14 = @12' */
  casadi_trans(w12,casadi_s17, w14, casadi_s16, iw);
  /* #104: @6 = zeros(4x4,0nz) */
  /* #105: @12 = horzcat(@6, @13) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #106: @13 = @12' */
  casadi_trans(w12,casadi_s19, w13, casadi_s18, iw);
  /* #107: @20 = horzcat(@14, @13) */
  rr=w20;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #108: @16 = @20' */
  casadi_trans(w20,casadi_s20, w16, casadi_s20, iw);
  /* #109: @2 = mac(@16,@15,@2) */
  casadi_mtimes(w16, casadi_s20, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #110: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #111: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #112: (@0[2, 6, 10, 14, 18, 26, 34, 42] = @2) */
  for (cii=casadi_s21, rr=w0, ss=w2; cii!=casadi_s21+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #113: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #114: @6 = 00 */
  /* #115: @5 = 00 */
  /* #116: @7 = 00 */
  /* #117: @4 = ones(8x1,1nz) */
  w4 = 1.;
  /* #118: {NULL, NULL, NULL, @10, NULL, NULL, NULL, NULL} = vertsplit(@4) */
  w10 = w4;
  /* #119: @4 = (-@10) */
  w4 = (- w10 );
  /* #120: @11 = horzcat(@6, @5, @7, @4) */
  rr=(&w11);
  *rr++ = w4;
  /* #121: @11 = @11' */
  /* #122: @5 = 00 */
  /* #123: @4 = (-@10) */
  w4 = (- w10 );
  /* #124: @7 = 00 */
  /* #125: @8 = horzcat(@5, @6, @4, @7) */
  rr=(&w8);
  *rr++ = w4;
  /* #126: @8 = @8' */
  /* #127: @9 = 00 */
  /* #128: @4 = horzcat(@7, @10, @6, @9) */
  rr=(&w4);
  *rr++ = w10;
  /* #129: @4 = @4' */
  /* #130: @7 = 00 */
  /* #131: @3 = horzcat(@10, @7, @5, @6) */
  rr=(&w3);
  *rr++ = w10;
  /* #132: @3 = @3' */
  /* #133: @14 = horzcat(@11, @8, @4, @3) */
  rr=w14;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w4;
  *rr++ = w3;
  /* #134: @13 = @14' */
  casadi_trans(w14,casadi_s22, w13, casadi_s22, iw);
  /* #135: @7 = zeros(4x4,0nz) */
  /* #136: @14 = horzcat(@13, @7) */
  rr=w14;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #137: @12 = @14' */
  casadi_trans(w14,casadi_s24, w12, casadi_s23, iw);
  /* #138: @7 = zeros(4x4,0nz) */
  /* #139: @14 = horzcat(@7, @13) */
  rr=w14;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #140: @13 = @14' */
  casadi_trans(w14,casadi_s26, w13, casadi_s25, iw);
  /* #141: @16 = horzcat(@12, @13) */
  rr=w16;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #142: @20 = @16' */
  casadi_trans(w16,casadi_s27, w20, casadi_s27, iw);
  /* #143: @2 = mac(@20,@15,@2) */
  casadi_mtimes(w20, casadi_s27, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #144: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #145: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #146: (@0[3, 7, 11, 15, 19, 27, 35, 43] = @2) */
  for (cii=casadi_s28, rr=w0, ss=w2; cii!=casadi_s28+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #147: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #148: @7 = zeros(8x4,0nz) */
  /* #149: @11 = ones(8x1,1nz) */
  w11 = 1.;
  /* #150: {NULL, NULL, NULL, NULL, @8, NULL, NULL, NULL} = vertsplit(@11) */
  w8 = w11;
  /* #151: @5 = 00 */
  /* #152: @6 = 00 */
  /* #153: @9 = 00 */
  /* #154: @11 = horzcat(@8, @5, @6, @9) */
  rr=(&w11);
  *rr++ = w8;
  /* #155: @11 = @11' */
  /* #156: @5 = 00 */
  /* #157: @6 = 00 */
  /* #158: @9 = 00 */
  /* #159: @4 = horzcat(@5, @8, @6, @9) */
  rr=(&w4);
  *rr++ = w8;
  /* #160: @4 = @4' */
  /* #161: @6 = 00 */
  /* #162: @21 = 00 */
  /* #163: @3 = horzcat(@9, @6, @8, @21) */
  rr=(&w3);
  *rr++ = w8;
  /* #164: @3 = @3' */
  /* #165: @9 = 00 */
  /* #166: @10 = horzcat(@6, @9, @5, @8) */
  rr=(&w10);
  *rr++ = w8;
  /* #167: @10 = @10' */
  /* #168: @12 = horzcat(@11, @4, @3, @10) */
  rr=w12;
  *rr++ = w11;
  *rr++ = w4;
  *rr++ = w3;
  *rr++ = w10;
  /* #169: @13 = @12' */
  casadi_trans(w12,casadi_s0, w13, casadi_s0, iw);
  /* #170: @6 = zeros(4x4,0nz) */
  /* #171: @12 = horzcat(@13, @6) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #172: @13 = @12' */
  casadi_trans(w12,casadi_s2, w13, casadi_s1, iw);
  /* #173: @12 = horzcat(@7, @13) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #174: @13 = @12' */
  casadi_trans(w12,casadi_s30, w13, casadi_s29, iw);
  /* #175: @2 = mac(@13,@15,@2) */
  casadi_mtimes(w13, casadi_s29, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #176: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #177: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #178: @13 = @2[4:8] */
  for (rr=w13, ss=w2+4; ss!=w2+8; ss+=1) *rr++ = *ss;
  /* #179: (@0[20:52:8] = @13) */
  for (rr=w0+20, ss=w13; rr!=w0+52; rr+=8) *rr = *ss++;
  /* #180: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #181: @7 = zeros(8x4,0nz) */
  /* #182: @6 = 00 */
  /* #183: @11 = ones(8x1,1nz) */
  w11 = 1.;
  /* #184: {NULL, NULL, NULL, NULL, NULL, @4, NULL, NULL} = vertsplit(@11) */
  w4 = w11;
  /* #185: @11 = (-@4) */
  w11 = (- w4 );
  /* #186: @9 = 00 */
  /* #187: @5 = 00 */
  /* #188: @3 = horzcat(@6, @11, @9, @5) */
  rr=(&w3);
  *rr++ = w11;
  /* #189: @3 = @3' */
  /* #190: @9 = 00 */
  /* #191: @5 = 00 */
  /* #192: @11 = horzcat(@4, @6, @9, @5) */
  rr=(&w11);
  *rr++ = w4;
  /* #193: @11 = @11' */
  /* #194: @9 = 00 */
  /* #195: @10 = (-@4) */
  w10 = (- w4 );
  /* #196: @8 = horzcat(@5, @9, @6, @10) */
  rr=(&w8);
  *rr++ = w10;
  /* #197: @8 = @8' */
  /* #198: @5 = 00 */
  /* #199: @10 = horzcat(@9, @5, @4, @6) */
  rr=(&w10);
  *rr++ = w4;
  /* #200: @10 = @10' */
  /* #201: @13 = horzcat(@3, @11, @8, @10) */
  rr=w13;
  *rr++ = w3;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w10;
  /* #202: @12 = @13' */
  casadi_trans(w13,casadi_s8, w12, casadi_s8, iw);
  /* #203: @9 = zeros(4x4,0nz) */
  /* #204: @13 = horzcat(@12, @9) */
  rr=w13;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #205: @12 = @13' */
  casadi_trans(w13,casadi_s10, w12, casadi_s9, iw);
  /* #206: @13 = horzcat(@7, @12) */
  rr=w13;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #207: @12 = @13' */
  casadi_trans(w13,casadi_s32, w12, casadi_s31, iw);
  /* #208: @2 = mac(@12,@15,@2) */
  casadi_mtimes(w12, casadi_s31, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #209: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #210: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #211: @12 = @2[4:8] */
  for (rr=w12, ss=w2+4; ss!=w2+8; ss+=1) *rr++ = *ss;
  /* #212: (@0[21:53:8] = @12) */
  for (rr=w0+21, ss=w12; rr!=w0+53; rr+=8) *rr = *ss++;
  /* #213: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #214: @7 = zeros(8x4,0nz) */
  /* #215: @9 = 00 */
  /* #216: @5 = 00 */
  /* #217: @3 = ones(8x1,1nz) */
  w3 = 1.;
  /* #218: {NULL, NULL, NULL, NULL, NULL, NULL, @11, NULL} = vertsplit(@3) */
  w11 = w3;
  /* #219: @3 = (-@11) */
  w3 = (- w11 );
  /* #220: @6 = 00 */
  /* #221: @8 = horzcat(@9, @5, @3, @6) */
  rr=(&w8);
  *rr++ = w3;
  /* #222: @8 = @8' */
  /* #223: @5 = 00 */
  /* #224: @6 = 00 */
  /* #225: @3 = horzcat(@5, @9, @6, @11) */
  rr=(&w3);
  *rr++ = w11;
  /* #226: @3 = @3' */
  /* #227: @6 = 00 */
  /* #228: @21 = 00 */
  /* #229: @10 = horzcat(@11, @6, @9, @21) */
  rr=(&w10);
  *rr++ = w11;
  /* #230: @10 = @10' */
  /* #231: @11 = (-@11) */
  w11 = (- w11 );
  /* #232: @4 = horzcat(@6, @11, @5, @9) */
  rr=(&w4);
  *rr++ = w11;
  /* #233: @4 = @4' */
  /* #234: @12 = horzcat(@8, @3, @10, @4) */
  rr=w12;
  *rr++ = w8;
  *rr++ = w3;
  *rr++ = w10;
  *rr++ = w4;
  /* #235: @13 = @12' */
  casadi_trans(w12,casadi_s15, w13, casadi_s15, iw);
  /* #236: @6 = zeros(4x4,0nz) */
  /* #237: @12 = horzcat(@13, @6) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #238: @13 = @12' */
  casadi_trans(w12,casadi_s17, w13, casadi_s16, iw);
  /* #239: @12 = horzcat(@7, @13) */
  rr=w12;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  /* #240: @13 = @12' */
  casadi_trans(w12,casadi_s34, w13, casadi_s33, iw);
  /* #241: @2 = mac(@13,@15,@2) */
  casadi_mtimes(w13, casadi_s33, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #242: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #243: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #244: @13 = @2[4:8] */
  for (rr=w13, ss=w2+4; ss!=w2+8; ss+=1) *rr++ = *ss;
  /* #245: (@0[22:54:8] = @13) */
  for (rr=w0+22, ss=w13; rr!=w0+54; rr+=8) *rr = *ss++;
  /* #246: @2 = zeros(8x1) */
  casadi_clear(w2, 8);
  /* #247: @7 = zeros(8x4,0nz) */
  /* #248: @6 = 00 */
  /* #249: @5 = 00 */
  /* #250: @9 = 00 */
  /* #251: @8 = ones(8x1,1nz) */
  w8 = 1.;
  /* #252: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, @3} = vertsplit(@8) */
  w3 = w8;
  /* #253: @8 = (-@3) */
  w8 = (- w3 );
  /* #254: @10 = horzcat(@6, @5, @9, @8) */
  rr=(&w10);
  *rr++ = w8;
  /* #255: @10 = @10' */
  /* #256: @5 = 00 */
  /* #257: @8 = (-@3) */
  w8 = (- w3 );
  /* #258: @9 = 00 */
  /* #259: @4 = horzcat(@5, @6, @8, @9) */
  rr=(&w4);
  *rr++ = w8;
  /* #260: @4 = @4' */
  /* #261: @21 = 00 */
  /* #262: @8 = horzcat(@9, @3, @6, @21) */
  rr=(&w8);
  *rr++ = w3;
  /* #263: @8 = @8' */
  /* #264: @9 = 00 */
  /* #265: @11 = horzcat(@3, @9, @5, @6) */
  rr=(&w11);
  *rr++ = w3;
  /* #266: @11 = @11' */
  /* #267: @13 = horzcat(@10, @4, @8, @11) */
  rr=w13;
  *rr++ = w10;
  *rr++ = w4;
  *rr++ = w8;
  *rr++ = w11;
  /* #268: @12 = @13' */
  casadi_trans(w13,casadi_s22, w12, casadi_s22, iw);
  /* #269: @9 = zeros(4x4,0nz) */
  /* #270: @13 = horzcat(@12, @9) */
  rr=w13;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #271: @12 = @13' */
  casadi_trans(w13,casadi_s24, w12, casadi_s23, iw);
  /* #272: @13 = horzcat(@7, @12) */
  rr=w13;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  /* #273: @12 = @13' */
  casadi_trans(w13,casadi_s36, w12, casadi_s35, iw);
  /* #274: @2 = mac(@12,@15,@2) */
  casadi_mtimes(w12, casadi_s35, w15, casadi_s6, w2, casadi_s6, w, 0);
  /* #275: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #276: @2 = (-@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) *rr++ = (- *cs++ );
  /* #277: @12 = @2[4:8] */
  for (rr=w12, ss=w2+4; ss!=w2+8; ss+=1) *rr++ = *ss;
  /* #278: (@0[23:55:8] = @12) */
  for (rr=w0+23, ss=w12; rr!=w0+55; rr+=8) *rr = *ss++;
  /* #279: @22 = @0' */
  casadi_trans(w0,casadi_s38, w22, casadi_s37, iw);
  /* #280: output[0][0] = @22 */
  casadi_copy(w22, 48, res[0]);
  /* #281: @2 = zeros(8x8,8nz) */
  casadi_clear(w2, 8);
  /* #282: @15 = ones(8x1) */
  casadi_fill(w15, 8, 1.);
  /* #283: (@2[:8] = @15) */
  for (rr=w2+0, ss=w15; rr!=w2+8; rr+=1) *rr = *ss++;
  /* #284: @15 = @2' */
  casadi_trans(w2,casadi_s5, w15, casadi_s5, iw);
  /* #285: output[1][0] = @15 */
  casadi_copy(w15, 8, res[1]);
  /* #286: @22 = zeros(6x8) */
  casadi_clear(w22, 48);
  /* #287: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #288: @10 = input[0][0] */
  w10 = arg[0] ? arg[0][0] : 0;
  /* #289: @4 = input[0][1] */
  w4 = arg[0] ? arg[0][1] : 0;
  /* #290: @8 = (-@4) */
  w8 = (- w4 );
  /* #291: @11 = input[0][2] */
  w11 = arg[0] ? arg[0][2] : 0;
  /* #292: @3 = (-@11) */
  w3 = (- w11 );
  /* #293: @17 = input[0][3] */
  w17 = arg[0] ? arg[0][3] : 0;
  /* #294: @18 = (-@17) */
  w18 = (- w17 );
  /* #295: @12 = horzcat(@10, @8, @3, @18) */
  rr=w12;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w3;
  *rr++ = w18;
  /* #296: @12 = @12' */
  /* #297: @8 = (-@17) */
  w8 = (- w17 );
  /* #298: @13 = horzcat(@4, @10, @8, @11) */
  rr=w13;
  *rr++ = w4;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w11;
  /* #299: @13 = @13' */
  /* #300: @8 = (-@4) */
  w8 = (- w4 );
  /* #301: @14 = horzcat(@11, @17, @10, @8) */
  rr=w14;
  *rr++ = w11;
  *rr++ = w17;
  *rr++ = w10;
  *rr++ = w8;
  /* #302: @14 = @14' */
  /* #303: @11 = (-@11) */
  w11 = (- w11 );
  /* #304: @23 = horzcat(@17, @11, @4, @10) */
  rr=w23;
  *rr++ = w17;
  *rr++ = w11;
  *rr++ = w4;
  *rr++ = w10;
  /* #305: @23 = @23' */
  /* #306: @24 = horzcat(@12, @13, @14, @23) */
  rr=w24;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w23; i<4; ++i) *rr++ = *cs++;
  /* #307: @25 = @24' */
  for (i=0, rr=w25, cs=w24; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #308: @24 = zeros(4x4) */
  casadi_clear(w24, 16);
  /* #309: @26 = horzcat(@25, @24) */
  rr=w26;
  for (i=0, cs=w25; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w24; i<16; ++i) *rr++ = *cs++;
  /* #310: @27 = @26' */
  for (i=0, rr=w27, cs=w26; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #311: @17 = input[0][4] */
  w17 = arg[0] ? arg[0][4] : 0;
  /* #312: @11 = input[0][5] */
  w11 = arg[0] ? arg[0][5] : 0;
  /* #313: @4 = (-@11) */
  w4 = (- w11 );
  /* #314: @10 = input[0][6] */
  w10 = arg[0] ? arg[0][6] : 0;
  /* #315: @8 = (-@10) */
  w8 = (- w10 );
  /* #316: @3 = input[0][7] */
  w3 = arg[0] ? arg[0][7] : 0;
  /* #317: @18 = (-@3) */
  w18 = (- w3 );
  /* #318: @12 = horzcat(@17, @4, @8, @18) */
  rr=w12;
  *rr++ = w17;
  *rr++ = w4;
  *rr++ = w8;
  *rr++ = w18;
  /* #319: @12 = @12' */
  /* #320: @4 = (-@3) */
  w4 = (- w3 );
  /* #321: @13 = horzcat(@11, @17, @4, @10) */
  rr=w13;
  *rr++ = w11;
  *rr++ = w17;
  *rr++ = w4;
  *rr++ = w10;
  /* #322: @13 = @13' */
  /* #323: @4 = (-@11) */
  w4 = (- w11 );
  /* #324: @14 = horzcat(@10, @3, @17, @4) */
  rr=w14;
  *rr++ = w10;
  *rr++ = w3;
  *rr++ = w17;
  *rr++ = w4;
  /* #325: @14 = @14' */
  /* #326: @10 = (-@10) */
  w10 = (- w10 );
  /* #327: @23 = horzcat(@3, @10, @11, @17) */
  rr=w23;
  *rr++ = w3;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w17;
  /* #328: @23 = @23' */
  /* #329: @24 = horzcat(@12, @13, @14, @23) */
  rr=w24;
  for (i=0, cs=w12; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w23; i<4; ++i) *rr++ = *cs++;
  /* #330: @28 = @24' */
  for (i=0, rr=w28, cs=w24; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #331: @26 = horzcat(@28, @25) */
  rr=w26;
  for (i=0, cs=w28; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w25; i<16; ++i) *rr++ = *cs++;
  /* #332: @29 = @26' */
  for (i=0, rr=w29, cs=w26; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #333: @30 = horzcat(@27, @29) */
  rr=w30;
  for (i=0, cs=w27; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w29; i<32; ++i) *rr++ = *cs++;
  /* #334: @31 = @30' */
  for (i=0, rr=w31, cs=w30; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #335: @7 = 00 */
  /* #336: @3 = ones(6x1,1nz) */
  w3 = 1.;
  /* #337: {@10, NULL, NULL, NULL, NULL, NULL} = vertsplit(@3) */
  w10 = w3;
  /* #338: @9 = 00 */
  /* #339: @5 = 00 */
  /* #340: @6 = 00 */
  /* #341: @21 = 00 */
  /* #342: @32 = 00 */
  /* #343: @33 = 00 */
  /* #344: @3 = vertcat(@7, @10, @9, @5, @6, @21, @32, @33) */
  rr=(&w3);
  *rr++ = w10;
  /* #345: @15 = mac(@31,@3,@15) */
  casadi_mtimes(w31, casadi_s40, (&w3), casadi_s39, w15, casadi_s6, w, 0);
  /* #346: @15 = (@1*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #347: @15 = (-@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) *rr++ = (- *cs++ );
  /* #348: (@22[:48:6] = @15) */
  for (rr=w22+0, ss=w15; rr!=w22+48; rr+=6) *rr = *ss++;
  /* #349: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #350: @7 = 00 */
  /* #351: @9 = 00 */
  /* #352: @3 = ones(6x1,1nz) */
  w3 = 1.;
  /* #353: {NULL, @10, NULL, NULL, NULL, NULL} = vertsplit(@3) */
  w10 = w3;
  /* #354: @5 = 00 */
  /* #355: @6 = 00 */
  /* #356: @21 = 00 */
  /* #357: @32 = 00 */
  /* #358: @33 = 00 */
  /* #359: @3 = vertcat(@7, @9, @10, @5, @6, @21, @32, @33) */
  rr=(&w3);
  *rr++ = w10;
  /* #360: @15 = mac(@31,@3,@15) */
  casadi_mtimes(w31, casadi_s40, (&w3), casadi_s41, w15, casadi_s6, w, 0);
  /* #361: @15 = (@1*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #362: @15 = (-@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) *rr++ = (- *cs++ );
  /* #363: (@22[1:49:6] = @15) */
  for (rr=w22+1, ss=w15; rr!=w22+49; rr+=6) *rr = *ss++;
  /* #364: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #365: @7 = 00 */
  /* #366: @9 = 00 */
  /* #367: @5 = 00 */
  /* #368: @3 = ones(6x1,1nz) */
  w3 = 1.;
  /* #369: {NULL, NULL, @10, NULL, NULL, NULL} = vertsplit(@3) */
  w10 = w3;
  /* #370: @6 = 00 */
  /* #371: @21 = 00 */
  /* #372: @32 = 00 */
  /* #373: @33 = 00 */
  /* #374: @3 = vertcat(@7, @9, @5, @10, @6, @21, @32, @33) */
  rr=(&w3);
  *rr++ = w10;
  /* #375: @15 = mac(@31,@3,@15) */
  casadi_mtimes(w31, casadi_s40, (&w3), casadi_s42, w15, casadi_s6, w, 0);
  /* #376: @15 = (@1*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #377: @15 = (-@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) *rr++ = (- *cs++ );
  /* #378: (@22[2:50:6] = @15) */
  for (rr=w22+2, ss=w15; rr!=w22+50; rr+=6) *rr = *ss++;
  /* #379: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #380: @7 = 00 */
  /* #381: @9 = 00 */
  /* #382: @5 = 00 */
  /* #383: @6 = 00 */
  /* #384: @21 = 00 */
  /* #385: @3 = ones(6x1,1nz) */
  w3 = 1.;
  /* #386: {NULL, NULL, NULL, @10, NULL, NULL} = vertsplit(@3) */
  w10 = w3;
  /* #387: @32 = 00 */
  /* #388: @33 = 00 */
  /* #389: @3 = vertcat(@7, @9, @5, @6, @21, @10, @32, @33) */
  rr=(&w3);
  *rr++ = w10;
  /* #390: @15 = mac(@31,@3,@15) */
  casadi_mtimes(w31, casadi_s40, (&w3), casadi_s43, w15, casadi_s6, w, 0);
  /* #391: @15 = (@1*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #392: @15 = (-@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) *rr++ = (- *cs++ );
  /* #393: (@22[3:51:6] = @15) */
  for (rr=w22+3, ss=w15; rr!=w22+51; rr+=6) *rr = *ss++;
  /* #394: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #395: @7 = 00 */
  /* #396: @9 = 00 */
  /* #397: @5 = 00 */
  /* #398: @6 = 00 */
  /* #399: @21 = 00 */
  /* #400: @32 = 00 */
  /* #401: @3 = ones(6x1,1nz) */
  w3 = 1.;
  /* #402: {NULL, NULL, NULL, NULL, @10, NULL} = vertsplit(@3) */
  w10 = w3;
  /* #403: @33 = 00 */
  /* #404: @3 = vertcat(@7, @9, @5, @6, @21, @32, @10, @33) */
  rr=(&w3);
  *rr++ = w10;
  /* #405: @15 = mac(@31,@3,@15) */
  casadi_mtimes(w31, casadi_s40, (&w3), casadi_s44, w15, casadi_s6, w, 0);
  /* #406: @15 = (@1*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #407: @15 = (-@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) *rr++ = (- *cs++ );
  /* #408: (@22[4:52:6] = @15) */
  for (rr=w22+4, ss=w15; rr!=w22+52; rr+=6) *rr = *ss++;
  /* #409: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #410: @7 = 00 */
  /* #411: @9 = 00 */
  /* #412: @5 = 00 */
  /* #413: @6 = 00 */
  /* #414: @21 = 00 */
  /* #415: @32 = 00 */
  /* #416: @33 = 00 */
  /* #417: @3 = ones(6x1,1nz) */
  w3 = 1.;
  /* #418: {NULL, NULL, NULL, NULL, NULL, @10} = vertsplit(@3) */
  w10 = w3;
  /* #419: @3 = vertcat(@7, @9, @5, @6, @21, @32, @33, @10) */
  rr=(&w3);
  *rr++ = w10;
  /* #420: @15 = mac(@31,@3,@15) */
  casadi_mtimes(w31, casadi_s40, (&w3), casadi_s45, w15, casadi_s6, w, 0);
  /* #421: @15 = (@1*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w1*(*cs++));
  /* #422: @15 = (-@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) *rr++ = (- *cs++ );
  /* #423: (@22[5:53:6] = @15) */
  for (rr=w22+5, ss=w15; rr!=w22+53; rr+=6) *rr = *ss++;
  /* #424: @0 = @22' */
  for (i=0, rr=w0, cs=w22; i<8; ++i) for (j=0; j<6; ++j) rr[i+j*8] = *cs++;
  /* #425: output[2][0] = @0 */
  casadi_copy(w0, 48, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_jac_x_xdot_u_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_jac_x_xdot_u_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_jac_x_xdot_u_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_jac_x_xdot_u_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_jac_x_xdot_u_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_jac_x_xdot_u_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_jac_x_xdot_u_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_jac_x_xdot_u_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_impl_dae_jac_x_xdot_u_z_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_impl_dae_jac_x_xdot_u_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_impl_dae_jac_x_xdot_u_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_impl_dae_jac_x_xdot_u_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_impl_dae_jac_x_xdot_u_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_impl_dae_jac_x_xdot_u_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s6;
    case 1: return casadi_s6;
    case 2: return casadi_s46;
    case 3: return casadi_s47;
    case 4: return casadi_s47;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_impl_dae_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s37;
    case 1: return casadi_s5;
    case 2: return casadi_s48;
    case 3: return casadi_s49;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 13;
  if (sz_res) *sz_res = 12;
  if (sz_iw) *sz_iw = 9;
  if (sz_w) *sz_w = 433;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
