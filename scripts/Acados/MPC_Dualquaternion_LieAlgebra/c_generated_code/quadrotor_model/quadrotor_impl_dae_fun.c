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
  #define CASADI_PREFIX(ID) quadrotor_impl_dae_fun_ ## ID
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
#define casadi_c3 CASADI_PREFIX(c3)
#define casadi_c4 CASADI_PREFIX(c4)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_f4 CASADI_PREFIX(f4)
#define casadi_f5 CASADI_PREFIX(f5)
#define casadi_f6 CASADI_PREFIX(f6)
#define casadi_mtimes CASADI_PREFIX(mtimes)
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

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
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

static const casadi_int casadi_s0[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s1[9] = {3, 3, 0, 1, 2, 3, 0, 1, 2};
static const casadi_int casadi_s2[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s3[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s4[3] = {0, 0, 0};
static const casadi_int casadi_s5[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

static const casadi_real casadi_c0[3] = {-3.7878787878787881e+02, -3.7878787878787881e+02, -2.0161290322580646e+02};
static const casadi_real casadi_c1[9] = {2.6400000000000000e-03, 0., 0., 0., 2.6400000000000000e-03, 0., 0., 0., 4.9600000000000000e-03};
static const casadi_real casadi_c2[3] = {3.7878787878787881e+02, 3.7878787878787881e+02, 2.0161290322580646e+02};
static const casadi_real casadi_c3[3] = {0., 0., 9.8000000000000007e+00};
static const casadi_real casadi_c4[3] = {0., 0., 1.};

/* f_real:(i0[8])->(o0[4]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
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

/* f_dual:(i0[8])->(o0[4]) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0;
  /* #0: @0 = input[0][4] */
  w0 = arg[0] ? arg[0][4] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #3: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #4: @0 = input[0][6] */
  w0 = arg[0] ? arg[0][6] : 0;
  /* #5: output[0][2] = @0 */
  if (res[0]) res[0][2] = w0;
  /* #6: @0 = input[0][7] */
  w0 = arg[0] ? arg[0][7] : 0;
  /* #7: output[0][3] = @0 */
  if (res[0]) res[0][3] = w0;
  return 0;
}

/* f_trans:(i0[8])->(o0[4]) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
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

/* f_velocity:(i0[6],i1[8])->(o0[6]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real **res1=res+1, *rr, *ss;
  const casadi_real **arg1=arg+2, *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, *w7=w+67, *w8=w+73, *w9=w+76, *w10=w+79, *w11=w+87, *w12=w+91, w13, w14;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = input[0][0] */
  w1 = arg[0] ? arg[0][0] : 0;
  /* #2: @2 = input[0][1] */
  w2 = arg[0] ? arg[0][1] : 0;
  /* #3: @3 = input[0][2] */
  w3 = arg[0] ? arg[0][2] : 0;
  /* #4: @4 = input[0][3] */
  w4 = arg[0] ? arg[0][3] : 0;
  /* #5: @5 = input[0][4] */
  w5 = arg[0] ? arg[0][4] : 0;
  /* #6: @6 = input[0][5] */
  w6 = arg[0] ? arg[0][5] : 0;
  /* #7: @7 = vertcat(@1, @2, @3, @4, @5, @6) */
  rr=w7;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #8: @8 = @7[:3] */
  for (rr=w8, ss=w7+0; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #9: @1 = 0 */
  w1 = 0.;
  /* #10: @9 = @7[3:6] */
  for (rr=w9, ss=w7+3; ss!=w7+6; ss+=1) *rr++ = *ss;
  /* #11: @10 = vertcat(@0, @8, @1, @9) */
  rr=w10;
  *rr++ = w0;
  for (i=0, cs=w8; i<3; ++i) *rr++ = *cs++;
  *rr++ = w1;
  for (i=0, cs=w9; i<3; ++i) *rr++ = *cs++;
  /* #12: @11 = f_real(@10) */
  arg1[0]=w10;
  res1[0]=w11;
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #13: @8 = @11[1:4] */
  for (rr=w8, ss=w11+1; ss!=w11+4; ss+=1) *rr++ = *ss;
  /* #14: output[0][0] = @8 */
  casadi_copy(w8, 3, res[0]);
  /* #15: @12 = f_dual(@10) */
  arg1[0]=w10;
  res1[0]=w12;
  if (casadi_f3(arg1, res1, iw, w, 0)) return 1;
  /* #16: @8 = @12[1:4] */
  for (rr=w8, ss=w12+1; ss!=w12+4; ss+=1) *rr++ = *ss;
  /* #17: @0 = @11[2] */
  for (rr=(&w0), ss=w11+2; ss!=w11+3; ss+=1) *rr++ = *ss;
  /* #18: @1 = input[1][0] */
  w1 = arg[1] ? arg[1][0] : 0;
  /* #19: @2 = input[1][1] */
  w2 = arg[1] ? arg[1][1] : 0;
  /* #20: @3 = input[1][2] */
  w3 = arg[1] ? arg[1][2] : 0;
  /* #21: @4 = input[1][3] */
  w4 = arg[1] ? arg[1][3] : 0;
  /* #22: @5 = input[1][4] */
  w5 = arg[1] ? arg[1][4] : 0;
  /* #23: @6 = input[1][5] */
  w6 = arg[1] ? arg[1][5] : 0;
  /* #24: @13 = input[1][6] */
  w13 = arg[1] ? arg[1][6] : 0;
  /* #25: @14 = input[1][7] */
  w14 = arg[1] ? arg[1][7] : 0;
  /* #26: @10 = vertcat(@1, @2, @3, @4, @5, @6, @13, @14) */
  rr=w10;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w13;
  *rr++ = w14;
  /* #27: @12 = f_trans(@10) */
  arg1[0]=w10;
  res1[0]=w12;
  if (casadi_f4(arg1, res1, iw, w, 0)) return 1;
  /* #28: @1 = @12[3] */
  for (rr=(&w1), ss=w12+3; ss!=w12+4; ss+=1) *rr++ = *ss;
  /* #29: @2 = (@0*@1) */
  w2  = (w0*w1);
  /* #30: @3 = @11[3] */
  for (rr=(&w3), ss=w11+3; ss!=w11+4; ss+=1) *rr++ = *ss;
  /* #31: @4 = @12[2] */
  for (rr=(&w4), ss=w12+2; ss!=w12+3; ss+=1) *rr++ = *ss;
  /* #32: @5 = (@3*@4) */
  w5  = (w3*w4);
  /* #33: @2 = (@2-@5) */
  w2 -= w5;
  /* #34: @5 = @12[1] */
  for (rr=(&w5), ss=w12+1; ss!=w12+2; ss+=1) *rr++ = *ss;
  /* #35: @3 = (@3*@5) */
  w3 *= w5;
  /* #36: @6 = @11[1] */
  for (rr=(&w6), ss=w11+1; ss!=w11+2; ss+=1) *rr++ = *ss;
  /* #37: @1 = (@6*@1) */
  w1  = (w6*w1);
  /* #38: @3 = (@3-@1) */
  w3 -= w1;
  /* #39: @6 = (@6*@4) */
  w6 *= w4;
  /* #40: @0 = (@0*@5) */
  w0 *= w5;
  /* #41: @6 = (@6-@0) */
  w6 -= w0;
  /* #42: @9 = vertcat(@2, @3, @6) */
  rr=w9;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w6;
  /* #43: @8 = (@8-@9) */
  for (i=0, rr=w8, cs=w9; i<3; ++i) (*rr++) -= (*cs++);
  /* #44: output[0][1] = @8 */
  if (res[0]) casadi_copy(w8, 3, res[0]+3);
  return 0;
}

/* f_quat:(i0[8])->(o0[4]) */
static int casadi_f5(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
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

/* f_rot:(i0[4],i1[3])->(o0[3]) */
static int casadi_f6(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, *w1=w+8, *w2=w+12, w3, w4, w5, w6, *w7=w+20, *w8=w+24, *w9=w+28, *w10=w+32, *w11=w+36, *w12=w+52, *w13=w+68;
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
  /* #5: @4 = (-@4) */
  w4 = (- w4 );
  /* #6: @5 = @2[2] */
  for (rr=(&w5), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #7: @5 = (-@5) */
  w5 = (- w5 );
  /* #8: @6 = @2[3] */
  for (rr=(&w6), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #9: @6 = (-@6) */
  w6 = (- w6 );
  /* #10: @7 = horzcat(@3, @4, @5, @6) */
  rr=w7;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #11: @7 = @7' */
  /* #12: @3 = @2[1] */
  for (rr=(&w3), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #13: @4 = @2[0] */
  for (rr=(&w4), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #14: @5 = @2[3] */
  for (rr=(&w5), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #15: @5 = (-@5) */
  w5 = (- w5 );
  /* #16: @6 = @2[2] */
  for (rr=(&w6), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #17: @8 = horzcat(@3, @4, @5, @6) */
  rr=w8;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #18: @8 = @8' */
  /* #19: @3 = @2[2] */
  for (rr=(&w3), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #20: @4 = @2[3] */
  for (rr=(&w4), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #21: @5 = @2[0] */
  for (rr=(&w5), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #22: @6 = @2[1] */
  for (rr=(&w6), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #23: @6 = (-@6) */
  w6 = (- w6 );
  /* #24: @9 = horzcat(@3, @4, @5, @6) */
  rr=w9;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #25: @9 = @9' */
  /* #26: @3 = @2[3] */
  for (rr=(&w3), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #27: @4 = @2[2] */
  for (rr=(&w4), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #28: @4 = (-@4) */
  w4 = (- w4 );
  /* #29: @5 = @2[1] */
  for (rr=(&w5), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #30: @6 = @2[0] */
  for (rr=(&w6), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #31: @10 = horzcat(@3, @4, @5, @6) */
  rr=w10;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #32: @10 = @10' */
  /* #33: @11 = horzcat(@7, @8, @9, @10) */
  rr=w11;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w8; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w9; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  /* #34: @12 = @11' */
  for (i=0, rr=w12, cs=w11; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #35: @3 = 0 */
  w3 = 0.;
  /* #36: @13 = input[1][0] */
  casadi_copy(arg[1], 3, w13);
  /* #37: @7 = vertcat(@3, @13) */
  rr=w7;
  *rr++ = w3;
  for (i=0, cs=w13; i<3; ++i) *rr++ = *cs++;
  /* #38: @1 = mac(@12,@7,@1) */
  for (i=0, rr=w1; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w12+j, tt=w7+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #39: @3 = @1[0] */
  for (rr=(&w3), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #40: @4 = @1[1] */
  for (rr=(&w4), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #41: @4 = (-@4) */
  w4 = (- w4 );
  /* #42: @5 = @1[2] */
  for (rr=(&w5), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #43: @5 = (-@5) */
  w5 = (- w5 );
  /* #44: @6 = @1[3] */
  for (rr=(&w6), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #45: @6 = (-@6) */
  w6 = (- w6 );
  /* #46: @7 = horzcat(@3, @4, @5, @6) */
  rr=w7;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #47: @7 = @7' */
  /* #48: @3 = @1[1] */
  for (rr=(&w3), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #49: @4 = @1[0] */
  for (rr=(&w4), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #50: @5 = @1[3] */
  for (rr=(&w5), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #51: @5 = (-@5) */
  w5 = (- w5 );
  /* #52: @6 = @1[2] */
  for (rr=(&w6), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #53: @8 = horzcat(@3, @4, @5, @6) */
  rr=w8;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #54: @8 = @8' */
  /* #55: @3 = @1[2] */
  for (rr=(&w3), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #56: @4 = @1[3] */
  for (rr=(&w4), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #57: @5 = @1[0] */
  for (rr=(&w5), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #58: @6 = @1[1] */
  for (rr=(&w6), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #59: @6 = (-@6) */
  w6 = (- w6 );
  /* #60: @9 = horzcat(@3, @4, @5, @6) */
  rr=w9;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #61: @9 = @9' */
  /* #62: @3 = @1[3] */
  for (rr=(&w3), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #63: @4 = @1[2] */
  for (rr=(&w4), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #64: @4 = (-@4) */
  w4 = (- w4 );
  /* #65: @5 = @1[1] */
  for (rr=(&w5), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #66: @6 = @1[0] */
  for (rr=(&w6), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #67: @1 = horzcat(@3, @4, @5, @6) */
  rr=w1;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #68: @1 = @1' */
  /* #69: @12 = horzcat(@7, @8, @9, @1) */
  rr=w12;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w8; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w9; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w1; i<4; ++i) *rr++ = *cs++;
  /* #70: @11 = @12' */
  for (i=0, rr=w11, cs=w12; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #71: @3 = @2[0] */
  for (rr=(&w3), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #72: @4 = @2[1] */
  for (rr=(&w4), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #73: @4 = (-@4) */
  w4 = (- w4 );
  /* #74: @5 = @2[2] */
  for (rr=(&w5), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #75: @5 = (-@5) */
  w5 = (- w5 );
  /* #76: @6 = @2[3] */
  for (rr=(&w6), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #77: @6 = (-@6) */
  w6 = (- w6 );
  /* #78: @2 = vertcat(@3, @4, @5, @6) */
  rr=w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #79: @0 = mac(@11,@2,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w11+j, tt=w2+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #80: @13 = @0[1:4] */
  for (rr=w13, ss=w0+1; ss!=w0+4; ss+=1) *rr++ = *ss;
  /* #81: output[0][0] = @13 */
  casadi_copy(w13, 3, res[0]);
  return 0;
}

/* quadrotor_impl_dae_fun:(i0[14],i1[14],i2[4],i3[],i4[18])->(o0[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+1, *rr, *ss, *tt;
  const casadi_real **arg1=arg+5, *cr, *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, *w14=w+111, *w15=w+125, *w16=w+133, *w17=w+137, *w18=w+141, *w19=w+145, *w20=w+149, *w21=w+165, *w22=w+181, *w23=w+213, *w24=w+245, *w25=w+261, *w26=w+293, *w27=w+357, w28, w29, w30, *w31=w+424, *w32=w+432, *w33=w+446, *w34=w+449, *w35=w+452, *w36=w+458, *w37=w+464, *w38=w+467, *w39=w+476, *w40=w+479, *w41=w+482;
  /* #0: @0 = input[1][0] */
  w0 = arg[1] ? arg[1][0] : 0;
  /* #1: @1 = input[1][1] */
  w1 = arg[1] ? arg[1][1] : 0;
  /* #2: @2 = input[1][2] */
  w2 = arg[1] ? arg[1][2] : 0;
  /* #3: @3 = input[1][3] */
  w3 = arg[1] ? arg[1][3] : 0;
  /* #4: @4 = input[1][4] */
  w4 = arg[1] ? arg[1][4] : 0;
  /* #5: @5 = input[1][5] */
  w5 = arg[1] ? arg[1][5] : 0;
  /* #6: @6 = input[1][6] */
  w6 = arg[1] ? arg[1][6] : 0;
  /* #7: @7 = input[1][7] */
  w7 = arg[1] ? arg[1][7] : 0;
  /* #8: @8 = input[1][8] */
  w8 = arg[1] ? arg[1][8] : 0;
  /* #9: @9 = input[1][9] */
  w9 = arg[1] ? arg[1][9] : 0;
  /* #10: @10 = input[1][10] */
  w10 = arg[1] ? arg[1][10] : 0;
  /* #11: @11 = input[1][11] */
  w11 = arg[1] ? arg[1][11] : 0;
  /* #12: @12 = input[1][12] */
  w12 = arg[1] ? arg[1][12] : 0;
  /* #13: @13 = input[1][13] */
  w13 = arg[1] ? arg[1][13] : 0;
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
  /* #15: @0 = 0.5 */
  w0 = 5.0000000000000000e-01;
  /* #16: @15 = zeros(8x1) */
  casadi_clear(w15, 8);
  /* #17: @1 = input[0][0] */
  w1 = arg[0] ? arg[0][0] : 0;
  /* #18: @2 = input[0][1] */
  w2 = arg[0] ? arg[0][1] : 0;
  /* #19: @3 = (-@2) */
  w3 = (- w2 );
  /* #20: @4 = input[0][2] */
  w4 = arg[0] ? arg[0][2] : 0;
  /* #21: @5 = (-@4) */
  w5 = (- w4 );
  /* #22: @6 = input[0][3] */
  w6 = arg[0] ? arg[0][3] : 0;
  /* #23: @7 = (-@6) */
  w7 = (- w6 );
  /* #24: @16 = horzcat(@1, @3, @5, @7) */
  rr=w16;
  *rr++ = w1;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w7;
  /* #25: @16 = @16' */
  /* #26: @3 = (-@6) */
  w3 = (- w6 );
  /* #27: @17 = horzcat(@2, @1, @3, @4) */
  rr=w17;
  *rr++ = w2;
  *rr++ = w1;
  *rr++ = w3;
  *rr++ = w4;
  /* #28: @17 = @17' */
  /* #29: @3 = (-@2) */
  w3 = (- w2 );
  /* #30: @18 = horzcat(@4, @6, @1, @3) */
  rr=w18;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w1;
  *rr++ = w3;
  /* #31: @18 = @18' */
  /* #32: @3 = (-@4) */
  w3 = (- w4 );
  /* #33: @19 = horzcat(@6, @3, @2, @1) */
  rr=w19;
  *rr++ = w6;
  *rr++ = w3;
  *rr++ = w2;
  *rr++ = w1;
  /* #34: @19 = @19' */
  /* #35: @20 = horzcat(@16, @17, @18, @19) */
  rr=w20;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w19; i<4; ++i) *rr++ = *cs++;
  /* #36: @21 = @20' */
  for (i=0, rr=w21, cs=w20; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #37: @20 = zeros(4x4) */
  casadi_clear(w20, 16);
  /* #38: @22 = horzcat(@21, @20) */
  rr=w22;
  for (i=0, cs=w21; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w20; i<16; ++i) *rr++ = *cs++;
  /* #39: @23 = @22' */
  for (i=0, rr=w23, cs=w22; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #40: @3 = input[0][4] */
  w3 = arg[0] ? arg[0][4] : 0;
  /* #41: @5 = input[0][5] */
  w5 = arg[0] ? arg[0][5] : 0;
  /* #42: @7 = (-@5) */
  w7 = (- w5 );
  /* #43: @8 = input[0][6] */
  w8 = arg[0] ? arg[0][6] : 0;
  /* #44: @9 = (-@8) */
  w9 = (- w8 );
  /* #45: @10 = input[0][7] */
  w10 = arg[0] ? arg[0][7] : 0;
  /* #46: @11 = (-@10) */
  w11 = (- w10 );
  /* #47: @16 = horzcat(@3, @7, @9, @11) */
  rr=w16;
  *rr++ = w3;
  *rr++ = w7;
  *rr++ = w9;
  *rr++ = w11;
  /* #48: @16 = @16' */
  /* #49: @7 = (-@10) */
  w7 = (- w10 );
  /* #50: @17 = horzcat(@5, @3, @7, @8) */
  rr=w17;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w7;
  *rr++ = w8;
  /* #51: @17 = @17' */
  /* #52: @7 = (-@5) */
  w7 = (- w5 );
  /* #53: @18 = horzcat(@8, @10, @3, @7) */
  rr=w18;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w3;
  *rr++ = w7;
  /* #54: @18 = @18' */
  /* #55: @7 = (-@8) */
  w7 = (- w8 );
  /* #56: @19 = horzcat(@10, @7, @5, @3) */
  rr=w19;
  *rr++ = w10;
  *rr++ = w7;
  *rr++ = w5;
  *rr++ = w3;
  /* #57: @19 = @19' */
  /* #58: @20 = horzcat(@16, @17, @18, @19) */
  rr=w20;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w19; i<4; ++i) *rr++ = *cs++;
  /* #59: @24 = @20' */
  for (i=0, rr=w24, cs=w20; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #60: @22 = horzcat(@24, @21) */
  rr=w22;
  for (i=0, cs=w24; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w21; i<16; ++i) *rr++ = *cs++;
  /* #61: @25 = @22' */
  for (i=0, rr=w25, cs=w22; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #62: @26 = horzcat(@23, @25) */
  rr=w26;
  for (i=0, cs=w23; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w25; i<32; ++i) *rr++ = *cs++;
  /* #63: @27 = @26' */
  for (i=0, rr=w27, cs=w26; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #64: @7 = 0 */
  w7 = 0.;
  /* #65: @9 = input[0][8] */
  w9 = arg[0] ? arg[0][8] : 0;
  /* #66: @11 = input[0][9] */
  w11 = arg[0] ? arg[0][9] : 0;
  /* #67: @12 = input[0][10] */
  w12 = arg[0] ? arg[0][10] : 0;
  /* #68: @13 = 0 */
  w13 = 0.;
  /* #69: @28 = input[0][11] */
  w28 = arg[0] ? arg[0][11] : 0;
  /* #70: @29 = input[0][12] */
  w29 = arg[0] ? arg[0][12] : 0;
  /* #71: @30 = input[0][13] */
  w30 = arg[0] ? arg[0][13] : 0;
  /* #72: @31 = vertcat(@7, @9, @11, @12, @13, @28, @29, @30) */
  rr=w31;
  *rr++ = w7;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w13;
  *rr++ = w28;
  *rr++ = w29;
  *rr++ = w30;
  /* #73: @15 = mac(@27,@31,@15) */
  for (i=0, rr=w15; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w27+j, tt=w31+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #74: @15 = (@0*@15) */
  for (i=0, rr=w15, cs=w15; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #75: @32 = vertcat(@1, @2, @4, @6, @3, @5, @8, @10, @9, @11, @12, @28, @29, @30) */
  rr=w32;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w8;
  *rr++ = w10;
  *rr++ = w9;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w28;
  *rr++ = w29;
  *rr++ = w30;
  /* #76: @16 = @32[:4] */
  for (rr=w16, ss=w32+0; ss!=w32+4; ss+=1) *rr++ = *ss;
  /* #77: @1 = 1 */
  w1 = 1.;
  /* #78: @2 = dot(@16, @16) */
  w2 = casadi_dot(4, w16, w16);
  /* #79: @1 = (@1-@2) */
  w1 -= w2;
  /* #80: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #81: @17 = (@16*@1) */
  for (i=0, rr=w17, cr=w16; i<4; ++i) (*rr++)  = ((*cr++)*w1);
  /* #82: @18 = @32[4:8] */
  for (rr=w18, ss=w32+4; ss!=w32+8; ss+=1) *rr++ = *ss;
  /* #83: @1 = 0 */
  w1 = 0.;
  /* #84: @16 = @16' */
  /* #85: @1 = mac(@16,@18,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w16+j, tt=w18+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #86: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #87: @18 = (@18*@1) */
  for (i=0, rr=w18; i<4; ++i) (*rr++) *= w1;
  /* #88: @31 = vertcat(@17, @18) */
  rr=w31;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  /* #89: @15 = (@15+@31) */
  for (i=0, rr=w15, cs=w31; i<8; ++i) (*rr++) += (*cs++);
  /* #90: @33 = zeros(3x1) */
  casadi_clear(w33, 3);
  /* #91: @34 = 
  [[-378.788, 00, 00], 
   [00, -378.788, 00], 
   [00, 00, -201.613]] */
  casadi_copy(casadi_c0, 3, w34);
  /* #92: @35 = @32[8:14] */
  for (rr=w35, ss=w32+8; ss!=w32+14; ss+=1) *rr++ = *ss;
  /* #93: @31 = @32[:8] */
  for (rr=w31, ss=w32+0; ss!=w32+8; ss+=1) *rr++ = *ss;
  /* #94: @36 = f_velocity(@35, @31) */
  arg1[0]=w35;
  arg1[1]=w31;
  res1[0]=w36;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #95: @37 = @36[:3] */
  for (rr=w37, ss=w36+0; ss!=w36+3; ss+=1) *rr++ = *ss;
  /* #96: @33 = mac(@34,@37,@33) */
  casadi_mtimes(w34, casadi_s1, w37, casadi_s0, w33, casadi_s0, w, 0);
  /* #97: @1 = @33[1] */
  for (rr=(&w1), ss=w33+1; ss!=w33+2; ss+=1) *rr++ = *ss;
  /* #98: @34 = zeros(3x1) */
  casadi_clear(w34, 3);
  /* #99: @38 = 
  [[0.00264, 0, 0], 
   [0, 0.00264, 0], 
   [0, 0, 0.00496]] */
  casadi_copy(casadi_c1, 9, w38);
  /* #100: @34 = mac(@38,@37,@34) */
  for (i=0, rr=w34; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w38+j, tt=w37+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #101: @2 = @34[2] */
  for (rr=(&w2), ss=w34+2; ss!=w34+3; ss+=1) *rr++ = *ss;
  /* #102: @4 = (@1*@2) */
  w4  = (w1*w2);
  /* #103: @6 = @33[2] */
  for (rr=(&w6), ss=w33+2; ss!=w33+3; ss+=1) *rr++ = *ss;
  /* #104: @3 = @34[1] */
  for (rr=(&w3), ss=w34+1; ss!=w34+2; ss+=1) *rr++ = *ss;
  /* #105: @5 = (@6*@3) */
  w5  = (w6*w3);
  /* #106: @4 = (@4-@5) */
  w4 -= w5;
  /* #107: @5 = @34[0] */
  for (rr=(&w5), ss=w34+0; ss!=w34+1; ss+=1) *rr++ = *ss;
  /* #108: @6 = (@6*@5) */
  w6 *= w5;
  /* #109: @8 = @33[0] */
  for (rr=(&w8), ss=w33+0; ss!=w33+1; ss+=1) *rr++ = *ss;
  /* #110: @2 = (@8*@2) */
  w2  = (w8*w2);
  /* #111: @6 = (@6-@2) */
  w6 -= w2;
  /* #112: @8 = (@8*@3) */
  w8 *= w3;
  /* #113: @1 = (@1*@5) */
  w1 *= w5;
  /* #114: @8 = (@8-@1) */
  w8 -= w1;
  /* #115: @33 = vertcat(@4, @6, @8) */
  rr=w33;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w8;
  /* #116: @34 = zeros(3x1) */
  casadi_clear(w34, 3);
  /* #117: @37 = 
  [[378.788, 00, 00], 
   [00, 378.788, 00], 
   [00, 00, 201.613]] */
  casadi_copy(casadi_c2, 3, w37);
  /* #118: @1 = input[2][0] */
  w1 = arg[2] ? arg[2][0] : 0;
  /* #119: @5 = input[2][1] */
  w5 = arg[2] ? arg[2][1] : 0;
  /* #120: @3 = input[2][2] */
  w3 = arg[2] ? arg[2][2] : 0;
  /* #121: @2 = input[2][3] */
  w2 = arg[2] ? arg[2][3] : 0;
  /* #122: @17 = vertcat(@1, @5, @3, @2) */
  rr=w17;
  *rr++ = w1;
  *rr++ = w5;
  *rr++ = w3;
  *rr++ = w2;
  /* #123: @39 = @17[1:4] */
  for (rr=w39, ss=w17+1; ss!=w17+4; ss+=1) *rr++ = *ss;
  /* #124: @34 = mac(@37,@39,@34) */
  casadi_mtimes(w37, casadi_s1, w39, casadi_s0, w34, casadi_s0, w, 0);
  /* #125: @33 = (@33+@34) */
  for (i=0, rr=w33, cs=w34; i<3; ++i) (*rr++) += (*cs++);
  /* #126: @17 = f_trans(@31) */
  arg1[0]=w31;
  res1[0]=w17;
  if (casadi_f4(arg1, res1, iw, w, 0)) return 1;
  /* #127: @5 = @17[3] */
  for (rr=(&w5), ss=w17+3; ss!=w17+4; ss+=1) *rr++ = *ss;
  /* #128: @3 = (@6*@5) */
  w3  = (w6*w5);
  /* #129: @2 = @17[2] */
  for (rr=(&w2), ss=w17+2; ss!=w17+3; ss+=1) *rr++ = *ss;
  /* #130: @10 = (@8*@2) */
  w10  = (w8*w2);
  /* #131: @3 = (@3-@10) */
  w3 -= w10;
  /* #132: @10 = @17[1] */
  for (rr=(&w10), ss=w17+1; ss!=w17+2; ss+=1) *rr++ = *ss;
  /* #133: @8 = (@8*@10) */
  w8 *= w10;
  /* #134: @5 = (@4*@5) */
  w5  = (w4*w5);
  /* #135: @8 = (@8-@5) */
  w8 -= w5;
  /* #136: @4 = (@4*@2) */
  w4 *= w2;
  /* #137: @6 = (@6*@10) */
  w6 *= w10;
  /* #138: @4 = (@4-@6) */
  w4 -= w6;
  /* #139: @34 = vertcat(@3, @8, @4) */
  rr=w34;
  *rr++ = w3;
  *rr++ = w8;
  *rr++ = w4;
  /* #140: @3 = @36[1] */
  for (rr=(&w3), ss=w36+1; ss!=w36+2; ss+=1) *rr++ = *ss;
  /* #141: @8 = @36[5] */
  for (rr=(&w8), ss=w36+5; ss!=w36+6; ss+=1) *rr++ = *ss;
  /* #142: @4 = (@3*@8) */
  w4  = (w3*w8);
  /* #143: @6 = @36[2] */
  for (rr=(&w6), ss=w36+2; ss!=w36+3; ss+=1) *rr++ = *ss;
  /* #144: @10 = @36[4] */
  for (rr=(&w10), ss=w36+4; ss!=w36+5; ss+=1) *rr++ = *ss;
  /* #145: @2 = (@6*@10) */
  w2  = (w6*w10);
  /* #146: @4 = (@4-@2) */
  w4 -= w2;
  /* #147: @2 = @36[3] */
  for (rr=(&w2), ss=w36+3; ss!=w36+4; ss+=1) *rr++ = *ss;
  /* #148: @6 = (@6*@2) */
  w6 *= w2;
  /* #149: @5 = @36[0] */
  for (rr=(&w5), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #150: @8 = (@5*@8) */
  w8  = (w5*w8);
  /* #151: @6 = (@6-@8) */
  w6 -= w8;
  /* #152: @5 = (@5*@10) */
  w5 *= w10;
  /* #153: @3 = (@3*@2) */
  w3 *= w2;
  /* #154: @5 = (@5-@3) */
  w5 -= w3;
  /* #155: @37 = vertcat(@4, @6, @5) */
  rr=w37;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w5;
  /* #156: @34 = (@34+@37) */
  for (i=0, rr=w34, cs=w37; i<3; ++i) (*rr++) += (*cs++);
  /* #157: @37 = [0, 0, 9.8] */
  casadi_copy(casadi_c3, 3, w37);
  /* #158: @34 = (@34-@37) */
  for (i=0, rr=w34, cs=w37; i<3; ++i) (*rr++) -= (*cs++);
  /* #159: @18 = f_quat(@31) */
  arg1[0]=w31;
  res1[0]=w18;
  if (casadi_f5(arg1, res1, iw, w, 0)) return 1;
  /* #160: @37 = [0, 0, 1] */
  casadi_copy(casadi_c4, 3, w37);
  /* #161: @40 = f_rot(@18, @37) */
  arg1[0]=w18;
  arg1[1]=w37;
  res1[0]=w40;
  if (casadi_f6(arg1, res1, iw, w, 0)) return 1;
  /* #162: @40 = (@1*@40) */
  for (i=0, rr=w40, cs=w40; i<3; ++i) (*rr++)  = (w1*(*cs++));
  /* #163: @37 = zeros(3x1) */
  casadi_clear(w37, 3);
  /* #164: @41 = 
  [[378.788, 00, 00], 
   [00, 378.788, 00], 
   [00, 00, 201.613]] */
  casadi_copy(casadi_c2, 3, w41);
  /* #165: @37 = mac(@41,@39,@37) */
  casadi_mtimes(w41, casadi_s1, w39, casadi_s0, w37, casadi_s0, w, 0);
  /* #166: @1 = @37[1] */
  for (rr=(&w1), ss=w37+1; ss!=w37+2; ss+=1) *rr++ = *ss;
  /* #167: @4 = @17[3] */
  for (rr=(&w4), ss=w17+3; ss!=w17+4; ss+=1) *rr++ = *ss;
  /* #168: @6 = (@1*@4) */
  w6  = (w1*w4);
  /* #169: @5 = @37[2] */
  for (rr=(&w5), ss=w37+2; ss!=w37+3; ss+=1) *rr++ = *ss;
  /* #170: @3 = @17[2] */
  for (rr=(&w3), ss=w17+2; ss!=w17+3; ss+=1) *rr++ = *ss;
  /* #171: @2 = (@5*@3) */
  w2  = (w5*w3);
  /* #172: @6 = (@6-@2) */
  w6 -= w2;
  /* #173: @2 = @17[1] */
  for (rr=(&w2), ss=w17+1; ss!=w17+2; ss+=1) *rr++ = *ss;
  /* #174: @5 = (@5*@2) */
  w5 *= w2;
  /* #175: @10 = @37[0] */
  for (rr=(&w10), ss=w37+0; ss!=w37+1; ss+=1) *rr++ = *ss;
  /* #176: @4 = (@10*@4) */
  w4  = (w10*w4);
  /* #177: @5 = (@5-@4) */
  w5 -= w4;
  /* #178: @10 = (@10*@3) */
  w10 *= w3;
  /* #179: @1 = (@1*@2) */
  w1 *= w2;
  /* #180: @10 = (@10-@1) */
  w10 -= w1;
  /* #181: @37 = vertcat(@6, @5, @10) */
  rr=w37;
  *rr++ = w6;
  *rr++ = w5;
  *rr++ = w10;
  /* #182: @40 = (@40+@37) */
  for (i=0, rr=w40, cs=w37; i<3; ++i) (*rr++) += (*cs++);
  /* #183: @34 = (@34+@40) */
  for (i=0, rr=w34, cs=w40; i<3; ++i) (*rr++) += (*cs++);
  /* #184: @32 = vertcat(@15, @33, @34) */
  rr=w32;
  for (i=0, cs=w15; i<8; ++i) *rr++ = *cs++;
  for (i=0, cs=w33; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w34; i<3; ++i) *rr++ = *cs++;
  /* #185: @14 = (@14-@32) */
  for (i=0, rr=w14, cs=w32; i<14; ++i) (*rr++) -= (*cs++);
  /* #186: output[0][0] = @14 */
  casadi_copy(w14, 14, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_impl_dae_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_impl_dae_fun_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_impl_dae_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_impl_dae_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_impl_dae_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_impl_dae_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_impl_dae_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s2;
    case 2: return casadi_s3;
    case 3: return casadi_s4;
    case 4: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_impl_dae_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_impl_dae_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 19;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 485;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
