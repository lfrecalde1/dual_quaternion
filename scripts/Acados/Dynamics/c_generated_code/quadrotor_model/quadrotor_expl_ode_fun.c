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
static const casadi_int casadi_s4[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

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

/* quadrotor_expl_ode_fun:(i0[14],i1[4],i2[18])->(o0[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+1, *rr, *ss, *tt;
  const casadi_real **arg1=arg+3, *cr, *cs;
  casadi_real w0, *w1=w+98, w2, w3, w4, w5, w6, w7, w8, *w9=w+113, *w10=w+117, *w11=w+121, *w12=w+125, *w13=w+129, *w14=w+145, *w15=w+161, *w16=w+193, w17, w18, w19, w20, *w21=w+229, *w22=w+245, *w23=w+277, *w24=w+341, w25, w26, w27, w28, w29, *w30=w+410, *w31=w+418, *w32=w+432, *w33=w+435, *w34=w+438, *w35=w+444, *w36=w+450, *w37=w+453, *w38=w+462, *w39=w+465;
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
  /* #50: @18 = input[0][8] */
  w18 = arg[0] ? arg[0][8] : 0;
  /* #51: @20 = input[0][9] */
  w20 = arg[0] ? arg[0][9] : 0;
  /* #52: @25 = input[0][10] */
  w25 = arg[0] ? arg[0][10] : 0;
  /* #53: @26 = 0 */
  w26 = 0.;
  /* #54: @27 = input[0][11] */
  w27 = arg[0] ? arg[0][11] : 0;
  /* #55: @28 = input[0][12] */
  w28 = arg[0] ? arg[0][12] : 0;
  /* #56: @29 = input[0][13] */
  w29 = arg[0] ? arg[0][13] : 0;
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
  /* #60: @31 = vertcat(@2, @3, @5, @7, @4, @6, @17, @19, @18, @20, @25, @27, @28, @29) */
  rr=w31;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w7;
  *rr++ = w4;
  *rr++ = w6;
  *rr++ = w17;
  *rr++ = w19;
  *rr++ = w18;
  *rr++ = w20;
  *rr++ = w25;
  *rr++ = w27;
  *rr++ = w28;
  *rr++ = w29;
  /* #61: @9 = @31[:4] */
  for (rr=w9, ss=w31+0; ss!=w31+4; ss+=1) *rr++ = *ss;
  /* #62: @2 = 1 */
  w2 = 1.;
  /* #63: @3 = dot(@9, @9) */
  w3 = casadi_dot(4, w9, w9);
  /* #64: @2 = (@2-@3) */
  w2 -= w3;
  /* #65: @2 = (2.*@2) */
  w2 = (2.* w2 );
  /* #66: @10 = (@9*@2) */
  for (i=0, rr=w10, cr=w9; i<4; ++i) (*rr++)  = ((*cr++)*w2);
  /* #67: @11 = @31[4:8] */
  for (rr=w11, ss=w31+4; ss!=w31+8; ss+=1) *rr++ = *ss;
  /* #68: @2 = 0 */
  w2 = 0.;
  /* #69: @9 = @9' */
  /* #70: @2 = mac(@9,@11,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w9+j, tt=w11+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #71: @2 = (2.*@2) */
  w2 = (2.* w2 );
  /* #72: @11 = (@11*@2) */
  for (i=0, rr=w11; i<4; ++i) (*rr++) *= w2;
  /* #73: @30 = vertcat(@10, @11) */
  rr=w30;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  /* #74: @1 = (@1+@30) */
  for (i=0, rr=w1, cs=w30; i<8; ++i) (*rr++) += (*cs++);
  /* #75: output[0][0] = @1 */
  casadi_copy(w1, 8, res[0]);
  /* #76: @32 = zeros(3x1) */
  casadi_clear(w32, 3);
  /* #77: @33 = 
  [[-378.788, 00, 00], 
   [00, -378.788, 00], 
   [00, 00, -201.613]] */
  casadi_copy(casadi_c0, 3, w33);
  /* #78: @34 = @31[8:14] */
  for (rr=w34, ss=w31+8; ss!=w31+14; ss+=1) *rr++ = *ss;
  /* #79: @1 = @31[:8] */
  for (rr=w1, ss=w31+0; ss!=w31+8; ss+=1) *rr++ = *ss;
  /* #80: @35 = f_velocity(@34, @1) */
  arg1[0]=w34;
  arg1[1]=w1;
  res1[0]=w35;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #81: @36 = @35[:3] */
  for (rr=w36, ss=w35+0; ss!=w35+3; ss+=1) *rr++ = *ss;
  /* #82: @32 = mac(@33,@36,@32) */
  casadi_mtimes(w33, casadi_s1, w36, casadi_s0, w32, casadi_s0, w, 0);
  /* #83: @2 = @32[1] */
  for (rr=(&w2), ss=w32+1; ss!=w32+2; ss+=1) *rr++ = *ss;
  /* #84: @33 = zeros(3x1) */
  casadi_clear(w33, 3);
  /* #85: @37 = 
  [[0.00264, 0, 0], 
   [0, 0.00264, 0], 
   [0, 0, 0.00496]] */
  casadi_copy(casadi_c1, 9, w37);
  /* #86: @33 = mac(@37,@36,@33) */
  for (i=0, rr=w33; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w37+j, tt=w36+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #87: @3 = @33[2] */
  for (rr=(&w3), ss=w33+2; ss!=w33+3; ss+=1) *rr++ = *ss;
  /* #88: @5 = (@2*@3) */
  w5  = (w2*w3);
  /* #89: @7 = @32[2] */
  for (rr=(&w7), ss=w32+2; ss!=w32+3; ss+=1) *rr++ = *ss;
  /* #90: @4 = @33[1] */
  for (rr=(&w4), ss=w33+1; ss!=w33+2; ss+=1) *rr++ = *ss;
  /* #91: @6 = (@7*@4) */
  w6  = (w7*w4);
  /* #92: @5 = (@5-@6) */
  w5 -= w6;
  /* #93: @6 = @33[0] */
  for (rr=(&w6), ss=w33+0; ss!=w33+1; ss+=1) *rr++ = *ss;
  /* #94: @7 = (@7*@6) */
  w7 *= w6;
  /* #95: @17 = @32[0] */
  for (rr=(&w17), ss=w32+0; ss!=w32+1; ss+=1) *rr++ = *ss;
  /* #96: @3 = (@17*@3) */
  w3  = (w17*w3);
  /* #97: @7 = (@7-@3) */
  w7 -= w3;
  /* #98: @17 = (@17*@4) */
  w17 *= w4;
  /* #99: @2 = (@2*@6) */
  w2 *= w6;
  /* #100: @17 = (@17-@2) */
  w17 -= w2;
  /* #101: @32 = vertcat(@5, @7, @17) */
  rr=w32;
  *rr++ = w5;
  *rr++ = w7;
  *rr++ = w17;
  /* #102: @33 = zeros(3x1) */
  casadi_clear(w33, 3);
  /* #103: @36 = 
  [[378.788, 00, 00], 
   [00, 378.788, 00], 
   [00, 00, 201.613]] */
  casadi_copy(casadi_c2, 3, w36);
  /* #104: @2 = input[1][0] */
  w2 = arg[1] ? arg[1][0] : 0;
  /* #105: @6 = input[1][1] */
  w6 = arg[1] ? arg[1][1] : 0;
  /* #106: @4 = input[1][2] */
  w4 = arg[1] ? arg[1][2] : 0;
  /* #107: @3 = input[1][3] */
  w3 = arg[1] ? arg[1][3] : 0;
  /* #108: @10 = vertcat(@2, @6, @4, @3) */
  rr=w10;
  *rr++ = w2;
  *rr++ = w6;
  *rr++ = w4;
  *rr++ = w3;
  /* #109: @38 = @10[1:4] */
  for (rr=w38, ss=w10+1; ss!=w10+4; ss+=1) *rr++ = *ss;
  /* #110: @33 = mac(@36,@38,@33) */
  casadi_mtimes(w36, casadi_s1, w38, casadi_s0, w33, casadi_s0, w, 0);
  /* #111: @32 = (@32+@33) */
  for (i=0, rr=w32, cs=w33; i<3; ++i) (*rr++) += (*cs++);
  /* #112: output[0][1] = @32 */
  if (res[0]) casadi_copy(w32, 3, res[0]+8);
  /* #113: @10 = f_trans(@1) */
  arg1[0]=w1;
  res1[0]=w10;
  if (casadi_f4(arg1, res1, iw, w, 0)) return 1;
  /* #114: @6 = @10[3] */
  for (rr=(&w6), ss=w10+3; ss!=w10+4; ss+=1) *rr++ = *ss;
  /* #115: @4 = (@7*@6) */
  w4  = (w7*w6);
  /* #116: @3 = @10[2] */
  for (rr=(&w3), ss=w10+2; ss!=w10+3; ss+=1) *rr++ = *ss;
  /* #117: @19 = (@17*@3) */
  w19  = (w17*w3);
  /* #118: @4 = (@4-@19) */
  w4 -= w19;
  /* #119: @19 = @10[1] */
  for (rr=(&w19), ss=w10+1; ss!=w10+2; ss+=1) *rr++ = *ss;
  /* #120: @17 = (@17*@19) */
  w17 *= w19;
  /* #121: @6 = (@5*@6) */
  w6  = (w5*w6);
  /* #122: @17 = (@17-@6) */
  w17 -= w6;
  /* #123: @5 = (@5*@3) */
  w5 *= w3;
  /* #124: @7 = (@7*@19) */
  w7 *= w19;
  /* #125: @5 = (@5-@7) */
  w5 -= w7;
  /* #126: @32 = vertcat(@4, @17, @5) */
  rr=w32;
  *rr++ = w4;
  *rr++ = w17;
  *rr++ = w5;
  /* #127: @4 = @35[1] */
  for (rr=(&w4), ss=w35+1; ss!=w35+2; ss+=1) *rr++ = *ss;
  /* #128: @17 = @35[5] */
  for (rr=(&w17), ss=w35+5; ss!=w35+6; ss+=1) *rr++ = *ss;
  /* #129: @5 = (@4*@17) */
  w5  = (w4*w17);
  /* #130: @7 = @35[2] */
  for (rr=(&w7), ss=w35+2; ss!=w35+3; ss+=1) *rr++ = *ss;
  /* #131: @19 = @35[4] */
  for (rr=(&w19), ss=w35+4; ss!=w35+5; ss+=1) *rr++ = *ss;
  /* #132: @3 = (@7*@19) */
  w3  = (w7*w19);
  /* #133: @5 = (@5-@3) */
  w5 -= w3;
  /* #134: @3 = @35[3] */
  for (rr=(&w3), ss=w35+3; ss!=w35+4; ss+=1) *rr++ = *ss;
  /* #135: @7 = (@7*@3) */
  w7 *= w3;
  /* #136: @6 = @35[0] */
  for (rr=(&w6), ss=w35+0; ss!=w35+1; ss+=1) *rr++ = *ss;
  /* #137: @17 = (@6*@17) */
  w17  = (w6*w17);
  /* #138: @7 = (@7-@17) */
  w7 -= w17;
  /* #139: @6 = (@6*@19) */
  w6 *= w19;
  /* #140: @4 = (@4*@3) */
  w4 *= w3;
  /* #141: @6 = (@6-@4) */
  w6 -= w4;
  /* #142: @33 = vertcat(@5, @7, @6) */
  rr=w33;
  *rr++ = w5;
  *rr++ = w7;
  *rr++ = w6;
  /* #143: @32 = (@32+@33) */
  for (i=0, rr=w32, cs=w33; i<3; ++i) (*rr++) += (*cs++);
  /* #144: @33 = [0, 0, 9.8] */
  casadi_copy(casadi_c3, 3, w33);
  /* #145: @32 = (@32-@33) */
  for (i=0, rr=w32, cs=w33; i<3; ++i) (*rr++) -= (*cs++);
  /* #146: @11 = f_quat(@1) */
  arg1[0]=w1;
  res1[0]=w11;
  if (casadi_f5(arg1, res1, iw, w, 0)) return 1;
  /* #147: @33 = [0, 0, 1] */
  casadi_copy(casadi_c4, 3, w33);
  /* #148: @36 = f_rot(@11, @33) */
  arg1[0]=w11;
  arg1[1]=w33;
  res1[0]=w36;
  if (casadi_f6(arg1, res1, iw, w, 0)) return 1;
  /* #149: @36 = (@2*@36) */
  for (i=0, rr=w36, cs=w36; i<3; ++i) (*rr++)  = (w2*(*cs++));
  /* #150: @33 = zeros(3x1) */
  casadi_clear(w33, 3);
  /* #151: @39 = 
  [[378.788, 00, 00], 
   [00, 378.788, 00], 
   [00, 00, 201.613]] */
  casadi_copy(casadi_c2, 3, w39);
  /* #152: @33 = mac(@39,@38,@33) */
  casadi_mtimes(w39, casadi_s1, w38, casadi_s0, w33, casadi_s0, w, 0);
  /* #153: @2 = @33[1] */
  for (rr=(&w2), ss=w33+1; ss!=w33+2; ss+=1) *rr++ = *ss;
  /* #154: @5 = @10[3] */
  for (rr=(&w5), ss=w10+3; ss!=w10+4; ss+=1) *rr++ = *ss;
  /* #155: @7 = (@2*@5) */
  w7  = (w2*w5);
  /* #156: @6 = @33[2] */
  for (rr=(&w6), ss=w33+2; ss!=w33+3; ss+=1) *rr++ = *ss;
  /* #157: @4 = @10[2] */
  for (rr=(&w4), ss=w10+2; ss!=w10+3; ss+=1) *rr++ = *ss;
  /* #158: @3 = (@6*@4) */
  w3  = (w6*w4);
  /* #159: @7 = (@7-@3) */
  w7 -= w3;
  /* #160: @3 = @10[1] */
  for (rr=(&w3), ss=w10+1; ss!=w10+2; ss+=1) *rr++ = *ss;
  /* #161: @6 = (@6*@3) */
  w6 *= w3;
  /* #162: @19 = @33[0] */
  for (rr=(&w19), ss=w33+0; ss!=w33+1; ss+=1) *rr++ = *ss;
  /* #163: @5 = (@19*@5) */
  w5  = (w19*w5);
  /* #164: @6 = (@6-@5) */
  w6 -= w5;
  /* #165: @19 = (@19*@4) */
  w19 *= w4;
  /* #166: @2 = (@2*@3) */
  w2 *= w3;
  /* #167: @19 = (@19-@2) */
  w19 -= w2;
  /* #168: @33 = vertcat(@7, @6, @19) */
  rr=w33;
  *rr++ = w7;
  *rr++ = w6;
  *rr++ = w19;
  /* #169: @36 = (@36+@33) */
  for (i=0, rr=w36, cs=w33; i<3; ++i) (*rr++) += (*cs++);
  /* #170: @32 = (@32+@36) */
  for (i=0, rr=w32, cs=w36; i<3; ++i) (*rr++) += (*cs++);
  /* #171: output[0][2] = @32 */
  if (res[0]) casadi_copy(w32, 3, res[0]+11);
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
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 468;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
