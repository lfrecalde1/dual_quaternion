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
  #define CASADI_PREFIX(ID) quadrotor_ocp_cost_ext_cost_fun_jac_ ## ID
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
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[36] = {2., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00, 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00, 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00};
static const casadi_real casadi_c1[16] = {6.7114093959731547e-01, 0., 0., 0., 0., 200., 0., 0., 0., 0., 200., 0., 0., 0., 0., 200.};

/* quadrotor_ocp_cost_ext_cost_fun_jac:(i0[14],i1[4],i2[],i3[18])->(o0,o1[18]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, *w4=w+17, *w5=w+20, *w6=w+28, w7, w8, w9, w10, *w11=w+50, w12, w13, *w14=w+56, *w15=w+60, *w16=w+64, *w17=w+68, *w18=w+84, *w19=w+100, *w20=w+132, *w21=w+164, *w22=w+180, *w23=w+212, *w24=w+276, w25, w26, w27, w28, w29, w30, w31, w32, *w33=w+348, *w34=w+362, *w35=w+370, w36, w37, w38, w39, *w40=w+377, w41, *w42=w+382, *w43=w+388, *w44=w+394, w45, *w46=w+431;
  /* #0: @0 = 15 */
  w0 = 15.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x6) */
  casadi_clear(w2, 6);
  /* #3: @3 = 0.5 */
  w3 = 5.0000000000000000e-01;
  /* #4: @4 = all_2.22045e-16(3x1) */
  casadi_fill(w4, 3, 2.2204460492503131e-16);
  /* #5: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #6: @6 = input[3][0] */
  casadi_copy(arg[3], 18, w6);
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
  /* #62: @27 = input[0][8] */
  w27 = arg[0] ? arg[0][8] : 0;
  /* #63: @28 = input[0][9] */
  w28 = arg[0] ? arg[0][9] : 0;
  /* #64: @29 = input[0][10] */
  w29 = arg[0] ? arg[0][10] : 0;
  /* #65: @30 = input[0][11] */
  w30 = arg[0] ? arg[0][11] : 0;
  /* #66: @31 = input[0][12] */
  w31 = arg[0] ? arg[0][12] : 0;
  /* #67: @32 = input[0][13] */
  w32 = arg[0] ? arg[0][13] : 0;
  /* #68: @33 = vertcat(@13, @7, @10, @8, @12, @9, @25, @26, @27, @28, @29, @30, @31, @32) */
  rr=w33;
  *rr++ = w13;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w8;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w25;
  *rr++ = w26;
  *rr++ = w27;
  *rr++ = w28;
  *rr++ = w29;
  *rr++ = w30;
  *rr++ = w31;
  *rr++ = w32;
  /* #69: @34 = @33[:8] */
  for (rr=w34, ss=w33+0; ss!=w33+8; ss+=1) *rr++ = *ss;
  /* #70: @5 = mac(@24,@34,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w24+j, tt=w34+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #71: @35 = @5[1:4] */
  for (rr=w35, ss=w5+1; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #72: @4 = (@4+@35) */
  for (i=0, rr=w4, cs=w35; i<3; ++i) (*rr++) += (*cs++);
  /* #73: @13 = ||@4||_F */
  w13 = sqrt(casadi_dot(3, w4, w4));
  /* #74: @7 = @5[0] */
  for (rr=(&w7), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #75: @10 = atan2(@13,@7) */
  w10  = atan2(w13,w7);
  /* #76: @8 = (@3*@10) */
  w8  = (w3*w10);
  /* #77: @12 = @5[1] */
  for (rr=(&w12), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #78: @9 = (@8*@12) */
  w9  = (w8*w12);
  /* #79: @9 = (@9/@13) */
  w9 /= w13;
  /* #80: @25 = 0.5 */
  w25 = 5.0000000000000000e-01;
  /* #81: @26 = (@25*@10) */
  w26  = (w25*w10);
  /* #82: @27 = @5[2] */
  for (rr=(&w27), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #83: @28 = (@26*@27) */
  w28  = (w26*w27);
  /* #84: @28 = (@28/@13) */
  w28 /= w13;
  /* #85: @29 = 0.5 */
  w29 = 5.0000000000000000e-01;
  /* #86: @10 = (@29*@10) */
  w10  = (w29*w10);
  /* #87: @30 = @5[3] */
  for (rr=(&w30), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #88: @31 = (@10*@30) */
  w31  = (w10*w30);
  /* #89: @31 = (@31/@13) */
  w31 /= w13;
  /* #90: @32 = 0.5 */
  w32 = 5.0000000000000000e-01;
  /* #91: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #92: @36 = @5[4] */
  for (rr=(&w36), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #93: @37 = @5[5] */
  for (rr=(&w37), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #94: @37 = (-@37) */
  w37 = (- w37 );
  /* #95: @38 = @5[6] */
  for (rr=(&w38), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #96: @38 = (-@38) */
  w38 = (- w38 );
  /* #97: @39 = @5[7] */
  for (rr=(&w39), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #98: @39 = (-@39) */
  w39 = (- w39 );
  /* #99: @14 = horzcat(@36, @37, @38, @39) */
  rr=w14;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #100: @14 = @14' */
  /* #101: @36 = @5[5] */
  for (rr=(&w36), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #102: @37 = @5[4] */
  for (rr=(&w37), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #103: @38 = @5[7] */
  for (rr=(&w38), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #104: @38 = (-@38) */
  w38 = (- w38 );
  /* #105: @39 = @5[6] */
  for (rr=(&w39), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #106: @15 = horzcat(@36, @37, @38, @39) */
  rr=w15;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #107: @15 = @15' */
  /* #108: @36 = @5[6] */
  for (rr=(&w36), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #109: @37 = @5[7] */
  for (rr=(&w37), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #110: @38 = @5[4] */
  for (rr=(&w38), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #111: @39 = @5[5] */
  for (rr=(&w39), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #112: @39 = (-@39) */
  w39 = (- w39 );
  /* #113: @16 = horzcat(@36, @37, @38, @39) */
  rr=w16;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #114: @16 = @16' */
  /* #115: @36 = @5[7] */
  for (rr=(&w36), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #116: @37 = @5[6] */
  for (rr=(&w37), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #117: @37 = (-@37) */
  w37 = (- w37 );
  /* #118: @38 = @5[5] */
  for (rr=(&w38), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #119: @39 = @5[4] */
  for (rr=(&w39), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #120: @40 = horzcat(@36, @37, @38, @39) */
  rr=w40;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #121: @40 = @40' */
  /* #122: @21 = horzcat(@14, @15, @16, @40) */
  rr=w21;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w40; i<4; ++i) *rr++ = *cs++;
  /* #123: @18 = @21' */
  for (i=0, rr=w18, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #124: @18 = (2.*@18) */
  for (i=0, rr=w18, cs=w18; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #125: @36 = @5[0] */
  for (rr=(&w36), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #126: @37 = @5[1] */
  for (rr=(&w37), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #127: @37 = (-@37) */
  w37 = (- w37 );
  /* #128: @38 = @5[2] */
  for (rr=(&w38), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #129: @38 = (-@38) */
  w38 = (- w38 );
  /* #130: @39 = @5[3] */
  for (rr=(&w39), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #131: @39 = (-@39) */
  w39 = (- w39 );
  /* #132: @14 = vertcat(@36, @37, @38, @39) */
  rr=w14;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #133: @11 = mac(@18,@14,@11) */
  for (i=0, rr=w11; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w14+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #134: @36 = @11[1] */
  for (rr=(&w36), ss=w11+1; ss!=w11+2; ss+=1) *rr++ = *ss;
  /* #135: @36 = (@32*@36) */
  w36  = (w32*w36);
  /* #136: @37 = 0.5 */
  w37 = 5.0000000000000000e-01;
  /* #137: @38 = @11[2] */
  for (rr=(&w38), ss=w11+2; ss!=w11+3; ss+=1) *rr++ = *ss;
  /* #138: @38 = (@37*@38) */
  w38  = (w37*w38);
  /* #139: @39 = 0.5 */
  w39 = 5.0000000000000000e-01;
  /* #140: @41 = @11[3] */
  for (rr=(&w41), ss=w11+3; ss!=w11+4; ss+=1) *rr++ = *ss;
  /* #141: @41 = (@39*@41) */
  w41  = (w39*w41);
  /* #142: @42 = vertcat(@9, @28, @31, @36, @38, @41) */
  rr=w42;
  *rr++ = w9;
  *rr++ = w28;
  *rr++ = w31;
  *rr++ = w36;
  *rr++ = w38;
  *rr++ = w41;
  /* #143: @43 = @42' */
  casadi_copy(w42, 6, w43);
  /* #144: @44 = 
  [[2, 0, 0, 0, 0, 0], 
   [0, 2, 0, 0, 0, 0], 
   [0, 0, 2, 0, 0, 0], 
   [0, 0, 0, 1.6, 0, 0], 
   [0, 0, 0, 0, 1.6, 0], 
   [0, 0, 0, 0, 0, 1.6]] */
  casadi_copy(casadi_c0, 36, w44);
  /* #145: @2 = mac(@43,@44,@2) */
  for (i=0, rr=w2; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w43+j, tt=w44+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #146: @1 = mac(@2,@42,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w42+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #147: @0 = (@0*@1) */
  w0 *= w1;
  /* #148: @1 = 0 */
  w1 = 0.;
  /* #149: @11 = zeros(1x4) */
  casadi_clear(w11, 4);
  /* #150: @15 = @6[14:18] */
  for (rr=w15, ss=w6+14; ss!=w6+18; ss+=1) *rr++ = *ss;
  /* #151: @36 = input[1][0] */
  w36 = arg[1] ? arg[1][0] : 0;
  /* #152: @38 = input[1][1] */
  w38 = arg[1] ? arg[1][1] : 0;
  /* #153: @41 = input[1][2] */
  w41 = arg[1] ? arg[1][2] : 0;
  /* #154: @45 = input[1][3] */
  w45 = arg[1] ? arg[1][3] : 0;
  /* #155: @16 = vertcat(@36, @38, @41, @45) */
  rr=w16;
  *rr++ = w36;
  *rr++ = w38;
  *rr++ = w41;
  *rr++ = w45;
  /* #156: @15 = (@15-@16) */
  for (i=0, rr=w15, cs=w16; i<4; ++i) (*rr++) -= (*cs++);
  /* #157: @16 = @15' */
  casadi_copy(w15, 4, w16);
  /* #158: @21 = 
  [[0.671141, 0, 0, 0], 
   [0, 200, 0, 0], 
   [0, 0, 200, 0], 
   [0, 0, 0, 200]] */
  casadi_copy(casadi_c1, 16, w21);
  /* #159: @11 = mac(@16,@21,@11) */
  for (i=0, rr=w11; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w16+j, tt=w21+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #160: @1 = mac(@11,@15,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w11+j, tt=w15+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #161: @0 = (@0+@1) */
  w0 += w1;
  /* #162: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #163: @11 = @11' */
  /* #164: @16 = zeros(1x4) */
  casadi_clear(w16, 4);
  /* #165: @15 = @15' */
  /* #166: @17 = @21' */
  for (i=0, rr=w17, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #167: @16 = mac(@15,@17,@16) */
  for (i=0, rr=w16; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w15+j, tt=w17+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #168: @16 = @16' */
  /* #169: @11 = (@11+@16) */
  for (i=0, rr=w11, cs=w16; i<4; ++i) (*rr++) += (*cs++);
  /* #170: @11 = (-@11) */
  for (i=0, rr=w11, cs=w11; i<4; ++i) *rr++ = (- *cs++ );
  /* #171: {@0, @1, @36, @38} = vertsplit(@11) */
  w0 = w11[0];
  w1 = w11[1];
  w36 = w11[2];
  w38 = w11[3];
  /* #172: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #173: output[1][1] = @1 */
  if (res[1]) res[1][1] = w1;
  /* #174: output[1][2] = @36 */
  if (res[1]) res[1][2] = w36;
  /* #175: output[1][3] = @38 */
  if (res[1]) res[1][3] = w38;
  /* #176: @33 = zeros(14x1) */
  casadi_clear(w33, 14);
  /* #177: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #178: @34 = zeros(8x1) */
  casadi_clear(w34, 8);
  /* #179: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #180: @17 = @18' */
  for (i=0, rr=w17, cs=w18; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #181: @16 = zeros(4x1) */
  casadi_clear(w16, 4);
  /* #182: @38 = 15 */
  w38 = 15.;
  /* #183: @2 = @2' */
  /* #184: @2 = (@38*@2) */
  for (i=0, rr=w2, cs=w2; i<6; ++i) (*rr++)  = (w38*(*cs++));
  /* #185: @43 = zeros(1x6) */
  casadi_clear(w43, 6);
  /* #186: @42 = @42' */
  /* #187: @42 = (@38*@42) */
  for (i=0, rr=w42, cs=w42; i<6; ++i) (*rr++)  = (w38*(*cs++));
  /* #188: @46 = @44' */
  for (i=0, rr=w46, cs=w44; i<6; ++i) for (j=0; j<6; ++j) rr[i+j*6] = *cs++;
  /* #189: @43 = mac(@42,@46,@43) */
  for (i=0, rr=w43; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w42+j, tt=w46+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #190: @43 = @43' */
  /* #191: @2 = (@2+@43) */
  for (i=0, rr=w2, cs=w43; i<6; ++i) (*rr++) += (*cs++);
  /* #192: {@38, @36, @1, @0, @41, @45} = vertsplit(@2) */
  w38 = w2[0];
  w36 = w2[1];
  w1 = w2[2];
  w0 = w2[3];
  w41 = w2[4];
  w45 = w2[5];
  /* #193: @39 = (@39*@45) */
  w39 *= w45;
  /* #194: (@16[3] += @39) */
  for (rr=w16+3, ss=(&w39); rr!=w16+4; rr+=1) *rr += *ss++;
  /* #195: @37 = (@37*@41) */
  w37 *= w41;
  /* #196: (@16[2] += @37) */
  for (rr=w16+2, ss=(&w37); rr!=w16+3; rr+=1) *rr += *ss++;
  /* #197: @32 = (@32*@0) */
  w32 *= w0;
  /* #198: (@16[1] += @32) */
  for (rr=w16+1, ss=(&w32); rr!=w16+2; rr+=1) *rr += *ss++;
  /* #199: @11 = mac(@17,@16,@11) */
  for (i=0, rr=w11; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w17+j, tt=w16+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #200: {@32, @0, @37, @41} = vertsplit(@11) */
  w32 = w11[0];
  w0 = w11[1];
  w37 = w11[2];
  w41 = w11[3];
  /* #201: @41 = (-@41) */
  w41 = (- w41 );
  /* #202: (@34[3] += @41) */
  for (rr=w34+3, ss=(&w41); rr!=w34+4; rr+=1) *rr += *ss++;
  /* #203: @37 = (-@37) */
  w37 = (- w37 );
  /* #204: (@34[2] += @37) */
  for (rr=w34+2, ss=(&w37); rr!=w34+3; rr+=1) *rr += *ss++;
  /* #205: @0 = (-@0) */
  w0 = (- w0 );
  /* #206: (@34[1] += @0) */
  for (rr=w34+1, ss=(&w0); rr!=w34+2; rr+=1) *rr += *ss++;
  /* #207: (@34[0] += @32) */
  for (rr=w34+0, ss=(&w32); rr!=w34+1; rr+=1) *rr += *ss++;
  /* #208: @17 = zeros(4x4) */
  casadi_clear(w17, 16);
  /* #209: @14 = @14' */
  /* #210: @17 = mac(@16,@14,@17) */
  for (i=0, rr=w17; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w16+j, tt=w14+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #211: @17 = (2.*@17) */
  for (i=0, rr=w17, cs=w17; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #212: @18 = @17' */
  for (i=0, rr=w18, cs=w17; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #213: {@16, @14, @11, @15} = horzsplit(@18) */
  casadi_copy(w18, 4, w16);
  casadi_copy(w18+4, 4, w14);
  casadi_copy(w18+8, 4, w11);
  casadi_copy(w18+12, 4, w15);
  /* #214: @15 = @15' */
  /* #215: {@32, @0, @37, @41} = horzsplit(@15) */
  w32 = w15[0];
  w0 = w15[1];
  w37 = w15[2];
  w41 = w15[3];
  /* #216: (@34[4] += @41) */
  for (rr=w34+4, ss=(&w41); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #217: (@34[5] += @37) */
  for (rr=w34+5, ss=(&w37); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #218: @0 = (-@0) */
  w0 = (- w0 );
  /* #219: (@34[6] += @0) */
  for (rr=w34+6, ss=(&w0); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #220: (@34[7] += @32) */
  for (rr=w34+7, ss=(&w32); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #221: @11 = @11' */
  /* #222: {@32, @0, @37, @41} = horzsplit(@11) */
  w32 = w11[0];
  w0 = w11[1];
  w37 = w11[2];
  w41 = w11[3];
  /* #223: @41 = (-@41) */
  w41 = (- w41 );
  /* #224: (@34[5] += @41) */
  for (rr=w34+5, ss=(&w41); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #225: (@34[4] += @37) */
  for (rr=w34+4, ss=(&w37); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #226: (@34[7] += @0) */
  for (rr=w34+7, ss=(&w0); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #227: (@34[6] += @32) */
  for (rr=w34+6, ss=(&w32); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #228: @14 = @14' */
  /* #229: {@32, @0, @37, @41} = horzsplit(@14) */
  w32 = w14[0];
  w0 = w14[1];
  w37 = w14[2];
  w41 = w14[3];
  /* #230: (@34[6] += @41) */
  for (rr=w34+6, ss=(&w41); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #231: @37 = (-@37) */
  w37 = (- w37 );
  /* #232: (@34[7] += @37) */
  for (rr=w34+7, ss=(&w37); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #233: (@34[4] += @0) */
  for (rr=w34+4, ss=(&w0); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #234: (@34[5] += @32) */
  for (rr=w34+5, ss=(&w32); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #235: @16 = @16' */
  /* #236: {@32, @0, @37, @41} = horzsplit(@16) */
  w32 = w16[0];
  w0 = w16[1];
  w37 = w16[2];
  w41 = w16[3];
  /* #237: @41 = (-@41) */
  w41 = (- w41 );
  /* #238: (@34[7] += @41) */
  for (rr=w34+7, ss=(&w41); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #239: @37 = (-@37) */
  w37 = (- w37 );
  /* #240: (@34[6] += @37) */
  for (rr=w34+6, ss=(&w37); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #241: @0 = (-@0) */
  w0 = (- w0 );
  /* #242: (@34[5] += @0) */
  for (rr=w34+5, ss=(&w0); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #243: (@34[4] += @32) */
  for (rr=w34+4, ss=(&w32); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #244: @32 = (@1/@13) */
  w32  = (w1/w13);
  /* #245: @10 = (@10*@32) */
  w10 *= w32;
  /* #246: (@34[3] += @10) */
  for (rr=w34+3, ss=(&w10); rr!=w34+4; rr+=1) *rr += *ss++;
  /* #247: @10 = (@36/@13) */
  w10  = (w36/w13);
  /* #248: @26 = (@26*@10) */
  w26 *= w10;
  /* #249: (@34[2] += @26) */
  for (rr=w34+2, ss=(&w26); rr!=w34+3; rr+=1) *rr += *ss++;
  /* #250: @26 = (@38/@13) */
  w26  = (w38/w13);
  /* #251: @8 = (@8*@26) */
  w8 *= w26;
  /* #252: (@34[1] += @8) */
  for (rr=w34+1, ss=(&w8); rr!=w34+2; rr+=1) *rr += *ss++;
  /* #253: @8 = sq(@13) */
  w8 = casadi_sq( w13 );
  /* #254: @0 = sq(@7) */
  w0 = casadi_sq( w7 );
  /* #255: @8 = (@8+@0) */
  w8 += w0;
  /* #256: @0 = (@13/@8) */
  w0  = (w13/w8);
  /* #257: @30 = (@30*@32) */
  w30 *= w32;
  /* #258: @29 = (@29*@30) */
  w29 *= w30;
  /* #259: @27 = (@27*@10) */
  w27 *= w10;
  /* #260: @25 = (@25*@27) */
  w25 *= w27;
  /* #261: @29 = (@29+@25) */
  w29 += w25;
  /* #262: @12 = (@12*@26) */
  w12 *= w26;
  /* #263: @3 = (@3*@12) */
  w3 *= w12;
  /* #264: @29 = (@29+@3) */
  w29 += w3;
  /* #265: @0 = (@0*@29) */
  w0 *= w29;
  /* #266: @0 = (-@0) */
  w0 = (- w0 );
  /* #267: (@34[0] += @0) */
  for (rr=w34+0, ss=(&w0); rr!=w34+1; rr+=1) *rr += *ss++;
  /* #268: @28 = (@28/@13) */
  w28 /= w13;
  /* #269: @28 = (@28*@36) */
  w28 *= w36;
  /* #270: @28 = (-@28) */
  w28 = (- w28 );
  /* #271: @31 = (@31/@13) */
  w31 /= w13;
  /* #272: @31 = (@31*@1) */
  w31 *= w1;
  /* #273: @28 = (@28-@31) */
  w28 -= w31;
  /* #274: @9 = (@9/@13) */
  w9 /= w13;
  /* #275: @9 = (@9*@38) */
  w9 *= w38;
  /* #276: @28 = (@28-@9) */
  w28 -= w9;
  /* #277: @7 = (@7/@8) */
  w7 /= w8;
  /* #278: @7 = (@7*@29) */
  w7 *= w29;
  /* #279: @28 = (@28+@7) */
  w28 += w7;
  /* #280: @28 = (@28/@13) */
  w28 /= w13;
  /* #281: @4 = (@28*@4) */
  for (i=0, rr=w4, cs=w4; i<3; ++i) (*rr++)  = (w28*(*cs++));
  /* #282: (@34[1:4] += @4) */
  for (rr=w34+1, ss=w4; rr!=w34+4; rr+=1) *rr += *ss++;
  /* #283: @5 = mac(@23,@34,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w34+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #284: (@33[:8] += @5) */
  for (rr=w33+0, ss=w5; rr!=w33+8; rr+=1) *rr += *ss++;
  /* #285: {@28, @13, @7, @29, @8, @9, @38, @31, @1, @36, @0, @3, @12, @26} = vertsplit(@33) */
  w28 = w33[0];
  w13 = w33[1];
  w7 = w33[2];
  w29 = w33[3];
  w8 = w33[4];
  w9 = w33[5];
  w38 = w33[6];
  w31 = w33[7];
  w1 = w33[8];
  w36 = w33[9];
  w0 = w33[10];
  w3 = w33[11];
  w12 = w33[12];
  w26 = w33[13];
  /* #286: output[1][4] = @28 */
  if (res[1]) res[1][4] = w28;
  /* #287: output[1][5] = @13 */
  if (res[1]) res[1][5] = w13;
  /* #288: output[1][6] = @7 */
  if (res[1]) res[1][6] = w7;
  /* #289: output[1][7] = @29 */
  if (res[1]) res[1][7] = w29;
  /* #290: output[1][8] = @8 */
  if (res[1]) res[1][8] = w8;
  /* #291: output[1][9] = @9 */
  if (res[1]) res[1][9] = w9;
  /* #292: output[1][10] = @38 */
  if (res[1]) res[1][10] = w38;
  /* #293: output[1][11] = @31 */
  if (res[1]) res[1][11] = w31;
  /* #294: output[1][12] = @1 */
  if (res[1]) res[1][12] = w1;
  /* #295: output[1][13] = @36 */
  if (res[1]) res[1][13] = w36;
  /* #296: output[1][14] = @0 */
  if (res[1]) res[1][14] = w0;
  /* #297: output[1][15] = @3 */
  if (res[1]) res[1][15] = w3;
  /* #298: output[1][16] = @12 */
  if (res[1]) res[1][16] = w12;
  /* #299: output[1][17] = @26 */
  if (res[1]) res[1][17] = w26;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_ocp_cost_ext_cost_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_ocp_cost_ext_cost_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_ocp_cost_ext_cost_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_ocp_cost_ext_cost_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_ocp_cost_ext_cost_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_ocp_cost_ext_cost_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 16;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 467;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
