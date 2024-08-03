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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_e_fun_jac_ ## ID
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
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_e_fun_jac:(i0[14],i1[],i2[],i3[18])->(o0,o1[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, *w4=w+17, *w5=w+20, *w6=w+28, w7, w8, w9, w10, *w11=w+50, w12, w13, *w14=w+56, *w15=w+60, *w16=w+64, *w17=w+68, *w18=w+84, *w19=w+100, *w20=w+132, *w21=w+164, *w22=w+180, *w23=w+212, *w24=w+276, w25, w26, w27, w28, w29, w30, w31, w32, *w33=w+348, *w34=w+362, *w35=w+370, w36, w37, w38, w39, *w40=w+377, w41, *w42=w+382, *w43=w+388, *w44=w+394, *w45=w+430, w46;
  /* #0: @0 = 10 */
  w0 = 10.;
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
  /* #76: @10 = (2.*@10) */
  w10 = (2.* w10 );
  /* #77: @8 = (@3*@10) */
  w8  = (w3*w10);
  /* #78: @12 = @5[1] */
  for (rr=(&w12), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #79: @9 = (@8*@12) */
  w9  = (w8*w12);
  /* #80: @9 = (@9/@13) */
  w9 /= w13;
  /* #81: @25 = 0.5 */
  w25 = 5.0000000000000000e-01;
  /* #82: @26 = (@25*@10) */
  w26  = (w25*w10);
  /* #83: @27 = @5[2] */
  for (rr=(&w27), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #84: @28 = (@26*@27) */
  w28  = (w26*w27);
  /* #85: @28 = (@28/@13) */
  w28 /= w13;
  /* #86: @29 = 0.5 */
  w29 = 5.0000000000000000e-01;
  /* #87: @10 = (@29*@10) */
  w10  = (w29*w10);
  /* #88: @30 = @5[3] */
  for (rr=(&w30), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #89: @31 = (@10*@30) */
  w31  = (w10*w30);
  /* #90: @31 = (@31/@13) */
  w31 /= w13;
  /* #91: @32 = 0.5 */
  w32 = 5.0000000000000000e-01;
  /* #92: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #93: @36 = @5[4] */
  for (rr=(&w36), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #94: @37 = @5[5] */
  for (rr=(&w37), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #95: @37 = (-@37) */
  w37 = (- w37 );
  /* #96: @38 = @5[6] */
  for (rr=(&w38), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #97: @38 = (-@38) */
  w38 = (- w38 );
  /* #98: @39 = @5[7] */
  for (rr=(&w39), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #99: @39 = (-@39) */
  w39 = (- w39 );
  /* #100: @14 = horzcat(@36, @37, @38, @39) */
  rr=w14;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #101: @14 = @14' */
  /* #102: @36 = @5[5] */
  for (rr=(&w36), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #103: @37 = @5[4] */
  for (rr=(&w37), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #104: @38 = @5[7] */
  for (rr=(&w38), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #105: @38 = (-@38) */
  w38 = (- w38 );
  /* #106: @39 = @5[6] */
  for (rr=(&w39), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #107: @15 = horzcat(@36, @37, @38, @39) */
  rr=w15;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #108: @15 = @15' */
  /* #109: @36 = @5[6] */
  for (rr=(&w36), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #110: @37 = @5[7] */
  for (rr=(&w37), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #111: @38 = @5[4] */
  for (rr=(&w38), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #112: @39 = @5[5] */
  for (rr=(&w39), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #113: @39 = (-@39) */
  w39 = (- w39 );
  /* #114: @16 = horzcat(@36, @37, @38, @39) */
  rr=w16;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #115: @16 = @16' */
  /* #116: @36 = @5[7] */
  for (rr=(&w36), ss=w5+7; ss!=w5+8; ss+=1) *rr++ = *ss;
  /* #117: @37 = @5[6] */
  for (rr=(&w37), ss=w5+6; ss!=w5+7; ss+=1) *rr++ = *ss;
  /* #118: @37 = (-@37) */
  w37 = (- w37 );
  /* #119: @38 = @5[5] */
  for (rr=(&w38), ss=w5+5; ss!=w5+6; ss+=1) *rr++ = *ss;
  /* #120: @39 = @5[4] */
  for (rr=(&w39), ss=w5+4; ss!=w5+5; ss+=1) *rr++ = *ss;
  /* #121: @40 = horzcat(@36, @37, @38, @39) */
  rr=w40;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #122: @40 = @40' */
  /* #123: @21 = horzcat(@14, @15, @16, @40) */
  rr=w21;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w15; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w40; i<4; ++i) *rr++ = *cs++;
  /* #124: @18 = @21' */
  for (i=0, rr=w18, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #125: @18 = (2.*@18) */
  for (i=0, rr=w18, cs=w18; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #126: @36 = @5[0] */
  for (rr=(&w36), ss=w5+0; ss!=w5+1; ss+=1) *rr++ = *ss;
  /* #127: @37 = @5[1] */
  for (rr=(&w37), ss=w5+1; ss!=w5+2; ss+=1) *rr++ = *ss;
  /* #128: @37 = (-@37) */
  w37 = (- w37 );
  /* #129: @38 = @5[2] */
  for (rr=(&w38), ss=w5+2; ss!=w5+3; ss+=1) *rr++ = *ss;
  /* #130: @38 = (-@38) */
  w38 = (- w38 );
  /* #131: @39 = @5[3] */
  for (rr=(&w39), ss=w5+3; ss!=w5+4; ss+=1) *rr++ = *ss;
  /* #132: @39 = (-@39) */
  w39 = (- w39 );
  /* #133: @14 = vertcat(@36, @37, @38, @39) */
  rr=w14;
  *rr++ = w36;
  *rr++ = w37;
  *rr++ = w38;
  *rr++ = w39;
  /* #134: @11 = mac(@18,@14,@11) */
  for (i=0, rr=w11; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w14+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #135: @36 = @11[1] */
  for (rr=(&w36), ss=w11+1; ss!=w11+2; ss+=1) *rr++ = *ss;
  /* #136: @36 = (@32*@36) */
  w36  = (w32*w36);
  /* #137: @37 = 0.5 */
  w37 = 5.0000000000000000e-01;
  /* #138: @38 = @11[2] */
  for (rr=(&w38), ss=w11+2; ss!=w11+3; ss+=1) *rr++ = *ss;
  /* #139: @38 = (@37*@38) */
  w38  = (w37*w38);
  /* #140: @39 = 0.5 */
  w39 = 5.0000000000000000e-01;
  /* #141: @41 = @11[3] */
  for (rr=(&w41), ss=w11+3; ss!=w11+4; ss+=1) *rr++ = *ss;
  /* #142: @41 = (@39*@41) */
  w41  = (w39*w41);
  /* #143: @42 = vertcat(@9, @28, @31, @36, @38, @41) */
  rr=w42;
  *rr++ = w9;
  *rr++ = w28;
  *rr++ = w31;
  *rr++ = w36;
  *rr++ = w38;
  *rr++ = w41;
  /* #144: @43 = @42' */
  casadi_copy(w42, 6, w43);
  /* #145: @44 = zeros(6x6) */
  casadi_clear(w44, 36);
  /* #146: @36 = 2 */
  w36 = 2.;
  /* #147: (@44[0] = @36) */
  for (rr=w44+0, ss=(&w36); rr!=w44+1; rr+=1) *rr = *ss++;
  /* #148: @36 = 2 */
  w36 = 2.;
  /* #149: (@44[7] = @36) */
  for (rr=w44+7, ss=(&w36); rr!=w44+8; rr+=1) *rr = *ss++;
  /* #150: @36 = 2 */
  w36 = 2.;
  /* #151: (@44[14] = @36) */
  for (rr=w44+14, ss=(&w36); rr!=w44+15; rr+=1) *rr = *ss++;
  /* #152: @36 = 1.6 */
  w36 = 1.6000000000000001e+00;
  /* #153: (@44[21] = @36) */
  for (rr=w44+21, ss=(&w36); rr!=w44+22; rr+=1) *rr = *ss++;
  /* #154: @36 = 1.6 */
  w36 = 1.6000000000000001e+00;
  /* #155: (@44[28] = @36) */
  for (rr=w44+28, ss=(&w36); rr!=w44+29; rr+=1) *rr = *ss++;
  /* #156: @36 = 1.6 */
  w36 = 1.6000000000000001e+00;
  /* #157: (@44[35] = @36) */
  for (rr=w44+35, ss=(&w36); rr!=w44+36; rr+=1) *rr = *ss++;
  /* #158: @2 = mac(@43,@44,@2) */
  for (i=0, rr=w2; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w43+j, tt=w44+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #159: @1 = mac(@2,@42,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w42+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #160: @0 = (@0*@1) */
  w0 *= w1;
  /* #161: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #162: @33 = zeros(14x1) */
  casadi_clear(w33, 14);
  /* #163: @5 = zeros(8x1) */
  casadi_clear(w5, 8);
  /* #164: @34 = zeros(8x1) */
  casadi_clear(w34, 8);
  /* #165: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #166: @21 = @18' */
  for (i=0, rr=w21, cs=w18; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #167: @15 = zeros(4x1) */
  casadi_clear(w15, 4);
  /* #168: @0 = 10 */
  w0 = 10.;
  /* #169: @2 = @2' */
  /* #170: @2 = (@0*@2) */
  for (i=0, rr=w2, cs=w2; i<6; ++i) (*rr++)  = (w0*(*cs++));
  /* #171: @43 = zeros(1x6) */
  casadi_clear(w43, 6);
  /* #172: @42 = @42' */
  /* #173: @42 = (@0*@42) */
  for (i=0, rr=w42, cs=w42; i<6; ++i) (*rr++)  = (w0*(*cs++));
  /* #174: @45 = @44' */
  for (i=0, rr=w45, cs=w44; i<6; ++i) for (j=0; j<6; ++j) rr[i+j*6] = *cs++;
  /* #175: @43 = mac(@42,@45,@43) */
  for (i=0, rr=w43; i<6; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w42+j, tt=w45+i*6; k<6; ++k) *rr += ss[k*1]**tt++;
  /* #176: @43 = @43' */
  /* #177: @2 = (@2+@43) */
  for (i=0, rr=w2, cs=w43; i<6; ++i) (*rr++) += (*cs++);
  /* #178: {@0, @1, @36, @38, @41, @46} = vertsplit(@2) */
  w0 = w2[0];
  w1 = w2[1];
  w36 = w2[2];
  w38 = w2[3];
  w41 = w2[4];
  w46 = w2[5];
  /* #179: @39 = (@39*@46) */
  w39 *= w46;
  /* #180: (@15[3] += @39) */
  for (rr=w15+3, ss=(&w39); rr!=w15+4; rr+=1) *rr += *ss++;
  /* #181: @37 = (@37*@41) */
  w37 *= w41;
  /* #182: (@15[2] += @37) */
  for (rr=w15+2, ss=(&w37); rr!=w15+3; rr+=1) *rr += *ss++;
  /* #183: @32 = (@32*@38) */
  w32 *= w38;
  /* #184: (@15[1] += @32) */
  for (rr=w15+1, ss=(&w32); rr!=w15+2; rr+=1) *rr += *ss++;
  /* #185: @11 = mac(@21,@15,@11) */
  for (i=0, rr=w11; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w21+j, tt=w15+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #186: {@32, @38, @37, @41} = vertsplit(@11) */
  w32 = w11[0];
  w38 = w11[1];
  w37 = w11[2];
  w41 = w11[3];
  /* #187: @41 = (-@41) */
  w41 = (- w41 );
  /* #188: (@34[3] += @41) */
  for (rr=w34+3, ss=(&w41); rr!=w34+4; rr+=1) *rr += *ss++;
  /* #189: @37 = (-@37) */
  w37 = (- w37 );
  /* #190: (@34[2] += @37) */
  for (rr=w34+2, ss=(&w37); rr!=w34+3; rr+=1) *rr += *ss++;
  /* #191: @38 = (-@38) */
  w38 = (- w38 );
  /* #192: (@34[1] += @38) */
  for (rr=w34+1, ss=(&w38); rr!=w34+2; rr+=1) *rr += *ss++;
  /* #193: (@34[0] += @32) */
  for (rr=w34+0, ss=(&w32); rr!=w34+1; rr+=1) *rr += *ss++;
  /* #194: @21 = zeros(4x4) */
  casadi_clear(w21, 16);
  /* #195: @14 = @14' */
  /* #196: @21 = mac(@15,@14,@21) */
  for (i=0, rr=w21; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w15+j, tt=w14+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #197: @21 = (2.*@21) */
  for (i=0, rr=w21, cs=w21; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #198: @18 = @21' */
  for (i=0, rr=w18, cs=w21; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #199: {@15, @14, @11, @16} = horzsplit(@18) */
  casadi_copy(w18, 4, w15);
  casadi_copy(w18+4, 4, w14);
  casadi_copy(w18+8, 4, w11);
  casadi_copy(w18+12, 4, w16);
  /* #200: @16 = @16' */
  /* #201: {@32, @38, @37, @41} = horzsplit(@16) */
  w32 = w16[0];
  w38 = w16[1];
  w37 = w16[2];
  w41 = w16[3];
  /* #202: (@34[4] += @41) */
  for (rr=w34+4, ss=(&w41); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #203: (@34[5] += @37) */
  for (rr=w34+5, ss=(&w37); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #204: @38 = (-@38) */
  w38 = (- w38 );
  /* #205: (@34[6] += @38) */
  for (rr=w34+6, ss=(&w38); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #206: (@34[7] += @32) */
  for (rr=w34+7, ss=(&w32); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #207: @11 = @11' */
  /* #208: {@32, @38, @37, @41} = horzsplit(@11) */
  w32 = w11[0];
  w38 = w11[1];
  w37 = w11[2];
  w41 = w11[3];
  /* #209: @41 = (-@41) */
  w41 = (- w41 );
  /* #210: (@34[5] += @41) */
  for (rr=w34+5, ss=(&w41); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #211: (@34[4] += @37) */
  for (rr=w34+4, ss=(&w37); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #212: (@34[7] += @38) */
  for (rr=w34+7, ss=(&w38); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #213: (@34[6] += @32) */
  for (rr=w34+6, ss=(&w32); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #214: @14 = @14' */
  /* #215: {@32, @38, @37, @41} = horzsplit(@14) */
  w32 = w14[0];
  w38 = w14[1];
  w37 = w14[2];
  w41 = w14[3];
  /* #216: (@34[6] += @41) */
  for (rr=w34+6, ss=(&w41); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #217: @37 = (-@37) */
  w37 = (- w37 );
  /* #218: (@34[7] += @37) */
  for (rr=w34+7, ss=(&w37); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #219: (@34[4] += @38) */
  for (rr=w34+4, ss=(&w38); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #220: (@34[5] += @32) */
  for (rr=w34+5, ss=(&w32); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #221: @15 = @15' */
  /* #222: {@32, @38, @37, @41} = horzsplit(@15) */
  w32 = w15[0];
  w38 = w15[1];
  w37 = w15[2];
  w41 = w15[3];
  /* #223: @41 = (-@41) */
  w41 = (- w41 );
  /* #224: (@34[7] += @41) */
  for (rr=w34+7, ss=(&w41); rr!=w34+8; rr+=1) *rr += *ss++;
  /* #225: @37 = (-@37) */
  w37 = (- w37 );
  /* #226: (@34[6] += @37) */
  for (rr=w34+6, ss=(&w37); rr!=w34+7; rr+=1) *rr += *ss++;
  /* #227: @38 = (-@38) */
  w38 = (- w38 );
  /* #228: (@34[5] += @38) */
  for (rr=w34+5, ss=(&w38); rr!=w34+6; rr+=1) *rr += *ss++;
  /* #229: (@34[4] += @32) */
  for (rr=w34+4, ss=(&w32); rr!=w34+5; rr+=1) *rr += *ss++;
  /* #230: @32 = (@36/@13) */
  w32  = (w36/w13);
  /* #231: @10 = (@10*@32) */
  w10 *= w32;
  /* #232: (@34[3] += @10) */
  for (rr=w34+3, ss=(&w10); rr!=w34+4; rr+=1) *rr += *ss++;
  /* #233: @10 = (@1/@13) */
  w10  = (w1/w13);
  /* #234: @26 = (@26*@10) */
  w26 *= w10;
  /* #235: (@34[2] += @26) */
  for (rr=w34+2, ss=(&w26); rr!=w34+3; rr+=1) *rr += *ss++;
  /* #236: @26 = (@0/@13) */
  w26  = (w0/w13);
  /* #237: @8 = (@8*@26) */
  w8 *= w26;
  /* #238: (@34[1] += @8) */
  for (rr=w34+1, ss=(&w8); rr!=w34+2; rr+=1) *rr += *ss++;
  /* #239: @8 = sq(@13) */
  w8 = casadi_sq( w13 );
  /* #240: @38 = sq(@7) */
  w38 = casadi_sq( w7 );
  /* #241: @8 = (@8+@38) */
  w8 += w38;
  /* #242: @38 = (@13/@8) */
  w38  = (w13/w8);
  /* #243: @30 = (@30*@32) */
  w30 *= w32;
  /* #244: @29 = (@29*@30) */
  w29 *= w30;
  /* #245: @27 = (@27*@10) */
  w27 *= w10;
  /* #246: @25 = (@25*@27) */
  w25 *= w27;
  /* #247: @29 = (@29+@25) */
  w29 += w25;
  /* #248: @12 = (@12*@26) */
  w12 *= w26;
  /* #249: @3 = (@3*@12) */
  w3 *= w12;
  /* #250: @29 = (@29+@3) */
  w29 += w3;
  /* #251: @29 = (2.*@29) */
  w29 = (2.* w29 );
  /* #252: @38 = (@38*@29) */
  w38 *= w29;
  /* #253: @38 = (-@38) */
  w38 = (- w38 );
  /* #254: (@34[0] += @38) */
  for (rr=w34+0, ss=(&w38); rr!=w34+1; rr+=1) *rr += *ss++;
  /* #255: @28 = (@28/@13) */
  w28 /= w13;
  /* #256: @28 = (@28*@1) */
  w28 *= w1;
  /* #257: @28 = (-@28) */
  w28 = (- w28 );
  /* #258: @31 = (@31/@13) */
  w31 /= w13;
  /* #259: @31 = (@31*@36) */
  w31 *= w36;
  /* #260: @28 = (@28-@31) */
  w28 -= w31;
  /* #261: @9 = (@9/@13) */
  w9 /= w13;
  /* #262: @9 = (@9*@0) */
  w9 *= w0;
  /* #263: @28 = (@28-@9) */
  w28 -= w9;
  /* #264: @7 = (@7/@8) */
  w7 /= w8;
  /* #265: @7 = (@7*@29) */
  w7 *= w29;
  /* #266: @28 = (@28+@7) */
  w28 += w7;
  /* #267: @28 = (@28/@13) */
  w28 /= w13;
  /* #268: @4 = (@28*@4) */
  for (i=0, rr=w4, cs=w4; i<3; ++i) (*rr++)  = (w28*(*cs++));
  /* #269: (@34[1:4] += @4) */
  for (rr=w34+1, ss=w4; rr!=w34+4; rr+=1) *rr += *ss++;
  /* #270: @5 = mac(@23,@34,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w23+j, tt=w34+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #271: (@33[:8] += @5) */
  for (rr=w33+0, ss=w5; rr!=w33+8; rr+=1) *rr += *ss++;
  /* #272: output[1][0] = @33 */
  casadi_copy(w33, 14, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_e_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_e_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_e_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_e_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_e_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_e_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 8;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 467;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
