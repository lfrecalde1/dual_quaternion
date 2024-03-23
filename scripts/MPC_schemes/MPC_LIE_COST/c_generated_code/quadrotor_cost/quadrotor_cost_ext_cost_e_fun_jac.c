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
#define casadi_c0 CASADI_PREFIX(c0)
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

static const casadi_real casadi_c0[64] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00, 0., 0., 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00, 0., 0., 0., 0., 0., 0., 0., 0., 1.6000000000000001e+00};

/* quadrotor_cost_ext_cost_e_fun_jac:(i0[14],i1[],i2[],i3[18])->(o0,o1[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, w4, *w5=w+20, w6, *w7=w+24, *w8=w+32, w9, w10, w11, w12, *w13=w+54, w14, w15, *w16=w+60, *w17=w+64, *w18=w+68, *w19=w+72, *w20=w+88, *w21=w+104, *w22=w+136, *w23=w+168, *w24=w+184, *w25=w+216, *w26=w+280, w27, w28, w29, w30, w31, w32, w33, w34, *w35=w+352, *w36=w+366, *w37=w+374, w38, w39, w40, w41, w42, w43, *w44=w+383, w45, *w46=w+388, *w47=w+396, *w48=w+404;
  /* #0: @0 = 10 */
  w0 = 10.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x8) */
  casadi_clear(w2, 8);
  /* #3: @3 = 0 */
  w3 = 0.;
  /* #4: @4 = 0.5 */
  w4 = 5.0000000000000000e-01;
  /* #5: @5 = all_2.22045e-16(3x1) */
  casadi_fill(w5, 3, 2.2204460492503131e-16);
  /* #6: @6 = 0 */
  w6 = 0.;
  /* #7: @7 = zeros(8x1) */
  casadi_clear(w7, 8);
  /* #8: @8 = input[3][0] */
  casadi_copy(arg[3], 18, w8);
  /* #9: @9 = @8[0] */
  for (rr=(&w9), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #10: @10 = @8[1] */
  for (rr=(&w10), ss=w8+1; ss!=w8+2; ss+=1) *rr++ = *ss;
  /* #11: @11 = @8[2] */
  for (rr=(&w11), ss=w8+2; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #12: @12 = @8[3] */
  for (rr=(&w12), ss=w8+3; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #13: @13 = horzcat(@9, @10, @11, @12) */
  rr=w13;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  *rr++ = w12;
  /* #14: @13 = @13' */
  /* #15: @9 = (-@10) */
  w9 = (- w10 );
  /* #16: @14 = @8[0] */
  for (rr=(&w14), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #17: @15 = (-@11) */
  w15 = (- w11 );
  /* #18: @16 = horzcat(@9, @14, @12, @15) */
  rr=w16;
  *rr++ = w9;
  *rr++ = w14;
  *rr++ = w12;
  *rr++ = w15;
  /* #19: @16 = @16' */
  /* #20: @12 = (-@12) */
  w12 = (- w12 );
  /* #21: @14 = @8[0] */
  for (rr=(&w14), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #22: @17 = horzcat(@15, @12, @14, @10) */
  rr=w17;
  *rr++ = w15;
  *rr++ = w12;
  *rr++ = w14;
  *rr++ = w10;
  /* #23: @17 = @17' */
  /* #24: @15 = @8[0] */
  for (rr=(&w15), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #25: @18 = horzcat(@12, @11, @9, @15) */
  rr=w18;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w9;
  *rr++ = w15;
  /* #26: @18 = @18' */
  /* #27: @19 = horzcat(@13, @16, @17, @18) */
  rr=w19;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  /* #28: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #29: @19 = zeros(4x4) */
  casadi_clear(w19, 16);
  /* #30: @21 = horzcat(@20, @19) */
  rr=w21;
  for (i=0, cs=w20; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w19; i<16; ++i) *rr++ = *cs++;
  /* #31: @22 = @21' */
  for (i=0, rr=w22, cs=w21; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #32: @12 = @8[4] */
  for (rr=(&w12), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #33: @11 = @8[5] */
  for (rr=(&w11), ss=w8+5; ss!=w8+6; ss+=1) *rr++ = *ss;
  /* #34: @9 = @8[6] */
  for (rr=(&w9), ss=w8+6; ss!=w8+7; ss+=1) *rr++ = *ss;
  /* #35: @15 = @8[7] */
  for (rr=(&w15), ss=w8+7; ss!=w8+8; ss+=1) *rr++ = *ss;
  /* #36: @13 = horzcat(@12, @11, @9, @15) */
  rr=w13;
  *rr++ = w12;
  *rr++ = w11;
  *rr++ = w9;
  *rr++ = w15;
  /* #37: @13 = @13' */
  /* #38: @12 = (-@11) */
  w12 = (- w11 );
  /* #39: @14 = @8[4] */
  for (rr=(&w14), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #40: @10 = (-@9) */
  w10 = (- w9 );
  /* #41: @16 = horzcat(@12, @14, @15, @10) */
  rr=w16;
  *rr++ = w12;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w10;
  /* #42: @16 = @16' */
  /* #43: @15 = (-@15) */
  w15 = (- w15 );
  /* #44: @14 = @8[4] */
  for (rr=(&w14), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #45: @17 = horzcat(@10, @15, @14, @11) */
  rr=w17;
  *rr++ = w10;
  *rr++ = w15;
  *rr++ = w14;
  *rr++ = w11;
  /* #46: @17 = @17' */
  /* #47: @10 = @8[4] */
  for (rr=(&w10), ss=w8+4; ss!=w8+5; ss+=1) *rr++ = *ss;
  /* #48: @18 = horzcat(@15, @9, @12, @10) */
  rr=w18;
  *rr++ = w15;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w10;
  /* #49: @18 = @18' */
  /* #50: @19 = horzcat(@13, @16, @17, @18) */
  rr=w19;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  /* #51: @23 = @19' */
  for (i=0, rr=w23, cs=w19; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #52: @21 = horzcat(@23, @20) */
  rr=w21;
  for (i=0, cs=w23; i<16; ++i) *rr++ = *cs++;
  for (i=0, cs=w20; i<16; ++i) *rr++ = *cs++;
  /* #53: @24 = @21' */
  for (i=0, rr=w24, cs=w21; i<8; ++i) for (j=0; j<4; ++j) rr[i+j*8] = *cs++;
  /* #54: @25 = horzcat(@22, @24) */
  rr=w25;
  for (i=0, cs=w22; i<32; ++i) *rr++ = *cs++;
  for (i=0, cs=w24; i<32; ++i) *rr++ = *cs++;
  /* #55: @26 = @25' */
  for (i=0, rr=w26, cs=w25; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #56: @15 = input[0][0] */
  w15 = arg[0] ? arg[0][0] : 0;
  /* #57: @9 = input[0][1] */
  w9 = arg[0] ? arg[0][1] : 0;
  /* #58: @12 = input[0][2] */
  w12 = arg[0] ? arg[0][2] : 0;
  /* #59: @10 = input[0][3] */
  w10 = arg[0] ? arg[0][3] : 0;
  /* #60: @14 = input[0][4] */
  w14 = arg[0] ? arg[0][4] : 0;
  /* #61: @11 = input[0][5] */
  w11 = arg[0] ? arg[0][5] : 0;
  /* #62: @27 = input[0][6] */
  w27 = arg[0] ? arg[0][6] : 0;
  /* #63: @28 = input[0][7] */
  w28 = arg[0] ? arg[0][7] : 0;
  /* #64: @29 = input[0][8] */
  w29 = arg[0] ? arg[0][8] : 0;
  /* #65: @30 = input[0][9] */
  w30 = arg[0] ? arg[0][9] : 0;
  /* #66: @31 = input[0][10] */
  w31 = arg[0] ? arg[0][10] : 0;
  /* #67: @32 = input[0][11] */
  w32 = arg[0] ? arg[0][11] : 0;
  /* #68: @33 = input[0][12] */
  w33 = arg[0] ? arg[0][12] : 0;
  /* #69: @34 = input[0][13] */
  w34 = arg[0] ? arg[0][13] : 0;
  /* #70: @35 = vertcat(@15, @9, @12, @10, @14, @11, @27, @28, @29, @30, @31, @32, @33, @34) */
  rr=w35;
  *rr++ = w15;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w10;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w27;
  *rr++ = w28;
  *rr++ = w29;
  *rr++ = w30;
  *rr++ = w31;
  *rr++ = w32;
  *rr++ = w33;
  *rr++ = w34;
  /* #71: @36 = @35[:8] */
  for (rr=w36, ss=w35+0; ss!=w35+8; ss+=1) *rr++ = *ss;
  /* #72: @7 = mac(@26,@36,@7) */
  for (i=0, rr=w7; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w26+j, tt=w36+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #73: @15 = @7[0] */
  for (rr=(&w15), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #74: @6 = (@6<@15) */
  w6  = (w6<w15);
  /* #75: @36 = (@6?@7:0) */
  for (i=0, rr=w36, cs=w7; i<8; ++i) (*rr++)  = (w6?(*cs++):0);
  /* #76: @15 = (!@6) */
  w15 = (! w6 );
  /* #77: @7 = (-@7) */
  for (i=0, rr=w7, cs=w7; i<8; ++i) *rr++ = (- *cs++ );
  /* #78: @7 = (@15?@7:0) */
  for (i=0, rr=w7, cs=w7; i<8; ++i) (*rr++)  = (w15?(*cs++):0);
  /* #79: @36 = (@36+@7) */
  for (i=0, rr=w36, cs=w7; i<8; ++i) (*rr++) += (*cs++);
  /* #80: @37 = @36[1:4] */
  for (rr=w37, ss=w36+1; ss!=w36+4; ss+=1) *rr++ = *ss;
  /* #81: @5 = (@5+@37) */
  for (i=0, rr=w5, cs=w37; i<3; ++i) (*rr++) += (*cs++);
  /* #82: @9 = ||@5||_F */
  w9 = sqrt(casadi_dot(3, w5, w5));
  /* #83: @12 = @36[0] */
  for (rr=(&w12), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #84: @10 = atan2(@9,@12) */
  w10  = atan2(w9,w12);
  /* #85: @14 = (@4*@10) */
  w14  = (w4*w10);
  /* #86: @11 = @36[1] */
  for (rr=(&w11), ss=w36+1; ss!=w36+2; ss+=1) *rr++ = *ss;
  /* #87: @27 = (@14*@11) */
  w27  = (w14*w11);
  /* #88: @27 = (@27/@9) */
  w27 /= w9;
  /* #89: @28 = 0.5 */
  w28 = 5.0000000000000000e-01;
  /* #90: @29 = (@28*@10) */
  w29  = (w28*w10);
  /* #91: @30 = @36[2] */
  for (rr=(&w30), ss=w36+2; ss!=w36+3; ss+=1) *rr++ = *ss;
  /* #92: @31 = (@29*@30) */
  w31  = (w29*w30);
  /* #93: @31 = (@31/@9) */
  w31 /= w9;
  /* #94: @32 = 0.5 */
  w32 = 5.0000000000000000e-01;
  /* #95: @10 = (@32*@10) */
  w10  = (w32*w10);
  /* #96: @33 = @36[3] */
  for (rr=(&w33), ss=w36+3; ss!=w36+4; ss+=1) *rr++ = *ss;
  /* #97: @34 = (@10*@33) */
  w34  = (w10*w33);
  /* #98: @34 = (@34/@9) */
  w34 /= w9;
  /* #99: @38 = 0 */
  w38 = 0.;
  /* #100: @39 = 0.5 */
  w39 = 5.0000000000000000e-01;
  /* #101: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #102: @40 = @36[4] */
  for (rr=(&w40), ss=w36+4; ss!=w36+5; ss+=1) *rr++ = *ss;
  /* #103: @41 = @36[5] */
  for (rr=(&w41), ss=w36+5; ss!=w36+6; ss+=1) *rr++ = *ss;
  /* #104: @41 = (-@41) */
  w41 = (- w41 );
  /* #105: @42 = @36[6] */
  for (rr=(&w42), ss=w36+6; ss!=w36+7; ss+=1) *rr++ = *ss;
  /* #106: @42 = (-@42) */
  w42 = (- w42 );
  /* #107: @43 = @36[7] */
  for (rr=(&w43), ss=w36+7; ss!=w36+8; ss+=1) *rr++ = *ss;
  /* #108: @43 = (-@43) */
  w43 = (- w43 );
  /* #109: @16 = horzcat(@40, @41, @42, @43) */
  rr=w16;
  *rr++ = w40;
  *rr++ = w41;
  *rr++ = w42;
  *rr++ = w43;
  /* #110: @16 = @16' */
  /* #111: @40 = @36[5] */
  for (rr=(&w40), ss=w36+5; ss!=w36+6; ss+=1) *rr++ = *ss;
  /* #112: @41 = @36[4] */
  for (rr=(&w41), ss=w36+4; ss!=w36+5; ss+=1) *rr++ = *ss;
  /* #113: @42 = @36[7] */
  for (rr=(&w42), ss=w36+7; ss!=w36+8; ss+=1) *rr++ = *ss;
  /* #114: @42 = (-@42) */
  w42 = (- w42 );
  /* #115: @43 = @36[6] */
  for (rr=(&w43), ss=w36+6; ss!=w36+7; ss+=1) *rr++ = *ss;
  /* #116: @17 = horzcat(@40, @41, @42, @43) */
  rr=w17;
  *rr++ = w40;
  *rr++ = w41;
  *rr++ = w42;
  *rr++ = w43;
  /* #117: @17 = @17' */
  /* #118: @40 = @36[6] */
  for (rr=(&w40), ss=w36+6; ss!=w36+7; ss+=1) *rr++ = *ss;
  /* #119: @41 = @36[7] */
  for (rr=(&w41), ss=w36+7; ss!=w36+8; ss+=1) *rr++ = *ss;
  /* #120: @42 = @36[4] */
  for (rr=(&w42), ss=w36+4; ss!=w36+5; ss+=1) *rr++ = *ss;
  /* #121: @43 = @36[5] */
  for (rr=(&w43), ss=w36+5; ss!=w36+6; ss+=1) *rr++ = *ss;
  /* #122: @43 = (-@43) */
  w43 = (- w43 );
  /* #123: @18 = horzcat(@40, @41, @42, @43) */
  rr=w18;
  *rr++ = w40;
  *rr++ = w41;
  *rr++ = w42;
  *rr++ = w43;
  /* #124: @18 = @18' */
  /* #125: @40 = @36[7] */
  for (rr=(&w40), ss=w36+7; ss!=w36+8; ss+=1) *rr++ = *ss;
  /* #126: @41 = @36[6] */
  for (rr=(&w41), ss=w36+6; ss!=w36+7; ss+=1) *rr++ = *ss;
  /* #127: @41 = (-@41) */
  w41 = (- w41 );
  /* #128: @42 = @36[5] */
  for (rr=(&w42), ss=w36+5; ss!=w36+6; ss+=1) *rr++ = *ss;
  /* #129: @43 = @36[4] */
  for (rr=(&w43), ss=w36+4; ss!=w36+5; ss+=1) *rr++ = *ss;
  /* #130: @44 = horzcat(@40, @41, @42, @43) */
  rr=w44;
  *rr++ = w40;
  *rr++ = w41;
  *rr++ = w42;
  *rr++ = w43;
  /* #131: @44 = @44' */
  /* #132: @23 = horzcat(@16, @17, @18, @44) */
  rr=w23;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w44; i<4; ++i) *rr++ = *cs++;
  /* #133: @20 = @23' */
  for (i=0, rr=w20, cs=w23; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #134: @20 = (2.*@20) */
  for (i=0, rr=w20, cs=w20; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #135: @40 = @36[0] */
  for (rr=(&w40), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #136: @41 = @36[1] */
  for (rr=(&w41), ss=w36+1; ss!=w36+2; ss+=1) *rr++ = *ss;
  /* #137: @41 = (-@41) */
  w41 = (- w41 );
  /* #138: @42 = @36[2] */
  for (rr=(&w42), ss=w36+2; ss!=w36+3; ss+=1) *rr++ = *ss;
  /* #139: @42 = (-@42) */
  w42 = (- w42 );
  /* #140: @43 = @36[3] */
  for (rr=(&w43), ss=w36+3; ss!=w36+4; ss+=1) *rr++ = *ss;
  /* #141: @43 = (-@43) */
  w43 = (- w43 );
  /* #142: @16 = vertcat(@40, @41, @42, @43) */
  rr=w16;
  *rr++ = w40;
  *rr++ = w41;
  *rr++ = w42;
  *rr++ = w43;
  /* #143: @13 = mac(@20,@16,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w20+j, tt=w16+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #144: @40 = @13[1] */
  for (rr=(&w40), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #145: @40 = (@39*@40) */
  w40  = (w39*w40);
  /* #146: @41 = 0.5 */
  w41 = 5.0000000000000000e-01;
  /* #147: @42 = @13[2] */
  for (rr=(&w42), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #148: @42 = (@41*@42) */
  w42  = (w41*w42);
  /* #149: @43 = 0.5 */
  w43 = 5.0000000000000000e-01;
  /* #150: @45 = @13[3] */
  for (rr=(&w45), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #151: @45 = (@43*@45) */
  w45  = (w43*w45);
  /* #152: @36 = vertcat(@3, @27, @31, @34, @38, @40, @42, @45) */
  rr=w36;
  *rr++ = w3;
  *rr++ = w27;
  *rr++ = w31;
  *rr++ = w34;
  *rr++ = w38;
  *rr++ = w40;
  *rr++ = w42;
  *rr++ = w45;
  /* #153: @7 = @36' */
  casadi_copy(w36, 8, w7);
  /* #154: @26 = 
  [[0, 0, 0, 0, 0, 0, 0, 0], 
   [0, 2, 0, 0, 0, 0, 0, 0], 
   [0, 0, 2, 0, 0, 0, 0, 0], 
   [0, 0, 0, 2, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0, 1.6, 0, 0], 
   [0, 0, 0, 0, 0, 0, 1.6, 0], 
   [0, 0, 0, 0, 0, 0, 0, 1.6]] */
  casadi_copy(casadi_c0, 64, w26);
  /* #155: @2 = mac(@7,@26,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w26+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #156: @1 = mac(@2,@36,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w36+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #157: @0 = (@0*@1) */
  w0 *= w1;
  /* #158: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #159: @35 = zeros(14x1) */
  casadi_clear(w35, 14);
  /* #160: @7 = zeros(8x1) */
  casadi_clear(w7, 8);
  /* #161: @0 = 1 */
  w0 = 1.;
  /* #162: @6 = (@6?@0:0) */
  w6  = (w6?w0:0);
  /* #163: @46 = zeros(8x1) */
  casadi_clear(w46, 8);
  /* #164: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #165: @23 = @20' */
  for (i=0, rr=w23, cs=w20; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #166: @17 = zeros(4x1) */
  casadi_clear(w17, 4);
  /* #167: @0 = 10 */
  w0 = 10.;
  /* #168: @2 = @2' */
  /* #169: @2 = (@0*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #170: @47 = zeros(1x8) */
  casadi_clear(w47, 8);
  /* #171: @36 = @36' */
  /* #172: @36 = (@0*@36) */
  for (i=0, rr=w36, cs=w36; i<8; ++i) (*rr++)  = (w0*(*cs++));
  /* #173: @48 = @26' */
  for (i=0, rr=w48, cs=w26; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #174: @47 = mac(@36,@48,@47) */
  for (i=0, rr=w47; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w36+j, tt=w48+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #175: @47 = @47' */
  /* #176: @2 = (@2+@47) */
  for (i=0, rr=w2, cs=w47; i<8; ++i) (*rr++) += (*cs++);
  /* #177: {NULL, @0, @1, @3, NULL, @38, @40, @42} = vertsplit(@2) */
  w0 = w2[1];
  w1 = w2[2];
  w3 = w2[3];
  w38 = w2[5];
  w40 = w2[6];
  w42 = w2[7];
  /* #178: @43 = (@43*@42) */
  w43 *= w42;
  /* #179: (@17[3] += @43) */
  for (rr=w17+3, ss=(&w43); rr!=w17+4; rr+=1) *rr += *ss++;
  /* #180: @41 = (@41*@40) */
  w41 *= w40;
  /* #181: (@17[2] += @41) */
  for (rr=w17+2, ss=(&w41); rr!=w17+3; rr+=1) *rr += *ss++;
  /* #182: @39 = (@39*@38) */
  w39 *= w38;
  /* #183: (@17[1] += @39) */
  for (rr=w17+1, ss=(&w39); rr!=w17+2; rr+=1) *rr += *ss++;
  /* #184: @13 = mac(@23,@17,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w17+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #185: {@39, @38, @41, @40} = vertsplit(@13) */
  w39 = w13[0];
  w38 = w13[1];
  w41 = w13[2];
  w40 = w13[3];
  /* #186: @40 = (-@40) */
  w40 = (- w40 );
  /* #187: (@46[3] += @40) */
  for (rr=w46+3, ss=(&w40); rr!=w46+4; rr+=1) *rr += *ss++;
  /* #188: @41 = (-@41) */
  w41 = (- w41 );
  /* #189: (@46[2] += @41) */
  for (rr=w46+2, ss=(&w41); rr!=w46+3; rr+=1) *rr += *ss++;
  /* #190: @38 = (-@38) */
  w38 = (- w38 );
  /* #191: (@46[1] += @38) */
  for (rr=w46+1, ss=(&w38); rr!=w46+2; rr+=1) *rr += *ss++;
  /* #192: (@46[0] += @39) */
  for (rr=w46+0, ss=(&w39); rr!=w46+1; rr+=1) *rr += *ss++;
  /* #193: @23 = zeros(4x4) */
  casadi_clear(w23, 16);
  /* #194: @16 = @16' */
  /* #195: @23 = mac(@17,@16,@23) */
  for (i=0, rr=w23; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w17+j, tt=w16+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #196: @23 = (2.*@23) */
  for (i=0, rr=w23, cs=w23; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #197: @20 = @23' */
  for (i=0, rr=w20, cs=w23; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #198: {@17, @16, @13, @18} = horzsplit(@20) */
  casadi_copy(w20, 4, w17);
  casadi_copy(w20+4, 4, w16);
  casadi_copy(w20+8, 4, w13);
  casadi_copy(w20+12, 4, w18);
  /* #199: @18 = @18' */
  /* #200: {@39, @38, @41, @40} = horzsplit(@18) */
  w39 = w18[0];
  w38 = w18[1];
  w41 = w18[2];
  w40 = w18[3];
  /* #201: (@46[4] += @40) */
  for (rr=w46+4, ss=(&w40); rr!=w46+5; rr+=1) *rr += *ss++;
  /* #202: (@46[5] += @41) */
  for (rr=w46+5, ss=(&w41); rr!=w46+6; rr+=1) *rr += *ss++;
  /* #203: @38 = (-@38) */
  w38 = (- w38 );
  /* #204: (@46[6] += @38) */
  for (rr=w46+6, ss=(&w38); rr!=w46+7; rr+=1) *rr += *ss++;
  /* #205: (@46[7] += @39) */
  for (rr=w46+7, ss=(&w39); rr!=w46+8; rr+=1) *rr += *ss++;
  /* #206: @13 = @13' */
  /* #207: {@39, @38, @41, @40} = horzsplit(@13) */
  w39 = w13[0];
  w38 = w13[1];
  w41 = w13[2];
  w40 = w13[3];
  /* #208: @40 = (-@40) */
  w40 = (- w40 );
  /* #209: (@46[5] += @40) */
  for (rr=w46+5, ss=(&w40); rr!=w46+6; rr+=1) *rr += *ss++;
  /* #210: (@46[4] += @41) */
  for (rr=w46+4, ss=(&w41); rr!=w46+5; rr+=1) *rr += *ss++;
  /* #211: (@46[7] += @38) */
  for (rr=w46+7, ss=(&w38); rr!=w46+8; rr+=1) *rr += *ss++;
  /* #212: (@46[6] += @39) */
  for (rr=w46+6, ss=(&w39); rr!=w46+7; rr+=1) *rr += *ss++;
  /* #213: @16 = @16' */
  /* #214: {@39, @38, @41, @40} = horzsplit(@16) */
  w39 = w16[0];
  w38 = w16[1];
  w41 = w16[2];
  w40 = w16[3];
  /* #215: (@46[6] += @40) */
  for (rr=w46+6, ss=(&w40); rr!=w46+7; rr+=1) *rr += *ss++;
  /* #216: @41 = (-@41) */
  w41 = (- w41 );
  /* #217: (@46[7] += @41) */
  for (rr=w46+7, ss=(&w41); rr!=w46+8; rr+=1) *rr += *ss++;
  /* #218: (@46[4] += @38) */
  for (rr=w46+4, ss=(&w38); rr!=w46+5; rr+=1) *rr += *ss++;
  /* #219: (@46[5] += @39) */
  for (rr=w46+5, ss=(&w39); rr!=w46+6; rr+=1) *rr += *ss++;
  /* #220: @17 = @17' */
  /* #221: {@39, @38, @41, @40} = horzsplit(@17) */
  w39 = w17[0];
  w38 = w17[1];
  w41 = w17[2];
  w40 = w17[3];
  /* #222: @40 = (-@40) */
  w40 = (- w40 );
  /* #223: (@46[7] += @40) */
  for (rr=w46+7, ss=(&w40); rr!=w46+8; rr+=1) *rr += *ss++;
  /* #224: @41 = (-@41) */
  w41 = (- w41 );
  /* #225: (@46[6] += @41) */
  for (rr=w46+6, ss=(&w41); rr!=w46+7; rr+=1) *rr += *ss++;
  /* #226: @38 = (-@38) */
  w38 = (- w38 );
  /* #227: (@46[5] += @38) */
  for (rr=w46+5, ss=(&w38); rr!=w46+6; rr+=1) *rr += *ss++;
  /* #228: (@46[4] += @39) */
  for (rr=w46+4, ss=(&w39); rr!=w46+5; rr+=1) *rr += *ss++;
  /* #229: @39 = (@3/@9) */
  w39  = (w3/w9);
  /* #230: @10 = (@10*@39) */
  w10 *= w39;
  /* #231: (@46[3] += @10) */
  for (rr=w46+3, ss=(&w10); rr!=w46+4; rr+=1) *rr += *ss++;
  /* #232: @10 = (@1/@9) */
  w10  = (w1/w9);
  /* #233: @29 = (@29*@10) */
  w29 *= w10;
  /* #234: (@46[2] += @29) */
  for (rr=w46+2, ss=(&w29); rr!=w46+3; rr+=1) *rr += *ss++;
  /* #235: @29 = (@0/@9) */
  w29  = (w0/w9);
  /* #236: @14 = (@14*@29) */
  w14 *= w29;
  /* #237: (@46[1] += @14) */
  for (rr=w46+1, ss=(&w14); rr!=w46+2; rr+=1) *rr += *ss++;
  /* #238: @14 = sq(@9) */
  w14 = casadi_sq( w9 );
  /* #239: @38 = sq(@12) */
  w38 = casadi_sq( w12 );
  /* #240: @14 = (@14+@38) */
  w14 += w38;
  /* #241: @38 = (@9/@14) */
  w38  = (w9/w14);
  /* #242: @33 = (@33*@39) */
  w33 *= w39;
  /* #243: @32 = (@32*@33) */
  w32 *= w33;
  /* #244: @30 = (@30*@10) */
  w30 *= w10;
  /* #245: @28 = (@28*@30) */
  w28 *= w30;
  /* #246: @32 = (@32+@28) */
  w32 += w28;
  /* #247: @11 = (@11*@29) */
  w11 *= w29;
  /* #248: @4 = (@4*@11) */
  w4 *= w11;
  /* #249: @32 = (@32+@4) */
  w32 += w4;
  /* #250: @38 = (@38*@32) */
  w38 *= w32;
  /* #251: @38 = (-@38) */
  w38 = (- w38 );
  /* #252: (@46[0] += @38) */
  for (rr=w46+0, ss=(&w38); rr!=w46+1; rr+=1) *rr += *ss++;
  /* #253: @31 = (@31/@9) */
  w31 /= w9;
  /* #254: @31 = (@31*@1) */
  w31 *= w1;
  /* #255: @31 = (-@31) */
  w31 = (- w31 );
  /* #256: @34 = (@34/@9) */
  w34 /= w9;
  /* #257: @34 = (@34*@3) */
  w34 *= w3;
  /* #258: @31 = (@31-@34) */
  w31 -= w34;
  /* #259: @27 = (@27/@9) */
  w27 /= w9;
  /* #260: @27 = (@27*@0) */
  w27 *= w0;
  /* #261: @31 = (@31-@27) */
  w31 -= w27;
  /* #262: @12 = (@12/@14) */
  w12 /= w14;
  /* #263: @12 = (@12*@32) */
  w12 *= w32;
  /* #264: @31 = (@31+@12) */
  w31 += w12;
  /* #265: @31 = (@31/@9) */
  w31 /= w9;
  /* #266: @5 = (@31*@5) */
  for (i=0, rr=w5, cs=w5; i<3; ++i) (*rr++)  = (w31*(*cs++));
  /* #267: (@46[1:4] += @5) */
  for (rr=w46+1, ss=w5; rr!=w46+4; rr+=1) *rr += *ss++;
  /* #268: @2 = (@6*@46) */
  for (i=0, rr=w2, cs=w46; i<8; ++i) (*rr++)  = (w6*(*cs++));
  /* #269: @6 = 1 */
  w6 = 1.;
  /* #270: @15 = (@15?@6:0) */
  w15  = (w15?w6:0);
  /* #271: @46 = (@15*@46) */
  for (i=0, rr=w46, cs=w46; i<8; ++i) (*rr++)  = (w15*(*cs++));
  /* #272: @2 = (@2-@46) */
  for (i=0, rr=w2, cs=w46; i<8; ++i) (*rr++) -= (*cs++);
  /* #273: @7 = mac(@25,@2,@7) */
  for (i=0, rr=w7; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w25+j, tt=w2+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #274: (@35[:8] += @7) */
  for (rr=w35+0, ss=w7; rr!=w35+8; rr+=1) *rr += *ss++;
  /* #275: output[1][0] = @35 */
  casadi_copy(w35, 14, res[1]);
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
  if (sz_res) *sz_res = 10;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 468;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
