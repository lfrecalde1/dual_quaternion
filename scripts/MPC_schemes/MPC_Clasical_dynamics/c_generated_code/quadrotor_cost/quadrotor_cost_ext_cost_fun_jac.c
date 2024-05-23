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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_fun_jac_ ## ID
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_fun_jac:(i0[13],i1[4],i2[],i3[17])->(o0,o1[17]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real w0, *w1=w+2, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+22, *w20=w+39, *w21=w+42, *w22=w+55, *w23=w+58, *w24=w+67, *w25=w+71, *w26=w+75, *w27=w+79, *w28=w+95, *w29=w+98, *w30=w+101, *w31=w+110, *w32=w+126, w33, w34, w35, w36, w37;
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
  /* #39: @2 = 4.5 */
  w2 = 4.5000000000000000e+00;
  /* #40: (@23[0] = @2) */
  for (rr=w23+0, ss=(&w2); rr!=w23+1; rr+=1) *rr = *ss++;
  /* #41: @2 = 4.5 */
  w2 = 4.5000000000000000e+00;
  /* #42: (@23[4] = @2) */
  for (rr=w23+4, ss=(&w2); rr!=w23+5; rr+=1) *rr = *ss++;
  /* #43: @2 = 4.5 */
  w2 = 4.5000000000000000e+00;
  /* #44: (@23[8] = @2) */
  for (rr=w23+8, ss=(&w2); rr!=w23+9; rr+=1) *rr = *ss++;
  /* #45: @1 = mac(@22,@23,@1) */
  for (i=0, rr=w1; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w23+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #46: @0 = mac(@1,@20,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w20+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #47: @2 = 0 */
  w2 = 0.;
  /* #48: @24 = zeros(1x4) */
  casadi_clear(w24, 4);
  /* #49: @25 = @19[13:17] */
  for (rr=w25, ss=w19+13; ss!=w19+17; ss+=1) *rr++ = *ss;
  /* #50: @3 = input[1][0] */
  w3 = arg[1] ? arg[1][0] : 0;
  /* #51: @4 = input[1][1] */
  w4 = arg[1] ? arg[1][1] : 0;
  /* #52: @5 = input[1][2] */
  w5 = arg[1] ? arg[1][2] : 0;
  /* #53: @6 = input[1][3] */
  w6 = arg[1] ? arg[1][3] : 0;
  /* #54: @26 = vertcat(@3, @4, @5, @6) */
  rr=w26;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #55: @25 = (@25-@26) */
  for (i=0, rr=w25, cs=w26; i<4; ++i) (*rr++) -= (*cs++);
  /* #56: @26 = @25' */
  casadi_copy(w25, 4, w26);
  /* #57: @27 = zeros(4x4) */
  casadi_clear(w27, 16);
  /* #58: @3 = 0.671141 */
  w3 = 6.7114093959731547e-01;
  /* #59: (@27[0] = @3) */
  for (rr=w27+0, ss=(&w3); rr!=w27+1; rr+=1) *rr = *ss++;
  /* #60: @3 = 600 */
  w3 = 600.;
  /* #61: (@27[5] = @3) */
  for (rr=w27+5, ss=(&w3); rr!=w27+6; rr+=1) *rr = *ss++;
  /* #62: @3 = 600 */
  w3 = 600.;
  /* #63: (@27[10] = @3) */
  for (rr=w27+10, ss=(&w3); rr!=w27+11; rr+=1) *rr = *ss++;
  /* #64: @3 = 600 */
  w3 = 600.;
  /* #65: (@27[15] = @3) */
  for (rr=w27+15, ss=(&w3); rr!=w27+16; rr+=1) *rr = *ss++;
  /* #66: @24 = mac(@26,@27,@24) */
  for (i=0, rr=w24; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w26+j, tt=w27+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #67: @2 = mac(@24,@25,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w24+j, tt=w25+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #68: @0 = (@0+@2) */
  w0 += w2;
  /* #69: @2 = 0 */
  w2 = 0.;
  /* #70: @22 = zeros(1x3) */
  casadi_clear(w22, 3);
  /* #71: @3 = (@8*@12) */
  w3  = (w8*w12);
  /* #72: @4 = (@9*@13) */
  w4  = (w9*w13);
  /* #73: @3 = (@3+@4) */
  w3 += w4;
  /* #74: @4 = (@10*@14) */
  w4  = (w10*w14);
  /* #75: @3 = (@3+@4) */
  w3 += w4;
  /* #76: @4 = (@11*@15) */
  w4  = (w11*w15);
  /* #77: @3 = (@3+@4) */
  w3 += w4;
  /* #78: @4 = (@8*@13) */
  w4  = (w8*w13);
  /* #79: @5 = (@9*@12) */
  w5  = (w9*w12);
  /* #80: @4 = (@4-@5) */
  w4 -= w5;
  /* #81: @5 = (@11*@14) */
  w5  = (w11*w14);
  /* #82: @4 = (@4+@5) */
  w4 += w5;
  /* #83: @5 = (@10*@15) */
  w5  = (w10*w15);
  /* #84: @4 = (@4-@5) */
  w4 -= w5;
  /* #85: @5 = (@3*@4) */
  w5  = (w3*w4);
  /* #86: @6 = (@10*@12) */
  w6  = (w10*w12);
  /* #87: @6 = (-@6) */
  w6 = (- w6 );
  /* #88: @7 = (@11*@13) */
  w7  = (w11*w13);
  /* #89: @6 = (@6-@7) */
  w6 -= w7;
  /* #90: @7 = (@8*@14) */
  w7  = (w8*w14);
  /* #91: @6 = (@6+@7) */
  w6 += w7;
  /* #92: @7 = (@9*@15) */
  w7  = (w9*w15);
  /* #93: @6 = (@6+@7) */
  w6 += w7;
  /* #94: @13 = (@10*@13) */
  w13  = (w10*w13);
  /* #95: @12 = (@11*@12) */
  w12  = (w11*w12);
  /* #96: @13 = (@13-@12) */
  w13 -= w12;
  /* #97: @14 = (@9*@14) */
  w14  = (w9*w14);
  /* #98: @13 = (@13-@14) */
  w13 -= w14;
  /* #99: @15 = (@8*@15) */
  w15  = (w8*w15);
  /* #100: @13 = (@13+@15) */
  w13 += w15;
  /* #101: @15 = (@6*@13) */
  w15  = (w6*w13);
  /* #102: @5 = (@5-@15) */
  w5 -= w15;
  /* #103: @15 = (@3*@6) */
  w15  = (w3*w6);
  /* #104: @14 = (@4*@13) */
  w14  = (w4*w13);
  /* #105: @15 = (@15+@14) */
  w15 += w14;
  /* #106: @28 = vertcat(@5, @15, @13) */
  rr=w28;
  *rr++ = w5;
  *rr++ = w15;
  *rr++ = w13;
  /* #107: @5 = 0.001 */
  w5 = 1.0000000000000000e-03;
  /* #108: @15 = sq(@3) */
  w15 = casadi_sq( w3 );
  /* #109: @14 = sq(@13) */
  w14 = casadi_sq( w13 );
  /* #110: @15 = (@15+@14) */
  w15 += w14;
  /* #111: @5 = (@5+@15) */
  w5 += w15;
  /* #112: @5 = sqrt(@5) */
  w5 = sqrt( w5 );
  /* #113: @28 = (@28/@5) */
  for (i=0, rr=w28; i<3; ++i) (*rr++) /= w5;
  /* #114: @29 = @28' */
  casadi_copy(w28, 3, w29);
  /* #115: @30 = zeros(3x3) */
  casadi_clear(w30, 9);
  /* #116: @15 = 10.5 */
  w15 = 1.0500000000000000e+01;
  /* #117: (@30[0] = @15) */
  for (rr=w30+0, ss=(&w15); rr!=w30+1; rr+=1) *rr = *ss++;
  /* #118: @15 = 10.5 */
  w15 = 1.0500000000000000e+01;
  /* #119: (@30[4] = @15) */
  for (rr=w30+4, ss=(&w15); rr!=w30+5; rr+=1) *rr = *ss++;
  /* #120: @15 = 10.5 */
  w15 = 1.0500000000000000e+01;
  /* #121: (@30[8] = @15) */
  for (rr=w30+8, ss=(&w15); rr!=w30+9; rr+=1) *rr = *ss++;
  /* #122: @22 = mac(@29,@30,@22) */
  for (i=0, rr=w22; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w29+j, tt=w30+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #123: @2 = mac(@22,@28,@2) */
  for (i=0, rr=(&w2); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w22+j, tt=w28+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #124: @0 = (@0+@2) */
  w0 += w2;
  /* #125: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #126: @24 = @24' */
  /* #127: @26 = zeros(1x4) */
  casadi_clear(w26, 4);
  /* #128: @25 = @25' */
  /* #129: @31 = @27' */
  for (i=0, rr=w31, cs=w27; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #130: @26 = mac(@25,@31,@26) */
  for (i=0, rr=w26; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w25+j, tt=w31+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #131: @26 = @26' */
  /* #132: @24 = (@24+@26) */
  for (i=0, rr=w24, cs=w26; i<4; ++i) (*rr++) += (*cs++);
  /* #133: @24 = (-@24) */
  for (i=0, rr=w24, cs=w24; i<4; ++i) *rr++ = (- *cs++ );
  /* #134: {@0, @2, @15, @14} = vertsplit(@24) */
  w0 = w24[0];
  w2 = w24[1];
  w15 = w24[2];
  w14 = w24[3];
  /* #135: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #136: output[1][1] = @2 */
  if (res[1]) res[1][1] = w2;
  /* #137: output[1][2] = @15 */
  if (res[1]) res[1][2] = w15;
  /* #138: output[1][3] = @14 */
  if (res[1]) res[1][3] = w14;
  /* #139: @21 = zeros(13x1) */
  casadi_clear(w21, 13);
  /* #140: @1 = @1' */
  /* #141: @29 = zeros(1x3) */
  casadi_clear(w29, 3);
  /* #142: @20 = @20' */
  /* #143: @32 = @23' */
  for (i=0, rr=w32, cs=w23; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #144: @29 = mac(@20,@32,@29) */
  for (i=0, rr=w29; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w32+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #145: @29 = @29' */
  /* #146: @1 = (@1+@29) */
  for (i=0, rr=w1, cs=w29; i<3; ++i) (*rr++) += (*cs++);
  /* #147: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<3; ++i) *rr++ = (- *cs++ );
  /* #148: (@21[:3] += @1) */
  for (rr=w21+0, ss=w1; rr!=w21+3; rr+=1) *rr += *ss++;
  /* #149: {@14, @15, @2, @0, @12, @7, @16, @17, @18, @33, @34, @35, @36} = vertsplit(@21) */
  w14 = w21[0];
  w15 = w21[1];
  w2 = w21[2];
  w0 = w21[3];
  w12 = w21[4];
  w7 = w21[5];
  w16 = w21[6];
  w17 = w21[7];
  w18 = w21[8];
  w33 = w21[9];
  w34 = w21[10];
  w35 = w21[11];
  w36 = w21[12];
  /* #150: output[1][4] = @14 */
  if (res[1]) res[1][4] = w14;
  /* #151: output[1][5] = @15 */
  if (res[1]) res[1][5] = w15;
  /* #152: output[1][6] = @2 */
  if (res[1]) res[1][6] = w2;
  /* #153: output[1][7] = @0 */
  if (res[1]) res[1][7] = w0;
  /* #154: output[1][8] = @12 */
  if (res[1]) res[1][8] = w12;
  /* #155: output[1][9] = @7 */
  if (res[1]) res[1][9] = w7;
  /* #156: @22 = @22' */
  /* #157: @1 = zeros(1x3) */
  casadi_clear(w1, 3);
  /* #158: @29 = @28' */
  casadi_copy(w28, 3, w29);
  /* #159: @32 = @30' */
  for (i=0, rr=w32, cs=w30; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #160: @1 = mac(@29,@32,@1) */
  for (i=0, rr=w1; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w29+j, tt=w32+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #161: @1 = @1' */
  /* #162: @22 = (@22+@1) */
  for (i=0, rr=w22, cs=w1; i<3; ++i) (*rr++) += (*cs++);
  /* #163: @1 = (@22/@5) */
  for (i=0, rr=w1, cr=w22; i<3; ++i) (*rr++)  = ((*cr++)/w5);
  /* #164: {@7, @12, @0} = vertsplit(@1) */
  w7 = w1[0];
  w12 = w1[1];
  w0 = w1[2];
  /* #165: @2 = (@3*@12) */
  w2  = (w3*w12);
  /* #166: @15 = (@13*@7) */
  w15  = (w13*w7);
  /* #167: @2 = (@2-@15) */
  w2 -= w15;
  /* #168: @15 = (@10*@2) */
  w15  = (w10*w2);
  /* #169: @15 = (-@15) */
  w15 = (- w15 );
  /* #170: @14 = (2.*@13) */
  w14 = (2.* w13 );
  /* #171: @28 = (@28/@5) */
  for (i=0, rr=w28; i<3; ++i) (*rr++) /= w5;
  /* #172: @28 = (-@28) */
  for (i=0, rr=w28, cs=w28; i<3; ++i) *rr++ = (- *cs++ );
  /* #173: @37 = dot(@28, @22) */
  w37 = casadi_dot(3, w28, w22);
  /* #174: @5 = (2.*@5) */
  w5 = (2.* w5 );
  /* #175: @37 = (@37/@5) */
  w37 /= w5;
  /* #176: @14 = (@14*@37) */
  w14 *= w37;
  /* #177: @14 = (@14+@0) */
  w14 += w0;
  /* #178: @0 = (@4*@12) */
  w0  = (w4*w12);
  /* #179: @14 = (@14+@0) */
  w14 += w0;
  /* #180: @0 = (@6*@7) */
  w0  = (w6*w7);
  /* #181: @14 = (@14-@0) */
  w14 -= w0;
  /* #182: @0 = (@11*@14) */
  w0  = (w11*w14);
  /* #183: @15 = (@15-@0) */
  w15 -= w0;
  /* #184: @13 = (@13*@12) */
  w13 *= w12;
  /* #185: @0 = (@3*@7) */
  w0  = (w3*w7);
  /* #186: @13 = (@13+@0) */
  w13 += w0;
  /* #187: @0 = (@9*@13) */
  w0  = (w9*w13);
  /* #188: @15 = (@15-@0) */
  w15 -= w0;
  /* #189: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #190: @3 = (@3*@37) */
  w3 *= w37;
  /* #191: @6 = (@6*@12) */
  w6 *= w12;
  /* #192: @3 = (@3+@6) */
  w3 += w6;
  /* #193: @4 = (@4*@7) */
  w4 *= w7;
  /* #194: @3 = (@3+@4) */
  w3 += w4;
  /* #195: @4 = (@8*@3) */
  w4  = (w8*w3);
  /* #196: @15 = (@15+@4) */
  w15 += w4;
  /* #197: @15 = (@15+@16) */
  w15 += w16;
  /* #198: output[1][10] = @15 */
  if (res[1]) res[1][10] = w15;
  /* #199: @15 = (@10*@14) */
  w15  = (w10*w14);
  /* #200: @16 = (@11*@2) */
  w16  = (w11*w2);
  /* #201: @15 = (@15-@16) */
  w15 -= w16;
  /* #202: @16 = (@8*@13) */
  w16  = (w8*w13);
  /* #203: @15 = (@15+@16) */
  w15 += w16;
  /* #204: @16 = (@9*@3) */
  w16  = (w9*w3);
  /* #205: @15 = (@15+@16) */
  w15 += w16;
  /* #206: @15 = (@15+@17) */
  w15 += w17;
  /* #207: output[1][11] = @15 */
  if (res[1]) res[1][11] = w15;
  /* #208: @15 = (@8*@2) */
  w15  = (w8*w2);
  /* #209: @17 = (@9*@14) */
  w17  = (w9*w14);
  /* #210: @15 = (@15-@17) */
  w15 -= w17;
  /* #211: @17 = (@11*@13) */
  w17  = (w11*w13);
  /* #212: @15 = (@15+@17) */
  w15 += w17;
  /* #213: @17 = (@10*@3) */
  w17  = (w10*w3);
  /* #214: @15 = (@15+@17) */
  w15 += w17;
  /* #215: @15 = (@15+@18) */
  w15 += w18;
  /* #216: output[1][12] = @15 */
  if (res[1]) res[1][12] = w15;
  /* #217: @8 = (@8*@14) */
  w8 *= w14;
  /* #218: @9 = (@9*@2) */
  w9 *= w2;
  /* #219: @8 = (@8+@9) */
  w8 += w9;
  /* #220: @10 = (@10*@13) */
  w10 *= w13;
  /* #221: @8 = (@8-@10) */
  w8 -= w10;
  /* #222: @11 = (@11*@3) */
  w11 *= w3;
  /* #223: @8 = (@8+@11) */
  w8 += w11;
  /* #224: @8 = (@8+@33) */
  w8 += w33;
  /* #225: output[1][13] = @8 */
  if (res[1]) res[1][13] = w8;
  /* #226: output[1][14] = @34 */
  if (res[1]) res[1][14] = w34;
  /* #227: output[1][15] = @35 */
  if (res[1]) res[1][15] = w35;
  /* #228: output[1][16] = @36 */
  if (res[1]) res[1][16] = w36;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 21;
  if (sz_res) *sz_res = 15;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 140;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
