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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_0_fun_jac_ ## ID
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_0_fun_jac:(i0[13],i1[4],i2[],i3[17])->(o0,o1[17]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, *w1=w+5, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, *w19=w+25, *w20=w+42, *w21=w+45, *w22=w+58, *w23=w+61, *w24=w+70, *w25=w+74, *w26=w+78, *w27=w+82, *w28=w+98, *w29=w+102, *w30=w+106, *w31=w+110, *w32=w+114, *w33=w+130, *w34=w+146, w35, *w36=w+150;
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
  /* #69: @2 = 12 */
  w2 = 12.;
  /* #70: @3 = 0 */
  w3 = 0.;
  /* #71: @4 = 0 */
  w4 = 0.;
  /* #72: @5 = 0.5 */
  w5 = 5.0000000000000000e-01;
  /* #73: @22 = all_2.22045e-16(3x1) */
  casadi_fill(w22, 3, 2.2204460492503131e-16);
  /* #74: @26 = zeros(4x1) */
  casadi_clear(w26, 4);
  /* #75: @28 = horzcat(@8, @9, @10, @11) */
  rr=w28;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w11;
  /* #76: @28 = @28' */
  /* #77: @6 = (-@9) */
  w6 = (- w9 );
  /* #78: @7 = (-@10) */
  w7 = (- w10 );
  /* #79: @29 = horzcat(@6, @8, @11, @7) */
  rr=w29;
  *rr++ = w6;
  *rr++ = w8;
  *rr++ = w11;
  *rr++ = w7;
  /* #80: @29 = @29' */
  /* #81: @11 = (-@11) */
  w11 = (- w11 );
  /* #82: @30 = horzcat(@7, @11, @8, @9) */
  rr=w30;
  *rr++ = w7;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w9;
  /* #83: @30 = @30' */
  /* #84: @31 = horzcat(@11, @10, @6, @8) */
  rr=w31;
  *rr++ = w11;
  *rr++ = w10;
  *rr++ = w6;
  *rr++ = w8;
  /* #85: @31 = @31' */
  /* #86: @32 = horzcat(@28, @29, @30, @31) */
  rr=w32;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w29; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w30; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w31; i<4; ++i) *rr++ = *cs++;
  /* #87: @33 = @32' */
  for (i=0, rr=w33, cs=w32; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #88: @28 = @21[6:10] */
  for (rr=w28, ss=w21+6; ss!=w21+10; ss+=1) *rr++ = *ss;
  /* #89: @26 = mac(@33,@28,@26) */
  for (i=0, rr=w26; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w33+j, tt=w28+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #90: @34 = @26[1:4] */
  for (rr=w34, ss=w26+1; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #91: @22 = (@22+@34) */
  for (i=0, rr=w22, cs=w34; i<3; ++i) (*rr++) += (*cs++);
  /* #92: @11 = ||@22||_F */
  w11 = sqrt(casadi_dot(3, w22, w22));
  /* #93: @10 = @26[0] */
  for (rr=(&w10), ss=w26+0; ss!=w26+1; ss+=1) *rr++ = *ss;
  /* #94: @6 = atan2(@11,@10) */
  w6  = atan2(w11,w10);
  /* #95: @8 = (@5*@6) */
  w8  = (w5*w6);
  /* #96: @7 = @26[1] */
  for (rr=(&w7), ss=w26+1; ss!=w26+2; ss+=1) *rr++ = *ss;
  /* #97: @9 = (@8*@7) */
  w9  = (w8*w7);
  /* #98: @9 = (@9/@11) */
  w9 /= w11;
  /* #99: @12 = 0.5 */
  w12 = 5.0000000000000000e-01;
  /* #100: @13 = (@12*@6) */
  w13  = (w12*w6);
  /* #101: @14 = @26[2] */
  for (rr=(&w14), ss=w26+2; ss!=w26+3; ss+=1) *rr++ = *ss;
  /* #102: @15 = (@13*@14) */
  w15  = (w13*w14);
  /* #103: @15 = (@15/@11) */
  w15 /= w11;
  /* #104: @16 = 0.5 */
  w16 = 5.0000000000000000e-01;
  /* #105: @6 = (@16*@6) */
  w6  = (w16*w6);
  /* #106: @17 = @26[3] */
  for (rr=(&w17), ss=w26+3; ss!=w26+4; ss+=1) *rr++ = *ss;
  /* #107: @18 = (@6*@17) */
  w18  = (w6*w17);
  /* #108: @18 = (@18/@11) */
  w18 /= w11;
  /* #109: @26 = vertcat(@4, @9, @15, @18) */
  rr=w26;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w15;
  *rr++ = w18;
  /* #110: @28 = @26' */
  casadi_copy(w26, 4, w28);
  /* #111: @3 = mac(@28,@26,@3) */
  for (i=0, rr=(&w3); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w28+j, tt=w26+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #112: @2 = (@2*@3) */
  w2 *= w3;
  /* #113: @0 = (@0+@2) */
  w0 += w2;
  /* #114: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #115: @24 = @24' */
  /* #116: @28 = zeros(1x4) */
  casadi_clear(w28, 4);
  /* #117: @25 = @25' */
  /* #118: @33 = @27' */
  for (i=0, rr=w33, cs=w27; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #119: @28 = mac(@25,@33,@28) */
  for (i=0, rr=w28; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w25+j, tt=w33+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #120: @28 = @28' */
  /* #121: @24 = (@24+@28) */
  for (i=0, rr=w24, cs=w28; i<4; ++i) (*rr++) += (*cs++);
  /* #122: @24 = (-@24) */
  for (i=0, rr=w24, cs=w24; i<4; ++i) *rr++ = (- *cs++ );
  /* #123: {@0, @2, @3, @4} = vertsplit(@24) */
  w0 = w24[0];
  w2 = w24[1];
  w3 = w24[2];
  w4 = w24[3];
  /* #124: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #125: output[1][1] = @2 */
  if (res[1]) res[1][1] = w2;
  /* #126: output[1][2] = @3 */
  if (res[1]) res[1][2] = w3;
  /* #127: output[1][3] = @4 */
  if (res[1]) res[1][3] = w4;
  /* #128: @21 = zeros(13x1) */
  casadi_clear(w21, 13);
  /* #129: @24 = zeros(4x1) */
  casadi_clear(w24, 4);
  /* #130: @28 = zeros(4x1) */
  casadi_clear(w28, 4);
  /* #131: @4 = 12 */
  w4 = 12.;
  /* #132: @25 = (@4*@26) */
  for (i=0, rr=w25, cs=w26; i<4; ++i) (*rr++)  = (w4*(*cs++));
  /* #133: @26 = @26' */
  /* #134: @26 = (@4*@26) */
  for (i=0, rr=w26, cs=w26; i<4; ++i) (*rr++)  = (w4*(*cs++));
  /* #135: @26 = @26' */
  /* #136: @25 = (@25+@26) */
  for (i=0, rr=w25, cs=w26; i<4; ++i) (*rr++) += (*cs++);
  /* #137: {NULL, @4, @3, @2} = vertsplit(@25) */
  w4 = w25[1];
  w3 = w25[2];
  w2 = w25[3];
  /* #138: @0 = (@2/@11) */
  w0  = (w2/w11);
  /* #139: @6 = (@6*@0) */
  w6 *= w0;
  /* #140: (@28[3] += @6) */
  for (rr=w28+3, ss=(&w6); rr!=w28+4; rr+=1) *rr += *ss++;
  /* #141: @6 = (@3/@11) */
  w6  = (w3/w11);
  /* #142: @13 = (@13*@6) */
  w13 *= w6;
  /* #143: (@28[2] += @13) */
  for (rr=w28+2, ss=(&w13); rr!=w28+3; rr+=1) *rr += *ss++;
  /* #144: @13 = (@4/@11) */
  w13  = (w4/w11);
  /* #145: @8 = (@8*@13) */
  w8 *= w13;
  /* #146: (@28[1] += @8) */
  for (rr=w28+1, ss=(&w8); rr!=w28+2; rr+=1) *rr += *ss++;
  /* #147: @8 = sq(@11) */
  w8 = casadi_sq( w11 );
  /* #148: @35 = sq(@10) */
  w35 = casadi_sq( w10 );
  /* #149: @8 = (@8+@35) */
  w8 += w35;
  /* #150: @35 = (@11/@8) */
  w35  = (w11/w8);
  /* #151: @17 = (@17*@0) */
  w17 *= w0;
  /* #152: @16 = (@16*@17) */
  w16 *= w17;
  /* #153: @14 = (@14*@6) */
  w14 *= w6;
  /* #154: @12 = (@12*@14) */
  w12 *= w14;
  /* #155: @16 = (@16+@12) */
  w16 += w12;
  /* #156: @7 = (@7*@13) */
  w7 *= w13;
  /* #157: @5 = (@5*@7) */
  w5 *= w7;
  /* #158: @16 = (@16+@5) */
  w16 += w5;
  /* #159: @35 = (@35*@16) */
  w35 *= w16;
  /* #160: @35 = (-@35) */
  w35 = (- w35 );
  /* #161: (@28[0] += @35) */
  for (rr=w28+0, ss=(&w35); rr!=w28+1; rr+=1) *rr += *ss++;
  /* #162: @15 = (@15/@11) */
  w15 /= w11;
  /* #163: @15 = (@15*@3) */
  w15 *= w3;
  /* #164: @15 = (-@15) */
  w15 = (- w15 );
  /* #165: @18 = (@18/@11) */
  w18 /= w11;
  /* #166: @18 = (@18*@2) */
  w18 *= w2;
  /* #167: @15 = (@15-@18) */
  w15 -= w18;
  /* #168: @9 = (@9/@11) */
  w9 /= w11;
  /* #169: @9 = (@9*@4) */
  w9 *= w4;
  /* #170: @15 = (@15-@9) */
  w15 -= w9;
  /* #171: @10 = (@10/@8) */
  w10 /= w8;
  /* #172: @10 = (@10*@16) */
  w10 *= w16;
  /* #173: @15 = (@15+@10) */
  w15 += w10;
  /* #174: @15 = (@15/@11) */
  w15 /= w11;
  /* #175: @22 = (@15*@22) */
  for (i=0, rr=w22, cs=w22; i<3; ++i) (*rr++)  = (w15*(*cs++));
  /* #176: (@28[1:4] += @22) */
  for (rr=w28+1, ss=w22; rr!=w28+4; rr+=1) *rr += *ss++;
  /* #177: @24 = mac(@32,@28,@24) */
  for (i=0, rr=w24; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w32+j, tt=w28+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #178: (@21[6:10] += @24) */
  for (rr=w21+6, ss=w24; rr!=w21+10; rr+=1) *rr += *ss++;
  /* #179: @1 = @1' */
  /* #180: @22 = zeros(1x3) */
  casadi_clear(w22, 3);
  /* #181: @20 = @20' */
  /* #182: @36 = @23' */
  for (i=0, rr=w36, cs=w23; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #183: @22 = mac(@20,@36,@22) */
  for (i=0, rr=w22; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w20+j, tt=w36+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #184: @22 = @22' */
  /* #185: @1 = (@1+@22) */
  for (i=0, rr=w1, cs=w22; i<3; ++i) (*rr++) += (*cs++);
  /* #186: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<3; ++i) *rr++ = (- *cs++ );
  /* #187: (@21[:3] += @1) */
  for (rr=w21+0, ss=w1; rr!=w21+3; rr+=1) *rr += *ss++;
  /* #188: {@15, @11, @10, @16, @8, @9, @4, @18, @2, @3, @35, @5, @7} = vertsplit(@21) */
  w15 = w21[0];
  w11 = w21[1];
  w10 = w21[2];
  w16 = w21[3];
  w8 = w21[4];
  w9 = w21[5];
  w4 = w21[6];
  w18 = w21[7];
  w2 = w21[8];
  w3 = w21[9];
  w35 = w21[10];
  w5 = w21[11];
  w7 = w21[12];
  /* #189: output[1][4] = @15 */
  if (res[1]) res[1][4] = w15;
  /* #190: output[1][5] = @11 */
  if (res[1]) res[1][5] = w11;
  /* #191: output[1][6] = @10 */
  if (res[1]) res[1][6] = w10;
  /* #192: output[1][7] = @16 */
  if (res[1]) res[1][7] = w16;
  /* #193: output[1][8] = @8 */
  if (res[1]) res[1][8] = w8;
  /* #194: output[1][9] = @9 */
  if (res[1]) res[1][9] = w9;
  /* #195: output[1][10] = @4 */
  if (res[1]) res[1][10] = w4;
  /* #196: output[1][11] = @18 */
  if (res[1]) res[1][11] = w18;
  /* #197: output[1][12] = @2 */
  if (res[1]) res[1][12] = w2;
  /* #198: output[1][13] = @3 */
  if (res[1]) res[1][13] = w3;
  /* #199: output[1][14] = @35 */
  if (res[1]) res[1][14] = w35;
  /* #200: output[1][15] = @5 */
  if (res[1]) res[1][15] = w5;
  /* #201: output[1][16] = @7 */
  if (res[1]) res[1][16] = w7;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_0_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_0_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_0_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_0_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_0_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_0_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_0_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_0_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_0_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 21;
  if (sz_res) *sz_res = 15;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 159;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
