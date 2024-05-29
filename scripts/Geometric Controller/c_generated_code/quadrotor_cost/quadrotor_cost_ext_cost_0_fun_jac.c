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

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

/* quadrotor_cost_ext_cost_0_fun_jac:(i0[7],i1[3],i2[],i3[10])->(o0,o1[10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real w0, w1, *w2=w+6, *w3=w+10, w4, w5, w6, w7, *w8=w+18, w9, w10, *w11=w+24, w12, *w13=w+29, *w14=w+33, *w15=w+37, *w16=w+53, w17, w18, w19, w20, *w21=w+73, *w22=w+80, *w23=w+84, *w24=w+100, *w25=w+110, *w26=w+113, *w27=w+117, *w28=w+121, *w29=w+125, *w30=w+141, *w31=w+144, *w32=w+147, *w33=w+150, *w34=w+159, *w35=w+168, w36, w37, w38, w39, w40, w41, w42, w43, w44, w45, w46, w47, w48;
  /* #0: @0 = 1000 */
  w0 = 1000.;
  /* #1: @1 = 0 */
  w1 = 0.;
  /* #2: @2 = zeros(1x4) */
  casadi_clear(w2, 4);
  /* #3: @3 = zeros(4x1) */
  casadi_clear(w3, 4);
  /* #4: @4 = input[3][0] */
  w4 = arg[3] ? arg[3][0] : 0;
  /* #5: @5 = input[3][1] */
  w5 = arg[3] ? arg[3][1] : 0;
  /* #6: @6 = input[3][2] */
  w6 = arg[3] ? arg[3][2] : 0;
  /* #7: @7 = input[3][3] */
  w7 = arg[3] ? arg[3][3] : 0;
  /* #8: @8 = horzcat(@4, @5, @6, @7) */
  rr=w8;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  /* #9: @8 = @8' */
  /* #10: @9 = (-@5) */
  w9 = (- w5 );
  /* #11: @10 = (-@6) */
  w10 = (- w6 );
  /* #12: @11 = horzcat(@9, @4, @7, @10) */
  rr=w11;
  *rr++ = w9;
  *rr++ = w4;
  *rr++ = w7;
  *rr++ = w10;
  /* #13: @11 = @11' */
  /* #14: @12 = (-@7) */
  w12 = (- w7 );
  /* #15: @13 = horzcat(@10, @12, @4, @5) */
  rr=w13;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w4;
  *rr++ = w5;
  /* #16: @13 = @13' */
  /* #17: @14 = horzcat(@12, @6, @9, @4) */
  rr=w14;
  *rr++ = w12;
  *rr++ = w6;
  *rr++ = w9;
  *rr++ = w4;
  /* #18: @14 = @14' */
  /* #19: @15 = horzcat(@8, @11, @13, @14) */
  rr=w15;
  for (i=0, cs=w8; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  /* #20: @16 = @15' */
  for (i=0, rr=w16, cs=w15; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #21: @12 = input[0][0] */
  w12 = arg[0] ? arg[0][0] : 0;
  /* #22: @9 = input[0][1] */
  w9 = arg[0] ? arg[0][1] : 0;
  /* #23: @10 = input[0][2] */
  w10 = arg[0] ? arg[0][2] : 0;
  /* #24: @17 = input[0][3] */
  w17 = arg[0] ? arg[0][3] : 0;
  /* #25: @18 = input[0][4] */
  w18 = arg[0] ? arg[0][4] : 0;
  /* #26: @19 = input[0][5] */
  w19 = arg[0] ? arg[0][5] : 0;
  /* #27: @20 = input[0][6] */
  w20 = arg[0] ? arg[0][6] : 0;
  /* #28: @21 = vertcat(@12, @9, @10, @17, @18, @19, @20) */
  rr=w21;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w17;
  *rr++ = w18;
  *rr++ = w19;
  *rr++ = w20;
  /* #29: @8 = @21[:4] */
  for (rr=w8, ss=w21+0; ss!=w21+4; ss+=1) *rr++ = *ss;
  /* #30: @3 = mac(@16,@8,@3) */
  for (i=0, rr=w3; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w16+j, tt=w8+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #31: @8 = zeros(4x1) */
  casadi_clear(w8, 4);
  /* #32: @11 = horzcat(@12, @9, @10, @17) */
  rr=w11;
  *rr++ = w12;
  *rr++ = w9;
  *rr++ = w10;
  *rr++ = w17;
  /* #33: @11 = @11' */
  /* #34: @18 = (-@9) */
  w18 = (- w9 );
  /* #35: @19 = (-@10) */
  w19 = (- w10 );
  /* #36: @13 = horzcat(@18, @12, @17, @19) */
  rr=w13;
  *rr++ = w18;
  *rr++ = w12;
  *rr++ = w17;
  *rr++ = w19;
  /* #37: @13 = @13' */
  /* #38: @17 = (-@17) */
  w17 = (- w17 );
  /* #39: @14 = horzcat(@19, @17, @12, @9) */
  rr=w14;
  *rr++ = w19;
  *rr++ = w17;
  *rr++ = w12;
  *rr++ = w9;
  /* #40: @14 = @14' */
  /* #41: @22 = horzcat(@17, @10, @18, @12) */
  rr=w22;
  *rr++ = w17;
  *rr++ = w10;
  *rr++ = w18;
  *rr++ = w12;
  /* #42: @22 = @22' */
  /* #43: @16 = horzcat(@11, @13, @14, @22) */
  rr=w16;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<4; ++i) *rr++ = *cs++;
  /* #44: @23 = @16' */
  for (i=0, rr=w23, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #45: @17 = input[3][4] */
  w17 = arg[3] ? arg[3][4] : 0;
  /* #46: @10 = input[3][5] */
  w10 = arg[3] ? arg[3][5] : 0;
  /* #47: @18 = input[3][6] */
  w18 = arg[3] ? arg[3][6] : 0;
  /* #48: @12 = input[3][7] */
  w12 = arg[3] ? arg[3][7] : 0;
  /* #49: @19 = input[3][8] */
  w19 = arg[3] ? arg[3][8] : 0;
  /* #50: @9 = input[3][9] */
  w9 = arg[3] ? arg[3][9] : 0;
  /* #51: @24 = vertcat(@4, @5, @6, @7, @17, @10, @18, @12, @19, @9) */
  rr=w24;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w17;
  *rr++ = w10;
  *rr++ = w18;
  *rr++ = w12;
  *rr++ = w19;
  *rr++ = w9;
  /* #52: @11 = @24[:4] */
  for (rr=w11, ss=w24+0; ss!=w24+4; ss+=1) *rr++ = *ss;
  /* #53: @8 = mac(@23,@11,@8) */
  for (i=0, rr=w8; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w23+j, tt=w11+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #54: @8 = (@3-@8) */
  for (i=0, rr=w8, cr=w3, cs=w8; i<4; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #55: @13 = @8' */
  casadi_copy(w8, 4, w13);
  /* #56: @23 = zeros(4x4) */
  casadi_clear(w23, 16);
  /* #57: @4 = 2 */
  w4 = 2.;
  /* #58: (@23[0] = @4) */
  for (rr=w23+0, ss=(&w4); rr!=w23+1; rr+=1) *rr = *ss++;
  /* #59: @4 = 2 */
  w4 = 2.;
  /* #60: (@23[10] = @4) */
  for (rr=w23+10, ss=(&w4); rr!=w23+11; rr+=1) *rr = *ss++;
  /* #61: @4 = 2 */
  w4 = 2.;
  /* #62: (@23[15] = @4) */
  for (rr=w23+15, ss=(&w4); rr!=w23+16; rr+=1) *rr = *ss++;
  /* #63: @4 = 2 */
  w4 = 2.;
  /* #64: (@23[5] = @4) */
  for (rr=w23+5, ss=(&w4); rr!=w23+6; rr+=1) *rr = *ss++;
  /* #65: @2 = mac(@13,@23,@2) */
  for (i=0, rr=w2; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w13+j, tt=w23+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #66: @1 = mac(@2,@8,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w8+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #67: @0 = (@0*@1) */
  w0 *= w1;
  /* #68: @1 = 10 */
  w1 = 10.;
  /* #69: @4 = 0 */
  w4 = 0.;
  /* #70: @25 = @21[4:7] */
  for (rr=w25, ss=w21+4; ss!=w21+7; ss+=1) *rr++ = *ss;
  /* #71: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #72: @14 = zeros(4x1) */
  casadi_clear(w14, 4);
  /* #73: @5 = @3[0] */
  for (rr=(&w5), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #74: @6 = @3[1] */
  for (rr=(&w6), ss=w3+1; ss!=w3+2; ss+=1) *rr++ = *ss;
  /* #75: @7 = @3[2] */
  for (rr=(&w7), ss=w3+2; ss!=w3+3; ss+=1) *rr++ = *ss;
  /* #76: @17 = @3[3] */
  for (rr=(&w17), ss=w3+3; ss!=w3+4; ss+=1) *rr++ = *ss;
  /* #77: @22 = horzcat(@5, @6, @7, @17) */
  rr=w22;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w17;
  /* #78: @22 = @22' */
  /* #79: @5 = (-@6) */
  w5 = (- w6 );
  /* #80: @10 = @3[0] */
  for (rr=(&w10), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #81: @18 = (-@7) */
  w18 = (- w7 );
  /* #82: @26 = horzcat(@5, @10, @17, @18) */
  rr=w26;
  *rr++ = w5;
  *rr++ = w10;
  *rr++ = w17;
  *rr++ = w18;
  /* #83: @26 = @26' */
  /* #84: @10 = (-@17) */
  w10 = (- w17 );
  /* #85: @12 = @3[0] */
  for (rr=(&w12), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #86: @27 = horzcat(@18, @10, @12, @6) */
  rr=w27;
  *rr++ = w18;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w6;
  /* #87: @27 = @27' */
  /* #88: @18 = @3[0] */
  for (rr=(&w18), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #89: @28 = horzcat(@10, @7, @5, @18) */
  rr=w28;
  *rr++ = w10;
  *rr++ = w7;
  *rr++ = w5;
  *rr++ = w18;
  /* #90: @28 = @28' */
  /* #91: @16 = horzcat(@22, @26, @27, @28) */
  rr=w16;
  for (i=0, cs=w22; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w27; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  /* #92: @29 = @16' */
  for (i=0, rr=w29, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #93: @10 = 0 */
  w10 = 0.;
  /* #94: @30 = @24[4:7] */
  for (rr=w30, ss=w24+4; ss!=w24+7; ss+=1) *rr++ = *ss;
  /* #95: @22 = vertcat(@10, @30) */
  rr=w22;
  *rr++ = w10;
  for (i=0, cs=w30; i<3; ++i) *rr++ = *cs++;
  /* #96: @14 = mac(@29,@22,@14) */
  for (i=0, rr=w14; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w29+j, tt=w22+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #97: @10 = @14[0] */
  for (rr=(&w10), ss=w14+0; ss!=w14+1; ss+=1) *rr++ = *ss;
  /* #98: @5 = @14[1] */
  for (rr=(&w5), ss=w14+1; ss!=w14+2; ss+=1) *rr++ = *ss;
  /* #99: @5 = (-@5) */
  w5 = (- w5 );
  /* #100: @18 = @14[2] */
  for (rr=(&w18), ss=w14+2; ss!=w14+3; ss+=1) *rr++ = *ss;
  /* #101: @18 = (-@18) */
  w18 = (- w18 );
  /* #102: @12 = @14[3] */
  for (rr=(&w12), ss=w14+3; ss!=w14+4; ss+=1) *rr++ = *ss;
  /* #103: @12 = (-@12) */
  w12 = (- w12 );
  /* #104: @26 = horzcat(@10, @5, @18, @12) */
  rr=w26;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #105: @26 = @26' */
  /* #106: @10 = @14[1] */
  for (rr=(&w10), ss=w14+1; ss!=w14+2; ss+=1) *rr++ = *ss;
  /* #107: @5 = @14[0] */
  for (rr=(&w5), ss=w14+0; ss!=w14+1; ss+=1) *rr++ = *ss;
  /* #108: @18 = @14[3] */
  for (rr=(&w18), ss=w14+3; ss!=w14+4; ss+=1) *rr++ = *ss;
  /* #109: @18 = (-@18) */
  w18 = (- w18 );
  /* #110: @12 = @14[2] */
  for (rr=(&w12), ss=w14+2; ss!=w14+3; ss+=1) *rr++ = *ss;
  /* #111: @27 = horzcat(@10, @5, @18, @12) */
  rr=w27;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #112: @27 = @27' */
  /* #113: @10 = @14[2] */
  for (rr=(&w10), ss=w14+2; ss!=w14+3; ss+=1) *rr++ = *ss;
  /* #114: @5 = @14[3] */
  for (rr=(&w5), ss=w14+3; ss!=w14+4; ss+=1) *rr++ = *ss;
  /* #115: @18 = @14[0] */
  for (rr=(&w18), ss=w14+0; ss!=w14+1; ss+=1) *rr++ = *ss;
  /* #116: @12 = @14[1] */
  for (rr=(&w12), ss=w14+1; ss!=w14+2; ss+=1) *rr++ = *ss;
  /* #117: @12 = (-@12) */
  w12 = (- w12 );
  /* #118: @28 = horzcat(@10, @5, @18, @12) */
  rr=w28;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #119: @28 = @28' */
  /* #120: @10 = @14[3] */
  for (rr=(&w10), ss=w14+3; ss!=w14+4; ss+=1) *rr++ = *ss;
  /* #121: @5 = @14[2] */
  for (rr=(&w5), ss=w14+2; ss!=w14+3; ss+=1) *rr++ = *ss;
  /* #122: @5 = (-@5) */
  w5 = (- w5 );
  /* #123: @18 = @14[1] */
  for (rr=(&w18), ss=w14+1; ss!=w14+2; ss+=1) *rr++ = *ss;
  /* #124: @12 = @14[0] */
  for (rr=(&w12), ss=w14+0; ss!=w14+1; ss+=1) *rr++ = *ss;
  /* #125: @14 = horzcat(@10, @5, @18, @12) */
  rr=w14;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #126: @14 = @14' */
  /* #127: @29 = horzcat(@26, @27, @28, @14) */
  rr=w29;
  for (i=0, cs=w26; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w27; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w28; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  /* #128: @16 = @29' */
  for (i=0, rr=w16, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #129: @10 = @3[0] */
  for (rr=(&w10), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #130: @3 = vertcat(@10, @6, @7, @17) */
  rr=w3;
  *rr++ = w10;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w17;
  /* #131: @13 = mac(@16,@3,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w16+j, tt=w3+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #132: @30 = @13[1:4] */
  for (rr=w30, ss=w13+1; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #133: @25 = (@25-@30) */
  for (i=0, rr=w25, cs=w30; i<3; ++i) (*rr++) -= (*cs++);
  /* #134: @30 = @25' */
  casadi_copy(w25, 3, w30);
  /* #135: @4 = mac(@30,@25,@4) */
  for (i=0, rr=(&w4); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w30+j, tt=w25+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #136: @1 = (@1*@4) */
  w1 *= w4;
  /* #137: @0 = (@0+@1) */
  w0 += w1;
  /* #138: @1 = 0 */
  w1 = 0.;
  /* #139: @30 = zeros(1x3) */
  casadi_clear(w30, 3);
  /* #140: @31 = @24[7:10] */
  for (rr=w31, ss=w24+7; ss!=w24+10; ss+=1) *rr++ = *ss;
  /* #141: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #142: @10 = input[1][1] */
  w10 = arg[1] ? arg[1][1] : 0;
  /* #143: @6 = input[1][2] */
  w6 = arg[1] ? arg[1][2] : 0;
  /* #144: @32 = vertcat(@4, @10, @6) */
  rr=w32;
  *rr++ = w4;
  *rr++ = w10;
  *rr++ = w6;
  /* #145: @31 = (@31-@32) */
  for (i=0, rr=w31, cs=w32; i<3; ++i) (*rr++) -= (*cs++);
  /* #146: @32 = @31' */
  casadi_copy(w31, 3, w32);
  /* #147: @33 = zeros(3x3) */
  casadi_clear(w33, 9);
  /* #148: @4 = 0.000166667 */
  w4 = 1.6666666666666669e-04;
  /* #149: (@33[0] = @4) */
  for (rr=w33+0, ss=(&w4); rr!=w33+1; rr+=1) *rr = *ss++;
  /* #150: @4 = 0.000166667 */
  w4 = 1.6666666666666669e-04;
  /* #151: (@33[4] = @4) */
  for (rr=w33+4, ss=(&w4); rr!=w33+5; rr+=1) *rr = *ss++;
  /* #152: @4 = 0.000166667 */
  w4 = 1.6666666666666669e-04;
  /* #153: (@33[8] = @4) */
  for (rr=w33+8, ss=(&w4); rr!=w33+9; rr+=1) *rr = *ss++;
  /* #154: @30 = mac(@32,@33,@30) */
  for (i=0, rr=w30; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w32+j, tt=w33+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #155: @1 = mac(@30,@31,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w30+j, tt=w31+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #156: @0 = (@0+@1) */
  w0 += w1;
  /* #157: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #158: @30 = @30' */
  /* #159: @32 = zeros(1x3) */
  casadi_clear(w32, 3);
  /* #160: @31 = @31' */
  /* #161: @34 = @33' */
  for (i=0, rr=w34, cs=w33; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #162: @32 = mac(@31,@34,@32) */
  for (i=0, rr=w32; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w31+j, tt=w34+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #163: @32 = @32' */
  /* #164: @30 = (@30+@32) */
  for (i=0, rr=w30, cs=w32; i<3; ++i) (*rr++) += (*cs++);
  /* #165: @30 = (-@30) */
  for (i=0, rr=w30, cs=w30; i<3; ++i) *rr++ = (- *cs++ );
  /* #166: {@0, @1, @4} = vertsplit(@30) */
  w0 = w30[0];
  w1 = w30[1];
  w4 = w30[2];
  /* #167: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #168: output[1][1] = @1 */
  if (res[1]) res[1][1] = w1;
  /* #169: output[1][2] = @4 */
  if (res[1]) res[1][2] = w4;
  /* #170: @16 = zeros(4x4) */
  casadi_clear(w16, 16);
  /* #171: @4 = 1000 */
  w4 = 1000.;
  /* #172: @2 = @2' */
  /* #173: @2 = (@4*@2) */
  for (i=0, rr=w2, cs=w2; i<4; ++i) (*rr++)  = (w4*(*cs++));
  /* #174: @13 = zeros(1x4) */
  casadi_clear(w13, 4);
  /* #175: @8 = @8' */
  /* #176: @8 = (@4*@8) */
  for (i=0, rr=w8, cs=w8; i<4; ++i) (*rr++)  = (w4*(*cs++));
  /* #177: @35 = @23' */
  for (i=0, rr=w35, cs=w23; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #178: @13 = mac(@8,@35,@13) */
  for (i=0, rr=w13; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w8+j, tt=w35+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #179: @13 = @13' */
  /* #180: @2 = (@2+@13) */
  for (i=0, rr=w2, cs=w13; i<4; ++i) (*rr++) += (*cs++);
  /* #181: @13 = (-@2) */
  for (i=0, rr=w13, cs=w2; i<4; ++i) *rr++ = (- *cs++ );
  /* #182: @11 = @11' */
  /* #183: @16 = mac(@13,@11,@16) */
  for (i=0, rr=w16; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w13+j, tt=w11+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #184: @35 = @16' */
  for (i=0, rr=w35, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #185: {@13, @11, @8, @26} = horzsplit(@35) */
  casadi_copy(w35, 4, w13);
  casadi_copy(w35+4, 4, w11);
  casadi_copy(w35+8, 4, w8);
  casadi_copy(w35+12, 4, w26);
  /* #186: @26 = @26' */
  /* #187: {@4, @1, @0, @10} = horzsplit(@26) */
  w4 = w26[0];
  w1 = w26[1];
  w0 = w26[2];
  w10 = w26[3];
  /* #188: @8 = @8' */
  /* #189: {@6, @7, @17, @5} = horzsplit(@8) */
  w6 = w8[0];
  w7 = w8[1];
  w17 = w8[2];
  w5 = w8[3];
  /* #190: @10 = (@10+@17) */
  w10 += w17;
  /* #191: @11 = @11' */
  /* #192: {@17, @18, @12, @19} = horzsplit(@11) */
  w17 = w11[0];
  w18 = w11[1];
  w12 = w11[2];
  w19 = w11[3];
  /* #193: @10 = (@10+@18) */
  w10 += w18;
  /* #194: @13 = @13' */
  /* #195: {@18, @9, @20, @36} = horzsplit(@13) */
  w18 = w13[0];
  w9 = w13[1];
  w20 = w13[2];
  w36 = w13[3];
  /* #196: @10 = (@10+@18) */
  w10 += w18;
  /* #197: @21 = zeros(7x1) */
  casadi_clear(w21, 7);
  /* #198: @18 = 10 */
  w18 = 10.;
  /* #199: @30 = (@18*@25) */
  for (i=0, rr=w30, cs=w25; i<3; ++i) (*rr++)  = (w18*(*cs++));
  /* #200: @25 = @25' */
  /* #201: @25 = (@18*@25) */
  for (i=0, rr=w25, cs=w25; i<3; ++i) (*rr++)  = (w18*(*cs++));
  /* #202: @25 = @25' */
  /* #203: @30 = (@30+@25) */
  for (i=0, rr=w30, cs=w25; i<3; ++i) (*rr++) += (*cs++);
  /* #204: (@21[4:7] += @30) */
  for (rr=w21+4, ss=w30; rr!=w21+7; rr+=1) *rr += *ss++;
  /* #205: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #206: @11 = zeros(4x1) */
  casadi_clear(w11, 4);
  /* #207: @8 = zeros(4x1) */
  casadi_clear(w8, 4);
  /* #208: @26 = zeros(4x1) */
  casadi_clear(w26, 4);
  /* #209: @30 = (-@30) */
  for (i=0, rr=w30, cs=w30; i<3; ++i) *rr++ = (- *cs++ );
  /* #210: (@26[1:4] += @30) */
  for (rr=w26+1, ss=w30; rr!=w26+4; rr+=1) *rr += *ss++;
  /* #211: @8 = mac(@29,@26,@8) */
  for (i=0, rr=w8; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w29+j, tt=w26+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #212: {@18, @37, @38, @39} = vertsplit(@8) */
  w18 = w8[0];
  w37 = w8[1];
  w38 = w8[2];
  w39 = w8[3];
  /* #213: (@11[0] += @18) */
  for (rr=w11+0, ss=(&w18); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #214: @29 = zeros(4x4) */
  casadi_clear(w29, 16);
  /* #215: @8 = zeros(4x1) */
  casadi_clear(w8, 4);
  /* #216: @35 = zeros(4x4) */
  casadi_clear(w35, 16);
  /* #217: @3 = @3' */
  /* #218: @35 = mac(@26,@3,@35) */
  for (i=0, rr=w35; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w26+j, tt=w3+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #219: @16 = @35' */
  for (i=0, rr=w16, cs=w35; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #220: {@26, @3, @27, @28} = horzsplit(@16) */
  casadi_copy(w16, 4, w26);
  casadi_copy(w16+4, 4, w3);
  casadi_copy(w16+8, 4, w27);
  casadi_copy(w16+12, 4, w28);
  /* #221: @28 = @28' */
  /* #222: {@18, @40, @41, @42} = horzsplit(@28) */
  w18 = w28[0];
  w40 = w28[1];
  w41 = w28[2];
  w42 = w28[3];
  /* #223: (@8[0] += @42) */
  for (rr=w8+0, ss=(&w42); rr!=w8+1; rr+=1) *rr += *ss++;
  /* #224: (@8[1] += @41) */
  for (rr=w8+1, ss=(&w41); rr!=w8+2; rr+=1) *rr += *ss++;
  /* #225: @40 = (-@40) */
  w40 = (- w40 );
  /* #226: (@8[2] += @40) */
  for (rr=w8+2, ss=(&w40); rr!=w8+3; rr+=1) *rr += *ss++;
  /* #227: (@8[3] += @18) */
  for (rr=w8+3, ss=(&w18); rr!=w8+4; rr+=1) *rr += *ss++;
  /* #228: @27 = @27' */
  /* #229: {@18, @40, @41, @42} = horzsplit(@27) */
  w18 = w27[0];
  w40 = w27[1];
  w41 = w27[2];
  w42 = w27[3];
  /* #230: @42 = (-@42) */
  w42 = (- w42 );
  /* #231: (@8[1] += @42) */
  for (rr=w8+1, ss=(&w42); rr!=w8+2; rr+=1) *rr += *ss++;
  /* #232: (@8[0] += @41) */
  for (rr=w8+0, ss=(&w41); rr!=w8+1; rr+=1) *rr += *ss++;
  /* #233: (@8[3] += @40) */
  for (rr=w8+3, ss=(&w40); rr!=w8+4; rr+=1) *rr += *ss++;
  /* #234: (@8[2] += @18) */
  for (rr=w8+2, ss=(&w18); rr!=w8+3; rr+=1) *rr += *ss++;
  /* #235: @3 = @3' */
  /* #236: {@18, @40, @41, @42} = horzsplit(@3) */
  w18 = w3[0];
  w40 = w3[1];
  w41 = w3[2];
  w42 = w3[3];
  /* #237: (@8[2] += @42) */
  for (rr=w8+2, ss=(&w42); rr!=w8+3; rr+=1) *rr += *ss++;
  /* #238: @41 = (-@41) */
  w41 = (- w41 );
  /* #239: (@8[3] += @41) */
  for (rr=w8+3, ss=(&w41); rr!=w8+4; rr+=1) *rr += *ss++;
  /* #240: (@8[0] += @40) */
  for (rr=w8+0, ss=(&w40); rr!=w8+1; rr+=1) *rr += *ss++;
  /* #241: (@8[1] += @18) */
  for (rr=w8+1, ss=(&w18); rr!=w8+2; rr+=1) *rr += *ss++;
  /* #242: @26 = @26' */
  /* #243: {@18, @40, @41, @42} = horzsplit(@26) */
  w18 = w26[0];
  w40 = w26[1];
  w41 = w26[2];
  w42 = w26[3];
  /* #244: @42 = (-@42) */
  w42 = (- w42 );
  /* #245: (@8[3] += @42) */
  for (rr=w8+3, ss=(&w42); rr!=w8+4; rr+=1) *rr += *ss++;
  /* #246: @41 = (-@41) */
  w41 = (- w41 );
  /* #247: (@8[2] += @41) */
  for (rr=w8+2, ss=(&w41); rr!=w8+3; rr+=1) *rr += *ss++;
  /* #248: @40 = (-@40) */
  w40 = (- w40 );
  /* #249: (@8[1] += @40) */
  for (rr=w8+1, ss=(&w40); rr!=w8+2; rr+=1) *rr += *ss++;
  /* #250: (@8[0] += @18) */
  for (rr=w8+0, ss=(&w18); rr!=w8+1; rr+=1) *rr += *ss++;
  /* #251: @22 = @22' */
  /* #252: @29 = mac(@8,@22,@29) */
  for (i=0, rr=w29; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w8+j, tt=w22+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #253: @16 = @29' */
  for (i=0, rr=w16, cs=w29; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #254: {@8, @22, @26, @3} = horzsplit(@16) */
  casadi_copy(w16, 4, w8);
  casadi_copy(w16+4, 4, w22);
  casadi_copy(w16+8, 4, w26);
  casadi_copy(w16+12, 4, w3);
  /* #255: @3 = @3' */
  /* #256: {@18, @40, @41, @42} = horzsplit(@3) */
  w18 = w3[0];
  w40 = w3[1];
  w41 = w3[2];
  w42 = w3[3];
  /* #257: (@11[0] += @42) */
  for (rr=w11+0, ss=(&w42); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #258: @26 = @26' */
  /* #259: {@42, @43, @44, @45} = horzsplit(@26) */
  w42 = w26[0];
  w43 = w26[1];
  w44 = w26[2];
  w45 = w26[3];
  /* #260: (@11[0] += @44) */
  for (rr=w11+0, ss=(&w44); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #261: @22 = @22' */
  /* #262: {@44, @46, @47, @48} = horzsplit(@22) */
  w44 = w22[0];
  w46 = w22[1];
  w47 = w22[2];
  w48 = w22[3];
  /* #263: (@11[0] += @46) */
  for (rr=w11+0, ss=(&w46); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #264: @18 = (@18+@43) */
  w18 += w43;
  /* #265: @39 = (@39-@18) */
  w39 -= w18;
  /* #266: @39 = (@39+@47) */
  w39 += w47;
  /* #267: @8 = @8' */
  /* #268: {@47, @18, @43, @46} = horzsplit(@8) */
  w47 = w8[0];
  w18 = w8[1];
  w43 = w8[2];
  w46 = w8[3];
  /* #269: @39 = (@39+@46) */
  w39 += w46;
  /* #270: (@11[3] += @39) */
  for (rr=w11+3, ss=(&w39); rr!=w11+4; rr+=1) *rr += *ss++;
  /* #271: @38 = (@38+@40) */
  w38 += w40;
  /* #272: @42 = (@42+@48) */
  w42 += w48;
  /* #273: @38 = (@38-@42) */
  w38 -= w42;
  /* #274: @38 = (@38+@43) */
  w38 += w43;
  /* #275: (@11[2] += @38) */
  for (rr=w11+2, ss=(&w38); rr!=w11+3; rr+=1) *rr += *ss++;
  /* #276: @37 = (@37+@45) */
  w37 += w45;
  /* #277: @41 = (@41+@44) */
  w41 += w44;
  /* #278: @37 = (@37-@41) */
  w37 -= w41;
  /* #279: @37 = (@37+@18) */
  w37 += w18;
  /* #280: (@11[1] += @37) */
  for (rr=w11+1, ss=(&w37); rr!=w11+2; rr+=1) *rr += *ss++;
  /* #281: (@11[0] += @47) */
  for (rr=w11+0, ss=(&w47); rr!=w11+1; rr+=1) *rr += *ss++;
  /* #282: @11 = (@11+@2) */
  for (i=0, rr=w11, cs=w2; i<4; ++i) (*rr++) += (*cs++);
  /* #283: @13 = mac(@15,@11,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w15+j, tt=w11+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #284: (@21[:4] += @13) */
  for (rr=w21+0, ss=w13; rr!=w21+4; rr+=1) *rr += *ss++;
  /* #285: {@47, @37, @18, @41, @44, @45, @38} = vertsplit(@21) */
  w47 = w21[0];
  w37 = w21[1];
  w18 = w21[2];
  w41 = w21[3];
  w44 = w21[4];
  w45 = w21[5];
  w38 = w21[6];
  /* #286: @10 = (@10+@47) */
  w10 += w47;
  /* #287: output[1][3] = @10 */
  if (res[1]) res[1][3] = w10;
  /* #288: @0 = (@0+@17) */
  w0 += w17;
  /* #289: @5 = (@5-@0) */
  w5 -= w0;
  /* #290: @5 = (@5+@9) */
  w5 += w9;
  /* #291: @5 = (@5+@37) */
  w5 += w37;
  /* #292: output[1][4] = @5 */
  if (res[1]) res[1][4] = w5;
  /* #293: @6 = (@6+@19) */
  w6 += w19;
  /* #294: @1 = (@1-@6) */
  w1 -= w6;
  /* #295: @1 = (@1+@20) */
  w1 += w20;
  /* #296: @1 = (@1+@18) */
  w1 += w18;
  /* #297: output[1][5] = @1 */
  if (res[1]) res[1][5] = w1;
  /* #298: @4 = (@4+@7) */
  w4 += w7;
  /* #299: @12 = (@12-@4) */
  w12 -= w4;
  /* #300: @12 = (@12+@36) */
  w12 += w36;
  /* #301: @12 = (@12+@41) */
  w12 += w41;
  /* #302: output[1][6] = @12 */
  if (res[1]) res[1][6] = w12;
  /* #303: output[1][7] = @44 */
  if (res[1]) res[1][7] = w44;
  /* #304: output[1][8] = @45 */
  if (res[1]) res[1][8] = w45;
  /* #305: output[1][9] = @38 */
  if (res[1]) res[1][9] = w38;
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
  if (sz_arg) *sz_arg = 14;
  if (sz_res) *sz_res = 9;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 197;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
