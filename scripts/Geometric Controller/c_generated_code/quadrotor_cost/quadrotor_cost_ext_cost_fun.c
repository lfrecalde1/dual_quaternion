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
  #define CASADI_PREFIX(ID) quadrotor_cost_ext_cost_fun_ ## ID
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

/* quadrotor_cost_ext_cost_fun:(i0[7],i1[3],i2[],i3[10])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cr, *cs;
  casadi_real w0, w1, *w2=w+6, *w3=w+10, w4, w5, w6, w7, *w8=w+18, w9, w10, *w11=w+24, w12, *w13=w+29, *w14=w+33, *w15=w+37, *w16=w+53, w17, w18, w19, w20, *w21=w+73, *w22=w+80, *w23=w+84, *w24=w+94, *w25=w+97, *w26=w+100, *w27=w+103;
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
  /* #44: @15 = @16' */
  for (i=0, rr=w15, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
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
  /* #51: @23 = vertcat(@4, @5, @6, @7, @17, @10, @18, @12, @19, @9) */
  rr=w23;
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
  /* #52: @11 = @23[:4] */
  for (rr=w11, ss=w23+0; ss!=w23+4; ss+=1) *rr++ = *ss;
  /* #53: @8 = mac(@15,@11,@8) */
  for (i=0, rr=w8; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w15+j, tt=w11+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #54: @8 = (@3-@8) */
  for (i=0, rr=w8, cr=w3, cs=w8; i<4; ++i) (*rr++)  = ((*cr++)-(*cs++));
  /* #55: @11 = @8' */
  casadi_copy(w8, 4, w11);
  /* #56: @15 = zeros(4x4) */
  casadi_clear(w15, 16);
  /* #57: @4 = 2 */
  w4 = 2.;
  /* #58: (@15[0] = @4) */
  for (rr=w15+0, ss=(&w4); rr!=w15+1; rr+=1) *rr = *ss++;
  /* #59: @4 = 2 */
  w4 = 2.;
  /* #60: (@15[10] = @4) */
  for (rr=w15+10, ss=(&w4); rr!=w15+11; rr+=1) *rr = *ss++;
  /* #61: @4 = 2 */
  w4 = 2.;
  /* #62: (@15[15] = @4) */
  for (rr=w15+15, ss=(&w4); rr!=w15+16; rr+=1) *rr = *ss++;
  /* #63: @4 = 2 */
  w4 = 2.;
  /* #64: (@15[5] = @4) */
  for (rr=w15+5, ss=(&w4); rr!=w15+6; rr+=1) *rr = *ss++;
  /* #65: @2 = mac(@11,@15,@2) */
  for (i=0, rr=w2; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w11+j, tt=w15+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #66: @1 = mac(@2,@8,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w8+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #67: @0 = (@0*@1) */
  w0 *= w1;
  /* #68: @1 = 10 */
  w1 = 10.;
  /* #69: @4 = 0 */
  w4 = 0.;
  /* #70: @24 = @21[4:7] */
  for (rr=w24, ss=w21+4; ss!=w21+7; ss+=1) *rr++ = *ss;
  /* #71: @2 = zeros(4x1) */
  casadi_clear(w2, 4);
  /* #72: @8 = zeros(4x1) */
  casadi_clear(w8, 4);
  /* #73: @5 = @3[0] */
  for (rr=(&w5), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #74: @6 = @3[1] */
  for (rr=(&w6), ss=w3+1; ss!=w3+2; ss+=1) *rr++ = *ss;
  /* #75: @7 = @3[2] */
  for (rr=(&w7), ss=w3+2; ss!=w3+3; ss+=1) *rr++ = *ss;
  /* #76: @17 = @3[3] */
  for (rr=(&w17), ss=w3+3; ss!=w3+4; ss+=1) *rr++ = *ss;
  /* #77: @11 = horzcat(@5, @6, @7, @17) */
  rr=w11;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w17;
  /* #78: @11 = @11' */
  /* #79: @5 = (-@6) */
  w5 = (- w6 );
  /* #80: @10 = @3[0] */
  for (rr=(&w10), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #81: @18 = (-@7) */
  w18 = (- w7 );
  /* #82: @13 = horzcat(@5, @10, @17, @18) */
  rr=w13;
  *rr++ = w5;
  *rr++ = w10;
  *rr++ = w17;
  *rr++ = w18;
  /* #83: @13 = @13' */
  /* #84: @10 = (-@17) */
  w10 = (- w17 );
  /* #85: @12 = @3[0] */
  for (rr=(&w12), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #86: @14 = horzcat(@18, @10, @12, @6) */
  rr=w14;
  *rr++ = w18;
  *rr++ = w10;
  *rr++ = w12;
  *rr++ = w6;
  /* #87: @14 = @14' */
  /* #88: @18 = @3[0] */
  for (rr=(&w18), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #89: @22 = horzcat(@10, @7, @5, @18) */
  rr=w22;
  *rr++ = w10;
  *rr++ = w7;
  *rr++ = w5;
  *rr++ = w18;
  /* #90: @22 = @22' */
  /* #91: @15 = horzcat(@11, @13, @14, @22) */
  rr=w15;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w22; i<4; ++i) *rr++ = *cs++;
  /* #92: @16 = @15' */
  for (i=0, rr=w16, cs=w15; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #93: @10 = 0 */
  w10 = 0.;
  /* #94: @25 = @23[4:7] */
  for (rr=w25, ss=w23+4; ss!=w23+7; ss+=1) *rr++ = *ss;
  /* #95: @11 = vertcat(@10, @25) */
  rr=w11;
  *rr++ = w10;
  for (i=0, cs=w25; i<3; ++i) *rr++ = *cs++;
  /* #96: @8 = mac(@16,@11,@8) */
  for (i=0, rr=w8; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w16+j, tt=w11+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #97: @10 = @8[0] */
  for (rr=(&w10), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #98: @5 = @8[1] */
  for (rr=(&w5), ss=w8+1; ss!=w8+2; ss+=1) *rr++ = *ss;
  /* #99: @5 = (-@5) */
  w5 = (- w5 );
  /* #100: @18 = @8[2] */
  for (rr=(&w18), ss=w8+2; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #101: @18 = (-@18) */
  w18 = (- w18 );
  /* #102: @12 = @8[3] */
  for (rr=(&w12), ss=w8+3; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #103: @12 = (-@12) */
  w12 = (- w12 );
  /* #104: @11 = horzcat(@10, @5, @18, @12) */
  rr=w11;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #105: @11 = @11' */
  /* #106: @10 = @8[1] */
  for (rr=(&w10), ss=w8+1; ss!=w8+2; ss+=1) *rr++ = *ss;
  /* #107: @5 = @8[0] */
  for (rr=(&w5), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #108: @18 = @8[3] */
  for (rr=(&w18), ss=w8+3; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #109: @18 = (-@18) */
  w18 = (- w18 );
  /* #110: @12 = @8[2] */
  for (rr=(&w12), ss=w8+2; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #111: @13 = horzcat(@10, @5, @18, @12) */
  rr=w13;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #112: @13 = @13' */
  /* #113: @10 = @8[2] */
  for (rr=(&w10), ss=w8+2; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #114: @5 = @8[3] */
  for (rr=(&w5), ss=w8+3; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #115: @18 = @8[0] */
  for (rr=(&w18), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #116: @12 = @8[1] */
  for (rr=(&w12), ss=w8+1; ss!=w8+2; ss+=1) *rr++ = *ss;
  /* #117: @12 = (-@12) */
  w12 = (- w12 );
  /* #118: @14 = horzcat(@10, @5, @18, @12) */
  rr=w14;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #119: @14 = @14' */
  /* #120: @10 = @8[3] */
  for (rr=(&w10), ss=w8+3; ss!=w8+4; ss+=1) *rr++ = *ss;
  /* #121: @5 = @8[2] */
  for (rr=(&w5), ss=w8+2; ss!=w8+3; ss+=1) *rr++ = *ss;
  /* #122: @5 = (-@5) */
  w5 = (- w5 );
  /* #123: @18 = @8[1] */
  for (rr=(&w18), ss=w8+1; ss!=w8+2; ss+=1) *rr++ = *ss;
  /* #124: @12 = @8[0] */
  for (rr=(&w12), ss=w8+0; ss!=w8+1; ss+=1) *rr++ = *ss;
  /* #125: @8 = horzcat(@10, @5, @18, @12) */
  rr=w8;
  *rr++ = w10;
  *rr++ = w5;
  *rr++ = w18;
  *rr++ = w12;
  /* #126: @8 = @8' */
  /* #127: @16 = horzcat(@11, @13, @14, @8) */
  rr=w16;
  for (i=0, cs=w11; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w13; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w14; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w8; i<4; ++i) *rr++ = *cs++;
  /* #128: @15 = @16' */
  for (i=0, rr=w15, cs=w16; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #129: @10 = @3[0] */
  for (rr=(&w10), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #130: @3 = vertcat(@10, @6, @7, @17) */
  rr=w3;
  *rr++ = w10;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w17;
  /* #131: @2 = mac(@15,@3,@2) */
  for (i=0, rr=w2; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w15+j, tt=w3+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #132: @25 = @2[1:4] */
  for (rr=w25, ss=w2+1; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #133: @24 = (@24-@25) */
  for (i=0, rr=w24, cs=w25; i<3; ++i) (*rr++) -= (*cs++);
  /* #134: @25 = @24' */
  casadi_copy(w24, 3, w25);
  /* #135: @4 = mac(@25,@24,@4) */
  for (i=0, rr=(&w4); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w25+j, tt=w24+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #136: @1 = (@1*@4) */
  w1 *= w4;
  /* #137: @0 = (@0+@1) */
  w0 += w1;
  /* #138: @1 = 0 */
  w1 = 0.;
  /* #139: @25 = zeros(1x3) */
  casadi_clear(w25, 3);
  /* #140: @24 = @23[7:10] */
  for (rr=w24, ss=w23+7; ss!=w23+10; ss+=1) *rr++ = *ss;
  /* #141: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #142: @10 = input[1][1] */
  w10 = arg[1] ? arg[1][1] : 0;
  /* #143: @6 = input[1][2] */
  w6 = arg[1] ? arg[1][2] : 0;
  /* #144: @26 = vertcat(@4, @10, @6) */
  rr=w26;
  *rr++ = w4;
  *rr++ = w10;
  *rr++ = w6;
  /* #145: @24 = (@24-@26) */
  for (i=0, rr=w24, cs=w26; i<3; ++i) (*rr++) -= (*cs++);
  /* #146: @26 = @24' */
  casadi_copy(w24, 3, w26);
  /* #147: @27 = zeros(3x3) */
  casadi_clear(w27, 9);
  /* #148: @4 = 0.000166667 */
  w4 = 1.6666666666666669e-04;
  /* #149: (@27[0] = @4) */
  for (rr=w27+0, ss=(&w4); rr!=w27+1; rr+=1) *rr = *ss++;
  /* #150: @4 = 0.000166667 */
  w4 = 1.6666666666666669e-04;
  /* #151: (@27[4] = @4) */
  for (rr=w27+4, ss=(&w4); rr!=w27+5; rr+=1) *rr = *ss++;
  /* #152: @4 = 0.000166667 */
  w4 = 1.6666666666666669e-04;
  /* #153: (@27[8] = @4) */
  for (rr=w27+8, ss=(&w4); rr!=w27+9; rr+=1) *rr = *ss++;
  /* #154: @25 = mac(@26,@27,@25) */
  for (i=0, rr=w25; i<3; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w26+j, tt=w27+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #155: @1 = mac(@25,@24,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w25+j, tt=w24+i*3; k<3; ++k) *rr += ss[k*1]**tt++;
  /* #156: @0 = (@0+@1) */
  w0 += w1;
  /* #157: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_ext_cost_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_ext_cost_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_ext_cost_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_ext_cost_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_ext_cost_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_ext_cost_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 14;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 112;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
