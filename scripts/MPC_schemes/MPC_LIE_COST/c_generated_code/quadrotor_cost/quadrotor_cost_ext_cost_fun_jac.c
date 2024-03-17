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

static const casadi_real casadi_c0[64] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 5.0000000000000000e-01, 0., 0., 0., 0., 0., 0., 0., 0., 5.0000000000000000e-01, 0., 0., 0., 0., 0., 0., 0., 0., 5.0000000000000000e-01};
static const casadi_real casadi_c1[16] = {6.7114093959731547e-01, 0., 0., 0., 0., 600., 0., 0., 0., 0., 600., 0., 0., 0., 0., 600.};

/* quadrotor_cost_ext_cost_fun_jac:(i0[14],i1[4],i2[],i3[18])->(o0,o1[18]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, *w2=w+10, w3, w4, *w5=w+20, w6, *w7=w+24, *w8=w+32, w9, w10, w11, w12, *w13=w+54, w14, w15, *w16=w+60, *w17=w+64, *w18=w+68, *w19=w+72, *w20=w+88, *w21=w+104, *w22=w+136, *w23=w+168, *w24=w+184, *w25=w+216, *w26=w+280, w27, w28, w29, w30, w31, w32, w33, w34, *w35=w+352, *w36=w+366, *w37=w+374, w38, w39, w40, w41, w42, w43, w44, w45, *w46=w+385, *w47=w+389, *w48=w+397, *w49=w+405, w50, w51, w52;
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
  /* #102: @40 = @36[0] */
  for (rr=(&w40), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #103: @41 = @36[1] */
  for (rr=(&w41), ss=w36+1; ss!=w36+2; ss+=1) *rr++ = *ss;
  /* #104: @42 = @36[2] */
  for (rr=(&w42), ss=w36+2; ss!=w36+3; ss+=1) *rr++ = *ss;
  /* #105: @43 = @36[3] */
  for (rr=(&w43), ss=w36+3; ss!=w36+4; ss+=1) *rr++ = *ss;
  /* #106: @16 = horzcat(@40, @41, @42, @43) */
  rr=w16;
  *rr++ = w40;
  *rr++ = w41;
  *rr++ = w42;
  *rr++ = w43;
  /* #107: @16 = @16' */
  /* #108: @40 = (-@41) */
  w40 = (- w41 );
  /* #109: @44 = @36[0] */
  for (rr=(&w44), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #110: @45 = (-@42) */
  w45 = (- w42 );
  /* #111: @17 = horzcat(@40, @44, @43, @45) */
  rr=w17;
  *rr++ = w40;
  *rr++ = w44;
  *rr++ = w43;
  *rr++ = w45;
  /* #112: @17 = @17' */
  /* #113: @43 = (-@43) */
  w43 = (- w43 );
  /* #114: @44 = @36[0] */
  for (rr=(&w44), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #115: @18 = horzcat(@45, @43, @44, @41) */
  rr=w18;
  *rr++ = w45;
  *rr++ = w43;
  *rr++ = w44;
  *rr++ = w41;
  /* #116: @18 = @18' */
  /* #117: @45 = @36[0] */
  for (rr=(&w45), ss=w36+0; ss!=w36+1; ss+=1) *rr++ = *ss;
  /* #118: @46 = horzcat(@43, @42, @40, @45) */
  rr=w46;
  *rr++ = w43;
  *rr++ = w42;
  *rr++ = w40;
  *rr++ = w45;
  /* #119: @46 = @46' */
  /* #120: @23 = horzcat(@16, @17, @18, @46) */
  rr=w23;
  for (i=0, cs=w16; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w46; i<4; ++i) *rr++ = *cs++;
  /* #121: @20 = @23' */
  for (i=0, rr=w20, cs=w23; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #122: @20 = (2.*@20) */
  for (i=0, rr=w20, cs=w20; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #123: @16 = @36[4:8] */
  for (rr=w16, ss=w36+4; ss!=w36+8; ss+=1) *rr++ = *ss;
  /* #124: @13 = mac(@20,@16,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w20+j, tt=w16+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #125: @43 = @13[1] */
  for (rr=(&w43), ss=w13+1; ss!=w13+2; ss+=1) *rr++ = *ss;
  /* #126: @43 = (@39*@43) */
  w43  = (w39*w43);
  /* #127: @42 = 0.5 */
  w42 = 5.0000000000000000e-01;
  /* #128: @40 = @13[2] */
  for (rr=(&w40), ss=w13+2; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #129: @40 = (@42*@40) */
  w40  = (w42*w40);
  /* #130: @45 = 0.5 */
  w45 = 5.0000000000000000e-01;
  /* #131: @44 = @13[3] */
  for (rr=(&w44), ss=w13+3; ss!=w13+4; ss+=1) *rr++ = *ss;
  /* #132: @44 = (@45*@44) */
  w44  = (w45*w44);
  /* #133: @36 = vertcat(@3, @27, @31, @34, @38, @43, @40, @44) */
  rr=w36;
  *rr++ = w3;
  *rr++ = w27;
  *rr++ = w31;
  *rr++ = w34;
  *rr++ = w38;
  *rr++ = w43;
  *rr++ = w40;
  *rr++ = w44;
  /* #134: @7 = @36' */
  casadi_copy(w36, 8, w7);
  /* #135: @26 = 
  [[0, 0, 0, 0, 0, 0, 0, 0], 
   [0, 2, 0, 0, 0, 0, 0, 0], 
   [0, 0, 2, 0, 0, 0, 0, 0], 
   [0, 0, 0, 2, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0, 0, 0, 0], 
   [0, 0, 0, 0, 0, 0.5, 0, 0], 
   [0, 0, 0, 0, 0, 0, 0.5, 0], 
   [0, 0, 0, 0, 0, 0, 0, 0.5]] */
  casadi_copy(casadi_c0, 64, w26);
  /* #136: @2 = mac(@7,@26,@2) */
  for (i=0, rr=w2; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w26+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #137: @1 = mac(@2,@36,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w36+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #138: @0 = (@0*@1) */
  w0 *= w1;
  /* #139: @1 = 0 */
  w1 = 0.;
  /* #140: @13 = zeros(1x4) */
  casadi_clear(w13, 4);
  /* #141: @17 = @8[14:18] */
  for (rr=w17, ss=w8+14; ss!=w8+18; ss+=1) *rr++ = *ss;
  /* #142: @3 = input[1][0] */
  w3 = arg[1] ? arg[1][0] : 0;
  /* #143: @38 = input[1][1] */
  w38 = arg[1] ? arg[1][1] : 0;
  /* #144: @43 = input[1][2] */
  w43 = arg[1] ? arg[1][2] : 0;
  /* #145: @40 = input[1][3] */
  w40 = arg[1] ? arg[1][3] : 0;
  /* #146: @18 = vertcat(@3, @38, @43, @40) */
  rr=w18;
  *rr++ = w3;
  *rr++ = w38;
  *rr++ = w43;
  *rr++ = w40;
  /* #147: @17 = (@17-@18) */
  for (i=0, rr=w17, cs=w18; i<4; ++i) (*rr++) -= (*cs++);
  /* #148: @18 = @17' */
  casadi_copy(w17, 4, w18);
  /* #149: @23 = 
  [[0.671141, 0, 0, 0], 
   [0, 600, 0, 0], 
   [0, 0, 600, 0], 
   [0, 0, 0, 600]] */
  casadi_copy(casadi_c1, 16, w23);
  /* #150: @13 = mac(@18,@23,@13) */
  for (i=0, rr=w13; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w18+j, tt=w23+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #151: @1 = mac(@13,@17,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w13+j, tt=w17+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #152: @0 = (@0+@1) */
  w0 += w1;
  /* #153: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #154: @13 = @13' */
  /* #155: @18 = zeros(1x4) */
  casadi_clear(w18, 4);
  /* #156: @17 = @17' */
  /* #157: @19 = @23' */
  for (i=0, rr=w19, cs=w23; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #158: @18 = mac(@17,@19,@18) */
  for (i=0, rr=w18; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w17+j, tt=w19+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #159: @18 = @18' */
  /* #160: @13 = (@13+@18) */
  for (i=0, rr=w13, cs=w18; i<4; ++i) (*rr++) += (*cs++);
  /* #161: @13 = (-@13) */
  for (i=0, rr=w13, cs=w13; i<4; ++i) *rr++ = (- *cs++ );
  /* #162: {@0, @1, @3, @38} = vertsplit(@13) */
  w0 = w13[0];
  w1 = w13[1];
  w3 = w13[2];
  w38 = w13[3];
  /* #163: output[1][0] = @0 */
  if (res[1]) res[1][0] = w0;
  /* #164: output[1][1] = @1 */
  if (res[1]) res[1][1] = w1;
  /* #165: output[1][2] = @3 */
  if (res[1]) res[1][2] = w3;
  /* #166: output[1][3] = @38 */
  if (res[1]) res[1][3] = w38;
  /* #167: @35 = zeros(14x1) */
  casadi_clear(w35, 14);
  /* #168: @7 = zeros(8x1) */
  casadi_clear(w7, 8);
  /* #169: @38 = 1 */
  w38 = 1.;
  /* #170: @6 = (@6?@38:0) */
  w6  = (w6?w38:0);
  /* #171: @47 = zeros(8x1) */
  casadi_clear(w47, 8);
  /* #172: @13 = zeros(4x1) */
  casadi_clear(w13, 4);
  /* #173: @19 = @20' */
  for (i=0, rr=w19, cs=w20; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #174: @18 = zeros(4x1) */
  casadi_clear(w18, 4);
  /* #175: @38 = 10 */
  w38 = 10.;
  /* #176: @2 = @2' */
  /* #177: @2 = (@38*@2) */
  for (i=0, rr=w2, cs=w2; i<8; ++i) (*rr++)  = (w38*(*cs++));
  /* #178: @48 = zeros(1x8) */
  casadi_clear(w48, 8);
  /* #179: @36 = @36' */
  /* #180: @36 = (@38*@36) */
  for (i=0, rr=w36, cs=w36; i<8; ++i) (*rr++)  = (w38*(*cs++));
  /* #181: @49 = @26' */
  for (i=0, rr=w49, cs=w26; i<8; ++i) for (j=0; j<8; ++j) rr[i+j*8] = *cs++;
  /* #182: @48 = mac(@36,@49,@48) */
  for (i=0, rr=w48; i<8; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w36+j, tt=w49+i*8; k<8; ++k) *rr += ss[k*1]**tt++;
  /* #183: @48 = @48' */
  /* #184: @2 = (@2+@48) */
  for (i=0, rr=w2, cs=w48; i<8; ++i) (*rr++) += (*cs++);
  /* #185: {NULL, @38, @3, @1, NULL, @0, @43, @40} = vertsplit(@2) */
  w38 = w2[1];
  w3 = w2[2];
  w1 = w2[3];
  w0 = w2[5];
  w43 = w2[6];
  w40 = w2[7];
  /* #186: @45 = (@45*@40) */
  w45 *= w40;
  /* #187: (@18[3] += @45) */
  for (rr=w18+3, ss=(&w45); rr!=w18+4; rr+=1) *rr += *ss++;
  /* #188: @42 = (@42*@43) */
  w42 *= w43;
  /* #189: (@18[2] += @42) */
  for (rr=w18+2, ss=(&w42); rr!=w18+3; rr+=1) *rr += *ss++;
  /* #190: @39 = (@39*@0) */
  w39 *= w0;
  /* #191: (@18[1] += @39) */
  for (rr=w18+1, ss=(&w39); rr!=w18+2; rr+=1) *rr += *ss++;
  /* #192: @13 = mac(@19,@18,@13) */
  for (i=0, rr=w13; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w19+j, tt=w18+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #193: (@47[4:8] += @13) */
  for (rr=w47+4, ss=w13; rr!=w47+8; rr+=1) *rr += *ss++;
  /* #194: @19 = zeros(4x4) */
  casadi_clear(w19, 16);
  /* #195: @16 = @16' */
  /* #196: @19 = mac(@18,@16,@19) */
  for (i=0, rr=w19; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w18+j, tt=w16+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #197: @19 = (2.*@19) */
  for (i=0, rr=w19, cs=w19; i<16; ++i) *rr++ = (2.* *cs++ );
  /* #198: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #199: {@18, @16, @13, @17} = horzsplit(@20) */
  casadi_copy(w20, 4, w18);
  casadi_copy(w20+4, 4, w16);
  casadi_copy(w20+8, 4, w13);
  casadi_copy(w20+12, 4, w17);
  /* #200: @17 = @17' */
  /* #201: {@39, @0, @42, @43} = horzsplit(@17) */
  w39 = w17[0];
  w0 = w17[1];
  w42 = w17[2];
  w43 = w17[3];
  /* #202: (@47[0] += @43) */
  for (rr=w47+0, ss=(&w43); rr!=w47+1; rr+=1) *rr += *ss++;
  /* #203: @13 = @13' */
  /* #204: {@43, @45, @40, @44} = horzsplit(@13) */
  w43 = w13[0];
  w45 = w13[1];
  w40 = w13[2];
  w44 = w13[3];
  /* #205: (@47[0] += @40) */
  for (rr=w47+0, ss=(&w40); rr!=w47+1; rr+=1) *rr += *ss++;
  /* #206: @16 = @16' */
  /* #207: {@40, @41, @50, @51} = horzsplit(@16) */
  w40 = w16[0];
  w41 = w16[1];
  w50 = w16[2];
  w51 = w16[3];
  /* #208: (@47[0] += @41) */
  for (rr=w47+0, ss=(&w41); rr!=w47+1; rr+=1) *rr += *ss++;
  /* #209: @39 = (@39+@45) */
  w39 += w45;
  /* #210: @50 = (@50-@39) */
  w50 -= w39;
  /* #211: @18 = @18' */
  /* #212: {@39, @45, @41, @52} = horzsplit(@18) */
  w39 = w18[0];
  w45 = w18[1];
  w41 = w18[2];
  w52 = w18[3];
  /* #213: @50 = (@50+@52) */
  w50 += w52;
  /* #214: (@47[3] += @50) */
  for (rr=w47+3, ss=(&w50); rr!=w47+4; rr+=1) *rr += *ss++;
  /* #215: @43 = (@43+@51) */
  w43 += w51;
  /* #216: @0 = (@0-@43) */
  w0 -= w43;
  /* #217: @0 = (@0+@41) */
  w0 += w41;
  /* #218: (@47[2] += @0) */
  for (rr=w47+2, ss=(&w0); rr!=w47+3; rr+=1) *rr += *ss++;
  /* #219: @42 = (@42+@40) */
  w42 += w40;
  /* #220: @44 = (@44-@42) */
  w44 -= w42;
  /* #221: @44 = (@44+@45) */
  w44 += w45;
  /* #222: (@47[1] += @44) */
  for (rr=w47+1, ss=(&w44); rr!=w47+2; rr+=1) *rr += *ss++;
  /* #223: (@47[0] += @39) */
  for (rr=w47+0, ss=(&w39); rr!=w47+1; rr+=1) *rr += *ss++;
  /* #224: @39 = (@1/@9) */
  w39  = (w1/w9);
  /* #225: @10 = (@10*@39) */
  w10 *= w39;
  /* #226: (@47[3] += @10) */
  for (rr=w47+3, ss=(&w10); rr!=w47+4; rr+=1) *rr += *ss++;
  /* #227: @10 = (@3/@9) */
  w10  = (w3/w9);
  /* #228: @29 = (@29*@10) */
  w29 *= w10;
  /* #229: (@47[2] += @29) */
  for (rr=w47+2, ss=(&w29); rr!=w47+3; rr+=1) *rr += *ss++;
  /* #230: @29 = (@38/@9) */
  w29  = (w38/w9);
  /* #231: @14 = (@14*@29) */
  w14 *= w29;
  /* #232: (@47[1] += @14) */
  for (rr=w47+1, ss=(&w14); rr!=w47+2; rr+=1) *rr += *ss++;
  /* #233: @14 = sq(@9) */
  w14 = casadi_sq( w9 );
  /* #234: @44 = sq(@12) */
  w44 = casadi_sq( w12 );
  /* #235: @14 = (@14+@44) */
  w14 += w44;
  /* #236: @44 = (@9/@14) */
  w44  = (w9/w14);
  /* #237: @33 = (@33*@39) */
  w33 *= w39;
  /* #238: @32 = (@32*@33) */
  w32 *= w33;
  /* #239: @30 = (@30*@10) */
  w30 *= w10;
  /* #240: @28 = (@28*@30) */
  w28 *= w30;
  /* #241: @32 = (@32+@28) */
  w32 += w28;
  /* #242: @11 = (@11*@29) */
  w11 *= w29;
  /* #243: @4 = (@4*@11) */
  w4 *= w11;
  /* #244: @32 = (@32+@4) */
  w32 += w4;
  /* #245: @44 = (@44*@32) */
  w44 *= w32;
  /* #246: @44 = (-@44) */
  w44 = (- w44 );
  /* #247: (@47[0] += @44) */
  for (rr=w47+0, ss=(&w44); rr!=w47+1; rr+=1) *rr += *ss++;
  /* #248: @31 = (@31/@9) */
  w31 /= w9;
  /* #249: @31 = (@31*@3) */
  w31 *= w3;
  /* #250: @31 = (-@31) */
  w31 = (- w31 );
  /* #251: @34 = (@34/@9) */
  w34 /= w9;
  /* #252: @34 = (@34*@1) */
  w34 *= w1;
  /* #253: @31 = (@31-@34) */
  w31 -= w34;
  /* #254: @27 = (@27/@9) */
  w27 /= w9;
  /* #255: @27 = (@27*@38) */
  w27 *= w38;
  /* #256: @31 = (@31-@27) */
  w31 -= w27;
  /* #257: @12 = (@12/@14) */
  w12 /= w14;
  /* #258: @12 = (@12*@32) */
  w12 *= w32;
  /* #259: @31 = (@31+@12) */
  w31 += w12;
  /* #260: @31 = (@31/@9) */
  w31 /= w9;
  /* #261: @5 = (@31*@5) */
  for (i=0, rr=w5, cs=w5; i<3; ++i) (*rr++)  = (w31*(*cs++));
  /* #262: (@47[1:4] += @5) */
  for (rr=w47+1, ss=w5; rr!=w47+4; rr+=1) *rr += *ss++;
  /* #263: @2 = (@6*@47) */
  for (i=0, rr=w2, cs=w47; i<8; ++i) (*rr++)  = (w6*(*cs++));
  /* #264: @6 = 1 */
  w6 = 1.;
  /* #265: @15 = (@15?@6:0) */
  w15  = (w15?w6:0);
  /* #266: @47 = (@15*@47) */
  for (i=0, rr=w47, cs=w47; i<8; ++i) (*rr++)  = (w15*(*cs++));
  /* #267: @2 = (@2-@47) */
  for (i=0, rr=w2, cs=w47; i<8; ++i) (*rr++) -= (*cs++);
  /* #268: @7 = mac(@25,@2,@7) */
  for (i=0, rr=w7; i<1; ++i) for (j=0; j<8; ++j, ++rr) for (k=0, ss=w25+j, tt=w2+i*8; k<8; ++k) *rr += ss[k*8]**tt++;
  /* #269: (@35[:8] += @7) */
  for (rr=w35+0, ss=w7; rr!=w35+8; rr+=1) *rr += *ss++;
  /* #270: {@15, @6, @31, @9, @12, @32, @14, @27, @38, @34, @1, @3, @44, @4} = vertsplit(@35) */
  w15 = w35[0];
  w6 = w35[1];
  w31 = w35[2];
  w9 = w35[3];
  w12 = w35[4];
  w32 = w35[5];
  w14 = w35[6];
  w27 = w35[7];
  w38 = w35[8];
  w34 = w35[9];
  w1 = w35[10];
  w3 = w35[11];
  w44 = w35[12];
  w4 = w35[13];
  /* #271: output[1][4] = @15 */
  if (res[1]) res[1][4] = w15;
  /* #272: output[1][5] = @6 */
  if (res[1]) res[1][5] = w6;
  /* #273: output[1][6] = @31 */
  if (res[1]) res[1][6] = w31;
  /* #274: output[1][7] = @9 */
  if (res[1]) res[1][7] = w9;
  /* #275: output[1][8] = @12 */
  if (res[1]) res[1][8] = w12;
  /* #276: output[1][9] = @32 */
  if (res[1]) res[1][9] = w32;
  /* #277: output[1][10] = @14 */
  if (res[1]) res[1][10] = w14;
  /* #278: output[1][11] = @27 */
  if (res[1]) res[1][11] = w27;
  /* #279: output[1][12] = @38 */
  if (res[1]) res[1][12] = w38;
  /* #280: output[1][13] = @34 */
  if (res[1]) res[1][13] = w34;
  /* #281: output[1][14] = @1 */
  if (res[1]) res[1][14] = w1;
  /* #282: output[1][15] = @3 */
  if (res[1]) res[1][15] = w3;
  /* #283: output[1][16] = @44 */
  if (res[1]) res[1][16] = w44;
  /* #284: output[1][17] = @4 */
  if (res[1]) res[1][17] = w4;
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
  if (sz_arg) *sz_arg = 18;
  if (sz_res) *sz_res = 16;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 472;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
