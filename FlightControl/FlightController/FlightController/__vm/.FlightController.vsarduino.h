/* 
	Editor: https://www.visualmicro.com/
			visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
			the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
			all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
			note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Adafruit Feather M0 Express, Platform=samd, Package=adafruit
*/

#if defined(_VMICRO_INTELLISENSE)

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define _VMDEBUG 1
#define F_CPU 48000000L
#define ARDUINO 10805
#define ARDUINO_SAMD_FEATHER_M0_EXPRESS
#define ARDUINO_ARCH_SAMD
#define ARDUINO_SAMD_ZERO
#define ARDUINO_SAMD_FEATHER_M0
#define ARM_MATH_CM0PLUS
#define ADAFRUIT_FEATHER_M0_EXPRESS
#define __SAMD21G18A__
#define USB_VID 0x239A
#define USB_PID 0x801B
#define USBCON
#define __cplusplus 201103L
#define __cplusplus 201103L
#define __PTRDIFF_TYPE__ int
#define __ARM__
#define __arm__
#define __inline__
#define __asm__(x)
#define __attribute__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __volatile__
typedef int __SIZE_TYPE__;
typedef int __builtin_va_list;
#define _Pragma(x)
#define __ASM
#define __INLINE

typedef long Sercom;
#define SERCOM0           ((Sercom   *)0x42000800UL) 
#define SERCOM1           ((Sercom   *)0x42000800UL) 
#define SERCOM2           ((Sercom   *)0x42000800UL) 
#define SERCOM3           ((Sercom   *)0x42000800UL) 
#define SERCOM4           ((Sercom   *)0x42000800UL) 
#define SERCOM5           ((Sercom   *)0x42000800UL) 
typedef long Tcc;
#define TCC0              ((Tcc      *)0x42002000UL)
#define TCC1              ((Tcc      *)0x42002000UL)
#define TCC2              ((Tcc      *)0x42002000UL)
typedef long Tc;
#define TC1               ((Tc       *)0x42002C00UL)
#define TC2               ((Tc       *)0x42003000UL)
#define TC3               ((Tc       *)0x42003400UL)
#define TC4               ((Tc       *)0x42003400UL)
#define TC5               ((Tc       *)0x42003400UL)

typedef long UsbHostDescBank;
typedef struct {
	UsbHostDescBank           HostDescBank[2];
} UsbHostDescriptor;

#define __IO
#define RoReg8
#define __I
#define USB_EPT_NUM
#define Reserved1[]
#define Reserved2[]
#define Reserved3[]
#define Reserved4[]
#define Reserved5[]
#define Reserved6[]
#define Reserved7[]
#define Reserved8[]
#define Reserved9[]
#define Reserved10[]

#define TC_INST_NUM 999
#define TCC_INST_NUM 999


#define prog_void
#define PGM_VOID_P int

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() { ; }
#include <arduino.h>
#include <pins_arduino.h> 
#include <variant.h> 
#include <variant.cpp> 

#ifndef __math_68881
extern double atan(double);
extern double cos(double);
extern double sin(double);
extern double tan(double);
extern double tanh(double);
extern double frexp(double, int *);
extern double modf(double, double *);
extern double ceil(double);
extern double fabs(double);
extern double floor(double);
#endif 

#ifndef __math_68881
extern double acos(double);
extern double asin(double);
extern double atan2(double, double);
extern double cosh(double);
extern double sinh(double);
extern double exp(double);
extern double ldexp(double, int);
extern double log(double);
extern double log10(double);
extern double pow(double, double);
extern double sqrt(double);
extern double fmod(double, double);
#endif 

extern int __isinff(float x);
extern int __isinfd(double x);
extern int __isnanf(float x);
extern int __isnand(double x);
extern int __fpclassifyf(float x);
extern int __fpclassifyd(double x);
extern int __signbitf(float x);
extern int __signbitd(double x);
extern int finitel(long double);
extern double infinity(void);
extern double nan(const char *);
extern int finite(double);
extern double copysign(double, double);
extern double logb(double);
extern int ilogb(double);

extern double asinh(double);
extern double cbrt(double);
extern double nextafter(double, double);
extern double rint(double);
extern double scalbn(double, int);

extern double exp2(double);
extern double scalbln(double, long int);
extern double tgamma(double);
extern double nearbyint(double);
extern long int lrint(double);
extern long long int llrint(double);
extern long int lround(double);
extern long long int llround(double);
extern double trunc(double);
extern double remquo(double, double, int *);
extern double fdim(double, double);
extern double fmax(double, double);
extern double fmin(double, double);
extern double fma(double, double, double);

#ifndef __math_68881
extern double log1p(double);
extern double expm1(double);
#endif 

extern double acosh(double);
extern double atanh(double);
extern double remainder(double, double);
extern double gamma(double);
extern double lgamma(double);
extern double erf(double);
extern double erfc(double);
extern double log2(double);

#ifndef __math_68881
extern double hypot(double, double);
#endif

extern float atanf(float);
extern float cosf(float);
extern float sinf(float);
extern float tanf(float);
extern float tanhf(float);
extern float frexpf(float, int *);
extern float modff(float, float *);
extern float ceilf(float);
extern float fabsf(float);
extern float floorf(float);

#ifndef _REENT_ONLY
extern float acosf(float);
extern float asinf(float);
extern float atan2f(float, float);
extern float coshf(float);
extern float sinhf(float);
extern float expf(float);
extern float ldexpf(float, int);
extern float logf(float);
extern float log10f(float);
extern float powf(float, float);
extern float sqrtf(float);
extern float fmodf(float, float);
#endif 

extern float exp2f(float);
extern float scalblnf(float, int);
extern float tgammaf(float);
extern float nearbyintf(float);
extern long int lrintf(float);
extern long long int llrintf(float);
extern float roundf(float);
extern long int lroundf(float);
extern long long int llroundf(float);
extern float truncf(float);
extern float remquof(float, float, int *);
extern float fdimf(float, float);
extern float fmaxf(float, float);
extern float fminf(float, float);
extern float fmaf(float, float, float);

extern float infinityf(void);
extern float nanf(const char *);
extern int finitef(float);
extern float copysignf(float, float);
extern float logbf(float);
extern int ilogbf(float);

extern float asinhf(float);
extern float cbrtf(float);
extern float nextafterf(float, float);
extern float rintf(float);
extern float scalbnf(float, int);
extern float log1pf(float);
extern float expm1f(float);

#ifndef _REENT_ONLY
extern float acoshf(float);
extern float atanhf(float);
extern float remainderf(float, float);
extern float gammaf(float);
extern float lgammaf(float);
extern float erff(float);
extern float erfcf(float);
extern float log2f(float);
extern float hypotf(float, float);
#endif 

extern double drem(double, double);
extern void sincos(double, double *, double *);
extern double gamma_r(double, int *);
extern double lgamma_r(double, int *);

extern double y0(double);
extern double y1(double);
extern double yn(int, double);
extern double j0(double);
extern double j1(double);
extern double jn(int, double);

extern float dremf(float, float);
extern void sincosf(float, float *, float *);
extern float gammaf_r(float, int *);
extern float lgammaf_r(float, int *);

extern float y0f(float);
extern float y1f(float);
extern float ynf(int, float);
extern float j0f(float);
extern float j1f(float);
extern float jnf(int, float);


#include "FlightController.ino"
#endif
#endif
