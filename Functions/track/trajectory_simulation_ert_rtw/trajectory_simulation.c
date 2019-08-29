/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: trajectory_simulation.c
 *
 * Code generated for Simulink model 'trajectory_simulation'.
 *
 * Model version                  : 1.44
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Thu Aug 29 16:11:45 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "trajectory_simulation.h"
#define NumBitsPerChar                 8U

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
# define rtmSetFirstInitCond(rtm, val) ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
# define rtmIsFirstInitCond(rtm)       ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

/* Continuous states */
X rtX;

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_powd_snf(real_T u0, real_T u1);

/* private model entry point functions */
extern void trajectory_simulation_derivatives(void);
extern real_T rtGetInf(void);
extern real32_T rtGetInfF(void);
extern real_T rtGetMinusInf(void);
extern real32_T rtGetMinusInfF(void);
extern real_T rtGetNaN(void);
extern real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)                                   /* do nothing */
# else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#   define UNUSED_PARAMETER(x)         (void) (x)
# endif
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
extern void rt_InitInfAndNaN(size_t realSize);
extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0 } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  trajectory_simulation_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  trajectory_simulation_step();
  trajectory_simulation_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  trajectory_simulation_step();
  trajectory_simulation_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  trajectory_simulation_step();
  trajectory_simulation_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void trajectory_simulation_step(void)
{
  real_T Uy;
  real_T F_z_f;
  real_T F_z_r;
  real_T z_f;
  real_T z_r;
  int32_T i;
  real_T u0;
  real_T F_z_f_tmp;
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  for (i = 0; i < 6; i++) {
    /* Integrator: '<Root>/Integrator' incorporates:
     *  Inport: '<Root>/xi_0'
     */
    if (rtDW.Integrator_IWORK != 0) {
      rtX.Integrator_CSTATE[i] = rtU.xi_0[i];
    }

    /* Outport: '<Root>/xi' incorporates:
     *  Integrator: '<Root>/Integrator'
     */
    rtY.xi[i] = rtX.Integrator_CSTATE[i];
  }

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/speed_ref'
   *  Integrator: '<Root>/Integrator'
   */
  rtDW.Sum1 = rtU.speed_ref - rtX.Integrator_CSTATE[2];
  if (rtmIsMajorTimeStep(rtM)) {
    /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
    rtDW.DiscreteTransferFcn_tmp = rtDW.Sum1 - -0.99 *
      rtDW.DiscreteTransferFcn_states;
    rtDW.DiscreteTransferFcn = 4.57 * rtDW.DiscreteTransferFcn_tmp + 4.57 *
      rtDW.DiscreteTransferFcn_states;

    /* UnitDelay: '<S1>/Unit Delay' */
    rtDW.UnitDelay = rtDW.UnitDelay_DSTATE;

    /* UnitDelay: '<S1>/Unit Delay1' */
    rtDW.UnitDelay1 = rtDW.UnitDelay1_DSTATE;
  }

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   */
  rtDW.integral_action = fmin(883.0, fmax(-5000.0, (0.045 * rtDW.Sum1 + 0.045 *
    rtDW.UnitDelay) + rtDW.UnitDelay1));

  /* Sum: '<S1>/Sum' */
  u0 = rtDW.integral_action + rtDW.DiscreteTransferFcn;

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/steer_input'
   *  Inport: '<Root>/theta'
   *  Integrator: '<Root>/Integrator'
   */
  Uy = tan(rtX.Integrator_CSTATE[3]) * rtX.Integrator_CSTATE[2];
  F_z_f_tmp = rtU.theta[2] + rtU.theta[3];
  F_z_f = rtU.theta[0] * 9.81 * rtU.theta[3] / F_z_f_tmp;
  F_z_r = rtU.theta[0] * 9.81 * rtU.theta[2] / F_z_f_tmp;
  z_f = tan(rt_atan2d_snf(rtU.theta[2] * rtX.Integrator_CSTATE[5] + Uy,
             rtX.Integrator_CSTATE[2]) - rtU.steer_input);
  z_r = tan(rt_atan2d_snf(Uy - rtU.theta[3] * rtX.Integrator_CSTATE[5],
             rtX.Integrator_CSTATE[2]));
  F_z_f_tmp = rtU.theta[7] * rtU.theta[7] * 27.0;
  F_z_f = fmin(rtU.theta[7] * F_z_f, fmax(-rtU.theta[7] * F_z_f, (rtU.theta[4] *
    rtU.theta[4] * fabs(z_f) * z_f / (3.0 * rtU.theta[7] * F_z_f) + -rtU.theta[4]
    * z_f) - rt_powd_snf(rtU.theta[4], 3.0) * rt_powd_snf(z_f, 3.0) / (F_z_f_tmp
    * (F_z_f * F_z_f))));
  F_z_r = fmin(rtU.theta[7] * F_z_r, fmax(-rtU.theta[7] * F_z_r, (rtU.theta[5] *
    rtU.theta[5] * fabs(z_r) * z_r / (3.0 * rtU.theta[7] * F_z_r) + -rtU.theta[5]
    * z_r) - rt_powd_snf(rtU.theta[5], 3.0) * rt_powd_snf(z_r, 3.0) / (F_z_f_tmp
    * (F_z_r * F_z_r))));
  F_z_f_tmp = sin(rtX.Integrator_CSTATE[4]);
  z_f = cos(rtX.Integrator_CSTATE[4]);
  rtDW.xi_dot[0] = rtX.Integrator_CSTATE[2] * z_f - Uy * F_z_f_tmp;
  rtDW.xi_dot[1] = rtX.Integrator_CSTATE[2] * F_z_f_tmp + Uy * z_f;

  /* Saturate: '<S1>/Saturation' */
  if (u0 > 883.0) {
    u0 = 883.0;
  } else {
    if (u0 < -5000.0) {
      u0 = -5000.0;
    }
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/steer_input'
   *  Inport: '<Root>/theta'
   *  Integrator: '<Root>/Integrator'
   */
  rtDW.xi_dot[2] = (((u0 / rtU.theta[6] - F_z_f * sin(rtU.steer_input)) -
                     rtU.theta[11] * rtX.Integrator_CSTATE[2]) - 0.5 *
                    rtU.theta[12] * rtU.theta[8] * rtU.theta[10] *
                    (rtX.Integrator_CSTATE[2] * rtX.Integrator_CSTATE[2])) *
    (1.0 / rtU.theta[0]);
  F_z_f_tmp = cos(rtU.steer_input);
  rtDW.xi_dot[3] = 1.0 / (rtU.theta[0] * rtX.Integrator_CSTATE[2]) * (F_z_f *
    F_z_f_tmp + F_z_r) - rtX.Integrator_CSTATE[5];
  rtDW.xi_dot[4] = rtX.Integrator_CSTATE[5];
  rtDW.xi_dot[5] = (rtU.theta[2] * F_z_f * F_z_f_tmp - rtU.theta[3] * F_z_r) *
    (1.0 / rtU.theta[1]);
  if (rtmIsMajorTimeStep(rtM)) {
    /* Update for Integrator: '<Root>/Integrator' */
    rtDW.Integrator_IWORK = 0;
    if (rtmIsMajorTimeStep(rtM)) {
      /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
      rtDW.DiscreteTransferFcn_states = rtDW.DiscreteTransferFcn_tmp;

      /* Update for UnitDelay: '<S1>/Unit Delay' */
      rtDW.UnitDelay_DSTATE = rtDW.Sum1;

      /* Update for UnitDelay: '<S1>/Unit Delay1' */
      rtDW.UnitDelay1_DSTATE = rtDW.integral_action;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.0001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.0001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void trajectory_simulation_derivatives(void)
{
  int32_T i;
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<Root>/Integrator' */
  for (i = 0; i < 6; i++) {
    _rtXdot->Integrator_CSTATE[i] = rtDW.xi_dot[i];
  }

  /* End of Derivatives for Integrator: '<Root>/Integrator' */
}

/* Model initialize function */
void trajectory_simulation_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->intgData.f[3] = rtM->odeF[3];
  rtM->contStates = ((X *) &rtX);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode4");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.0001;
  rtmSetFirstInitCond(rtM, 1);

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  if (rtmIsFirstInitCond(rtM)) {
    rtX.Integrator_CSTATE[0] = 0.0;
    rtX.Integrator_CSTATE[1] = 0.0;
    rtX.Integrator_CSTATE[2] = 0.0;
    rtX.Integrator_CSTATE[3] = 0.0;
    rtX.Integrator_CSTATE[4] = 0.0;
    rtX.Integrator_CSTATE[5] = 0.0;
  }

  rtDW.Integrator_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<Root>/Integrator' */

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(rtM)) {
    rtmSetFirstInitCond(rtM, 0);
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
