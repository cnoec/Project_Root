/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: trajectory_simulation.h
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

#ifndef RTW_HEADER_trajectory_simulation_h_
#define RTW_HEADER_trajectory_simulation_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#ifndef trajectory_simulation_COMMON_INCLUDES_
# define trajectory_simulation_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* trajectory_simulation_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T xi_dot[6];                    /* '<Root>/MATLAB Function1' */
  real_T Sum1;                         /* '<S1>/Sum1' */
  real_T DiscreteTransferFcn;          /* '<S1>/Discrete Transfer Fcn' */
  real_T UnitDelay;                    /* '<S1>/Unit Delay' */
  real_T UnitDelay1;                   /* '<S1>/Unit Delay1' */
  real_T integral_action;              /* '<S1>/MATLAB Function1' */
  real_T DiscreteTransferFcn_states;   /* '<S1>/Discrete Transfer Fcn' */
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<S1>/Unit Delay1' */
  real_T DiscreteTransferFcn_tmp;      /* '<S1>/Discrete Transfer Fcn' */
  int_T Integrator_IWORK;              /* '<Root>/Integrator' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE[6];         /* '<Root>/Integrator' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE[6];         /* '<Root>/Integrator' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE[6];      /* '<Root>/Integrator' */
} XDis;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T steer_input;                  /* '<Root>/steer_input' */
  real_T theta[13];                    /* '<Root>/theta' */
  real_T speed_ref;                    /* '<Root>/speed_ref' */
  real_T xi_0[6];                      /* '<Root>/xi_0' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T xi[6];                        /* '<Root>/xi' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[6];
  real_T odeF[4][6];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Continuous states (default storage) */
extern X rtX;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void trajectory_simulation_initialize(void);
extern void trajectory_simulation_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'trajectory_simulation'
 * '<S1>'   : 'trajectory_simulation/Cruise Control'
 * '<S2>'   : 'trajectory_simulation/MATLAB Function1'
 * '<S3>'   : 'trajectory_simulation/Cruise Control/MATLAB Function1'
 */
#endif                                 /* RTW_HEADER_trajectory_simulation_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
