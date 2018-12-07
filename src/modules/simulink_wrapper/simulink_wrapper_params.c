/**
 * @file simulink_wrapper_params.c
 * Parameters for simulink wrapper attitutde control.
 *
 * @author
 */


/**
 * Rate control maximum sample rate.
 *
 * @unit Hz
 * @min 10
 * @max 1000
 * @decimal 0
 * @increment 1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ATT_RATE_MAX, 500.0f);

/**
 * INDI control actuator roll control effectiveness.
 *
 * @unit Hz
 * @min 1
 * @max 100
 * @decimal 0
 * @increment .1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ATT_ROLL_EFF, 15.0f);

/**
 * INDI control actuator pitch control effectiveness.
 *
 * @unit Hz
 * @min 1
 * @max 100
 * @decimal 0
 * @increment .1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ATT_PITCH_EFF, 12.0f);

/**
 * INDI control actuator yaw control effectiveness.
 *
 * @unit Hz
 * @min .01
 * @max 10
 * @decimal 0
 * @increment .01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ATT_YAW_EFF, 0.02f);

/**
 * INDI control actuator z-acceleration control effectiveness.
 *
 * @unit Hz
 * @min 1
 * @max 100
 * @decimal 0
 * @increment .1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ATT_AZ_EFF, 10.0f);
