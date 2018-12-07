uint64 timestamp		# time since system start (microseconds)

float32[3]  rates
float32[3]  rates_sp

float32     thrust_sp
float32     accel_z

# RateController.h:
# typedef struct {
#   real32_T rates[3];                   /* '<Root>/rates' */
#   real32_T rates_dot_sp[3];            /* '<Root>/rates_dot_sp' */
#   real32_T thrust_sp;                  /* '<Root>/thrust_sp' */
#   real32_T accel_z;                    /* '<Root>/accel_z' */
#   real_T wRotor[4];                    /* '<Root>/wRotor' */
# } ExtU_RateController_T;
