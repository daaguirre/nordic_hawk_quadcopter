/*
 * nordic_hawk_pwm_motor.c
 *
 *  Created on: 16 oct. 2017
 *      Author: diego.aguirre
 */

#include "nordic_hawk_pwm_motor.h"
#include "real_time_debugger.h"
#include "nrf_delay.h"
#include "common.h"

static void nordic_hawk_motor_handler(nrf_drv_pwm_evt_type_t event_type);
//static uint16_t get_pwm_scaled_value(uint8_t value);

#ifdef ESC_CALIBRATION
static void esc_controller_calibration();
static int calibration_step= 0;
static int second_unit;
#endif

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static bool can_update_motor_values = false;
static nrf_pwm_values_individual_t nordic_hawk_motor_seq_values;
static nrf_pwm_sequence_t const    nordic_hawk_motor_seq =
{
    .values.p_individual = &nordic_hawk_motor_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(nordic_hawk_motor_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};
static motor_values_t current_motor_values;

void pwm_init(pwm_frequency_t pwm_frequency)
{
	nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
        		NORDIC_HAWK_PWM_MOTOR1,             // channel 0
				NORDIC_HAWK_PWM_MOTOR2,             // channel 1
				NORDIC_HAWK_PWM_MOTOR3,             // channel 2
				NORDIC_HAWK_PWM_MOTOR4,             // channel 3
        },
        .base_clock = NRF_PWM_CLK_16MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value  = TOP_PWM/(pwm_frequency + 1),
        .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode  = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, nordic_hawk_motor_handler));

    nordic_hawk_motor_seq_values.channel_0 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN; //125 us pulse
    nordic_hawk_motor_seq_values.channel_1 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;
    nordic_hawk_motor_seq_values.channel_2 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;
    nordic_hawk_motor_seq_values.channel_3 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;

    memset(&current_motor_values, 0, sizeof(motor_values_t));

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &nordic_hawk_motor_seq, 1,
                                      NRF_DRV_PWM_FLAG_LOOP);
#ifdef ESC_CALIBRATION
    second_unit = 500*(pwm_frequency + 1);
#endif
}


bool update_motor_values(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
	uint16_t motor_value1 = motor1 + ONE_SHOT_125_MIN;
	uint16_t motor_value2 = motor2 + ONE_SHOT_125_MIN;
	uint16_t motor_value3 = motor3 + ONE_SHOT_125_MIN;
	uint16_t motor_value4 = motor4 + ONE_SHOT_125_MIN;

	if( motor_value1 < ONE_SHOT_125_MAX && motor_value2 < ONE_SHOT_125_MAX
			&& motor_value3 < ONE_SHOT_125_MAX && motor_value4 < ONE_SHOT_125_MAX)
	{
		current_motor_values.motor1 = motor_value1;
		current_motor_values.motor2 = motor_value2;
		current_motor_values.motor3 = motor_value3;
		current_motor_values.motor4 = motor_value4;

		can_update_motor_values = true;
		return true;
	}

	return false;
}

static void nordic_hawk_motor_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
    	if(can_update_motor_values)
    	{
    		nordic_hawk_motor_seq_values.channel_0 = PWM_INVERT_POLARITY | current_motor_values.motor1;
    		nordic_hawk_motor_seq_values.channel_1 = PWM_INVERT_POLARITY | current_motor_values.motor2;
    		nordic_hawk_motor_seq_values.channel_2 = PWM_INVERT_POLARITY | current_motor_values.motor3;
    		nordic_hawk_motor_seq_values.channel_3 = PWM_INVERT_POLARITY | current_motor_values.motor4;
    		rtt_print_int("Motor value updated:", current_motor_values.motor4);
    		can_update_motor_values = false;
    	}

#ifdef ESC_CALIBRATION
    	esc_controller_calibration();
#endif

    }
}

//static uint16_t get_pwm_scaled_value(uint8_t value)
//{
//	uint16_t scaled_value = 500 + value * 5;
//
//	return scaled_value;
//}

#ifdef ESC_CALIBRATION
static void esc_controller_calibration()
{
	if(calibration_step == 0)
	{
		rtt_println("ESC Calibration is about to start, connect motor!\n");
	    nordic_hawk_motor_seq_values.channel_0 = PWM_INVERT_POLARITY | ONE_SHOT_125_MAX;// 250 us pulse width
	    nordic_hawk_motor_seq_values.channel_1 = PWM_INVERT_POLARITY | ONE_SHOT_125_MAX;
	    nordic_hawk_motor_seq_values.channel_2 = PWM_INVERT_POLARITY | ONE_SHOT_125_MAX;
	    nordic_hawk_motor_seq_values.channel_3 = PWM_INVERT_POLARITY | ONE_SHOT_125_MAX;
	    rtt_println("Max throttle value sent\n");
	}

	if(calibration_step == 10*second_unit - 1) //3 seconds
	{
	    nordic_hawk_motor_seq_values.channel_0 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;// 125 us pulse width
	    nordic_hawk_motor_seq_values.channel_1 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;
	    nordic_hawk_motor_seq_values.channel_2 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;
	    nordic_hawk_motor_seq_values.channel_3 = PWM_INVERT_POLARITY | ONE_SHOT_125_MIN;
	    rtt_println("Min throttle value sent\n");
	}


//	if(calibration_step == 20*second_unit - 1) //6 seconds
//	{
//		rtt_println("Calibration finished\n");
//		rtt_println("Test is about to start\n");
//	    nordic_hawk_motor_seq_values.channel_0 = PWM_INVERT_POLARITY | 3000;// 1.5 ms pulse width
//	    nordic_hawk_motor_seq_values.channel_1 = PWM_INVERT_POLARITY | 3000;
//	    nordic_hawk_motor_seq_values.channel_2 = PWM_INVERT_POLARITY | 3000;
//	    nordic_hawk_motor_seq_values.channel_3 = PWM_INVERT_POLARITY | 3000;
//	    rtt_println("Medium throttle value sent\n");
//	}

	if(calibration_step < 60*second_unit)
		calibration_step++;

}
#endif
