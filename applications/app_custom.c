/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "conf_general.h"

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define PEDELEC_STOP_TIMEOUT 500

// Threads
static THD_FUNCTION(pedelec_thread, arg);
static THD_WORKING_AREA(pedelec_thread_wa, 2048);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float rot_speed = 0;
static volatile bool prev_state = 1;
static volatile time_t prev_magnet_time = 0;
static volatile app_configuration config;

// function called when the pedelec triggers the PFS
static void set_speed(time_t now) {
  rot_speed = 2 * 3.14  / (12 * (now - prev_magnet_time));
}
static void reset_speed(void) {
  rot_speed = 0;
}
void app_custom_configure(app_configuration *conf) {
    config = *conf;
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
  /* PB5 = PFS */
	palSetPadMode(GPIOB, 5, PAL_MODE_INPUT_PULLUP);
  /* PC8 = CRUISE */
	palSetPadMode(GPIOC, 8, PAL_MODE_INPUT_PULLUP);

	chThdCreateStatic(pedelec_thread_wa, sizeof(pedelec_thread_wa),
			NORMALPRIO, pedelec_thread, NULL);

	stop_now = false;
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	stop_now = true;

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

static THD_FUNCTION(pedelec_thread, arg) {
	(void)arg;

	chRegSetThreadName("pedelec App");

	is_running = true;


	commands_init_plot("Sample", "Voltage");
	commands_plot_add_graph("Current");
	commands_plot_add_graph("Max");

	uint32_t sample = 0.0;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.
    bool current_state = palReadPad(GPIOB, 5);
		time_t magnet_time = ST2MS(chVTGetSystemTime());
    if(current_state != prev_state && prev_state == true){
      set_speed(magnet_time);
			prev_magnet_time = magnet_time;
    }
		if (magnet_time - prev_magnet_time >= PEDELEC_STOP_TIMEOUT) {
			reset_speed();
		}
		prev_state = current_state;

		float pedelec_current = utils_map((float)rot_speed, 0.0, 0.012, 0.0, 1.0);
		utils_truncate_number(&pedelec_current, 0.0, 1.0);

        float throttle_current = (float)ADC_Value[ADC_IND_EXT];
		throttle_current /= 4095;
		throttle_current *= V_REG;
        throttle_current = utils_map(throttle_current, config.app_adc_conf.voltage_start, config.app_adc_conf.voltage_end, 0.0, 1.0);
		utils_truncate_number(&throttle_current, 0.0, 1.0);
        utils_deadband(&throttle_current, config.app_adc_conf.hyst, 1.0);


        float current = pedelec_current >= throttle_current ? pedelec_current : throttle_current;
        // The ADC throttle curve also impacts the pedelec
		current = utils_throttle_curve(current, config.app_adc_conf.throttle_exp, config.app_adc_conf.throttle_exp_brake, config.app_adc_conf.throttle_exp_mode);
		mc_interface_set_current_rel(current);

		commands_plot_set_graph(0);
		commands_send_plot_points(sample % 2000, current);
		commands_plot_set_graph(1);
		commands_send_plot_points(sample % 2000, 1);
		++sample;

		chThdSleepMilliseconds(1);
	}
}
