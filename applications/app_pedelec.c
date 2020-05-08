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

// Threads
static THD_FUNCTION(pedelec_thread, arg);
static THD_WORKING_AREA(pedelec_thread_wa, 2048);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float rot_speed = 0;
static volatile bool prev_state = 1;
static volatile time_t prev_tick_time = 0;

// function called when the pedelec triggers the PFS
static void set_speed() {
  rot_speed = 2 * 3.14  / (12 * ST2MS(chVTGetSystemTime() - prev_tick_time));
  prev_tick_time = chVTGetSystemTime();
}
// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_pedelec_start(void) {

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
void app_pedelec_stop(void) {
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
	commands_plot_add_graph("PFS");
	commands_plot_add_graph("CRUISE");

	uint32_t sample = 0.0;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.
    bool current_state = palReadPad(GPIOB, 5);
    if(current_state != prev_state && prev_state == false){
      set_speed();
      prev_state = current_state;
    }

		commands_plot_set_graph(0);
		commands_send_plot_points(sample, rot_speed);
		// commands_plot_set_graph(1);
		// commands_send_plot_points(sample, palReadPad(GPIOC, 8));
		++sample;
		chThdSleepMilliseconds(10);
	}
}
