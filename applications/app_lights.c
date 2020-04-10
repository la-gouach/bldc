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
static THD_FUNCTION(lights_thread, arg);
static THD_WORKING_AREA(lights_thread_wa, 2048);

// Private functions
static void light_command(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool lights_on = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_lights_start(void) {

  /* Configure GPIO pin : PC3 */
	palSetPadMode(GPIOC, 3, PAL_MODE_OUTPUT_PUSHPULL);

	chThdCreateStatic(lights_thread_wa, sizeof(lights_thread_wa),
			NORMALPRIO, lights_thread, NULL);

	stop_now = false;
	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"lights",
			"Turn the lights ON/OFF",
			"[v]",
			light_command);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_lights_stop(void) {
	terminal_unregister_callback(light_command);
	stop_now = true;

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

static THD_FUNCTION(lights_thread, arg) {
	(void)arg;

	chRegSetThreadName("Lights App");

	is_running = true;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

    if (lights_on) {
      palWritePad(GPIOC, 3, 1);
    } else {
      palWritePad(GPIOC, 3, 0);
    }

		chThdSleepMilliseconds(100);
	}
}

void app_lights_set_state(bool state) {
	commands_printf("Turning the lights %s\n", state ? "ON" : "OFF");
	lights_on = state;
}

// Callback function for the terminal command with arguments.
static void light_command(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);


		if (d != 0 && d != 1) {
      commands_printf("Invalid lights values. Expected 0 or 1\n");
		} else {
			app_lights_set_state((bool)d);
		}
  }
}
