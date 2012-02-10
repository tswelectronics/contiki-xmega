/**
 *
 */

#include <contiki.h>
#include <autostart.h>

PROCESS_NAME(hello_world_process);
PROCESS_NAME(on_chip_sensors_monitor_process);
PROCESS_NAME(on_chip_sensors_display_process);

AUTOSTART_PROCESSES(
	&hello_world_process,
	&on_chip_sensors_monitor_process,
	&on_chip_sensors_display_process
);

