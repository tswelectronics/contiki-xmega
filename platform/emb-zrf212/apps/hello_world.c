#include "common.h"
PROCESS(hello_world_process, "Hello world process");
//AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
	printf("Hello, I'm \"Hello world process\":\n my greetings\n");

  printf("Hello, world\n");
  
  PROCESS_END();
}

