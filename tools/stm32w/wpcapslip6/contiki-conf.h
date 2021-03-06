#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__
#include <stdint.h>
#define CCIF
#define CLIF

typedef uint8_t   u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef  int32_t s32_t;
typedef unsigned short uip_stats_t;

#define UIP_CONF_UIP_IP4ADDR_T_WITH_U32 1

typedef unsigned long clock_time_t;
#define CLOCK_CONF_SECOND 1000

#endif /* __CONTIKI_CONF_H__ */
