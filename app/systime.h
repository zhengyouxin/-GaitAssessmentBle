#ifndef _SYSTIME_H
#define _SYSTIME_H

void systime_init(void);
void sysRtc_init(void);

typedef struct 
{
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
	uint8_t  week;
}
systime_t;

#endif

