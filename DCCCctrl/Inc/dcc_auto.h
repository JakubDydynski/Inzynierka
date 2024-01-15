#include "dccctrl_config.h"

struct autostep_ {
	uint8_t stime;	// step time in seconds, 0 = back to 1st step
	uint8_t locnum;
	_Bool rev;
	uint8_t speed;
	uint32_t f0_28;
	uint8_t itof;
};

extern struct autostep_ autopgm[NAUTOSTEPS];
extern _Bool itof_flag;
extern uint8_t curr_step;
extern 
uint8_t curr_step_time;
extern _Bool autorun_active;

void autorun_start(void);
void autorun_stop(void);
void autorun(void);
