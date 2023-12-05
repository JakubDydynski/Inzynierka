#include "dcc_tx.h"
#include "dcc_auto.h"

struct autostep_ __attribute__((aligned)) autopgm[NAUTOSTEPS];

uint8_t curr_step;
uint8_t curr_step_time;
_Bool autorun_active;

static void autorun_exec(void)
{
	curr_step_time = 0;
	// set state
	const struct autostep_ *sp = &autopgm[curr_step];
	uint8_t l = sp->locnum;
	loco[l].rev = sp->rev;
	loco[l].dspeed = sp->speed;
	loco[l].fun.w[0] = sp->f0_28;
}

void autorun_start(void)
{
	curr_step = 0;
	if (autopgm[0].stime)
	{
		autorun_active = 1;
		autorun_exec();
	}
}

void autorun_stop(void)
{
	autorun_active = 0;
}

// called every second
void autorun(void)
{
	if (autorun_active && ++curr_step_time == autopgm[curr_step].stime)
	{
		if (++curr_step == NAUTOSTEPS || autopgm[curr_step].stime == 0)
			curr_step = 0;
		if (autopgm[curr_step].stime == 0)
			autorun_active = 0;
		else
			autorun_exec();
	}
}
