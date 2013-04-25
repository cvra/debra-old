#ifndef _STRAT_JOB_H_
#define _STRAT_JOB_H_

#include "strat.h"
#include <aversive/queue.h>

/** Run all the jobs from the pool, until the pool is empty. */
void strat_do_jobs(void);

/** Adds a job to the pool. */
void strat_schedule_job(void (*f)(void *), void *param);

/** @returns 1 if the job pool is empty. */
int strat_job_pool_is_empty(void);
#endif
