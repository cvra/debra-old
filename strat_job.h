#ifndef _STRAT_JOB_H_
#define _STRAT_JOB_H_

#include "strat.h"
#include <aversive/queue.h>

/** Represents a single strategy job (tasks are for OS scheduling). */
struct strat_job {
    void (*f)(void *); /** The function to call. */
    void (*param); /** The parameter that will be provided to f. */
    SIMPLEQ_ENTRY(strat_job) next; /** The poinbter to the next task, should not be filled manually. */
};

/** Runs a single job from the task pool in a round robin fashion. */
void strat_do_job(void);

/** Adds a job to the pool. */
void strat_schedule_job(void (*f)(void *), void *param);

/** @returns 1 if the job pool is empty. */
int strat_job_pool_is_empty(void);
#endif
