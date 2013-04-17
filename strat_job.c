#include <stdlib.h>
#include "strat_job.h"
#include <aversive.h>

SIMPLEQ_HEAD(, strat_job) jobs_queue;

void strat_schedule_job(void (*f)(void *), void *param) {
    struct strat_job *s = malloc(sizeof(struct strat_job));

    if(s == NULL)
        panic();

    s->f = f;
    s->param = param;

    SIMPLEQ_INSERT_TAIL(&jobs_queue, s, next);
}

int strat_job_pool_is_empty(void) {
    return SIMPLEQ_EMPTY(&jobs_queue);
}

void strat_do_job(void) { 
    struct strat_job *s;
    s = SIMPLEQ_FIRST(&jobs_queue);
    SIMPLEQ_REMOVE_HEAD(&jobs_queue, next);

    if(s->f)
        s->f(s->param);
}

