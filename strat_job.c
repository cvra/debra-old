#include <stdlib.h>
#include "strat_job.h"
#include <aversive.h>

struct strat_job {
    int (*f)(void *);
    void *param;
    int active;
};

struct strat_job *jobs = NULL;
int job_count = 0;
int active_count = 0;

void strat_schedule_job(void (*f)(void *), void *param) {

    job_count++;
    active_count++;
    jobs = realloc(jobs, job_count * sizeof(struct strat_job));

    if(jobs == NULL)
        panic();

    jobs[job_count-1].f = f;
    jobs[job_count-1].param = param;
}

int strat_job_pool_is_empty(void) {
    return active_count == 0;
}

void strat_do_jobs(void) { 
    int i=0;
    while(!strat_job_pool_is_empty()) {
        if(jobs[i].active)
            if(jobs[i].f(jobs[i].param)==0) {
                jobs[i].active = 0;
                active_count--;
            }

        i++;
        if(i>=job_count)
            i = 0;
    }
}

