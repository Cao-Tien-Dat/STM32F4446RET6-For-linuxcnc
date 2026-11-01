/*
 * module.c — Module base class implementation
 * Port of Mbed modules/module.cpp
 */

#include "module.h"

void module_init(Module *m, int32_t thread_freq, int32_t slow_update_freq)
{
    if (!m) return;
    m->threadFreq = thread_freq;
    m->slowUpdateFreq = slow_update_freq;
    m->counter = 0;
    m->updateCount = (slow_update_freq > 0) ?
        (thread_freq / slow_update_freq) : 1;
    m->update = NULL;
    m->updatePost = NULL;
    m->slowUpdate = NULL;
    m->configure = NULL;
    m->priv = NULL;
}

/* module_run and module_run_post are defined as static inline in module.h */
