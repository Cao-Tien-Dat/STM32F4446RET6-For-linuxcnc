#ifndef MODULE_H
#define MODULE_H

#include <stdint.h>
#include <stddef.h>

/*
 * Module base class — bare-metal port of Mbed Module class.
 * All modules are derived from this base class.
 * update() runs at thread frequency, updatePost() runs after all modules
 * in the thread have updated, slowUpdate() runs at a divisor of thread freq.
 */

typedef struct Module Module;

struct Module {
    int32_t threadFreq;
    int32_t slowUpdateFreq;
    int32_t updateCount;
    int32_t counter;

    void (*update)(Module *self);
    void (*updatePost)(Module *self);
    void (*slowUpdate)(Module *self);
    void (*configure)(Module *self);
    void *priv;           /* module-specific private data */
};

/* Initialise a module (implemented in module.c) */
void module_init(Module *m, int32_t thread_freq, int32_t slow_update_freq);

/* Run one module cycle: handle slow update + main update */
static inline void module_run(Module *m)
{
    if (m == NULL) return;

    m->counter++;
    if (m->counter >= m->updateCount) {
        if (m->slowUpdate) m->slowUpdate(m);
        m->counter = 0;
    }
    if (m->update) m->update(m);
}

/* Run post-update (after all modules in thread) */
static inline void module_run_post(Module *m)
{
    if (m == NULL || !m->updatePost) return;
    m->updatePost(m);
}

#endif /* MODULE_H */
