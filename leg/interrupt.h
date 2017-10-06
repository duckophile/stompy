#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

extern volatile int interrupts_enabled;
extern uint32_t isr_max;
extern uint32_t isr_min;
extern uint32_t isr_count;

int set_interrupt_state(int state);
void interrupt_setup(void);

#endif /* __INTERRUPT_H__ */
