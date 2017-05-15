/* ; -*- mode: C ;-*- */

#define TIE 0x2
#define TEN 0x1

/*
 * XXX fixme: I need some form of mutex to keep the ISR and main loop
 * from colliding.  Maybe just disable interrupts in the main loop
 * when needed?
 */

void pit0_isr(void)
{
    uint32_t start_time, end_time, total_time;

    start_time = micros();

    velocity_loop();

    /* This is supposed to do acceleration, which is currently disabled. */
    adjust_pwms();

    end_time = micros();

    if (end_time > start_time) { /* Avoid micros() wrapping every 1:11 */
        total_time = end_time - start_time;

        if (total_time < isr_min)
            isr_min = total_time;
        if (total_time > isr_max)
            isr_max = total_time;
    }
    isr_count++;

    PIT_TFLG0 = 1;

    return;
}

void disable_interrupts(void)
{
    PIT_TCTRL0 &= ~TIE;
    PIT_TCTRL0 &= ~TEN;
    PIT_TFLG0 &= ~1;

    return;
}

void enable_interrupts(void)
{
    PIT_TCTRL0 = TIE;
    PIT_TCTRL0 |= TEN;
    PIT_TFLG0 |= 1;

    return;
}

void interrupt_setup(void)
{
/*    pinMode(13,OUTPUT);*/
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = 0x007A120; /* 500000 - about 100hz. */

    enable_interrupts();

    return;
}
