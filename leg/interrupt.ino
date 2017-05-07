/* ; -*- mode: C ;-*- */

#define TIE 0x2
#define TEN 0x1

uint32_t isr_max = 0;
uint32_t isr_min = 1 << 31;

void pit0_isr(void)
{
    uint32_t start_time, end_time, total_time;

    start_time = micros();

    adjust_pwms();

    velocity_loop();

    end_time = micros();

    total_time = end_time - start_time;

    if (total_time < isr_min)
        isr_min = total_time;
    if (total_time > isr_max)
        isr_max = total_time;

    PIT_TFLG0 = 1;
}

void interrupt_setup(void)
{
/*    pinMode(13,OUTPUT);*/
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = 0x007A120; /* 500000 - about 100hz. */
    PIT_TCTRL0 = TIE;
    PIT_TCTRL0 |= TEN;
    PIT_TFLG0 |= 1;
}
