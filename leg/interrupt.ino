/* ; -*- mode: C ;-*- */

#define TIE 0x2
#define TEN 0x1

void pit0_isr(void)
{
    adjust_pwms();

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
