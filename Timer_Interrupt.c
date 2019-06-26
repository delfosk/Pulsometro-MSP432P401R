
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
extern uint16_t estado;
 void T32_INT1_IRQHandler(void)
{

    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    int16_t vx = getSampleSensor();
    if(estado==0){
        CountPulse(vx);

    }
    else if(estado==1){
        FftPulse(vx);
    }
    else{
        dinamicpulse(vx);
    }





}

