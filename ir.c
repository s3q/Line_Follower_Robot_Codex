#include "ir.h"
#include "robot_pins.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void IR_Init(void)
{
    /*
     * Center IR analog input on P6.1.
     * P6.1 maps to ADC14 channel A14 on MSP432P401R.
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        IR_CENTER_PORT, IR_CENTER_PIN, GPIO_TERTIARY_MODULE_FUNCTION);

    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0U);
    ADC14_setResolution(ADC_14BIT);
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(
        ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, IR_CENTER_ADC_INPUT, false);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();
}

uint16_t IR_ReadRaw(void)
{
    ADC14_toggleConversionTrigger();
    while (ADC14_isBusy()) {
    }
    return (uint16_t)ADC14_getResult(ADC_MEM0);
}
