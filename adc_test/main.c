//*****************************************************************************
//*****************************    C Source Code    ***************************
//*****************************************************************************
//  DESIGNER NAME:  TBD
//
//       LAB NAME:  TBD
//
//      FILE NAME:  main.c
//
//-----------------------------------------------------------------------------
//
// DESCRIPTION:
//    This program serves as a ... 
//
//*****************************************************************************
//*****************************************************************************

//-----------------------------------------------------------------------------
// Loads standard C include files
//-----------------------------------------------------------------------------
#include <stdio.h>

//-----------------------------------------------------------------------------
// Loads MSP launchpad board support macros and definitions
//-----------------------------------------------------------------------------
#include <ti/devices/msp/msp.h>
#include "LaunchPad.h"
#include "clock.h"
#include "adc.h"
#include "lcd1602.h"

//-----------------------------------------------------------------------------
// Define function prototypes used by the program
//-----------------------------------------------------------------------------
void ADC1_init(uint32_t reference);
uint32_t ADC1_in(uint8_t channel);
void OPA0_init();

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define DEADZONE_LOWER        (1900)
#define DEADZONE_UPPER        (4095 - DEADZONE_LOWER)
#define DEADZONE_SIZE         (DEADZONE_UPPER - DEADZONE_LOWER)
#define ADC_TO_PERCENT(x)     (((x)*(100.0/DEADZONE_LOWER)))

//-----------------------------------------------------------------------------
// Define global variables and structures here.
// NOTE: when possible avoid using global variables
//-----------------------------------------------------------------------------


// Define a structure to hold different data types

int main(void)
{
  clock_init_40mhz();
  launchpad_gpio_init();

  I2C_init();
  lcd1602_init();
  lcd_clear();
  led_disable();

  OPA0_init();
  ADC0_init(ADC12_MEMCTL_VRSEL_VDDA_VSSA);
  ADC1_init(ADC12_MEMCTL_VRSEL_VDDA_VSSA);

  uint16_t joystick_x;
  uint16_t joystick_y;
  uint8_t motor_pwm;
  uint8_t servo_pwm;
  uint16_t motor_speed;
  uint16_t steer_degrees;
  uint16_t ADC_light_value;

  bool run = true;

  char* lcd_x_message = "X:";
  char* lcd_y_message = "Y:";
  char* lcd_light_message = "L:";

  while(run)
  {
    joystick_x = ADC1_in(1); //pa16 for adc1 channel 1
    joystick_y = ADC1_in(3); //pa18 for adc1 channel 3
    ADC_light_value = ADC0_in(7); //

    if(joystick_x < DEADZONE_LOWER)
    {
      servo_pwm = ADC_TO_PERCENT(DEADZONE_LOWER - joystick_x);
      //turn on servo in left direction
    } 
    else if (joystick_x > DEADZONE_UPPER)
    {
      servo_pwm = ADC_TO_PERCENT(joystick_x - DEADZONE_UPPER);
      //turn on servo in right direction
    } 
    else 
    {
      servo_pwm = 0;
      //turn off servo
    }
      
    if(joystick_y < DEADZONE_LOWER)
    {
      motor_pwm = ADC_TO_PERCENT(DEADZONE_LOWER - joystick_y);
      //turn on motor in forwards direction
    }
    else if (joystick_y > DEADZONE_UPPER)
    {
      motor_pwm = ADC_TO_PERCENT(joystick_y - DEADZONE_UPPER);
      //turn on motor in backwards direction
    } 
    else 
    {
      motor_pwm = 0;
      //turn off motor
    }

    lcd_set_ddram_addr(0x00);
    lcd_write_string(lcd_x_message);
    lcd_write_byte(servo_pwm);

    lcd_set_ddram_addr(0x40);
    lcd_write_string(lcd_y_message);
    lcd_write_byte(motor_pwm);

    lcd_set_ddram_addr(0x8);
    lcd_write_string(lcd_light_message);
    lcd_write_byte(ADC_light_value);

    msec_delay(50);

  }

 // Endless loop to prevent program from ending
 while (1);

} /* main */

void ADC1_init(uint32_t reference)
{
  // Reset ADC and VREF
  ADC1->ULLMEM.GPRCM.RSTCTL = (ADC12_RSTCTL_KEY_UNLOCK_W | 
                               ADC12_RSTCTL_RESETSTKYCLR_CLR | 
                               ADC12_RSTCTL_RESETASSERT_ASSERT);
  
  if(reference == ADC12_MEMCTL_VRSEL_INTREF_VSSA)
  {
    VREF->GPRCM.RSTCTL = 0xB1000003;
  } /* if */
  
  // Enable power ADC and VREF
  ADC1->ULLMEM.GPRCM.PWREN = (ADC12_PWREN_KEY_UNLOCK_W |
                              ADC12_PWREN_ENABLE_ENABLE);
  
  if(reference == ADC12_MEMCTL_VRSEL_INTREF_VSSA)
  {
    VREF->GPRCM.PWREN = 0x26000001;
  } /* if */
  
  clock_delay(24); // time for ADC and VREF to power up
  
  // Set ADC clock configuration
  ADC1->ULLMEM.GPRCM.CLKCFG = (ADC12_CLKCFG_KEY_UNLOCK_W | 
                               ADC12_CLKCFG_CCONSTOP_DISABLE | 
                               ADC12_CLKCFG_CCONRUN_DISABLE | 
                               ADC12_CLKCFG_SAMPCLK_ULPCLK); 

  // Set sampling clock frequency range
  ADC1->ULLMEM.CLKFREQ = ADC12_CLKFREQ_FRANGE_RANGE40TO48;
  
  // Configure ADC Control Register 0
  ADC1->ULLMEM.CTL0 = ADC12_CTL0_SCLKDIV_DIV_BY_8 | ADC12_CTL0_PWRDN_MANUAL |
                      ADC12_CTL0_ENC_OFF;

  // Configure Sample Time Compare 0 Register
  ADC1->ULLMEM.SCOMP0 = 0; // 8 sample clocks
  
  if(reference == ADC12_MEMCTL_VRSEL_INTREF_VSSA)
  {
    VREF->CLKSEL = 0x00000008; // bus clock
    VREF->CLKDIV = 0; // divide by 1

    // bit 8 SHMODE = off
    // bit 7 BUFCONFIG=0 for 2.4 (=1 for 1.4)
    // bit 0 is enable
    VREF->CTL0 = 0x0001;

    // bits 31-16 HCYCLE=0
    // bits 15-0 SHCYCLE=0
    VREF->CTL2 = 0;
    while((VREF->CTL1 & 0x01)==0){}; // wait for VREF to be ready
  } /* if */

} /* ADC1_init */

uint32_t ADC1_in(uint8_t channel)
{
  // Configure ADC Control Register 1
  ADC1->ULLMEM.CTL1 = (ADC12_CTL1_AVGD_SHIFT0 | ADC12_CTL1_AVGN_DISABLE |
                       ADC12_CTL1_SAMPMODE_AUTO | ADC12_CTL1_CONSEQ_SINGLE |
                       ADC12_CTL1_SC_STOP | ADC12_CTL1_TRIGSRC_SOFTWARE);
                       
  // Configure ADC Control Register 2
  ADC1->ULLMEM.CTL2 = (ADC12_CTL2_ENDADD_ADDR_00 | ADC12_CTL2_STARTADD_ADDR_00 |
                       ADC12_CTL2_SAMPCNT_MIN | ADC12_CTL2_FIFOEN_DISABLE |
                       ADC12_CTL2_DMAEN_DISABLE | ADC12_CTL2_RES_BIT_12 |
                       ADC12_CTL2_DF_UNSIGNED);

  // Configure Conversion Memory Control Register
  ADC1->ULLMEM.MEMCTL[0] =  ADC12_MEMCTL_WINCOMP_DISABLE | 
                      ADC12_MEMCTL_TRIG_AUTO_NEXT | ADC12_MEMCTL_BCSEN_DISABLE | 
                      ADC12_MEMCTL_AVGEN_DISABLE | ADC12_MEMCTL_STIME_SEL_SCOMP0 | 
                      ADC12_MEMCTL_VRSEL_VDDA_VSSA | channel;

  ADC1->ULLMEM.CTL0 |= ADC12_CTL0_ENC_ON;
  ADC1->ULLMEM.CTL1 |= ADC12_CTL1_SC_START; 
  
  clock_delay(2); // TODO: required for 80Mhz clock to work TBR TODO:

  volatile uint32_t *status_reg = (volatile uint32_t *)&(ADC1->ULLMEM.STATUS);

  // wait here until the conversion completes
  while((*status_reg & ADC12_STATUS_BUSY_MASK) == ADC12_STATUS_BUSY_ACTIVE);
  
  return ADC1->ULLMEM.MEMRES[0];

} /* ADC1_in */

void OPA0_init()
{
  OPA0->GPRCM.RSTCTL = (OA_RSTCTL_KEY_UNLOCK_W | OA_RSTCTL_RESETSTKYCLR_CLR |
                        OA_RSTCTL_RESETASSERT_ASSERT);

  OPA0->GPRCM.PWREN = (OA_PWREN_KEY_UNLOCK_W | OA_PWREN_ENABLE_ENABLE);

  clock_delay(24);

  OPA0->CFGBASE &= ~(OA_CFGBASE_GBW_HIGHGAIN);

  OPA0->CFGBASE |= ((uint32_t) OA_CFGBASE_GBW_HIGHGAIN);

  OPA0->CFG |= (OA_CFG_GAIN_MINIMUM | OA_CFG_MSEL_NC | OA_CFG_NSEL_EXTPIN0|
                OA_CFG_PSEL_EXTPIN0 | OA_CFG_OUTPIN_ENABLED | OA_CFG_CHOP_OFF);
  
  OPA0->CTL |= OA_CTL_ENABLE_ON;
}