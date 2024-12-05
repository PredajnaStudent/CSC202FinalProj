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
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_iomux.h"

//-----------------------------------------------------------------------------
// Define function prototypes used by the program
//-----------------------------------------------------------------------------
void ADC1_init(uint32_t reference);
uint32_t ADC1_in(uint8_t channel);
void OPA0_init();
void motor1_init(void);
void motor1_pwm_init(uint32_t load_value, uint32_t compare_value);
void motor1_pwm_enable(void);
void motor1_set_pwm_count(uint32_t count_value);

//-----------------------------------------------------------------------------
// Define symbolic constants used by the program
//-----------------------------------------------------------------------------
#define DEADZONE_LOWER              (1900)
#define DEADZONE_UPPER              (4095 - DEADZONE_LOWER)
#define DEADZONE_SIZE               (DEADZONE_UPPER - DEADZONE_LOWER)
#define ADC_TO_PERCENT(x)           (((x) * (100.0 / DEADZONE_LOWER)))
#define Motor1_LOAD_VALUE           (4000)
#define ADC_TO_SERVO_COUNT(x)       (600.0 - (((x * (500.0 - 100)) / 4095) + 100))
//-----------------------------------------------------------------------------
// Define global variables and structures here.
// NOTE: when possible avoid using global variables
//-----------------------------------------------------------------------------


// Define a structure to hold different data types

int main(void)
{
  clock_init_40mhz();
  launchpad_gpio_init();

  led_init();
  led_enable();

  I2C_init();
  lcd1602_init();
  lcd_clear();

  OPA0_init();
  ADC0_init(ADC12_MEMCTL_VRSEL_VDDA_VSSA);
  ADC1_init(ADC12_MEMCTL_VRSEL_VDDA_VSSA);

  motor0_init();
  motor0_pwm_init(Motor1_LOAD_VALUE, 0);
  motor0_pwm_enable();

  motor1_init();
  motor1_pwm_init(Motor1_LOAD_VALUE, 0);
  motor1_pwm_enable();
  

  uint16_t joystick_x;
  uint16_t joystick_y;
  uint8_t motor_pwm;
  uint16_t servo_pwm;
  uint16_t motor_speed;
  uint16_t steer_degrees;
  uint16_t ADC_light_value;

  bool run = true;

  char* lcd_x_message = "X:";
  char* lcd_y_message = "Y:";
  char* lcd_light_message = "L:";

  while(run)
  {
    joystick_x = ADC1_in(5); 
    joystick_y = ADC1_in(4); 
    ADC_light_value = ADC0_in(7); 

    if(joystick_x < DEADZONE_LOWER && joystick_x > DEADZONE_UPPER)
    {
      servo_pwm = 295;
    }
    else 
    {
      servo_pwm = ADC_TO_SERVO_COUNT(joystick_x);
    }

    motor1_set_pwm_count(servo_pwm);
    
    if(joystick_y < DEADZONE_LOWER)
    {
      motor_pwm = ADC_TO_PERCENT(DEADZONE_LOWER - joystick_y);
      //turn on motor in forwards direction (CW)
      led_off(LED_BAR_LD1_IDX);
      led_on(LED_BAR_LD2_IDX);

    }
    else if (joystick_y > DEADZONE_UPPER)
    {
      motor_pwm = ADC_TO_PERCENT(joystick_y - DEADZONE_UPPER);
      //turn on motor in backwards direction (CCW)
      led_on(LED_BAR_LD1_IDX);
      led_off(LED_BAR_LD2_IDX);
    } 
    else 
    {
      motor_pwm = 0;
      //turn off motor
      led_off(LED_BAR_LD1_IDX);
      led_off(LED_BAR_LD2_IDX);
    }

    motor0_set_pwm_dc(motor_pwm);

    lcd_set_ddram_addr(0x00);
    lcd_write_string(lcd_x_message);
    lcd_write_doublebyte(servo_pwm);

    lcd_set_ddram_addr(0x40);
    lcd_write_string(lcd_y_message);
    lcd_write_byte(motor_pwm);

    /*lcd_set_ddram_addr(0x8);
    lcd_write_string(lcd_light_message);
    lcd_write_byte(ADC_light_value);*/

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

//A variant of motor0_init, which creates a pwm used to control the servo position.
void motor1_init (void)
{

 // Set PA17 (LD7) for TIMA1_C0
  IOMUX->SECCFG.PINCM[LED7_IOMUX] = IOMUX_PINCM39_PF_TIMA1_CCP0| 
                                    IOMUX_PINCM_PC_CONNECTED;
  GPIOA->DOESET31_0 = LED7_MASK;

}/*motor1_init*/


void motor1_pwm_init(uint32_t load_value, uint32_t compare_value)
{
  // Reset TIMA1
  TIMA1->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W | 
                           GPTIMER_RSTCTL_RESETSTKYCLR_CLR |
                           GPTIMER_RSTCTL_RESETASSERT_ASSERT);

  // Enable power to TIMA1
  TIMA1->GPRCM.PWREN = (GPTIMER_PWREN_KEY_UNLOCK_W | 
                          GPTIMER_PWREN_ENABLE_ENABLE);

  clock_delay(24);

  TIMA1->CLKSEL = (GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE | 
                   GPTIMER_CLKSEL_MFCLK_SEL_DISABLE |  
                   GPTIMER_CLKSEL_LFCLK_SEL_DISABLE);

  TIMA1->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_8;

  // set the pre-scale count value that divides select clock by PCNT+1
  // TimerClock = BusCock / (DIVIDER * (PRESCALER))
  // 200,000 Hz = 40,000,000 Hz / (8 * (24 + 1))
  TIMA1->COMMONREGS.CPS = GPTIMER_CPS_PCNT_MASK & 0x18;

  // Set action for compare
  // On Zero, set output HIGH; On Compares up, set output LOW
  TIMA1->COUNTERREGS.CCACT_01[0] = (GPTIMER_CCACT_01_FENACT_DISABLED | 
        GPTIMER_CCACT_01_CC2UACT_DISABLED | GPTIMER_CCACT_01_CC2DACT_DISABLED |
        GPTIMER_CCACT_01_CUACT_CCP_LOW | GPTIMER_CCACT_01_CDACT_DISABLED | 
        GPTIMER_CCACT_01_LACT_DISABLED | GPTIMER_CCACT_01_ZACT_CCP_HIGH);

  // set timer reload value
  TIMA1->COUNTERREGS.LOAD = GPTIMER_LOAD_LD_MASK & (load_value - 1);

  // set timer compare value
  TIMA1->COUNTERREGS.CC_01[0] = GPTIMER_CC_01_CCVAL_MASK & compare_value;

  // set compare control for PWM func with output initially low
  TIMA1->COUNTERREGS.OCTL_01[0] = (GPTIMER_OCTL_01_CCPIV_LOW | 
                GPTIMER_OCTL_01_CCPOINV_NOINV | GPTIMER_OCTL_01_CCPO_FUNCVAL);
  //
  TIMA1->COUNTERREGS.CCCTL_01[0] = GPTIMER_CCCTL_01_CCUPD_IMMEDIATELY;


  // When enabled load 0, set timer to count up
  TIMA1->COUNTERREGS.CTRCTL = GPTIMER_CTRCTL_CVAE_ZEROVAL | 
                 GPTIMER_CTRCTL_REPEAT_REPEAT_1 | GPTIMER_CTRCTL_CM_UP;

  TIMA1->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED;

  // No interrupt is required
  TIMA1->CPU_INT.IMASK = GPTIMER_CPU_INT_IMASK_L_CLR;

  // set C0 as output
  TIMA1->COMMONREGS.CCPD =(GPTIMER_CCPD_C0CCP3_INPUT | 
         GPTIMER_CCPD_C0CCP2_INPUT | GPTIMER_CCPD_C0CCP1_INPUT | 
         GPTIMER_CCPD_C0CCP0_OUTPUT);

} /* motor0_pwm_init */

void motor1_pwm_enable(void)
{
    TIMA1->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);
} /* motor_pwm_enable */

void motor1_set_pwm_count(uint32_t count)
{
  TIMA1->COUNTERREGS.CC_01[0] = GPTIMER_CC_01_CCVAL_MASK & count;
} /* motor0_set_pwm_count */