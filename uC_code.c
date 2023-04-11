/*###########################################################################
 $TI Release: F2837xD Support Library v210 $
 $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
 $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
             http://www.ti.com/ ALL RIGHTS RESERVED $

###########################################################################*/

//
// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_device.h"     // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"   // F2837xD Examples Include File
#include "SFO_V8.h"
//#include <math.h>
//

float Kp_i = -4.5e+03;          //Designed at 29.2V 10A
float Ki_i = -9e+07;            //

//float Kp_v = 0.0183;           //10 Hz (without charging cable resistance)
//float Ki_v = 340;

float Kp_v = 0.0367;            //20 Hz (with charging cable resistance)
float Ki_v = 679;

//float Kp_v = 0.0601;          //30 Hz (without charging cable resistance)
//float Ki_v = 1.18e3;

//float Kp_v = 0.0734;          //40 Hz (with charging cable resistance)
//float Ki_v = 1.36e3;


//
// Timer interrupt function execution rates all in [us]
//
#define CpuTimer0_us_period	1.5625	//Currently 80*8 = 640 kHz
#define CpuTimer1_us_period	12.5	//Currently 80 kHz

#define ADC_AVG_NUM          16
#define ADC_AVG_LOG           4

//
// SCI Defines (Suitable for RPi4 Operation @ a baud rate of 115200
//
#define CPU_FREQ        200E6
#define LSPCLK_FREQ     CPU_FREQ/4
#define SCI_FREQ        115200         //specific for the new RPi3
#define SCI_PRD         ((LSPCLK_FREQ/(SCI_FREQ*8))-1)

volatile int k = 0;
volatile bool transmit = false, received = false;
volatile unsigned char tx_MSG = 0;
volatile unsigned char rx_MSG = 0, receive1 = 0;
unsigned char main_msg = 0;
volatile unsigned char MSG[] = {0,0,0,0};   //MSG[0] > received message from RPi4; MSG[1] = Vref_max; MSG[2] = Iref_max; MSG[3] = Vbat_min

volatile bool plugged_in = false; //To check plugged in or not

//(__1__)These are all the states that needs to be addressed
enum states {
    CHARGER_IS_AVAILABLE,
    AWAITING_BATTERY_CONNECTION,
    READY_TO_CHARGE,
    CHARGING_IN_PROGRESS,
    PLUGGED_FAULTY_CHARGER,
    BATTERY_FULLY_CHARGED,
    FAULTY_BATTERY,
    TERMINATED_BY_USER,
    UNPLUGGED_FAULTY_CHARGER
} state;

enum states charger_state = CHARGER_IS_AVAILABLE;     //State 1 == The charging station is available

//
// SCI Function Prototypes
//
interrupt void scibTxFifoIsr(void);
interrupt void scibRxFifoIsr(void);
void scib_fifo_init(void);
void charger_state_event(void);
void charger_state_execute(void);

//
// MAIN CODE DEFINITIONS WITHOUT SCI
//
volatile struct EPWM_REGS *ePWM[PWM_CH] =
{  &EPwm4Regs, &EPwm4Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
   &EPwm6Regs, &EPwm7Regs, &EPwm8Regs};

#define PWM_CH 9        // # of PWM channels
int MEP_ScaleFactor = 0;
uint16_t TBPRD = 0;
int TBPRD_new = 0, TBPRD_old = 0, CMPA_new = 0, CMPA = 0, CMPA_old = 0;

// Prototype functions
//void Sample_average(void); // SAMPLING
void fan_control();
void check_output_voltage();
void disable_and_reset_charger();
void check_input_voltage();
void reset_hardware_fault();
void pre_charge_ouput_capacitor();
void detect_fault(void);
void ramp_pwm(void);                // Initializing function
void CurrentLoop_Battery(void);
void VoltageLoop_Battery(void);     // PI Controller Loop

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);

// ADC configure, startup and data read functions
interrupt void adca1_isr(void);
interrupt void adcb1_isr(void);

void ConfigureADC(void);
void SetupADCSoftware(void);

//PWM functions
void InitEPwm4(void);           // Battery
void InitEPwmGpio_TZ(void);     // PWM gpio init
__interrupt void epwm4_tzint_isr(void);

long int Time_counter = 100000;
long temp_1 = 0, temp_2 = 0, temp_d = 0, temp_2d = 0;
uint16_t TBPRD_Counter = 0, TBPRD_fine = 0, TBPRD_fine_new = 0, TBPRD_fine_old = 0, TBPRD_HR_1 = 0, TBPRD_HR_2 = 0, TBPRD_fine_d = 0, CMPA_HR_2 = 0;

int i = 0;
int v_out_max_trip = 2300;      // Approximately 30 V
bool is_input_voltage_equal_380V = false;
bool load_connected = 0;         // Set S_Loop to zero for current control
bool inlet_fan = 0, outlet_fan = 0, pfc_fan = 0, sr_fan = 0;

//
//ADC calibration parameters
//
volatile long int i_out[ADC_AVG_NUM] = {0}, v_out[ADC_AVG_NUM] = {0};
volatile int v_bat[8] = {0}, v_in[8] = {0};
volatile int PFC_adc_temp = 0, GaN_adc_temp = 0, SR_adc_temp = 0;

float Vbat_factor = 0.008385, Vbat_offset = -0.04023;           // Updated on 29/5/2022 ()
float Vout_factor = 0.01325, Vout_offset = -0.1475;            // Updated on 31/5/2022 (Needed for control and protection)
float Iout_factor = 0.008289, Iout_offset = -3.783;      	    // Updated on 29/5/2022 (Needed for control)
//float Iout_factor = 0.006317, Iout_offset = -2.754;           // Updated on 29/5/2022 (Needed for control)
float T_factor = -0.08975, T_offset = 225.6;             // Updated on 29/5/2022 (Needed for control)

volatile float T_gan = 0, T_sr = 0, T_pfc = 0;

uint16_t ConversionCounta = 0;
uint16_t ConversionCountb = 0; //(needed for 2nd generation)

//
// Inner current control loop parameter definitions
//
float TS = 0;
float Omega_2_kHz = 0;
float PI = 3.14159;
float Kp_current_loop = 0, Ki_Ts_current_loop = 0, Kb_current_loop = 0;

volatile float Fref_new_kHz = 0, Fref_kHz_input = 170, fmax = 170, fmin = 98; 		// in kHz
uint16_t frequency_ref_KHz = 160;       // Initializing PWM frequency

volatile float ei_k = 0, ei_km1 = 0, e_k_i = 0, ey_km1_i = 0;
volatile float f_kHz_unsat_k = 0, f_kHz_sat_k = 0, f_kHz_sat_km1 = 0;

bool current_loop_control = 0, current_loop_RESET = 0;

volatile float Iout_meas = 0, Iout_meas_test = 0;
volatile float Iout_meas_INPUT = 0;
volatile int Vout_meas_INPUT = 0;

volatile long int i_out_avg = 0;

//
// Outer voltage control loop parameter definitions
//

float Kp_voltage_loop = 0, Ki_Ts_voltage_loop = 0, Kb_voltage_loop = 0;

volatile float ev_k = 0, ev_km1 = 0, e_k_v = 0, ey_km1_v = 0;
volatile float i_unsat_k = 0, i_sat_k = 0, i_sat_km1 = 0;

volatile float Vref_V = 23.5, Vref_min_V = 23.2, Vref_max_V = 29;
volatile float Vref_ft = 0;

volatile float Iref = 0;

float Imax = 10.5, Imin = 0; 	//11*2^17
bool voltage_loop_control = 0, voltage_loop_RESET = 0, RELAY = 0, MOSFET = 0;

volatile long int v_out_avg = 0, Vout_meas = 0, Vref = 0;

volatile float v_bat_V = 0, v_out_V = 0;
bool run_once = 1, run_once1 = 1;

//
//Software lowpass filter initialization (Ask Ujjwal for the reference)
//
float alpha = 0.015456, beta = 0.96908; // 400 Hz filter
volatile int v_out_avg_LPF = 0, v_out_avg_LPF_m1 = 0, v_out_avg_m1 = 0;

//float alpha_i = 0.33, beta_i = 0.33;            //12.73 kHz filter
//float alpha_i = 0.282, beta_i = 0.4361;        //10 kHz filter
float alpha_i = 0.1641, beta_i = 0.6718;        //5 kHz filter
volatile double i_out_avg_LPF = 0, i_out_avg_LPF_m1 = 0, i_out_avg_m1 = 0;

//
// Voltage reference ramp function
//
float delta_V = 0.0001, Vref_ramp_max_V = 29;


// end of initialization

void main(void)
{

//MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();

// Enable PWM1 through PWM8
   CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;


// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//

   InitEPwmGpio_TZ();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Only init the pins for the SCI-B port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
//
   GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

//
// All GPIO definitions (Except for SCI-B)
//
    EALLOW;
    GpioCtrlRegs.GPCPUD.bit.GPIO95 = 1;   // Disable pullup on GPIO122
    //GpioDataRegs.GPCSET.bit.GPIO95 = 0;   // Load output latch
    GpioCtrlRegs.GPCMUX2.bit.GPIO95 = 0;  // GPIO94 = GPIO94
    GpioCtrlRegs.GPCDIR.bit.GPIO95 = 0;   // GPIO94 = input

    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;   // Enable pullup on GPIO66 (Hardware fault reset)
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;   // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;  // GPIO66 = GPIO66
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = 1;   // GPIO66 = output
    GpioDataRegs.GPCDAT.bit.GPIO66 = 0;   // initially off

    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;   // Enable pullup on GPIO66 (Hardware fault reset)
    GpioDataRegs.GPCSET.bit.GPIO65 = 1;   // Load output latch
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;  // GPIO66 = GPIO66
    GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;   // GPIO66 = output
    GpioDataRegs.GPCDAT.bit.GPIO65 = 0;   // initially off

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pull-up on GPIO0 (Battery upper MOSFET)
    GpioDataRegs.GPASET.bit.GPIO0 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;  // GPIO0 = GPIO0
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;   // GPIO0 = output
    GpioDataRegs.GPADAT.bit.GPIO0 = 0;   // Initially OFF

    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pull-up on GPIO1 (Battery precharge relay)
    GpioDataRegs.GPASET.bit.GPIO1 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;  // GPIO1 = GPIO1
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;   // GPIO1 = output
    GpioDataRegs.GPADAT.bit.GPIO1 = 0;   // Initially OFF

    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pull-up on GPIO14 (PFC small fan)
    GpioDataRegs.GPASET.bit.GPIO14 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;  // GPIO14 = GPIO14
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;   // GPIO14 = output
    GpioDataRegs.GPADAT.bit.GPIO14 = 0;   // Initially OFF

    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // Enable pull-up on GPIO15 (inlet big fan)
    GpioDataRegs.GPASET.bit.GPIO15 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;  // GPIO15 = GPIO15
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;   // GPIO15 = output
    GpioDataRegs.GPADAT.bit.GPIO15 = 0;   // Initially OFF

    GpioCtrlRegs.GPDPUD.bit.GPIO122 = 0;   // Enable pull-up on GPIO22 (outlet big fan)
    GpioDataRegs.GPDSET.bit.GPIO122 = 1;   // Load output latch
    GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0;  // GPIO14 = GPIO14
    GpioCtrlRegs.GPDDIR.bit.GPIO122 = 1;   // GPIO14 = output
    GpioDataRegs.GPDDAT.bit.GPIO122 = 0;   // Initially OFF

    GpioCtrlRegs.GPDPUD.bit.GPIO123 = 0;   // Enable pull-up on GPIO23 (SR or GaN transistors fan)
    GpioDataRegs.GPDSET.bit.GPIO123 = 1;   // Load output latch
    GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0;  // GPIO14 = GPIO14
    GpioCtrlRegs.GPDDIR.bit.GPIO123 = 1;   // GPIO14 = output
    GpioDataRegs.GPDDAT.bit.GPIO123 = 0;   // Initially OFF
    EDIS;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0; // SYSCLKOUT equals EPWMCLK
    EDIS;

    InitSysPll(INT_OSC2,IMULT_40,FMULT_0,PLLCLK_BY_2);      //Read page 122 in the technical manual (External OSC = XTAL_OSC)

    InitCpuTimers();   // For this example, only initialize the Cpu Timers

    ConfigCpuTimer(&CpuTimer0, 200, CpuTimer0_us_period); 	//Currently 400 kHz
    ConfigCpuTimer(&CpuTimer1, 200, CpuTimer1_us_period); 	//Currently 80 kHz
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
//
//Configure the ADCs and power them up
//
    ConfigureADC();
    SetupADCSoftware();

//
// Initialize the SCI-B Port Device Peripherals:
//
    DELAY_US(30000000);       //>>Necessary to allow the RASPBERRY PI TO BOOT UP FIRST WITHOUT INTERFERING WITH THE uC UNIT
    scib_fifo_init();  // Init SCI-A


//
// Initialize PWM4
//
    EALLOW;
    EPwm4Regs.TZFRC.bit.OST = 1; // Disabling PWM
    EDIS;

    InitEPwm4();                //Grid

    EALLOW;

    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;

    PieVectTable.ADCA1_INT = &adca1_isr;                //function for ADCA interrupt 1
    PieVectTable.ADCB1_INT = &adcb1_isr;                //function for ADCB interrupt 1 (needed for the 2nd generation)

    PieVectTable.EPWM4_TZ_INT = &epwm4_tzint_isr;       //EPWM4 instead of EPWM1

    PieVectTable.SCIB_RX_INT = &scibRxFifoIsr;
    PieVectTable.SCIB_TX_INT = &scibTxFifoIsr;

    EDIS;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
	
	//
	// PI inner current controller initialization
	//
	TS = (CpuTimer1_us_period*1e-6);	//TS = 1/80 kHz
	Omega_2_kHz= 0.001/(2*3.14);
	Kp_current_loop = Kp_i*Omega_2_kHz;			//Initialize as float
	Ki_Ts_current_loop = Ki_i*TS*Omega_2_kHz;	//Initialize as float
	//Kb_current_loop = Ki_i*Omega_2_kHz*0.00001;
	Kb_current_loop = -1;        //This coefficient relates the output to the input. It needs to be scaled appropriately to be proportional to the input error

	//
	// PI outer voltage controller initialization
	//
	Kp_voltage_loop = Kp_v*Vout_factor;
	Ki_Ts_voltage_loop = Ki_v*TS*Vout_factor;	//Initialize as long integers
	//Kb_voltage_loop = Ki_v*Vout_factor;
	Kb_voltage_loop = 1;
	
	//Vref=floor(Vref_ft);
	Vref_ft = (Vref_V - Vout_offset)/Vout_factor;
	Vref = Vref_ft;
	if((Vref_ft - Vref) >= 0.5) Vref++;

    while (SFO() == 0) {}; // one time run to initialize MEP factor

//
// Enable interrupts required for this example
//
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;      // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;      // ADCB1
    PieCtrlRegs.PIEIER2.bit.INTx4 = 1;      // EPWM4 TZ
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 __interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // PIE Group 9, INT3  (for SCI-B)
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;      // PIE Group 9, INT4  (for SCI-B)

//
// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= M_INT1;      // Enable CPU INT1 which is connected to CPU-Timer 0:
    IER |= M_INT2;
    IER |= M_INT3;
    //IER |= M_INT13;   // An alternative to 0x1101 used for timer1
    IER |= 0x1101;      // Needed for timer1
    IER |= M_INT9;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    for(;;)
    {

        if(transmit==1){ScibRegs.SCICTL1.bit.TXENA = 1; transmit = 0;}      //Control transmit action

        charger_state_execute();              //Executes the commands of each state
        charger_state_event();                //Determines the state (needs to be modified with Zhansen)

        detect_fault();                       //Check for faults

//
//Continuously calculate the battery and filter capacitor voltage and check if the battery is plugged in or not
//
        v_bat_V = Vbat_factor*v_bat[0] + Vbat_offset;   //battery voltage
        v_out_V = Vout_factor*v_out_avg + Vout_offset;  //capacitor voltage

//
//Continuously check if a batter is detected or not and perform pre-charging based on that
//
        pre_charge_ouput_capacitor();

//
//This function controls the cooling fans based on the sensed temperatures
//
        fan_control();

//
//Ramp up Vref gradually
//
        if (Vref_V < Vref_ramp_max_V && voltage_loop_control == true )  Vref_V = Vref_V + delta_V;

        Vref_ft = (Vref_V - Vout_offset)/Vout_factor;
        Vref = Vref_ft;
        if((Vref_ft - Vref) >= 0.5) {Vref++;};

        int status;
        status = SFO_INCOMPLETE;
        while (status==SFO_INCOMPLETE) {
        status = SFO();
        }
        if(status != SFO_ERROR) { // IF SFO() is complete with no errors
        //EALLOW;
        //EPwm4Regs.HRMSTEP=MEP_ScaleFactor;
        //EDIS;
        }

    }
}

void ConfigureADC(void)
{
    EALLOW;

    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4 (needed for 2nd generation)

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //(needed for 2nd generation)

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1; //(needed for 2nd generation)

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; //(needed for 2nd generation)

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCSoftware - Setup ADC channels and acquisition window
//
void SetupADCSoftware(void)
{
    Uint16 acqps;

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //ADCA
    //
    EALLOW;


    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;      //SOC0 will convert pin A0 >> (Output rectified current)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  //sample window is acqps

    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;      //SOC0 will convert pin A2 >> (Output voltage)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;  //sample window is acqps

    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3;      //SOC0 will convert pin A3 >> (battery voltage)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;  //sample window is acqps

    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 1;      //SOC0 will convert pin A3 >> (DC-bus voltage)
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;  //sample window is acqps

    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;      //SOC0 will convert pin A4 >> (PFC Thermistor)
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;  //sample window is acqps

    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;      //SOC0 will convert pin A5 >> (GaN Thermistor)
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;  //sample window is acqps

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5; //end of SOC5 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
                                           //1 SYSCLK cycles

    AdcbRegs.ADCSOC6CTL.bit.TRIGSEL = 1;    //trigger on Timer 0 SOCA/C
    AdcbRegs.ADCSOC6CTL.bit.CHSEL = 3;      //SOC6 will convert pin B3  >> (SR Thermistor)
    AdcbRegs.ADCSOC6CTL.bit.ACQPS = acqps;  //sample window is acqps +

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 6;  //end of SOC6 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared

    EDIS;
}

void InitEPwmGpio_TZ(void)
{

    InitEPwm4Gpio();

}

void InitEPwm4(void)
{
    EALLOW;

        TBPRD_Counter=Time_counter/frequency_ref_KHz; // Calculation
        EPwm4Regs.TBPRD = TBPRD_Counter;                 // Set timer period
        EPwm4Regs.TBPRDHR = 0x0000;
        EPwm4Regs.TBPHS.bit.TBPHS == 0x0000;
       // EPwm4Regs.TBPHS.half.TBPHS = 0x0000;            //  Used for Sync. of multiple PWM channels, we are using only one channel (2 output) , It os already disabled below
        EPwm4Regs.TBCTR = 0x0000;                       // Clear counter
        EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;             // set Shadow load so that TBPRD is safe this way

        // Setup TBCLK
        EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count updown
        EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable  loading // To enable
        EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
        EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow so we can observe on the scope
        EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;// Recommended for High Resolution : zero or Comp B

        /*
         The PWM Uses EPWM1A to create EPWM1B signal (It does not use EPWM1B)
         Refer to fig. 3.29
         The PWM is initialised to active high. The Deadband full enable allows Dead band on both rising edges
         */
        // Load registers every ZERO
        EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // This allows shadowing of Duty cycle compare registers
        //EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW; // This allows shadowing of Duty cycle compare registers
        EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // The data will be transferred when counter is zero
        //EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // The data will be transferred when counter is zero

        // Setup compare
        EPwm4Regs.CMPA.bit.CMPA =TBPRD_Counter>>1; //New line
        //EPwm4Regs.CMPA.half.CMPA = TBPRD_Counter>>1;      // Duty cycle initialized to 50%
        EPwm4Regs.CMPA.bit.CMPAHR = 0x00;       // initialize HRPWM extension for duty cycle 1<<8

        // Set actions
        //Channel B
        EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;         // When Duty counter reaches the set value while incrementing : The PWM goes high
        EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // When Duty counter reaches the set value while decrementing : The PWM goes low

        // Active high complementary PWMs - Setup the band
        EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // Fig:3.29 | S1 and S0 are 1 and 1 | Both of them come from delay ckt
        EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // Fig:3.29 | It creates EPWM1B by inverting  and delaying EPWM1B
        EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;  // Fig:3.29 | It uses only EPWM1A as an input
        //EPwm4Regs.DBRED = EPWM1_MAX_DB; // This initializes dead band for rising edge
        //EPwm4Regs.DBFED = EPWM1_MAX_DB; // This initializes dead band for falling edge
        EPwm4Regs.DBRED.bit.DBRED=0x00018; // This initializes dead band for rising edge
        EPwm4Regs.DBFED.bit.DBFED = 0x000018;

        // Interrupt where we will change the deadband
        EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // The interrupt ISR happens after counter reaches 0 after decrementing
        EPwm4Regs.ETSEL.bit.INTEN = 0;                  // Enable INT
        EPwm4Regs.ETPS.bit.INTPRD = ET_DISABLE;             // Generate INT on 3rd event ET_DISABLE

        // Settings for Enabling/Disabling PWM
        //EPwm4Regs.TZCLR.all = 0; // No Effect
       // EPwm4Regs.TZCLR.bit.OST = 1; // Clears previous trip
        //EPwm4Regs.TZCLR.bit.INT = 1;
        //EPwm4Regs.TZSEL.bit.OSHT1 = 1;  // Initialize TZ1 for one-shot trip
        //EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
        //EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
        //EPwm4Regs.TZEINT.bit.OST = 1; // allowing interrupt
        EPwm4Regs.TZFRC.bit.OST= 1; // Disabling PWM, reset counter (TBCTR)  before allowing

        //HR Initialisation
        EPwm4Regs.HRCNFG.all = 0x0;
        EPwm4Regs.HRCNFG.bit.EDGMODE = HR_BEP;          // MEP control on both edges
        EPwm4Regs.HRCNFG.bit.CTLMODE = HR_CMP;          // CMPAHR and TBPRDHR HR control
        EPwm4Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD; // load on CTR = 0 and CTR = TBPRD
        EPwm4Regs.HRCNFG.bit.AUTOCONV = 1;  // Enable autoconversion for HR period then calculation for TBPRDHR using MEP steps is not needed

        EPwm4Regs.HRPCTL.bit.TBPHSHRLOADE = 1;  // Enable TBPHSHR sync (required for up-down count HR control even if phase control is not used)
        EPwm4Regs.HRPCTL.bit.HRPE = 1;    // Turn on high-resolution period or duty control.

        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Synchronizes TBCLK with the EPWM
        EPwm4Regs.TBCTL.bit.SWFSYNC = 1;      // This generates one time synchronization signal for EPWM

        EDIS;

    }

__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void)
{
    //GpioDataRegs.GPCDAT.bit.GPIO65 = 1;   // set high when enter interrupt (Used for precharge now)

//
// Check if the input voltage is sustained or not
//
    check_input_voltage();

//
// Fast reaction if battery is unplugged to disable the gate signals
//
    check_output_voltage();

//
// Taking Samples here does not obstruct data
//
    i_out_avg = (long int) (i_out[0] + i_out[1] + i_out[2] + i_out[3] + i_out[4] + i_out[5] + i_out[6] + i_out[7] + i_out[8] + i_out[9] + i_out[10] + i_out[11] + i_out[12] + i_out[13] + i_out[14] + i_out[15]) >> ADC_AVG_LOG;
    v_out_avg = (long int) (v_out[0] + v_out[1] + v_out[2] + v_out[3] + v_out[4] + v_out[5] + v_out[6] + v_out[7] + v_out[8] + v_out[9] + v_out[10] + v_out[11] + v_out[12] + v_out[13] + v_out[14] + v_out[15]) >> ADC_AVG_LOG;

//
// Nesting Interrupt (Nested Interrupts)
//
    IER |= 0x0001;                          //Enable interrupt groups
    IER &= 0x0001;                          // Disable everything else
    //PieCtrlRegs.PIEIER1.all &= 0x0041;    //adca, and timer0 isr
    PieCtrlRegs.PIEIER1.all &= 0x0043;      //adca, adcb, and timer0 isr
    //PieCtrlRegs.PIEIER9.bit.INTx3 = 1;    //SCI-B
    //PieCtrlRegs.PIEIER9.bit.INTx4 = 1;    //SCI-B
    PieCtrlRegs.PIEACK.all &= 0xFFFF;
    asm("    NOP");                         // Wait one cycle
    EINT;

//
//Software low-pass filter (Used in the currentController (bypass by setting Iout_meas = i_out_avg))
//
//    i_out_avg_LPF = alpha_i*(i_out_avg + i_out_avg_m1) + beta_i*i_out_avg_LPF_m1;   //(double) 12.73 kHz LPF
//    i_out_avg_LPF_m1 = i_out_avg_LPF;	//(double)
//    i_out_avg_m1 = i_out_avg;	//(double)
//    Iout_meas_test = (float) i_out_avg_LPF*Iout_factor + Iout_offset;	//(float)

	Iout_meas = i_out_avg*Iout_factor + Iout_offset;

// Low pass filter for voltage sampling
//     v_out_avg_LPF = alpha*(v_out_avg+v_out_avg_m1)+beta*v_out_avg_LPF_m1; // % 400 Hz LPF
//     v_out_avg_LPF_m1 = v_out_avg_LPF;
//     v_out_avg_m1 = v_out_avg;
//
//     Vout_meas = v_out_avg_LPF;
	Vout_meas = v_out_avg;

    //The controller can only start if a battery voltage that is more than 22 V is detected and a 380-V bus is established by the PFC
    if (load_connected == true) //&& is_input_voltage_equal_380V == true
    {
       if (current_loop_control == true && voltage_loop_control == true) // Dual loop control when both loops are on
           {
            VoltageLoop_Battery();
            CurrentLoop_Battery();

            current_loop_RESET = true; // Activate the reset controller
            voltage_loop_RESET = true;

            }
       else if (current_loop_control == true && voltage_loop_control == false) // Start integration : Saturated at minimum frequency, we want error to be negative so that frequency increases
           {

           CurrentLoop_Battery();
           current_loop_RESET = true;

           if (voltage_loop_RESET == true)
           {
               ev_km1 = 0;
               ey_km1_v = 0;
               i_sat_km1 = 0;
               Iref = 0;
               Vref_V = 23.5;
               voltage_loop_RESET = false; // RESET Parameters and OFF the reset parameters
              }
           }
       else if (current_loop_control == false && voltage_loop_control == false)
           {
               Fref_new_kHz = Fref_kHz_input; // Set frequency reference using Fref_kHz_input

               // Check the output - It should be in kHz
               if (Fref_new_kHz >= fmax)               //Up limit
                   {
                   Fref_new_kHz = fmax;
                   }

               if (Fref_new_kHz <= fmin)               //Up limit
                   {
                   Fref_new_kHz = fmin;
                   }

               if (current_loop_RESET == true)
                   {
                       // Here, we reset the current loop parameters when the controller is turned-false
                       ei_km1 = 0;
                       ey_km1_i = 0;
                       f_kHz_sat_km1 = 0;
                       Iref = 0;
                       current_loop_RESET = false; // RESET Parameters and OFF the reset parameters
                    }

               if (voltage_loop_RESET == true)
                   {
                       ev_km1 = 0;
                       ey_km1_v = 0;
                       i_sat_km1 = 0;
                       Iref = 0;
                       Vref_V = 23.5;
                       voltage_loop_RESET = false; // RESET Parameters and OFF the reset parameters
                    }
           }


       temp_1 = (6553600000/Fref_new_kHz) ;
       TBPRD = (temp_1)>>16;
       TBPRD_new = TBPRD;

       TBPRD_fine = (temp_1)>>16;
       temp_2 = temp_1 - ((long)TBPRD_fine<<16);
       TBPRD_HR_2 = temp_2 << 8; // The calculation is automatically done

       temp_d = temp_1>>1;
       CMPA = (temp_d)>>16;
       CMPA_new = CMPA;

       TBPRD_fine_d = (temp_d)>>16;
       temp_2d = temp_d - ((long)TBPRD_fine_d<<16);
       CMPA_HR_2 = temp_2d << 8;


       TBPRD_fine_new = TBPRD_HR_2;

        if (TBPRD_new != TBPRD_old || TBPRD_fine_new != TBPRD_fine_old)
        {
            EPwm4Regs.TBPRD =TBPRD_new;
            EPwm4Regs.CMPA.bit.CMPA = CMPA_new ;

            EPwm4Regs.TBPRDHR = TBPRD_HR_2;
            EPwm4Regs.CMPA.bit.CMPAHR = CMPA_HR_2;

        }

        TBPRD_old = TBPRD;
        TBPRD_fine_old = TBPRD_HR_2;
        CMPA_old = CMPA;
    }
      //GpioDataRegs.GPCDAT.bit.GPIO65 = 0;   // set low when exit interupt (used for precharge now)
      DINT;
}

interrupt void adca1_isr(void)
{
    //GpioDataRegs.GPCDAT.bit.GPIO94 = 1;   // set high when enter interupt
    //sensorSample1 = AdcaResultRegs.ADCRESULT0;
    i_out[ConversionCounta] = AdcaResultRegs.ADCRESULT0;        //ADC result collection
    v_out[ConversionCounta] = AdcaResultRegs.ADCRESULT1;        //ADC result collection
    v_bat[0] = AdcaResultRegs.ADCRESULT2;                       //ADC result collection (No need for averaging)
    v_in[0] = AdcaResultRegs.ADCRESULT3;                        //ADC result collection (No need for averaging)
    PFC_adc_temp = AdcaResultRegs.ADCRESULT4;
    GaN_adc_temp = AdcaResultRegs.ADCRESULT5;

    if (ConversionCounta == ADC_AVG_NUM-1)
    {
        ConversionCounta = 0;
    }
    else
        ConversionCounta++;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void adcb1_isr(void)
{

    SR_adc_temp = AdcbResultRegs.ADCRESULT6; //ADC result collection ()

    if (ConversionCountb == 7)
    {
        ConversionCountb = 0;
    }
    else
        ConversionCountb++;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //clear INT1 flag
    //AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //clear INT1 flag (I may need to find a solution for this or ask TI)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void CurrentLoop_Battery(void) // PI Controller Loop
{
   // GpioDataRegs.GPADAT.bit.GPIO5 = 1;

	//Calculate the error
	ei_k = Iref - Iout_meas;	//Difference

	//Back-calculation error
	e_k_i = ei_k + ey_km1_i;

	//(Unsaturated) Calculate output
	f_kHz_unsat_k = Kp_current_loop*(ei_k - ei_km1) + Ki_Ts_current_loop*e_k_i + f_kHz_sat_km1;

	//Check for saturation and the calculated saturated output
    if (f_kHz_unsat_k >= fmax)
    { 
        f_kHz_sat_k = fmax; 
    } 
    else if (f_kHz_unsat_k <= fmin)
    { 
        f_kHz_sat_k = fmin; 
    } 
    else 
    { 
         f_kHz_sat_k = f_kHz_unsat_k; 
    } 

	//Calculate and save variables for the next iteration
    ey_km1_i = Kb_current_loop*(f_kHz_sat_k - f_kHz_unsat_k);
    ei_km1 = ei_k;
	f_kHz_sat_km1 = f_kHz_sat_k;

	// Controller output reference current
	Fref_new_kHz = f_kHz_sat_k;

}

void VoltageLoop_Battery(void) // PI Controller Loop
// This function processes the ADC value of the output voltage 
// Kp and Ki parameters are divided by Vout_factor since Vout_meas is in ADC
{
	//GpioDataRegs.GPADAT.bit.GPIO5 = 1;

	//Calculate the error
	ev_k = Vref - Vout_meas;	// (volatile float) Difference of ADC values
    //ev_k = Vref - Vout_meas_INPUT;

	//Back-calculation error
	e_k_v = ev_k + ey_km1_v;		// (volatile double)

	//(Unsaturated) Calculate output
	i_unsat_k = Kp_voltage_loop*(ev_k - ev_km1) + Ki_Ts_voltage_loop*e_k_v + i_sat_km1; // Outputs current in [A]

	//Check for saturation and the calculated saturated output 
	if (i_unsat_k >= Imax) 
	{ 
		i_sat_k = Imax; 
	} 
	else if (i_unsat_k <= Imin) 
	{ 
		i_sat_k = Imin; 
	} 
	else 
	{ 
		i_sat_k = i_unsat_k; 
	} 

	//Calculate and save variables for the next iteration 
	ey_km1_v = Kb_voltage_loop*(i_sat_k - i_unsat_k);
	ev_km1 = ev_k;
	i_sat_km1 = i_sat_k;

	// Controller output reference current 
	Iref = i_sat_k; 

	//GpioDataRegs.GPADAT.bit.GPIO5 = 0;
}

void ramp_pwm(void)
{

    //fmax=125; //kHz
    temp_1 = (6553600000/fmax) ;        // Initializing with maximum frequency
    TBPRD = (temp_1)>>16;
    TBPRD_new = TBPRD;

    TBPRD_fine = (temp_1)>>16;
    temp_2 = temp_1 - ((long)TBPRD_fine<<16);
    TBPRD_HR_2 = temp_2 << 8;           // The calculation is automatically done

    temp_d = temp_1>>1;
    CMPA = (temp_d)>>16;
    CMPA_new = CMPA;

    TBPRD_fine_d = (temp_d)>>16;
    temp_2d = temp_d - ((long)TBPRD_fine_d<<16);
    CMPA_HR_2 = temp_2d << 8;

    EPwm4Regs.TBPRD = TBPRD_new;
    EPwm4Regs.TBPRDHR = TBPRD_HR_2;

    EPwm4Regs.CMPA.bit.CMPA = (int) CMPA_new*0.003; // 1.5% starting DUTY
    EPwm4Regs.CMPA.bit.CMPAHR = 0;

    EALLOW;

    EPwm4Regs.TZCLR.bit.OST = 1;    // Enable PWM
    EPwm4Regs.TZCLR.bit.INT = 1;    // CLears interrupt

    EDIS;

    for (i = 4; i<101; i++)
    {
    EPwm4Regs.CMPA.bit.CMPA = (int) CMPA_new*0.01*(i);
    EPwm4Regs.CMPA.bit.CMPAHR = (int) CMPA_HR_2*0.01*(i);
    DELAY_US(2000); // Total time is approximately 3 sec
    }

    //EPwm4Regs.TZCLR.bit.OST = 0;  // Enable PWM

}

__interrupt void epwm4_tzint_isr(void)
{
    //EPwm1TZIntCount++;

    //
    // To Re-enable the OST Interrupt, do the following:
     /*EALLOW;
     EPwm4Regs.TZCLR.bit.OST = 1;
     EPwm4Regs.TZCLR.bit.INT = 1;
     EDIS;*/

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}


void detect_fault(void){ // Add output over-current protection at 15A and 32V
    if((!GpioDataRegs.GPCDAT.bit.GPIO95 || !is_input_voltage_equal_380V) && plugged_in){
        main_msg = 8;
        tx_MSG = main_msg;
        transmit = 1;
        charger_state = PLUGGED_FAULTY_CHARGER;
}
    else if((!GpioDataRegs.GPCDAT.bit.GPIO95 || !is_input_voltage_equal_380V) && !plugged_in){
        main_msg = 12;
        tx_MSG = main_msg;
        transmit = 1;
        charger_state = UNPLUGGED_FAULTY_CHARGER;
}
}
//
// scia_fifo_init - Configure SCIB FIFO
//
void scib_fifo_init()
{
   ScibRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   ScibRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   ScibRegs.SCICTL2.bit.TXINTENA = 0;   //(0 Disable TXRDY interrupt)
   ScibRegs.SCICTL2.bit.RXBKINTENA = 1; // 1 Receiver-buffer break enable

   ScibRegs.SCIHBAUD.all = ((uint16_t)SCI_PRD  & 0xFF00U) >> 8U;        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   ScibRegs.SCILBAUD.all = (uint16_t)SCI_PRD  & 0x00FFU;                //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

   //ScibRegs.SCIHBAUD.all = 0x0000;
   //ScibRegs.SCILBAUD.all = SCI_PRD;

   //ScibRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back
   ScibRegs.SCIFFTX.all = 0xC022;
   ScibRegs.SCIFFRX.all = 0x0022;                                       //Try to understand this more thoroughly
   //ScibRegs.SCIFFRX.all = 0x0028;

   ScibRegs.SCIFFCT.all = 0x00;
   ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
   ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
   ScibRegs.SCICTL1.bit.TXENA = 0;
}

//
// scibRxFifoIsr - SCIB Receive FIFO ISR
//
interrupt void scibRxFifoIsr(void)   //Change to 4-byte buffer to receive Iref, Vref, and Vmin
                                     //Initialize Iref, Vref, and Vmin as zeros until you receive them from the RPi4 (Do the assignment one time by creating a flag for setting these values once)
                                     //Reset them after one charging cycle
{
    //ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    //int i;

    //rx_status = ScibRegs.SCIFFRX.bit.RXFFST;        //Ask a question about FIFO here

    rx_MSG = ScibRegs.SCIRXBUF.all;
    //receive1 = ScibRegs.SCIFFRX.bit.RXFFST;
    receive1 = ScibRegs.SCIFFRX.bit.RXFFINT;

    if(receive1 == 1){
        tx_MSG = rx_MSG;
        transmit = 1;

        switch(rx_MSG) {
           case 3 :
               MSG[0] = 3;              //Request charging
               break;

           case 7 :   // 69 = 'I_ref_max [A]'
               MSG[0] = 7;              //Start charging
               break;

           case 11 :   // 96 = 'V_char_max [V]'
               MSG[0] = 11;             //Stop charging
               break;

           case 15 :
               MSG[0] = 15;             //Reset because the wheelchair RPi4 is disconnected from Bluetooth
               break;

           case 118 :       //118 is the character value of 'v' >> Vref_max
               MSG[0] = 118;
               MSG[1] = ScibRegs.SCIRXBUF.all;
               Vref_ramp_max_V = MSG[1];
               break;

           case 105 :       //105 is the character value of 'i' >> Iref
               MSG[0] = 105;
               MSG[2] = ScibRegs.SCIRXBUF.all;
               Imax = MSG[2];
               break;

           case 115 :       //115 is the character value of 's' >> Vmin
               MSG[0] = 115;
               MSG[3] = ScibRegs.SCIRXBUF.all;
               break;

           default :
                MSG[0] = 0;
                MSG[1] = 0;
                MSG[2] = 0;
                MSG[3] = 0;
        }

        receive1 = 0;
    }

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

//
// scibTxFifoIsr - SCIB Transmit FIFO ISR
//
interrupt void scibTxFifoIsr(void)
{
    for(k=0; k<2; k++){
    ScibRegs.SCITXBUF.all = tx_MSG;     // Send data
    }

    ScibRegs.SCICTL1.bit.TXENA = 0;
    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
//(__2__) This function handles the events that trigger a state change. All events are triggered by the Rx message that is received from the RPi4.
//
void charger_state_event(){
    if(MSG[0]==3){ //MSG[0]==3 is received when the user requests to initiate the charging sequence
        if(charger_state==CHARGER_IS_AVAILABLE && !plugged_in){
            main_msg=5;
            tx_MSG = main_msg;
            transmit = 1;                                   //Send "Awaiting battery connection"
            charger_state = AWAITING_BATTERY_CONNECTION;   //State 5 == Waiting for the user to connect the battery to the charger
        }else if((charger_state==CHARGER_IS_AVAILABLE || charger_state==AWAITING_BATTERY_CONNECTION) && plugged_in){    //The OR here is to make it jump directly to READY_TO_CHARGE if the battery was plugged in before the user sends the request
            main_msg=6;          //Send "Ready to charge"
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = READY_TO_CHARGE;   //State 6 == Ready to start charging the wheelchair battery
        }
    }else if(MSG[0]==7){ //MSG[0]==7 is received when the user requests to start the charging process
        if(charger_state==READY_TO_CHARGE || charger_state==TERMINATED_BY_USER){    //The OR here is to allow the user to toggle start/stop the charging process as long as he is at this stage of the code (battery is secured and plugged in)
            main_msg = 7;
            tx_MSG = main_msg;       //Sends charging started
            transmit = 1;
            charger_state = CHARGING_IN_PROGRESS;  //State 7 == The charger is currently charging the battery
        }
    }else if(MSG[0]==11){ //MSG[0]==11 is received when the user requests to terminate the charging process
        main_msg=11;
        tx_MSG = main_msg;
        transmit = 1;
        charger_state = TERMINATED_BY_USER;  //State 11 == The charging process has been terminated by the user
    }else if(MSG[0]==15){ //MSG[0]==15 is received when the user disconnects his Bluethooth from the charging station. Once received, the charger RESETS and become available again for the next user.
        main_msg=1;
        tx_MSG = main_msg;
        transmit = 1;
        charger_state = CHARGER_IS_AVAILABLE;
    }else{         //MSG[0] always resets to 0 after every event trigger (if there is no special MSG[0] == {3,7,11,15} that is sent by the user)
        if(charger_state==AWAITING_BATTERY_CONNECTION && plugged_in){    //Plugged in the battery after requesting
            main_msg = 6;
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = READY_TO_CHARGE;   //State 6 == Ready to start charging the wheelchair battery
        }else if(charger_state==READY_TO_CHARGE && !plugged_in){ //Unplugged the battery after requesting
            main_msg=5;
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = AWAITING_BATTERY_CONNECTION;   //State 5 == Waiting for the user to connect the battery to the charger
        }else if(charger_state==CHARGING_IN_PROGRESS && !plugged_in){ //Unplugged the battery during the charging process
            main_msg=14;         //Battery has been unplugged by the user
            tx_MSG = 14;
            transmit = 1;
            charger_state = CHARGER_IS_AVAILABLE;  //Make the charging station available again
        }else if(charger_state==TERMINATED_BY_USER && !plugged_in){ //Battery is unplugged after the charging process was terminated by the user
            main_msg=14;
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = CHARGER_IS_AVAILABLE;  //Make the charging station available again
        }else if(charger_state==FAULTY_BATTERY && !plugged_in){// Battery fault resolved by unplugging the battery. Triggered if the battery voltage is not around 24V or if the battery voltage is less that its own minimum voltage (i.e., damaged battery).
            main_msg=14;
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = CHARGER_IS_AVAILABLE;  //Make the charging station available again
        }else if(charger_state==BATTERY_FULLY_CHARGED && !plugged_in){//Completed the charging process and unplugged the battery. This is triggered when the battery voltage exceeds the maximum allowed charging voltage (inside the battery voltage loop)
            main_msg = 14;
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = CHARGER_IS_AVAILABLE;  //Make the charging station available again
        }else if(charger_state==PLUGGED_FAULTY_CHARGER && !plugged_in){//Charger goes into the fault condition and the battery is unplugged. The fault function triggers the event the changes the charger state to PLUGGED_FAULTY_CHARGER.
            main_msg = 14;
            tx_MSG = main_msg;
            transmit = 1;
            charger_state = UNPLUGGED_FAULTY_CHARGER;                  //Now charger is faulty and unplugged
        }else{  //This is never executed
            tx_MSG = main_msg;
            transmit = 1;
        }
    }
    MSG[0] = 0;
}

//
//(__3__) execute commands relavent to each state
//
void charger_state_execute(){
    switch(charger_state) {
       case CHARGER_IS_AVAILABLE:   //The charger is available but not being used. PWMs, controller, and the PRE-CHARGER circuit are all disabled

           disable_and_reset_charger();
           run_once = true;                           //Reset "Run controller action"
            break;

       case AWAITING_BATTERY_CONNECTION:    //The charger is awaiting the user to plug in the battery so it remains in the CHARGER_IS_AVAILABLE state. PWMs, controller

           disable_and_reset_charger();
            break;

       case READY_TO_CHARGE:    //The battery is connected now after requesting charging, the charger becomes ready to start the charging process upon the user's request.
            //Check if battery voltage is incompatible with our 24-Volt Battery Charger
            //I think MSG[3] will eventually be removed
            if(v_bat_V <= MSG[3]){
               main_msg = 10;
               tx_MSG = main_msg;
               transmit = 1;
               charger_state = FAULTY_BATTERY;
            }else if(v_bat_V >= Vref_ramp_max_V){
                main_msg = 9;
                tx_MSG = main_msg;
                transmit = 1;
                charger_state = BATTERY_FULLY_CHARGED;
            }

            if(Vref_ramp_max_V > Vref_max_V)   Vref_ramp_max_V = Vref_max_V;
            else if(Vref_ramp_max_V < Vref_min_V){
                main_msg = 10;
                tx_MSG = main_msg;
                transmit = 1;
                charger_state = FAULTY_BATTERY;
            }

            if(Imax > 10.5)   Imax = 10.5;
            else if(Imax < 1.5){
                main_msg = 10;
                tx_MSG = main_msg;
                transmit = 1;
                charger_state = FAULTY_BATTERY;
            }

            break;

       case CHARGING_IN_PROGRESS:             //Start charging and go into charging-in-progress state (Verify Iref, Vref, and Vmin before charging the controller)
                            //We come here after the user allows the charging process to start

            if(run_once==true && load_connected==true){

               reset_hardware_fault();

               ramp_pwm();            // Ramp up duty cycle to 50 %

               current_loop_control = true;   // Turn-on the current loop first
               voltage_loop_control = true;   // Turn-on the voltage loop second

               run_once = false;
            }

            //During the charging process, once Iref and Iout_meas is less than 1.5 A, stop the charging process and change the charger state
            if(Iref <= 1.5 && Iout_meas <= 1.5 && Vref_V >= Vref_ramp_max_V){
                disable_and_reset_charger();
                charger_state = BATTERY_FULLY_CHARGED;
            }
            break;

       case TERMINATED_BY_USER:                 //The charging process is terminated by the user

           disable_and_reset_charger();
           run_once = true;
            break;

       case BATTERY_FULLY_CHARGED:            //Completed the charging process
           disable_and_reset_charger();
           run_once = true;                   //Should be able to run the controller again (if charger_state changes to 7)
            break;

       case FAULTY_BATTERY:                 //Fault

           disable_and_reset_charger();
           run_once = false;                  //Should not be able to run the controller again (if charger_state changes to 7)
            break;            //Should be able to run the controller again (if charger_state changes to 7)

       case PLUGGED_FAULTY_CHARGER:            //Fault

           disable_and_reset_charger();
           run_once = false;                      //Should not be able to run the controller again (if charger_state changes to 7)
            break;

       case UNPLUGGED_FAULTY_CHARGER:            //Fault

           disable_and_reset_charger();
           run_once = false;                      //Should not be able to run the controller again (if charger_state changes to 7)
            break;
       default:            //Charging is disabled by default / all battery relays/switches are open / dual-loop controller is OFF

           disable_and_reset_charger();
           run_once = true;
    }
}

void pre_charge_ouput_capacitor() {
    //
    //Perform pre-charging before starting the charging process
    //

    if(v_bat_V >= 5){  //If the battery voltage is larger than 5 volts
        plugged_in = true;
    }else{
        plugged_in = false;
        GpioDataRegs.GPADAT.bit.GPIO0 = 0;      // Turn on UPPER MOSFET
        GpioDataRegs.GPADAT.bit.GPIO1 = 0;      // Turn off Output Relay (for the precharge circuit)
        load_connected = false;                   //LOAD IS CONNECTED
        run_once1 = true;
    }

    if(run_once1 == true && plugged_in == true){
        if(v_out_V < 1){//Only do this when a battery is detected at the output
           GpioDataRegs.GPADAT.bit.GPIO1 = 1;   // Turn on Output Relay
           DELAY_US(500000);
        }

        if(v_out_V > 20){
           GpioDataRegs.GPADAT.bit.GPIO0 = 1;      // Turn on UPPER MOSFET
           DELAY_US(5000);
           GpioDataRegs.GPADAT.bit.GPIO1 = 0;      // Turn off Output Relay (for the precharge circuit)
           load_connected = true;                 //LOAD IS CONNECTED
           run_once1 = false;
        }else{
           load_connected = false;
        }
    }

}

//
//This function is needed to reset the hardware fault
//
void reset_hardware_fault() {
    GpioDataRegs.GPCDAT.bit.GPIO66 = 1;      //RESET hardware faults to enable gate driver
    DELAY_US(100000);
    GpioDataRegs.GPCDAT.bit.GPIO66 = 0;      //RESET hardware faults to enable gate driver

}

//
//As long as the DC bus voltage is maintained at 383 V (above 330 V == 2835), is_input_voltage_equal_380V should remain true.
//Otherwise, is_input_voltage_equal_380V will be false disabling the PWMs and preventing the controller from starting again.
//
void check_input_voltage() {
    if(v_in[0]>2835)    is_input_voltage_equal_380V = true;
    else{
        is_input_voltage_equal_380V = false;
    }
}

//
//This function is needed to disable the PWM signals once the battery is suddenly unplugged during charging.
//The voltage would suddenly increase beyond 30 V triggering this function.
//
void check_output_voltage() {
    if(v_out_avg > v_out_max_trip){
        disable_and_reset_charger();
    }
}

void disable_and_reset_charger(){

    EALLOW;
    EPwm4Regs.TZFRC.bit.OST= 1;         // Disabling PWM
    EDIS;

    voltage_loop_control = false;
    current_loop_control = false;
    voltage_loop_RESET = true;    // Make a controller reset function or modify initialization
    current_loop_RESET = true;

    //initialize = false;           // To stop calculation in control loop

    inlet_fan = false;
    outlet_fan = false;
    pfc_fan = false;
    sr_fan = false;

}

void fan_control(){

    T_sr = T_factor*SR_adc_temp + T_offset;
    T_gan = T_factor*GaN_adc_temp + T_offset;
    T_pfc = T_factor*PFC_adc_temp + T_offset;

    if(T_pfc >= 50) pfc_fan = true;
    else pfc_fan = false;

    if(T_sr >= 50) sr_fan = true;
    else sr_fan = false;

    if(T_gan >= 50 || T_sr >= 60 || T_pfc >= 60) {
        inlet_fan = true;
        outlet_fan = true;}
    else {
        inlet_fan = false;
        outlet_fan = false;}

    if(inlet_fan == 1) GpioDataRegs.GPADAT.bit.GPIO15 = 1;
    else GpioDataRegs.GPADAT.bit.GPIO15 = 0;

    if(outlet_fan == 1) GpioDataRegs.GPDDAT.bit.GPIO122 = 1;
    else GpioDataRegs.GPDDAT.bit.GPIO122 = 0;

    if(pfc_fan == 1) GpioDataRegs.GPADAT.bit.GPIO14 = 1;
    else GpioDataRegs.GPADAT.bit.GPIO14 = 0;

    if(sr_fan == 1) GpioDataRegs.GPDDAT.bit.GPIO123 = 1;
    else GpioDataRegs.GPDDAT.bit.GPIO123 = 0;

}
