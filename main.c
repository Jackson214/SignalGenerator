// EMBEDDED II PROJECT
// Jackson Spray

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

/*
Outputs - DAC R Value to Voltage
R=0 	| 2.97mV / 0.003V	| 4.541V
R=2048	| 1.021V			| -0.008V / -7.9mV
R=4095 	| 2.04V				| -4.90V
*/
/*
Gains - Used to calculate GainA and GainB
2.5V 	|   Ra = 1086	| 43E
0V 		|	Ra = 2026	| 7EA
-2.5V 	|   Ra = 2969	| B99

2.5V    |   Rb = 1098   | 44A
0V      |   Rb = 2045   | 7FD
-2.5V   |   Rb = 2990   | BAE

*/


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "spi1.h"
#include "wait.h"
#include "uart0.h"
#include "adc0.h"
#include "nvic.h"
#define _USE_MATH_DEFINES

// Pins
#define SSI1CLK PORTD,0
#define CS PORTD,1
#define SSI1TX PORTD,3
#define LDAC PORTB,5
#define AIN9_MASK 16
#define AIN8_MASK 32

// Variables
#define MAX_CHARS 7000
#define MAX_FIELDS 2050
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char buffer[10000];
int R = 0;
int Ra = 0x7EA;
int Rb = 0x7FD;
int GainA = -376.6;
int GainB = -378.4;
int LUTsize = 2048;
uint16_t LUTa[2048];
uint16_t LUTb[2048];
uint32_t phase_a = 0;
bool stopCyclesA = false;
uint32_t phase_b = 0;
bool stopCyclesB = false;
uint32_t delta_phase_a = 0;
uint32_t delta_phase_b = 0;
bool AisWave = false;
bool BisWave = false;
int Fs = 100000;
int cyclesA = -1;
int cyclesB = -1;
bool usingCyclesA = false;
bool usingCyclesB = false;
bool differential = false;
bool hilbert_b = false;
bool showVoltageA = false;
bool showVoltageB = false;
bool levelA = false;
bool levelB = false;
float levelMultiplierA = 1;
float levelMultiplierB = 1;
int arbArr[1024];
char* arbStr;
int levelCheckVoltage = 2;
bool isSine = false;
int phaseShift = 0;
char str[32];

// Structs
typedef struct _instruction
{
    uint8_t command;
    uint16_t arguments[MAX_FIELDS-1];
} instruction;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    initSystemClockTo40Mhz();

    enablePort(PORTB);
    enablePort(PORTD);
    enablePort(PORTE);

    initSpi1(USE_SSI_FSS);
    setSpi1BaudRate(20e6, 40e6);
    setSpi1Mode(0, 0);

    selectPinPushPullOutput(LDAC);
    setPinValue(LDAC, 1);

    GPIO_PORTE_AFSEL_R |= AIN9_MASK;                 // select alternative functions for AIN0 (PE4)
    GPIO_PORTE_DEN_R &= ~AIN9_MASK;                  // turn off digital operation on pin PE4
    GPIO_PORTE_AMSEL_R |= AIN9_MASK;                 // turn on analog operation on pin PE4

    GPIO_PORTE_AFSEL_R |= AIN8_MASK;
    GPIO_PORTE_DEN_R &= ~AIN8_MASK;
    GPIO_PORTE_AMSEL_R |= AIN8_MASK;

    initAdc0Ss3();
    setAdc0Ss3Mux(8); //Pin select, input is AIN number
    setAdc0Ss3Log2AverageCount(2);

    initAdc0Ss2();
    setAdc0Ss2Mux(9); //Pin select, input is AIN number
    setAdc0Ss2Log2AverageCount(2);

    initUart0();
    setUart0BaudRate(115200,40e6);

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;          	             // set load value to 400 for 100 kHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

}

char* getsUart0(instruction* instr){
    uint8_t count = 0;
    uint8_t empty = 0;
    char c;

    while(true){
        c = getcUart0();
        if((c == 8 || c == 127) && count > 0){
            count--;
            buffer[count] = empty;
        }
        else if(c == 13){
            buffer[count] = empty;
            return 0;
        }
        if(c >= 32 && c < 127){
            buffer[count] = c;
            count++;
            if(count == MAX_CHARS){
                buffer[count] = empty;
                return 0;
            }
        }
    }
}

char* parseFields(instruction* instr){
    int buffCt = 0;
    char c, prev;
    bool field = false;
    while(buffer[buffCt]){
        c = buffer[buffCt];
        if(buffCt > 0){
            prev = buffer[buffCt-1];
        }
        if(fieldCount == MAX_FIELDS){
            return 0;
        }
        if((buffCt > 0) && (prev >= 'A' && prev <= 'Z') || (prev >= '0' && prev <= '9') || (prev >= 'a' && prev <= 'z') || (prev == '-')){
            field = false;
        }
        else {
            field = true;
            if((c >= 'A' && c <= 'Z') && field == true){
                fieldPosition[fieldCount] = buffCt;
                fieldCount++;
                if(buffCt != 0){
                    buffer[buffCt-1] = '\0';
                }
            }
            if((c >= 'a' && c <= 'z') && field == true){
                fieldPosition[fieldCount] = buffCt;
                fieldCount++;
                if(buffCt != 0){
                    buffer[buffCt-1] = '\0';
                }
            }
            if((c >= '0' && c <= '9') && field == true){
                fieldPosition[fieldCount] = buffCt;
                fieldCount++;
                if(buffCt != 0){
                    buffer[buffCt-1] = '\0';
                }
            }
            if(c == '-' && field == true){
                fieldPosition[fieldCount] = buffCt;
                fieldCount++;
                if(buffCt != 0){
                    buffer[buffCt-1] = '\0';
                }
            }

            field = false;
            prev = c;
        }
        buffCt++;
    }
    return 0;
}

bool isCommand(instruction* instr, const char strCommand[], uint8_t minArguments){
    if(strcmp(buffer,strCommand) == 0 && fieldCount-1 >= minArguments){
        return true;
    }

    else {
        return false;
    }
}

char* getFieldString(instruction* instr, uint16_t fieldNumber){
    if(fieldNumber >= MAX_FIELDS){
        return NULL;
    }
    else {
        return &buffer[fieldPosition[fieldNumber]];
    }
}

int32_t getFieldInteger(instruction* instr, uint16_t fieldNumber){
    if(fieldNumber >= MAX_FIELDS){
        return NULL;
    }
    else {
        return arraytoint(instr, fieldNumber);
    }
}

int32_t arraytoint(instruction* instr, uint16_t fieldNumber){
    int i, value = 0;
    bool negative = false;
    for(i = fieldPosition[fieldNumber]; buffer[i] != '\0'; i++){
    	if (buffer[i] == '-'){
    		negative = true;
    	}
    	else{
    		value = (buffer[i] - '0') + (value * 10);
    	}
    }
    if (negative){
    	value = value * -1;
    }
    return value;
}

int32_t getArbNumber(uint16_t fieldNumber){
    int i, value = 0;
    int numCount = 0;
    for(i = fieldPosition[fieldNumber]; buffer[i] != '\0'; i++){
		value = (buffer[i] - '0') + (value * 10);
    }
    return value;
}

void stopA(void){
	AisWave = false;
	int R = Ra;
	R = R + 0x3000;
	writeSpi1Data(R);
    setPinValue(LDAC,0);
    _delay_cycles(4);
    setPinValue(LDAC,1);
}

void stopB(void){
	BisWave = false;
	int R = Rb;
	R = R + 0xB000;
	writeSpi1Data(R);
    setPinValue(LDAC,0);
    _delay_cycles(4);
    setPinValue(LDAC,1);
}

void differentialFunction(void){
	int i = 0;
    BisWave = true;
    putsUart0("Calculating LUT B\n");
    for(i=0; i < LUTsize; i++){
    	LUTb[i] = Rb - ((LUTa[i] - 0x3000) - Ra);
    	LUTb[i] = LUTb[i] + 0xB000;
    }
    delta_phase_b = delta_phase_a;
    phase_b = phase_a;
}

void hilbertFunction(void){
	int i = 0;
    BisWave = true;
    putsUart0("Calculating LUT B\n");
    for(i=0; i < LUTsize; i++){
    	LUTb[i] = Rb + ((LUTa[i] - 0x3000) - Ra);
    	LUTb[i] = LUTb[i] + 0xB000;
    }
    delta_phase_b = delta_phase_a;
    phase_a = 0;
    phase_b = phase_a + 0x60000000;
}

void stop_all(void){
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // Stop Timer
	showVoltageA = false;
	showVoltageB = false;
	AisWave = false;
	BisWave = false;
	R = Ra;
	R = R + 0x3000;
	writeSpi1Data(R);
	R = Rb;
	R = R + 0xB000;
	writeSpi1Data(R);
    setPinValue(LDAC,0);
    _delay_cycles(4);
    setPinValue(LDAC,1);
}

void timerIsr(void){
	if (AisWave){
		phase_a += delta_phase_a;
	    if ((phase_a >> 20) >= LUTsize){
	    	phase_a = 0;
	    	if (usingCyclesA){
	    		cyclesA--;
	    		if (cyclesA <= 0){
	    			cyclesA = -1;
	    			stopCyclesA = true;
	    			stopA();
	    		}
	    	}
	    	
	    }
	    if (!stopCyclesA){
	    	writeSpi1Data(LUTa[(phase_a >> 20)]);
	    }
	    stopCyclesA = false;
	}
	if (BisWave){
		phase_b += delta_phase_b;
	    if ((phase_b >> 20) >= LUTsize){
	    	phase_b = 0;
	    	if (usingCyclesB){
	    		cyclesB--;
	    		if (cyclesB <= 0){
	    			cyclesB = -1;
	    			stopCyclesB = true;
	    			stopB();
	    		}
	    	}
	    	
	    }
	    if (!stopCyclesB){
	    	writeSpi1Data(LUTb[(phase_b >> 20)]);
	    }
	    stopCyclesB = false;
	}
    setPinValue(LDAC,0);
    _delay_cycles(4);
    setPinValue(LDAC,1);
    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // clear interrupt flag
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void){
    initHw();
    putsUart0("BEGIN PROGRAM\n\n");
    instruction instr;
    uint8_t newCommand = 0;
    char* channel;
    char* toggle;
    int voltage = 0;
    int amplitude = 0;
    int freq = 0;
    int offset = 0;
    int N = 0;
    int dutyCycle = 50; // Percentage
    uint16_t raw = 0;
    uint16_t rawA = 0;
    uint16_t rawB = 0;
    int reqVoltsA = 0;
    int reqVoltsB = 0;
    float ADC_voltsA = 0;
    float ADC_voltsB = 0;
    float differenceA = 0;
    float differenceB = 0;
    int differenceR_A = 0;
    int differenceR_B = 0;
    int DC_A_R = 0;
    int DC_B_R = 0;

    //Set both outputs to 0V
    R = Ra;
    R = R + 0x3000;
    writeSpi1Data(R);
    R = Rb;
    R = R + 0xB000;
    writeSpi1Data(R);
    setPinValue(LDAC,0);
    _delay_cycles(4);
    setPinValue(LDAC,1);

	while(true) {
		
		if (showVoltageA){
			raw = readAdc0Ss3();
			ADC_voltsA = ((float)raw * (3.3/4095));
			sprintf(str, "Voltage A: %.4f", ADC_voltsA);
			putsUart0(str);
			putsUart0("\n");
		}
		if (showVoltageB){
			raw = readAdc0Ss2();
			ADC_voltsB = ((float)raw * (3.3/4095));
			sprintf(str, "Voltage B: %.4f", ADC_voltsB);
			putsUart0(str);
			putsUart0("\n");
		}

		waitMicrosecond(500000);
		if (kbhitUart0()){
			fieldCount = 0;
			getsUart0(&instr);
			putcUart0('\n');
			parseFields(&instr);
			newCommand = 1;
		}
		if (newCommand) {
			channel = NULL;
			voltage = -2;
			if(isCommand(&instr, "start", 0)){
				if (AisWave || BisWave){
					TIMER1_CTL_R |= TIMER_CTL_TAEN; // Start Timer
				}
	            setPinValue(LDAC,0);
                _delay_cycles(4);
                setPinValue(LDAC,1);
	        }
	        if(isCommand(&instr, "stop", 0)){
				stop_all();
	        }
			if(isCommand(&instr, "dc", 2)){
	            channel = getFieldString(&instr, 1);
	            voltage= getFieldInteger(&instr, 2);
	            if ((voltage > 5) || (voltage < -5)){
	            	putsUart0("Voltage cannot be more than 5V or less than -5V.\n");
	            	break;
	            }
	            else if (strcmp(channel,"a") == 0){
	            	reqVoltsA = voltage;
	                R = (GainA * voltage) + Ra;
	                R = R + 0x3000 + differenceR_A;
	                DC_A_R = R;
	            }
	            else if (strcmp(channel,"b") == 0){
	            	reqVoltsB = voltage;
	                R = (GainB * voltage) + Rb;
	                R = R + 0xB000 + differenceR_A;
	                DC_B_R = R;
	            }
	            writeSpi1Data(R); 
	        }
	        if(isCommand(&instr, "sine", 5)){
	            channel = getFieldString(&instr, 1);
	            freq = getFieldInteger(&instr, 2);
	            amplitude = getFieldInteger(&instr, 3);
	            offset = getFieldInteger(&instr, 4);
	            phaseShift = getFieldInteger(&instr, 5);
	            phase_b = 0;
	            phase_a = 0;
	            if (strcmp(channel,"a") == 0){
	                int i = 0;
	                int offsetR = 0;
	                float slice = 0;
	                AisWave = true;
	                offsetR = (GainA * offset) + Ra;
	                for(i=0; i < LUTsize; i++){
	                	slice = (float)i / LUTsize;
	                	LUTa[i] = offsetR + ((GainA * amplitude) * sin(2*M_PI*slice+(phaseShift*(M_PI/(float)180))));
	                	LUTa[i] = LUTa[i] + 0x3000; 
	                }
	                delta_phase_a = 4294967296 * ((float)freq / (float)Fs) / 2;
	               	phase_b = 0;
	            	phase_a = 0;
	                if (differential){
						differentialFunction();
					}
					if (hilbert_b){
						hilbertFunction();
					}
	            }
	            else if (strcmp(channel,"b") == 0){
	            	phase_b = 0;
	                int i = 0;
	                int offsetR = 0;
	                float slice = 0;
	                BisWave = true;
	                offsetR = (GainB * offset) + Rb;
	                for(i=0; i < LUTsize; i++){
	                	slice = (float)i / LUTsize;
	                	LUTb[i] = offsetR + ((GainB * amplitude) * sin(2*M_PI*slice+(phaseShift*(M_PI/(float)180))));
	                	LUTb[i] = LUTb[i] + 0xB000 + differenceR_A;
	                }
	                delta_phase_b = 4294967296 * ((float)freq / (float)Fs) / 2;
	            }
	        }
	        if(isCommand(&instr, "square", 4)){
	        	phase_a = 0;
	        	phase_b = 0;
	            channel = getFieldString(&instr, 1);
	            freq = getFieldInteger(&instr, 2);
	            amplitude = getFieldInteger(&instr, 3);
	            offset = getFieldInteger(&instr, 4);
	            dutyCycle = getFieldInteger(&instr, 5);
	            float dutyCycleFloat = (float)dutyCycle/100;
	            if (strcmp(channel,"a") == 0){
	                int i = 0;
	                int offsetR = 0;
	                AisWave = true;
	                int upperR = (GainA * amplitude);
	                for(i=0; i < LUTsize; i++){
	                	if (i <= (LUTsize * dutyCycleFloat)){
	                		LUTa[i] = (GainA * offset) + Ra + upperR;
	                	}
	                	else {
	                		LUTa[i] = (GainA * offset) + Ra - upperR;
	                	}
	                	LUTa[i] = LUTa[i] + 0x3000;
	                }
	                delta_phase_a = 4294967296 * ((float)freq / (float)Fs) / 2;
	                if (differential){
						differentialFunction();
					}
	            }
	            else if (strcmp(channel,"b") == 0){
	                int i = 0;
	                int offsetR = 0;
	                BisWave = true;
	                int upperR = (GainB * amplitude);
	                for(i=0; i < LUTsize; i++){
	                	if (i <= (LUTsize / 2)){
	                		LUTb[i] = (GainB * offset) + Rb + upperR;
	                	}
	                	else {
	                		LUTb[i] = (GainB * offset) + Rb - upperR;
	                	}
	                	LUTb[i] = LUTb[i] + 0xB000;
	                }
	                delta_phase_b = 4294967296 * ((float)freq / (float)Fs) / 2;
	            }
	        }
	        if(isCommand(&instr, "sawtooth", 3)){
	        	phase_a = 0;
	        	phase_b = 0;
	            channel = getFieldString(&instr, 1);
	            freq = getFieldInteger(&instr, 2);
	            amplitude = getFieldInteger(&instr, 3);
	            offset = getFieldInteger(&instr, 4);
	            if (strcmp(channel,"a") == 0){
	                int i = 0;
	                AisWave = true;
	                for(i=0; i < LUTsize; i++){
	                	LUTa[i] = ((GainA * ((float)(i-1024)/2048)) * amplitude) + (GainA * offset) + Ra;
	                	LUTa[i] = LUTa[i] + 0x3000;
	                }
	                delta_phase_a = 4294967296 * ((float)freq / (float)Fs) / 2;
	                if (differential){
						differentialFunction();
					}
	            }
	            else if (strcmp(channel,"b") == 0){
	                int i = 0;
	                BisWave = true;
	                for(i=0; i < LUTsize; i++){
	                	LUTb[i] = ((GainB * ((float)(i-1024)/2048)) * amplitude) + (GainB * offset) + Rb;
	                	LUTb[i] = LUTb[i] + 0xB000;
	                }
	                delta_phase_b = 4294967296 * ((float)freq / (float)Fs) / 2;
	            }
	        }
	        if(isCommand(&instr, "triangle", 3)){
	        	phase_a = 0;
	        	phase_b = 0;
	            channel = getFieldString(&instr, 1);
	            freq = getFieldInteger(&instr, 2);
	            amplitude = getFieldInteger(&instr, 3);
	            offset = getFieldInteger(&instr, 4);
	            if (strcmp(channel,"a") == 0){
	                int i = 0;
	                AisWave = true;
	                for(i=0; i < LUTsize; i++){
	                	if (i <= (LUTsize/2)){
	                		LUTa[i] = -((GainA * ((float)(i-1024)/1024)) * amplitude) + (GainA * offset) + Ra - (((float)amplitude/2) * GainA);
	                	}
	                	else {
	                		LUTa[i] = ((GainA * ((float)(i-1024)/1024)) * amplitude) + (GainA * offset) + Ra - (((float)amplitude/2) * GainA);
	                	}
	                	LUTa[i] = LUTa[i] + 0x3000;
	                }
	                delta_phase_a = 4294967296 * ((float)freq / (float)Fs) / 2;
	                if (differential){
						differentialFunction();
					}
	            }
	            else if (strcmp(channel,"b") == 0){
	                int i = 0;
	                BisWave = true;
	                for(i=0; i < LUTsize; i++){
	                	if (i <= (LUTsize/2)){
	                		LUTb[i] = -((GainB * ((float)(i-1024)/1024)) * amplitude) + (GainB * offset) + Rb - (((float)amplitude/2) * GainB);
	                	}
	                	else {
	                		LUTb[i] = ((GainB * ((float)(i-1024)/1024)) * amplitude) + (GainB * offset) + Rb - (((float)amplitude/2) * GainB);
	                	}
	                	LUTb[i] = LUTb[i] + 0xB000;
	                }
	                delta_phase_b = 4294967296 * ((float)freq / (float)Fs) / 2;
	            }
	        }
	        if(isCommand(&instr, "cycles", 2)){
	            channel = getFieldString(&instr, 1);
	            N = getFieldInteger(&instr, 2);
	            if (N < 0){
	            	putsUart0("Must select 1 or more cycles.\n");
	            	break;
	            }
	            else if (strcmp(channel,"a") == 0){
	                if (N <= 0){
	                	usingCyclesA = false;
	                	cyclesA = -1;
	                }
	                else {
	                	cyclesA = N;
	                	usingCyclesA = true;
	                }
	            }
	            else if (strcmp(channel,"b") == 0){
	                if (N <= 0){
	                	usingCyclesB = false;
	                	cyclesB = -1;
	                }
	                else {
	                	cyclesB = N;
	                	usingCyclesB = true;
	                }
	            }
	        }
	        if(isCommand(&instr, "differential", 1)){
				differential = getFieldInteger(&instr, 1);
				if (differential){
					differentialFunction();
				}
	            else {
	            	stopB();
	            }
	        }
	        if(isCommand(&instr, "voltage", 1)){
				channel = getFieldString(&instr, 1);
				if (strcmp(channel,"a") == 0){
					showVoltageA = !showVoltageA;
				}
				if (strcmp(channel,"b") == 0){
					showVoltageB = !showVoltageB;
				}
	        }
	        if(isCommand(&instr, "level", 1)){
	        	AisWave = false;
				toggle = getFieldString(&instr, 1);
				stop_all();
				if (strcmp(toggle,"on") == 0){
					R = (GainA * levelCheckVoltage) + Ra;
		            R = R + 0x3000;
		            DC_A_R = R;
		            writeSpi1Data(R);
		            setPinValue(LDAC,0);
		            _delay_cycles(4);
		            setPinValue(LDAC,1);
		            waitMicrosecond(1000000);

					rawA = readAdc0Ss3();
					rawB = readAdc0Ss2();
					sprintf(str, "Ra: %d | Rb = %d\n", rawA, rawB);
					putsUart0(str);
					int16_t levelDiff = rawA - rawB;
					if(levelDiff < 0){
						levelDiff = levelDiff * -1;
					}
					sprintf(str, "Difference in R: %d\n", levelDiff);
					putsUart0(str);
					float voltDiff = ((float)levelDiff * (3.3/4095));

					sprintf(str, "Difference in RMS Voltage: %.4fV given 2 Volts\n", voltDiff);
					putsUart0(str);

				}

			}
			if (isCommand(&instr, "reset", 0)){
				stop_all();
				__asm("    .global _c_int00\n"
          			  "    b.w     _c_int00");
			}
			if(isCommand(&instr, "gain", 2)){
				int freqStart = getFieldInteger(&instr, 1);
				int freqEnd = getFieldInteger(&instr, 2);
				int numDecades = log10((double)(freqEnd/freqStart));
				float steps = 30/numDecades;
				float stepSize = (pow(10, (1/steps)))-1;
				int i = 0;
				float freq[31];
				float dbArr[31];
				float reqFreq = freqStart;
				char testChar[64];
				for(i=0; i < 31; i++){
					freq[i] = reqFreq;
					reqFreq = reqFreq + (reqFreq * stepSize);
				}
				i = 0;
				int offsetR = 0;
                float slice = 0;
                AisWave = true;
                offsetR = Ra;
                for(i=0; i < LUTsize; i++){
                	slice = (float)i / LUTsize;
                	LUTa[i] = offsetR + ((GainA * 2) * sin(2*M_PI*slice));
                	LUTa[i] = LUTa[i] + 0x3000; 
                }
                TIMER1_CTL_R |= TIMER_CTL_TAEN; // Start Timer
                float testA, testB;
				for(i=0; i<31; i++){
					delta_phase_a = 4294967296 * ((float)freq[i] / (float)Fs) / 2;
					phase_a = 0;
					waitMicrosecond(100000);
					rawA = readAdc0Ss3();
					rawB = readAdc0Ss2();
					testA = (float)rawA;
					testB = (float)rawB;
					dbArr[i] = 20.0 * log10(testA/testB);
				}
				i = 0;
				putsUart0("DB ARRAY:\n\n");
				for(i=0; i<31; i++){
					sprintf(str, "%f\n", (float)dbArr[i]);
					putsUart0(str);
				}
				putsUart0("\nFREQ ARRAY:\n\n");
				for(i=0; i<31; i++){
					sprintf(str, "%f\n", (float)freq[i]);
					putsUart0(str);
				}
				stop_all();
				putsUart0("\nReady for next command!\n\n");
			}
	        if(isCommand(&instr, "arb", 1)){
	        	freq = getFieldInteger(&instr, 1);
	        	AisWave = true;
	        	phase_a = 0;
	        	delta_phase_a = 4294967296 * ((float)freq / (float)Fs) / 2;
				int i = 0;
				int index = 0;
				int setsRecvd = 0;
				for(setsRecvd = 0; setsRecvd < 128; setsRecvd++){
					getsUart0(&arbStr);
					for(i=0; buffer[i] != '\0'; i++){
						if (buffer[i] == ' '){
							index++;
						}
						else {
							arbArr[index+(setsRecvd)] = arbArr[index+(setsRecvd)] * 10 + (buffer[i] - 48);
						}
					}
				}
				//Transfer arbArr into LUT
				for(i=0; i<1024; i++){
					LUTa[i*2] = arbArr[i];
				}
				for(i=0; i<LUTsize; i++){
					if ((i % 2) != 0){
						LUTa[i] = LUTa[i-1];
					}
				}

				//Clear arbArr for next transmission
				for(i=0; i<1024; i++){
					arbArr[i] = 0;
				}

	        }
	        if (isCommand(&instr, "hilbert", 1)){
	        	hilbert_b = getFieldInteger(&instr, 1);
				if (hilbert_b){
					hilbertFunction();
				}
	            else {
	            	stopB();
	            }
	        }

	        newCommand = 0;
		}
	}
}
