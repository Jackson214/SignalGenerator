//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// 16 MHz external crystal oscillator

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef CLOCK_TEMPH_
#define CLOCK_TEMPH_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSystemClockTo40Mhz(void);

#endif
