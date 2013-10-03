


# ***************************************************************************************************************************
# *                                                                                                                         *
# *                                           Global Symbols                                                                *
# *                                                                                                                         *
# ***************************************************************************************************************************

.GLOBAL main

# ***************************************************************************************************************************
# *                                                                                                                         *
# *                                           Data Segment                                                                  *
# *                                                                                                                         *
# ***************************************************************************************************************************

# This is where all variables are defined.
# We generally assign pointer/address registers to these variables and access them indirectly via these registers.

.DATA                        # The start of the data segment

enable_display: .BYTE 27, '[', '3', 'e', '\0'
set_cursor: .BYTE 27, '[', '1', 'c', '\0'
home_cursor: .BYTE 27, '[', 'j', '\0'
wrap_line: .BYTE 27, '[', '0', 'h', '\0'
duty_cycle2: .WORD 0x0000 # Duty cycle of right motor (OC2)
duty_cycle3: .WORD 0x0000 # Duty cycle of left motor (OC3)
dir0: .WORD 0 # Direction of right motor (RD07) (OC2)
dir1: .WORD 1 # Direction of left motor (RD06) (OC3)
dir0change: .WORD 1
dir1change: .WORD 1


# ***************************************************************************************************************************
# *                                                                                                                         *
# *                                           Code Segment                                                                  *
# *                                                                                                                         *
# ***************************************************************************************************************************

.TEXT                        # The start of the code segment

.ENT main                    # Setup a main entry point
main:

	DI	# Disable system wide interrupts; dont respond to spurious interrupts

	JAL setupPORTs	# Setup pins to H-bridge
    JAL setupUART1
    JAL setupUART2

	# NOTE: H-bridge connected to Cerebot MX4cK Port JH-10:07
	# Set motor direction; DIR - JH-07, RE08
	# Be careful not to change the direction at the same time the motor is pulsed; may cause short cirucit in H-bridge
	LI $t0, 1 << 8
	SW $t0, LATECLR

	JAL setupMultiVectoredMode	# Want mult-vectored interrupts - each interrupt may have a location in interrupt vector table
	
    JAL setupOutputCompare2Module
    JAL setupOutputCompare3Module
	JAL setupTimer2	# Configure Timer 2 - Only Timers 2 and 3 should be used for compare time base

    JAL enableUART1rxInt
    JAL setupINT1
    JAL enableINT1

    LI $s4, 0x9000
    LI $s5, 0x0001

	EI	# Enable system wide interrupts
    # LA $s1, enable_display # Load address so can cycle through character commands

    LI $t0, 0b110
    SW $t0, (LATD)
 
	loop:
    JAL runRobot
    
    LI $t0, 0x0002
    BNE $s5, $t0, mode2
    JAL lineFollow
    mode2:

    LI $t0, 0x0003
    BNE $s5, $t0, mode3
    JAL squareRoutine
    mode3:

 LI $t0, 0x0004
    BNE $s5, $t0, mode4
    JAL circleRoutine
    mode4:


 LI $t0, 0x0005
    BNE $s5, $t0, mode5
    JAL figureRoutine
    mode5:


    LI $t0, 0x0006
    BNE $s5, $t0, mode6
    JAL triangleRoutine
    mode6:


		J loop      # Embedded programs require that they run forever! So jump back to the beginning of the loop

.END main

.ENT setupPORTs
setupPORTs:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

    # UART1 CLS TX JE02 RF08, RX JE03 RF02
    LA $s0, TRISF
	LW $t0, ($s0)
    ORI $t0, $t0, 1 << 2 #RF02
	ANDI $t0, $t0, 0b1111111011111111 #RF08
	SW $t0, ($s0)

    # UART2 CLS TX JH02 RF05, RX JH03 RF04
    LA $s0, TRISF
	LW $t0, ($s0)
    ORI $t0, $t0, 1 << 4 #RF04
	ANDI $t0, $t0, 0b1111111111011111 #RF05
	SW $t0, ($s0)

/*
	# H-bridge DIR - RG09, RG12; EN - RG8, RG13; Sense - RG7,6,14,15
	# Set these two pins to outputs
	LA $s0, TRISG
    LW $t0, ($s0)
    ANDI $t0, $t0, 0b1100110011111110
    ORI $t0, $t0, 0b1100000011000000
    SW $t0, ($s0)
*/

    # H-Bridge EN on RD01, RD02
    # H-Bridge DIR on RD07, RD06
    LA $s0, TRISD
    LW $t0, ($s0)
    ANDI $t0, $t0, 0b1111111100111001
    SW $t0, ($s0)

    # Onboard LEDs set to outputs
    LA $s0, TRISB
    LW $t0, ($s0)
	ANDI $t0, $t0, 0b1100001111111111# Preserve other pins on PORTB
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupPORTs


.ENT setupMultiVectoredMode
setupMultiVectoredMode:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Interrupt control register
	LA $s0, INTCON # Register necessary for setting multi-vectored mode
	LW $t0, ($s0)
	ORI $t0, $t0, 1 << 12 # Set for mutli-vectored mode
	#SW $t0, ($s0)
	SW $t0, INTCON

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupMultiVectoredMode

.ENT setupOutputCompare2Module

setupOutputCompare2Module:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Ensure OC1 is off while setting up module 1
	LA $s0, OC2CON # Output compare 1 control register
	MOVE $t0, $zero
	SW $t0, ($s0)

	# Initialize the OC2R register before the output compare module, this register determins duty cycle
	LA $s0, OC2R
	LI $t0, 0x0000 # Initialize to OFF
	SW $t0, ($s0)
	# The OC2RS secondary output compare register will contain the actual duty cycle
	LA $s0, OC2RS
	LI $t0, 0x0000 # Initialize to OFF
	SW $t0, ($s0)

	# Now configure the compare module using OC2CON
	# Bits 2:0 - 110 = PWM mode on OC1, 011 = compare event toggles OC1 pin
	# Bit 3 - 1 = Timer 3 clk src, 0 = Timer 2 clk src
	# Bit 5 - 1 = 32-bit comparisons, 0 = 16-bit comparisons
	# Bit 15 - 1 = enable output compare, 0 = disabled, not drawing current
	LA $s0, OC2CON

	LI $t0, 6 # PWM mode
	#ORI $t0, $t0, 0b1000000000000110 # Enable output compare module
	SW $t0, ($s0)

	# Set priority of compare match interrupt IPC1<20:18>
	#LA $s0, IPC1SET
	#LI $t0, 6 # priority 6
	#SLL $t0, $t0, 18
	#SW $t0, ($s0)

/*
    # Set priority
	LA $s0, IPC1 # Interrupt priority register
	# IPC1 <20:18> for UART1
	LW $t0, ($s0)
	LI $t1, 6
	SLL $t1, $t1, 18
	OR $t0, $t0, $t1
	SW $t0, ($s0)
*/
    #Turn On OC2
    LA $s0, OC2CON
    LW $t1, ($s0)
    ORI $t1, 0x8000
    SW $t1, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupOutputCompare2Module

.ENT setupOutputCompare3Module
setupOutputCompare3Module:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Ensure OC1 is off while setting up module 1
	LA $s0, OC3CON # Output compare 1 control register
	MOVE $t0, $zero
	SW $t0, ($s0)

	# Initialize the OC3R register before the output compare module, this register determins duty cycle
	LA $s0, OC3R
	LI $t0, 0x0000 # Initialize to OFF
	SW $t0, ($s0)
	# The OC3RS secondary output compare register will contain the actual duty cycle
	LA $s0, OC3RS
	LI $t0, 0x0000 # Initialize to OFF
	SW $t0, ($s0)

	# Now configure the compare module using OC3CON
	# Bits 2:0 - 110 = PWM mode on OC1, 011 = compare event toggles OC1 pin
	# Bit 3 - 1 = Timer 3 clk src, 0 = Timer 2 clk src
	# Bit 5 - 1 = 32-bit comparisons, 0 = 16-bit comparisons
	# Bit 15 - 1 = enable output compare, 0 = disabled, not drawing current
	LA $s0, OC3CON

	LI $t0, 6 # PWM mode
	#ORI $t0, $t0, 0b1000000000000110 # Enable output compare module
	SW $t0, ($s0)

	# Set priority of compare match interrupt IPC1<20:18>
	#LA $s0, IPC1SET
	#LI $t0, 6 # priority 6
	#SLL $t0, $t0, 18
	#SW $t0, ($s0)

/*
    # Set priority
	LA $s0, IPC1 # Interrupt priority register
	# IPC1 <20:18> for UART1
	LW $t0, ($s0)
	LI $t1, 6
	SLL $t1, $t1, 18
	OR $t0, $t0, $t1
	SW $t0, ($s0)
*/
    #Turn On OC3
    LA $s0, OC3CON
    LW $t1, ($s0)
    ORI $t1, 0x8000
    SW $t1, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupOutputCompare3Module

# Used for the clock source for the compare time base
.ENT setupTimer2
setupTimer2:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# T2CON - Control Register for Timer 2
	# Bit 15 - ON Timer On bit, 1 = timer enabled, 0 = disabled
	LA $s0, T2CON
	LI $t0, 0x0 # stop Timer 2
	SW $t0, ($s0)

	# TMR2 register contains 16-bit current value of Timer 2
	LA $s0, TMR2
	MOVE $t0, $zero # clear timer value
	SW $t0, ($s0)

	# PR2 register contains 16-bit period match value, i.e. TMR2 value == PR1 value ==> timer resets
	LA $s0, PR2
	LI $t0, 0x0800 # Affects how often the timer is reset! Contains PWM period! Allows ~50% duty cycle based on OC1R values
	SW $t0, ($s0)

	# T2CON - Control Register for Timer 2 - can combine with Timer 3 to form 32-bit timer!
	# Bit 1 - TCS Timer Clock Source, 0 = internal peripheral clk (PBCLK)
	# Bit 3 - 1 = 32-bit timer, 0 = 16-bit timer
	# Bits 6:4 - TCKPS Timer Clock Prescale Select bits, these are slightly different than with Timer 1; 100 = /16
	# Bit 15 - ON Timer On bit, 1 = timer enabled, 0 = disabled

	LA $s0, T2CON
	LI $t0, 0x0050 # PBCLK / 16, Timer 2 on, 16-bit timer mode, use PBCLK
	SW $t0, ($s0)

	# Will not set an interrupt priority for Timer 2, but will for Ouput Compare Module 1

    LA $s0, T2CON
    LW $t1, ($s0)
	LI $t0, 0x8000 # Timer 2 on
    OR $t0, $t0, $t1
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupTimer2

/*
.ENT enableOutputCompare1Int
enableOutputCompare1Int:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	LA $s0, IEC0 # Interrupt enable control register - our mask register
	LW $t0, ($s0)
    LI $t1, 1
	SLL $t1, $t1, 6 # Set corresponding mask bit to 1 to enable
    OR $t0, $t0, $t1
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END enableOutputCompare1Int

.SECTION .vector_6, code	# Attach jump to handler in vector corresponding to OC1

   J OutputCompare1IntHandler


.TEXT

.ENT OutputCompare1IntHandler
OutputCompare1IntHandler:

	# We want our handlers to be as short as possible. We do want them
	# to execute in as few clock cycles as possible. We generally do
	# not want to call procedures from within these handlers.

	DI	# Disable system wide interrupts

	# Clear Output Compare 1 Interrupt status flag
	LA $t0, IFS0CLR
	LW $t1, ($t0)
	ORI $t1, $t1, 1 << 6
	SW $t1, ($t0)

    # Can set new duty cycle in this handler
	# Use OC1RS, the secondary compare register

  # Toggle LEDs
    LA $s0, LATB
    LW $t0, ($s0)
    XORI $t0, $t0, 0b0011110000000000
    SW $t0, ($s0)

# toggle motor en

    LA $s0, LATG
    LW $t0, ($s0)
    ORI $t0, $t0, 0b0010000100000000
    SW $t0, ($s0)

	EI	# Enable system wide interrupts

	ERET # PC = EPC

.END OutputCompare1IntHandler

*/

.ENT setupUART1
setupUART1:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# U1MODE - UART1 Mode Register, i.e. control/config register
	# Bit 15 - ON UART enable bit, 1 = UART enabled, 0 = disabled
	LA $s0, U1MODE
	LI $t0, 0 # disable UART1, reset it
	SW $t0, ($s0)

	# Clear the transmit and receive buffers
	LA $s0, U1TXREG # UART1 transmit register
	MOVE $t0, $zero # clear register value
	SW $t0, ($s0)
	LA $s0, U1RXREG # UART1 receive register
	MOVE $t0, $zero # clear register value
	SW $t0, ($s0)

	# Setup the baud rate - bits per second (bps) of data received and transmitted
	LA $s0, U1BRG # UART1 Baud Rate Register
	LI $t0, 259 # U1BRG = (PBCLK / (16 * baud rate)) - 1; PBCLK = 40 MHz, desired baud rate = 9600 bps
	SW $t0, ($s0)


	# U1STA - UART1 Status and Control Register
	# Bits 15:14 - UTXISEL TX Interrupt Mode Selection bits 10 = interrupt generated tranmit buffer becomes empty, 01 = interrupt is generated when all character transmitted,
	#                                                       00 = interrupt generated when transmit buffer becomes not full
	# Bit 12 - URXEN Receive Enable bit 1 = UART1 receiver is enabled, U1RX pin controlled by UART1; 0 = UART1 receiver disabled, U1RX pin ignored
	# Bit 10 - UXTEN Transmit Enable bit 1 = UART1 transmitter enabled, U1TX pin controlled by UART1; 0 = UART1 trasmitter diabled, transmissin aborted, buffer reset
	# Bit 9 - UTXBF Transmit Buffer Full Status bit 1 = Transmit buffer is full, 0 = not full
	# Bit 8 - TRMT Transmit Shift Registr is Empty bit 1 = Transmit shift register and transmit buffer empty, 0 = Transmit shift register is not empty, transmission in progress
	# Bits 7:6 - URXISEL Receive Interrupt Mode Selection bit 11 = Interrup flag bit set when receive buffer is full, 10 set when buffer 3/4 full, 0x flag bit set when character is received
	# Bit 3 - Parity Error Status bit 1 = parity error detected, 0 = parity error not detected
	# Bit 2 - FERR Framing Error Status bit 1 = framing error detected, 0 = no framing error detected
	# Bit 1 - OERR Receive Buffer Overrun Error Status bit 1 = receive buffer overflowed, 0 = buffer has not overflowed
	# Bit 0 - URXDA Receive Buffer Data Available bit 1 = receive buffer has data, 0 = receive buffer is empty
	LA $s0, U1STA
	LI $t0, 0b0001010000000000 # Dont need to preserve any bits, overwrite with constant; not applying interrupts; enable transmission only
	SW $t0, ($s0)

	# Set priority
	# IPC6 <4:2> IEC0<27> = U1RX, IEC0<28> = U1TX

    # Set priority
	LA $s0, IPC6 # Interrupt priority register
	# IPC6 <4:2> for UART1
	LW $t0, ($s0)
	LI $t1, 6
	SLL $t1, $t1, 2
	OR $t0, $t0, $t1
	SW $t0, ($s0)

	# U1MODE - UART1 Mode Register, i.e. control/config register
	# Bit 0 - STSEL Stop Selection bit 1 = 2 stop bits, 0 = 1 stop bit
	# Bits 2:1 - PDSEL Parity and Data Selection bits 11 = 9 data, no parity; 10 = 8 data, odd parity; 01 = 8 data, even parity; 00 = 8 data, no parity
	# Bit 3 - BRGH High Baud Rate Enable bit 1 = high speed, 0 = standard speed
	# Bits 9:8 - UEN UART Enable bits 00 = U1TX and U1RX pins enabled and used
	# Bit 15 - ON UART enable bit, 1 = UART enabled, 0 = disabled
	LA $s0, U1MODE
	LI $t0, 0b1000000000000000 # Dont need to preserve any bits, overwrite with constant
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupUART1

.ENT enableUART1rxInt
enableUART1rxInt:
	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	LA $s0, IEC0 # Interrupt enable control register - our mask register
	LW $t0, ($s0)
    LI $t1, 1
	SLL $t1, $t1, 27 # Set corresponding mask bit to 1 to enable
    OR $t0, $t0, $t1
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra
.END enableUART1rxInt

.SECTION .vector_24,code	# Attach jump to handler in vector corresponding to UART1RX

   J UART1rxIntHandler


.TEXT

.ENT UART1rxIntHandler
UART1rxIntHandler:
# Preserve registers - push to stack
	ADDI $sp, $sp, -20
    SW $t2, 16($sp)
    SW $t1, 12($sp)
    SW $t0, 8($sp)
	SW $ra, 4($sp)
	SW $s0, 0($sp)

# We want our handlers to be as short as possible. We do want them
	# to execute in as few clock cycles as possible. We generally do
	# not want to call procedures from within these handlers.

	DI	# Disable system wide interrupts

	# Clear Output Compare 1 Interrupt status flag
	#LA $t0, IFS0CLR
	#LW $t2, ($t0)
	#LI $t1, 1
	#SLL $t1, $t1, 27 # Set corresponding mask bit to 1 to enable
    #OR $t2, $t2, $t1
	#SW $t2, ($t0)

   # Clear out flag corresponding to UART IRQ
    LW $t0, IFS0
    LI $t1, 1
    SLL $t1, $t1, 27
    NOT $t1, $t1
    AND $t0, $t0, $t1
    SW $t0, IFS0

    # Echo to U1TXREG and U2TXREG
    LW $t0, U1RXREG
    SW $t0, U1TXREG
    SW $t0, U2TXREG
    SW $zero, U1RXREG

# IF w, set motors to forward and equalize duty cycles
LI $t1, 'w'
BNE $t0, $t1, forward
    LI $t1, 0x0800
    SW $t1, duty_cycle2
    SW $t1, duty_cycle3
forward:

# IF s, stop motors
LI $t1, 's'
BNE $t0, $t1, stop
    LI $s5, 0x0001
    SW $zero, duty_cycle2
    SW $zero, duty_cycle3
stop:

# IF a, decrease duty cycle on OC3
LI $t1, 'a'
BNE $t0, $t1, down3
    LW $t2, duty_cycle3
    ADDI $t2, $t2, -0x0080
    SW $t2, duty_cycle3

down3:

# IF q, increase duty cycle on OC3

LI $t1, 'q'
BNE $t0, $t1, up3
    LW $t2, duty_cycle3
    ADDI $t2, $t2, 0x0080
    SW $t2, duty_cycle3
up3:

# IF d, decrease duty cycle on OC2
LI $t1, 'd'
BNE $t0, $t1, down2
    LW $t3, duty_cycle2
    ADDI $t3, $t3, -0x0080
    SW $t3, duty_cycle2
down2:
# IF e, increase duty cycle on OC2
LI $t1, 'e'
BNE $t0, $t1, up2
    LW $t2, duty_cycle2
    ADDI $t2, $t2, 0x0080
    SW $t2, duty_cycle2

up2:

# IF z, zero OC3 duty cycle then toggle RD6, then start OC3
LI $t1, 'z'
BNE $t0, $t1, reverseleft
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
reverseleft:

# IF c, zero OC2 duty cycle then toggle RD7, then start OC2
LI $t1, 'c'
BNE $t0, $t1, reverseright
    LW $t2, dir0
    XORI $t2, $t2, 1
    SW $t2, dir0
    LI $t2, 1
    SW $t2, dir0change
reverseright:

# IF u, circle
LI $t1, 'u'
BNE $t0, $t1, circle
LI $s5, 0x0004
circle:

# IF i, square
LI $t1, 'i'
BNE $t0, $t1, square
LI $s5, 0x0003
square:

# IF o, figure 8
LI $t1, 'o'
BNE $t0, $t1, figure8
LI $s5, 0x0005
figure8:

# IF p, triangle
LI $t1, 'p'
BNE $t0, $t1, triangle
LI $s5, 0x0006
triangle:

# IF l, linefollow
LI $t1, 'l'
BNE $t0, $t1, u1lmode
LI $s5, 0x0002
u1lmode:

    # Toggle LEDs to make sure UART is actually working
    LW $t0, LATB
	LI $t1, 0b0011110000000000
    XOR $t0, $t0, $t1
	SW $t0, LATB

    # Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
    LW $t0, 8($sp)
    LW $t1, 12($sp)
    LW $t2, 16($sp)
	ADDI $sp, $sp, 20

	EI	# Enable system wide interrupts

	ERET # PC = EPC


.END UART1rxIntHandler

.ENT setupUART2
setupUART2:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# U2MODE - UART2 Mode Register, i.e. control/config register
	# Bit 15 - ON UART enable bit, 1 = UART enabled, 0 = disabled
	LA $s0, U2MODE
	LI $t0, 0 # disable UART2, reset it
	SW $t0, ($s0)

	# Clear the transmit and receive buffers
	LA $s0, U2TXREG # UART2 transmit register
	MOVE $t0, $zero # clear register value
	SW $t0, ($s0)
	LA $s0, U2RXREG # UART2 receive register
	MOVE $t0, $zero # clear register value
	SW $t0, ($s0)

	# Setup the baud rate - bits per second (bps) of data received and transmitted
	LA $s0, U2BRG # UART2 Baud Rate Register
	LI $t0, 259 # U2BRG = (PBCLK / (16 * baud rate)) - 1; PBCLK = 40 MHz, desired baud rate = 9600 bps
	SW $t0, ($s0)


	# U2STA - UART2 Status and Control Register
	# Bits 15:14 - UTXISEL TX Interrupt Mode Selection bits 10 = interrupt generated tranmit buffer becomes empty, 01 = interrupt is generated when all character transmitted,
	#                                                       00 = interrupt generated when transmit buffer becomes not full
	# Bit 12 - URXEN Receive Enable bit 1 = UART1 receiver is enabled, U1RX pin controlled by UART1; 0 = UART1 receiver disabled, U1RX pin ignored
	# Bit 10 - UXTEN Transmit Enable bit 1 = UART1 transmitter enabled, U1TX pin controlled by UART1; 0 = UART1 trasmitter diabled, transmissin aborted, buffer reset
	# Bit 9 - UTXBF Transmit Buffer Full Status bit 1 = Transmit buffer is full, 0 = not full
	# Bit 8 - TRMT Transmit Shift Registr is Empty bit 1 = Transmit shift register and transmit buffer empty, 0 = Transmit shift register is not empty, transmission in progress
	# Bits 7:6 - URXISEL Receive Interrupt Mode Selection bit 11 = Interrup flag bit set when receive buffer is full, 10 set when buffer 3/4 full, 0x flag bit set when character is received
	# Bit 3 - Parity Error Status bit 1 = parity error detected, 0 = parity error not detected
	# Bit 2 - FERR Framing Error Status bit 1 = framing error detected, 0 = no framing error detected
	# Bit 1 - OERR Receive Buffer Overrun Error Status bit 1 = receive buffer overflowed, 0 = buffer has not overflowed
	# Bit 0 - URXDA Receive Buffer Data Available bit 1 = receive buffer has data, 0 = receive buffer is empty
	LA $s0, U2STA
	LI $t0, 1 << 10 # Dont need to preserve any bits, overwrite with constant; not applying interrupts; enable transmission only
	SW $t0, ($s0)

	# Set priority
	# IPC6 <4:2> IEC0<27> = U1RX, IEC0<28> = U1TX

	# U2MODE - UART2 Mode Register, i.e. control/config register
	# Bit 0 - STSEL Stop Selection bit 1 = 2 stop bits, 0 = 1 stop bit
	# Bits 2:1 - PDSEL Parity and Data Selection bits 11 = 9 data, no parity; 10 = 8 data, odd parity; 01 = 8 data, even parity; 00 = 8 data, no parity
	# Bit 3 - BRGH High Baud Rate Enable bit 1 = high speed, 0 = standard speed
	# Bits 9:8 - UEN UART Enable bits 00 = U1TX and U1RX pins enabled and used
	# Bit 15 - ON UART enable bit, 1 = UART enabled, 0 = disabled
	LA $s0, U2MODE
	LI $t0, 0b1000000000000000 # Dont need to preserve any bits, overwrite with constant
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupUART2

.ENT sendUART1a0
sendS0:


.END sendUART1a0


.ENT setupINT1
setupINT1:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	# Set priority
	LA $s0, IPC1 # Interrupt priority register
	# IPC1 <28:26> for INT1
	LW $t0, ($s0)
	LI $t1, 2
	SLL $t1, $t1, 26
	OR $t0, $t0, $t1
	SW $t0, ($s0)

	# Set the polarity
	# Interrupt control register
	LA $s0, INTCON # Register necessary for setting polarity of interrupt trigger
	LW $t0, ($s0)
	ORI $t0, $t0, 1 << 1 # Set INT4 rising edge trigger
	SW $t0, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END setupINT1

.ENT enableINT1
enableINT1:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	LA $s0, IEC0 # Interrupt enable control register - our mask register
	LW $t0, ($s0)
	LI $t1, 1
	SLL $t1, $t1, 7
	OR $t0, $t0, $t1 # Set corresponding mask bit to 1 to enable, 7 is INT4 position
	SW $t0, ($s0)


	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END enableINT1

.ENT disableINT1
disableINT1:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)

	LA $s0, IEC0CLR # Used to clear bit to disable INT1
	LI $t1, 1
	SLL $t1, $t1, 7
	SW $t1, ($s0)

	# Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8

	JR $ra

.END disableINT1

.SECTION .vector_7,code
   J      ExtInt1Handler


.TEXT
.ENT ExtInt1Handler
ExtInt1Handler:

    # Disable global interrupts
    DI

    # Clear out flag corresponding to INT1 IRQ
    LW $t0, IFS0
    LI $t1, 1
    SLL $t1, $t1, 7
    NOT $t1, $t1
    AND $t0, $t0, $t1
    SW $t0, IFS0

    #Toggle on-board LEDS so we know we are reaching here for each interrupt request
    LW $t0, LATB
	LI $t1, 0b0011110000000000
    XOR $t0, $t0, $t1
	SW $t0, LATB

    LI $t1, 27
    SW $t1, U2TXREG

    # Re-enable global interrupts
    EI

	ERET

.END ExtInt1Handler

.ENT runRobot
runRobot:
# Check directions
LW $t0, dir0change
BEQ $t0, $zero, skipdir0

    # Get the current duty cycle, set duty cycle to 0, nop a few times,  toggle direction, nop a few times, resume duty cycle
    LW $t0, dir0
    SLL $t0, $t0, 7
    LW $t2, PORTD
    ANDI $t2, $t2, 0b1111111101111111
    OR $t0, $t0, $t2
    LW $t1, duty_cycle2
    SW $zero, OC2R
    SW $zero, OC2RS
    nop
    nop
    nop
    nop
    SW $t0, LATD
    nop
    nop
    nop
    nop
    SW $t1, OC2R
    SW $t1, OC2RS
    SW $zero, dir0change
skipdir0:

LW $t0, dir1change
BEQ $t0, $zero, skipdir1
    # Get the current duty cycle, set duty cycle to 0, nop a few times,  toggle direction, nop a few times, resume duty cycle
    LW $t0, dir1
    SLL $t0, $t0, 6
    LW $t2, PORTD
    ANDI $t2, $t2, 0b1111111110111111
    OR $t0, $t0, $t2
    LW $t1, duty_cycle3
    SW $zero, OC3R
    SW $zero, OC3RS
    nop
    nop
    nop
    nop
    SW $t0, LATD
    nop
    nop
    nop
    nop
    SW $t1, OC3R
    SW $t1, OC3RS
    SW $zero, dir1change
skipdir1:


# Check duty cycles
LW $t0, OC2RS
LW $t1, duty_cycle2
BEQ $t0, $t1, skipcycle2
    SW $t1, OC2R
    SW $t1, OC2RS

skipcycle2:

LW $t0, OC3RS
LW $t1, duty_cycle3
BEQ $t0, $t1, skipcycle3
    SW $t1, OC3R
    SW $t1, OC3RS

skipcycle3:

JR $ra

.END runRobot

.ENT lineFollow
lineFollow:
    # The line should be under RG14 and RG13 at all times
    # If it fall under RG12 or RG15 we need to start turning one way or the other
    # If we see it fall off of RG15 or RG12, we need to stop and turn back onto it at a low speed
    # If the line falls off RG14 or RG13, just keep driving
    # If the line Falls on ALL four, just keep driving until its only on 1-2

    #First, read portg and mask
    LW $t0, PORTG
    ANDI $t0, $t0, 0xF000

    # If this isnt 0xF000, save it to the last seen global reg, s4
    LI $t1, 0xF000
    BEQ $t0, $t1, saveinfra
        MOVE $s4, $t0
    saveinfra:

    # Conditionals
    LI $t1, 0xF000
    BEQ $t0, $t1, allon

    LI $t1, 0x000
    BEQ $t0, $t1, alloff

    LI $t1, 0xE000
    BEQ $t0, $t1, lsideoff

    LI $t1, 0x7000
    BEQ $t0, $t1, rsideoff

    LI $t1, 0x9000
    BEQ $t0, $t1, moff

    LI $t1, 0xB000
    BEQ $t0, $t1, moff

    LI $t1, 0xD000
    BEQ $t0, $t1, moff

    # Other, continue forward. Break in line or sensor error
    B moff

   allon:
   # Load last known position of line and operate on that condition
   MOVE $t0, $s4
   B saveinfra

lsideoff:
   LI $t1, 0x0200
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3
   B contline

rsideoff:
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0200
   SW $t1, duty_cycle3
   B contline

alloff:
   LI $t1, 0x0280
   SW $t1, duty_cycle2
   LI $t1, 0x0280
   SW $t1, duty_cycle3
   B contline

moff:
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3
   B contline

contline:

JR $ra

.END lineFollow

.ENT wait
wait:
    LI $t0, 30000
    MOVE $t1, $a0

        waitwhile1:
        BLEZ $t1, waitdone
        NOP
        ADDI $t1, $t1, -1
        LI $t0, 30000
        waitwhile:
            BLEZ $t0, waitwhile1
            NOP
            NOP
            ADDI $t0, $t0, -1
            B waitwhile
            NOP

waitdone:
    JR $ra
.END wait

.ENT circleRoutine
circleRoutine:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)
#wait a second
LI $a0, 0x000F
JAL wait


#set duty cycles for a fast circle
   LI $t1, 0x02A0
   SW $t1, duty_cycle2
   LI $t1, 0x0480
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x008F
    JAL wait

   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3

   # return to bluetooth mode
   LI $s5, 1

    # Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8
    JR $ra
.END circleRoutine

.ENT squareRoutine
squareRoutine:
# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)
#wait a second
LI $a0, 0x000F
JAL wait



   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
     #wait a second
    LI $a0, 0x000A
    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
     #wait a second
    LI $a0, 0x000A
    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
     #wait a second
    LI $a0, 0x000A
    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
     #wait a second
    LI $a0, 0x000A

    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change

    LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   # return to bluetooth mode
   LI $s5, 1

    # Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8
    JR $ra
.END squareRoutine

.ENT triangleRoutine
triangleRoutine:
# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)
#wait a second
LI $a0, 0x000F
JAL wait



   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
     #wait a second
    LI $a0, 0x000D
    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
     #wait a second
    LI $a0, 0x000D
    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
   LI $t1, 0x0300
   SW $t1, duty_cycle2
   LI $t1, 0x0300
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x003F
    JAL wait
    #stop
   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   #turn
   LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change
    JAL runRobot
    LI $t1, 0x0700
   SW $t1, duty_cycle2
   LI $t1, 0x0700
   SW $t1, duty_cycle3
    JAL runRobot
    #wait a second
    LI $a0, 0x000D
    JAL wait
    LW $t2, dir1
    XORI $t2, $t2, 1
    SW $t2, dir1
    LI $t2, 1
    SW $t2, dir1change

    LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3
   # return to bluetooth mode
   LI $s5, 1

    # Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8
    JR $ra
.END triangleRoutine

.ENT figureRoutine
figureRoutine:

	# Preserve registers - push to stack
	ADDI $sp, $sp, -8
	SW $ra, 4($sp)
	SW $s0, 0($sp)
#wait a second
LI $a0, 0x000F
JAL wait


#set duty cycles for a fast circle
   LI $t1, 0x02A0
   SW $t1, duty_cycle2
   LI $t1, 0x0480
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x004F
    JAL wait
#set duty cycles for a fast circle
   LI $t1, 0x0480
   SW $t1, duty_cycle2
   LI $t1, 0x02A0
   SW $t1, duty_cycle3

   JAL runRobot
    #wait a second
    LI $a0, 0x004F
    JAL wait

   LI $t1, 0x0000
   SW $t1, duty_cycle2
   LI $t1, 0x0000
   SW $t1, duty_cycle3

   # return to bluetooth mode
   LI $s5, 1

    # Pop registers
	LW $s0, 0($sp)
	LW $ra, 4($sp)
	ADDI $sp, $sp, 8
    JR $ra
.END figureRoutine


