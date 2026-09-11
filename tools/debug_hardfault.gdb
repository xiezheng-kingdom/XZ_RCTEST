file c:/Users/Xiezheng_kingdom/Desktop/PZtest/build/H7_Arm.elf
target extended-remote localhost:3333
monitor reset init
# Read stacked PC to find where fault occurred
# After HardFault, the CPU pushes r0-r3, r12, LR, PC, xPSR onto stack
# The SP at time of fault is MSP - we can get it from the HardFault stack frame
set $msp = 0x2001ff60
x/8x $msp
# Check UFSR (part of CFSR at 0xE000ED28, upper half is UFSR at 0xE000ED2A)
x/1xw 0xE000ED28
x/1xw 0xE000ED2C
x/1xw 0xE000ED30
x/1xw 0xE000ED34
x/1xw 0xE000ED38
# Check VTOR
x/1xw 0xE000ED08
# Check flash at vector table
x/4xw 0x08000000
quit
