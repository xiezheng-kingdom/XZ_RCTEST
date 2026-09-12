set pagination off
file c:/Users/Xiezheng_kingdom/Desktop/PZtest/build/H7_Arm.elf
target extended-remote localhost:3333

# Do a full reset and halt immediately
monitor reset init

# We should now be halted at the reset vector
# Let's verify by checking PC
info registers pc sp xpsr

# Set a breakpoint at the data copy loop
break *CopyDataInit
break *FillZerobss
break *LoopForever
break HardFault_Handler

# Run - see where we stop
continue

info registers pc sp xpsr
bt

quit
