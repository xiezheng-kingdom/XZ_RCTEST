set pagination off
file c:/Users/Xiezheng_kingdom/Desktop/PZtest/build/H7_Arm.elf
target extended-remote localhost:3333

# Full reset and halt at reset vector
monitor reset init

# Set breakpoints at key init stages
break Reset_Handler
break ExitRun0Mode
break SystemInit
break main
break HardFault_Handler

# Step through init
continue
# Should be at Reset_Handler
info registers pc xpsr sp

continue
# Should be at ExitRun0Mode
info registers pc

continue
# Should be at SystemInit
info registers pc

continue
# Should be at main or HardFault_Handler
info registers pc xpsr sp

quit
