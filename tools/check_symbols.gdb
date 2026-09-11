set pagination off
file c:/Users/Xiezheng_kingdom/Desktop/PZtest/build/H7_Arm.elf
target extended-remote localhost:3333
monitor reset init

# Check key symbols
info variables _sidata
info variables _sdata
info variables _edata
info variables _sbss
info variables _ebss
info variables _estack

# Step from reset vector
# Set breakpoint at first instruction of Reset_Handler
break *Reset_Handler
continue
info registers pc sp

# Remove breakpoints
delete

# Single step through the FPU enable code
stepi 10
info registers pc sp

# Now step into ExitRun0Mode
stepi
info registers pc

# Step through ExitRun0Mode (which is just BX LR)
stepi 5
info registers pc

# Should be in SystemInit now
stepi 10
info registers pc

# Check if we're still alive
info registers pc xpsr

quit
