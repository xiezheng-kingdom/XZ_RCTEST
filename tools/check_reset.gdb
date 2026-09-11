set pagination off
file c:/Users/Xiezheng_kingdom/Desktop/PZtest/build/H7_Arm.elf
target extended-remote localhost:3333

# Try software reset via AIRCR
monitor reset
info registers pc sp xpsr

# Try breakpoint at Reset_Handler
break *Reset_Handler
continue

# If we hit Reset_Handler, step through
info registers pc sp xpsr

quit
