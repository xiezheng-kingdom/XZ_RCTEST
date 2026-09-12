init
reset halt
echo "=== CPU State ==="
reg pc sp xPSR
echo "=== RCC_APB1LENR (USART3 clock enable) ==="
mdw 0x40024400 1
echo "=== GPIOD_MODER (PD8,PD9) ==="
mdw 0x40020C00 1
echo "=== GPIOD_AFRH (PD8=AF7,PD9=AF7) ==="
mdw 0x40020C24 1
echo "=== USART3_CR1 ==="
mdw 0x40004800 1
echo "=== USART3_CR2 ==="
mdw 0x40004804 1
echo "=== USART3_CR3 ==="
mdw 0x40004808 1
echo "=== USART3_BRR ==="
mdw 0x4000480C 1
echo "=== USART3_ISR ==="
mdw 0x4000481C 1
echo "=== USART3_TDR ==="
mdw 0x40004828 1
echo "=== CFSR (fault status) ==="
mdw 0xE000ED28 1
resume
exit
