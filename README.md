# UART_TIMER

FND 7 Segment 제어 

UART Command line 

SFR Read, Write  Command  

Function running time check  

==============================
22/02/14 수정

TIM3 1ms로 변경
해당 start ,end , run 값 변경 
( 10us -> tim3->cnt register value + count * 1000 ( tim interrupt 1ms )
