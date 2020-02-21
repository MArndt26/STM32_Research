# STM32_Research
This Project is aimed to measure voltages from 10 ACD ports and then
write them to a micro sd card via an SPI connection

USART
    BAUD RATE = 115200
    TX --> PA9
    RX --> PA10
 
Push Button
  PA0
  
ADC:
    1 --> PA1
    2 --> PA2
    3 --> PA3
    4 --> PA4
    5 --> PA5
    6 --> PA6
    7 --> PA7
    8 --> PB0
    9 --> PB1
    10 -> PC0

SCI:
    CS    -> PB6
    MOSI  -> PB5
    MISO  -> PB4
    SCK   -> PB3

Questions to ponder:
    - what value of Max_ss --> max sector size are needed
        I cannot do 4096 --> currently at 512

Resources Used:
  https://www.youtube.com/watch?v=spVIZO-jbxE