/*
 * ESP32 ULP SHT3x Library
 *
 * Copyright (C) 2017 KIMATA Tetsuya <kimata@green-rabbit.net>
 *
 * This program is free software ; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "util_macro.S"

.set SHT31_DEV_ADDR,0x88 // 8bit representation (0x44 in 7bit)
.set SHT31_CMD_MSB,0x24
.set SHT31_CMD_LSB,0x00

////////////////////////////////////////////////////////////
	.bss

    .global sht3x_sense_value
sht3x_sense_value: .long 0
    .skip 4

////////////////////////////////////////////////////////////
	.text
    
    .global sense_sht3x
    .global sht3x_read_value

////////////////////////////////////////////////////////////
sense_sht3x:
 	move r1,SHT31_DEV_ADDR
 	push r1
 	move r1,SHT31_CMD_MSB
 	push r1
 	move r1,SHT31_CMD_LSB
 	push r1
    
    psr
    jump i2c_write_reg8
    add r3,r3,3
    jumpr sht3x_sense_fail,1,ge
    
    wait (8*8000) // 8ms (RTC_FAST_CLK = 8Mhz)
    wait (8*8000) // 8ms (RTC_FAST_CLK = 8Mhz)
    wait (8*8000) // 8ms (RTC_FAST_CLK = 8Mhz)

 	move r1,SHT31_DEV_ADDR
 	push r1
	move r1,sht3x_sense_value
 	push r1
    
    psr
    jump sht3x_read_value
    add r3,r3,2
    
    ret

////////////////////////////////////////////////////////////
sht3x_read_value:
    psr
    jump i2c_start

    ld r2,r3,12 // device addeess
    or r2,r2,1 // read

    psr
    jump i2c_write_byte
    jumpr sht3x_sense_fail,1,ge
     
    move r2,0 // ACK
    psr
    jump i2c_read_byte
    
    push r0
    
    move r2,0 // ACK
    psr
    jump i2c_read_byte
    
    pop r1
    lsh r1,r1,8
    or r0,r0,r1
    ld r1,r3,8 
    st r0,r1,0 // sensor value[0]

    move r2,0 // ack
    psr
    jump i2c_read_byte
    
    ld r1,r3,8 
    st r0,r1,4 // sensor value[1]

    move r2,0 // ACK
    psr
    jump i2c_read_byte
    
    push r0

    move r2,0 // ACK
    psr
    jump i2c_read_byte

    pop r1
    lsh r1,r1,8
    or r0,r0,r1
    ld r1,r3,8 
    st r0,r1,8 // sensor value[2]
    
    move r2,1 // NACK
    psr
    jump i2c_read_byte
    ld r2,r3,8
    st r0,r2,12 // sensor value[3]

    jump i2c_return_success

////////////////////////////////////////////////////////////
sht3x_sense_fail:
    ld r1,r3,8 
    move r0,0xFFFF
    st r0,r1,0 // sensor value[0]
    st r0,r1,4 // sensor value[1]
    st r0,r1,8 // sensor value[2]
    st r0,r1,12 // sensor value[3]

    jump i2c_return_fail
        
// Local Variables:
// mode: asm
// comment-start: "// "
// End:     
