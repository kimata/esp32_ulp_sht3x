/*
 * ESP32 ULP Bit Bang I2C Library
 *
 * Copyright (C) 2017 KIMATA Tetsuya <kimata@green-rabbit.net>
 *
 * This program is free software ; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Bibliography: https://github.com/tomtor/ulp-i2c
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "util_macro.S"

.set I2C_SCL_REG_OFFSET,7 // GPIO_NUM_26
.set I2C_SDA_REG_OFFSET,6 // GPIO_NUM_25

.macro i2c_wait_quarter_clock
    wait 10 // 8Mhz /10 = 800kHz, clock = 800kHz /4 = 200kHz
.endm

.macro i2c_scl_L
    WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + I2C_SCL_REG_OFFSET, 1, 1)
.endm

.macro i2c_scl_H
    WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + I2C_SCL_REG_OFFSET, 1, 1)
.endm
    
.macro i2c_sda_L
    WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + I2C_SDA_REG_OFFSET, 1, 1)
.endm

.macro i2c_sda_H
    WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + I2C_SDA_REG_OFFSET, 1, 1)
.endm

.macro i2c_sda_read
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + I2C_SDA_REG_OFFSET, 1)
.endm

    .text

    .global i2c_write_reg8
    
    .global i2c_return_fail
    .global i2c_return_success

    .global i2c_stop    
    .global i2c_start
    .global i2c_write_byte
    .global i2c_read_byte

    .global i2c_write_bit
    .global i2c_read_bit
    
////////////////////////////////////////////////////////////
// usage:
// move r1,"DEVICE ADDRESS"
// push r1
// move r1,"REGISTER ADDRESS"
// push r1
// move r1,"VALUE"
// push r1
// psr
// jump i2c_write_reg8
// add r3,r3,3
i2c_write_reg8:
    psr
    jump i2c_start

    ld r2,r3,16 // device addeess

    psr
    jump i2c_write_byte
    jumpr i2c_return_fail,1,ge

    ld r2,r3,12 // register address
    psr
    jump i2c_write_byte
    jumpr i2c_return_fail,1,ge

    ld r2,r3,8 // value
    psr
    jump i2c_write_byte

    psr
    jump i2c_stop
    
    jumpr i2c_return_fail,1,ge

    jump i2c_return_success

////////////////////////////////////////////////////////////
i2c_return_fail:
    move r0,1
    ret

////////////////////////////////////////////////////////////
i2c_return_success:
    move r0,0
    ret
    
////////////////////////////////////////////////////////////
i2c_start:
    WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + I2C_SCL_REG_OFFSET, 1, 0)
    WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + I2C_SDA_REG_OFFSET, 1, 0)
    
    i2c_sda_L
    i2c_wait_quarter_clock
  
    ret

////////////////////////////////////////////////////////////
i2c_stop:
    i2c_wait_quarter_clock
    i2c_scl_L
    i2c_wait_quarter_clock
    i2c_wait_quarter_clock
    i2c_scl_H
    i2c_wait_quarter_clock

    ret
    
////////////////////////////////////////////////////////////
// r2: the byte to write
i2c_write_byte:
    stage_rst
_i2c_write_byte_next_bit:
    and r0,r2,0x80
    psr
    jump i2c_write_bit
    lsh r2,r2,1
    stage_inc 1
    jumps _i2c_write_byte_next_bit,8,lt

    psr
    jump i2c_read_bit
   
    ret

////////////////////////////////////////////////////////////
// r2: send ack (0: ACK, 1: NACK)
i2c_read_byte:
    push r2
    move r2,0
    stage_rst
_i2c_read_byte_next_bit:
    psr
    jump i2c_read_bit
    lsh r2,r2,1
    or r2,r2,r0
    stage_inc 1
    jumps _i2c_read_byte_next_bit,8,lt

    pop r0
    psr
    jump i2c_write_bit

    move r0,r2
    
    ret
  
////////////////////////////////////////////////////////////
// r0: the bit to write (0 or 1)    
i2c_write_bit:
    i2c_wait_quarter_clock
    i2c_scl_L
    i2c_wait_quarter_clock
    jumpr _i2c_write_bit_0,1,lt
_i2c_write_bit_1:
    i2c_sda_H
    jump _i2c_write_tick
_i2c_write_bit_0:
    i2c_sda_L
_i2c_write_tick:
    i2c_wait_quarter_clock
    i2c_scl_H
    i2c_wait_quarter_clock
    ret
    
////////////////////////////////////////////////////////////
i2c_read_bit:
    i2c_wait_quarter_clock
    i2c_scl_L
    i2c_wait_quarter_clock
    i2c_sda_H
    i2c_wait_quarter_clock
    i2c_scl_H
    i2c_sda_read
    i2c_wait_quarter_clock
    ret
   
// Local Variables:
// mode: asm
// comment-start: "// "
// End:     
