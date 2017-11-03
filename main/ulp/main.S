/*
 * Demo of I2C ULP routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "util_macro.S"

    .set SENSE_SIZE,4
    .set SENSE_BUF_ENTRY,10
    .set SENSE_BUF_SIZE,(SENSE_BUF_ENTRY*SENSE_SIZE)

	.bss

	.global	sense_count
sense_count: .long 0

    .balign 4
	.global	sense_data
sense_data: .long 0
    .skip SENSE_BUF_SIZE

	.global stack
stack:
	.skip 100
	.global stackEnd
stackEnd:
	.long 0

	.text
	.global entry
entry:
    move r3,stackEnd
    
    // Check whether CPU is in deep sleep or not
    //
    // https://github.com/espressif/esp-idf/issues/484
    // > This bit indicates that the SoC is sleeping and RTC is ready to receive wakeup command. 
    READ_RTC_REG(RTC_CNTL_DIAG0_REG, 19, 1)
    and r0, r0, 1
    jump exit, eq

    sleep 0
    
    psr
    jump sense_sht3x

    move r2,sense_count
    ld r0,r2,0
    move r1,sense_data

_sense_data_pos:
    sub r0,r0,1
	jumpr _copy_sense_data,1,lt
    add r1,r1,SENSE_SIZE
    jump _sense_data_pos

_copy_sense_data:
    move r2,sht3x_sense_value
    ld r0,r2,0
    st r0,r1,0
    ld r0,r2,4
    st r0,r1,4
    ld r0,r2,8
    st r0,r1,8
    ld r0,r2,12
    st r0,r1,12
    
    // decrement sense_count
    move r2,sense_count
    ld r0,r2,0
    sub r0,r0,1
    jumpr wake_cpu,1,lt
    st r0,r2,0
    halt

wake_cpu:
    // reset sense_count
    move r0,0
    st r0,r2,0
    
    wake
    // stop wake up timer
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)

exit:   
    halt
    
// Local Variables:
// mode: asm
// comment-start: "// "
// End:     