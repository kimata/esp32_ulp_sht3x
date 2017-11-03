/*
 * ESP32 ULP Utility Macro
 *
 * Copyright (C) 2017 KIMATA Tetsuya <kimata@green-rabbit.net>
 *
 * This program is free software ; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Bibliography: https://github.com/tomtor/ulp-i2c
 */

.macro push rx
    st \rx,r3,0
    sub r3,r3,1
.endm

.macro pop rx
    add r3,r3,1
    ld \rx,r3,0
.endm
    
.macro psr
    .set Lret_addr,(.+16)
    move r1,Lret_addr
    push r1
.endm

.macro ret
    pop r1
    jump r1
.endm

// Local Variables:
// mode: asm
// comment-start: "// "
// End:     
