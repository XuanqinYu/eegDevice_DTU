#include "soc/soc_ulp.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"

#include "stack.S"


#define N_SAMPLE      1400

.bss
.global buffer_index
buffer_index:
    .align 4
    .long 0

.global data_buffer
data_buffer:
    .align 4
    .space 4*N_SAMPLE

.global doread
doread:
    .align 4
    .long 0

.global stackEnd
stackEnd:
    .long 0

.text
.global entry
entry:
    move r1, doread
    ld r0, r1, 0
    jumpr readvalue, 1, ge

    // write config
    move r3, stackEnd
    psr
    jump i2c_start_cond

    move r2, 0x90
    move r1, 0x00
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge

    move r2, 0x01
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge

    move r2, 0xD2
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge

    move r2, 0xe3
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge

    psr
    jump i2c_stop_cond

    wait 100000

    // write read mode
    psr
    jump i2c_start_cond

    move r2, 0x90
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge

    move r2, 0x00
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge

    psr
    jump i2c_stop_cond

    move r1, doread
    move r0, 0x02
    st r0, r1, 0

readvalue:
    psr
    jump i2c_start_cond
    move r2, 0x91
    psr
    jump i2c_write_byte
    jumpr noack, 1, ge
    
    psr
    jump i2c_read_byte
    push r0
    move r2,1
    psr
    jump i2c_read_byte
    push r0  
    psr
    jump i2c_stop_cond

    pop r0
    pop r2
    lsh r2,r2,8
    or r2,r2,r0
    move r3,r2

finish:
    MOVE r1, buffer_index
    MOVE r2, data_buffer
    //MOVE r3, 37

    LD r0, r1, 0

    ADD r2, r2, r0
    ST r3, r2, 0

    ADD r0, r0, 1
    ST r0, r1, 0

check:
    //move r3, N_SAMPLE
    //sub r3, r0, r3
    jumpr buffer_full, N_SAMPLE, ge

.global exit
exit:
	halt

.global buffer_full
buffer_full:
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq

	/* Wake up the SoC, end program */
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
	halt

noack:
    move r3, 99
    jump finish
    //jump buffer_full
