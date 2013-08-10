#include <linux/linkage.h>
#include <asm/memory.h>
#include <asm/glue.h>
#include <asm/vfpmacros.h>
#include <asm/arch/entry-macro.S>
#include <asm/thread_notify.h>
#include <asm/arch/hardware.h>
#include <asm/ptrace.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>

	.align	5
	.globl __scu_call_wrap
#__scu_call_wrap:
#
# call to the defined funcion.
# r0: func argument array , max = 6
# r1: argument num.
# r2: the function number.

#;	EXPORT __scu_call_wrap
#;	
#;	CODE32
#;	AREA   ||funwrap||, CODE, READONLY	
__scu_call_wrap:
	STMFD    r13!,{r4,r5,r6,lr}
	mov		r6,r2
	ldmia	r0, {r0-r5}
	STMFD    r13!,{r4,r5}
	mov	lr , pc
	mov pc , r6
	add     r13 , r13 ,#8
	LDMFD    r13!,{r4,r5,r6,pc}
#	END

/**
 * 20091126,HSL@RK,change to get syscall struct pt_regs *.
 * way: search stack frame for lr = ret_fast_syscall.
 * code from bachtrace.S -- __backtrace.
 */
#define frame	r4
#define sv_fp	r5
#define sv_pc	r6
#define mask	r7
#define offset	r8
        
ENTRY(__scu_get_regs)
		mov	r1, #0x10
		mov	r0, fp
#if !defined(CONFIG_FRAME_POINTER) || !defined(CONFIG_PRINTK)
    mov r0, #0
		mov	pc, lr
#else
		stmfd	sp!, {r4 - r8, lr}	@ Save an extra register so we have a location...
		movs	frame, r0		@ if frame pointer is zero
		beq	no_frame		@ we have no stack frames

		tst	r1, #0x10		@ 26 or 32-bit mode?
		moveq	mask, #0xfc000003	@ mask for 26-bit
		movne	mask, #0		@ mask for 32-bit

1:		stmfd	sp!, {pc}		@ calculate offset of PC stored
		ldr	r0, [sp], #4		@ by stmfd for this CPU
		adr	r1, 1b
		sub	offset, r0, r1

/*
 * Stack frame layout:
 *             optionally saved caller registers (r4 - r10)
 *             saved fp
 *             saved sp
 *             saved lr
 *    frame => saved pc
 *             optionally saved arguments (r0 - r3)
 * saved sp => <next word>
 *
 * Functions start with the following code sequence:
 *                  mov   ip, sp
 *                  stmfd sp!, {r0 - r3} (optional)
 * corrected pc =>  stmfd sp!, {..., fp, ip, lr, pc}
 */
for_each_frame:	tst	frame, mask		@ Check for address exceptions
		bne	no_frame

1001:		ldr	sv_pc, [frame, #0]	@ get saved pc
1002:		ldr	sv_fp, [frame, #-12]	@ get saved fp

		sub	sv_pc, sv_pc, offset	@ Correct PC for prefetching
		bic	sv_pc, sv_pc, mask	@ mask PC/LR for the mode

1003:		ldr	r2, [sv_pc, #-4]	@ if stmfd sp!, {args} exists,
		ldr	r3, .Ldsi+4		@ adjust saved 'pc' back one
		teq	r3, r2, lsr #10		@ instruction
		subne	r0, sv_pc, #4		@ allow for mov
		subeq	r0, sv_pc, #8		@ allow for mov + stmia

		ldr	r1, [frame, #-4]	@ get saved lr
		ldr	r2, .Ldsi+8
		bic	r1, r1, mask		@ mask PC/LR for the mode
		bic	r2, r2, mask		@ mask PC/LR for the mode
		cmp r1, r2
		beq found_frame
		
		teq	sv_fp, #0		@ zero saved fp means
		beq	no_frame		@ no further frames

		cmp	sv_fp, frame		@ next frame must be
		mov	frame, sv_fp		@ above the current frame
		bhi	for_each_frame
    b no_frame

found_frame:
    add r0, frame , #0x0c 	
    b return_r
no_frame:	
     mov r0, #0
return_r:
    ldmfd	sp!, {r4 - r8, pc}
		
		.section __ex_table,"a"
		.align	3
		.long	1001b, no_frame
		.long	1002b, no_frame
		.long	1003b, no_frame
		.previous
.Ldsi:		
    .word	0xe92dd800 >> 10	@ stmfd sp!, {... fp, ip, lr, pc}
		.word	0xe92d0000 >> 10	@ stmfd sp!, {}
		.word ret_fast_syscall
#endif	

/*
 * 20091215,continue running from break point.
 * r0 is the struct pt_regs *.
 */
ENTRY(__scu_bk_continue)
    mov r12, r0	 
    ldr	r0, [r12, #S_PSR]		
	  msr	spsr_cxsf, r0
	  ldmia	r12, {r0 - pc}^			@ load r0 - pc, cpsr
          
        .align 5
ENTRY(__rk28_tcm_map)
    STMFD    r13!,{r4,r14}
		adr r4 , itcm
		ldr r1 , [r4,#4]
		ldr r0 , [r4,#0]
		MRC p15,0,r2,c9,c1,1
 		LDR r3,=0xfffff001
 		BIC r2,r2,r3
 		ORR r2,r2,r0
  	MCR p15,0,r2,c9,c1,1  @enable ITCM and map to r2(addr=bit[32-12])
  	
  	MRC p15,0,r2,c9,c1,0
 		BIC r2,r2,r3
 		ORR r2,r2,r1
  	MCR p15,0,r2,c9,c1,0
  	add r3 , r4,#0x08
  	ldmia r3 , {r0-r2}
  	bl memcpy
  	mov     r0 , r4
  	LDMFD    r13!,{r4,pc}
itcm:  	
	.long 0xff400001    @ ITCM ADDR = 0XFF400000
	.long 0xff404001    @ DTCM ADDR = 0XFF400000
	.long 0xff400000    @ dst of memcpy
	.word	__rk28_halt_enter @ src of memcpy
  .long __rk28_halt_end-__rk28_halt_enter @len of memcpy 	  
	.long restart_code - __rk28_halt_enter 
	
#define SDRC_OFFSET       	    0X10
#define SCU_OFFSET       	      (SDRC_OFFSET+0X4)  
#define GATE0_OFFSET            (SCU_OFFSET+0x4)
#define SCU_GAT1_REG_OFFSET     0X20

/*
 * 20090924,HSL,take care of soft reset reg have no usage.
 * gate 0 have to much use clk,so only close gate 1,2.
*/
  .section ".meminit.text", "ax"
	.align	10
ENTRY(__rk28_halt_enter)
	adr r0 , sdram
	stmia R0 , {r4-r6, lr }
	mov	r4 , r0		
	ldr r0 , [r4 , #SDRC_OFFSET] @set sdram ctroler
	ldr r1 , [r0 , #0xc]
	ORR	r1 , r1, #0x2
	str	r1 , [r0 , #0x0c]   
waitfor_refresh_enter:
	ldr	r2 , [r0 , #0xc]
	TST	R2 , #(0X1<<11)
	beq	waitfor_refresh_enter
	
	@close gate clk control 	
	ldr r3 , [r4 , #SCU_OFFSET]	@ r3 = scu gate 1 reg 
	ldmia r3,{r5-r6}            @ save gate1-gate2,not soft reset value	
	
#if 1	
	add lr , r4 , #GATE0_OFFSET
	ldmia lr , { r0 - r1 } @r0-r2:gate1-gate2
  stmia r3,  { r0 - r1 }
#endif

   
		
	bl __rk28_halt_here

	@open gate clk control
	ldr r3 , [r4 , #SCU_OFFSET]   @ r3 = scu gate 1 
	stmia r3,{r5-r6}        @ restore gate1-gate2 .
	
	mov	r0 , #2
	bl 	__rk28_delay_cycle
		
	ldr r0 , [r4 , #SDRC_OFFSET]   @sdram ctrl
	ldr r1 , [r0 , #0xc]
	bic	r1 , r1 , #0X2
	str r1 , [r0 , #0x0c ]	
waitfor_refresh_exit:
	ldr	r2 , [r0 , #0xc]
	TST	R2 , #(0X1<<11)
	bne	waitfor_refresh_exit
	
	mov	r0 , #16
	bl 	__rk28_delay_cycle
	
	mov r0 , r4
	ldmia r0 , {r4-r6 , pc}

sdram:  	
	.long	0			@ FOR r4.
	.long	0			@ FOR r5.      @bake gat1
	.long	0			@ FOR r6.      @bake gat2
	.long	0			@ FOR lr. 
	.long SDRAMC_BASE_ADDR_VA
	.long SCU_BASE_ADDR_VA+SCU_GAT1_REG_OFFSET     

   @CLOCK GATE 0, 0:arm,7:usb phy,9:intc,14:sdmmc0,25:timer,27:rtc
   @CLOCK GATE 1,15:extern hclk,16:sdram clk,17:msdram clk,19:lcdc clk
   @CLOCK GATE 2,2:dsp ahb ,5:efuse ip
  .long ((1<<19)|(7<<15)) @0xe7fff (0xfff<<20)|
  .long 0x24
  
  .align	3     
__rk28_delay_cycle:	
	MOV		R0 , R0 , LSL#12
1:
	SUBS 	R0 , R0 , #1
	BNE		1b
	mov 	pc , lr
	
ENTRY(__rk28_halt_here)
	mov	r0, #0
	mrc	p15, 0, r1, c1, c0, 0		@ Read control register
	mcr	p15, 0, r0, c7, c10, 4		@ Drain write buffer
	bic	r2, r1, #1 << 12
	mrs	r3, cpsr			@ Disable FIQs while Icache
	orr	ip, r3, #PSR_F_BIT		@ is disabled
	msr	cpsr_c, ip
	mcr	p15, 0, r2, c1, c0, 0		@ Disable I cache
	mcr	p15, 0, r0, c7, c0, 4		@ Wait for interrupt
	nop             @20091221,flush cpu line
	nop
	nop
	nop
	nop
	mcr	p15, 0, r1, c1, c0, 0		@ Restore ICache enable
	msr	cpsr_c, r3			@ Restore FIQ state
	mov	pc, lr

#if 0 	
#define UART_USR        0X7C
#define UART_THR        0X00

    .align	3
        @r0 = write value , will crash r4
uart_write_reg:
    @ldr r0,[r0,#0]
    mov r1,#0xf
    adr r3,tmpp_buffer
    add r4,r3,#1
    add r3,r4,#8
2:
    and r2,r1,r0
    cmp r2,#9
    add r2,r2,#0x30
    ble 1f
    add r2,r2,#0x7
1:  strb r2,[r3],#-1
    lsr r0,r0,#4
    cmp r4,r3
    bne 2b
    SUB r0,r4,#1
    b uart_little_write
    
tmpp_buffer:
   .asciz "0x        \r\n"        

   .align	3
uart_little_write:
   adr r2,uart_reg
   ldr r2,[r2,#0]
2:   
   ldrb r1,[r0],#1
   CMP R1,#0
   beq 2f
1:   
   ldr r3,[r2,#UART_USR]
   TST r3,#(1<<1)
   beq 1b
   str r1,[r2,#UART_THR]
   b 2b
2:
   mov pc , lr   
      
uart_reg:
   .long 0x18000000 
#endif
   
   .align	3		
restart_code:
  MRC p15,0,r0,c1,c0,0
  BIC r0,r0,#(1<<0)     @disable mmu
  BIC r0,r0,#(1<<13)    @set vector to 0x00000000
  MCR p15,0,r0,c1,c0,0  
  mov r1,r1
  mov r1,r1
  adr r0,clear_reg
  
  ldr r2,[r0,#0]        @usb soft disconnect.
  mov r3,#2
  str r3,[r2,#0]
  
  ldr r2,[r0,#4]        @ BCH reset.
  mov r3,#1
  str r3,[r2,#0]
  
  ldr r2,[r0,#8]        @ set dsp power domain on
  ldr r3,[r2,#0x10]
  bic r3,r3,#(1<<0)
  str r3,[r2,#0x10]
  
  ldr r3,[r2,#0x24]     @ enable dsp ahb clk
  bic r3,r3,#(1<<2)
  str r3,[r2,#0x24]
  
  ldr r3,[r2,#0x1c]     @ gate 0 ,enable dsp clk
  bic r3,r3,#(1<<1)
  str r3,[r2,#0x1c]
  
  ldr r3,[r2,#0x28]     @dsp soft reset , for dsp l2 cache as maskrom stack.
  ORR r3,r3,#((1<<5))
  str r3,[r2,#0x28]
  BIC r3,r3,#((1<<5))
  str r3,[r2,#0x28]

#if 0 
  ldr r3,[r2,#0xc]      @ arm enter slow mode
  ORR r3,r3,#(3<<2)
  str r3,[r2,#0xc]
#endif

#if 0  
  adr r0,string0 
  bl  uart_little_write
  
  mov r6,#0
  mov r5,#0x100
  
  ldr r0,[r6,#0] 
  bl  uart_write_reg
  
  adr r0,string0 
  bl  uart_little_write
  
6:
  ldr r0,[r6],#4
  bl uart_write_reg
  cmp r6,r5
  bne 6b
#endif
  
  mov r12,#0
@dead:  b dead   @20090925,may dead here for axd jtag.
  bx r12 	
	
clear_reg:
  .long 0x10040804	        @USB_OTG_BASE_ADDR
  .long 0x100AE00C	        @NANDC_BASE_ADDR
  .long 0x18018000	        @ SCU_BASE_ADDR
@string0: .asciz "\n####go here\r\n"
    
  .align	3
__rk28_halt_end: @ MUST BE THE LAST STAMENT
	.previous

