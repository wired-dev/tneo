/*******************************************************************************
 *
 * TNeo: real-time kernel initially based on TNKernel
 *
 *    TNKernel:                  copyright 2004, 2013 Yuri Tiomkin.
 *    PIC32-specific routines:   copyright 2013, 2014 Anders Montonen.
 *    TNeo:                      copyright 2014       Dmitry Frank.
 *
 *    TNeo was born as a thorough review and re-implementation of
 *    TNKernel. The new kernel has well-formed code, inherited bugs are fixed
 *    as well as new features being added, and it is tested carefully with
 *    unit-tests.
 *
 *    API is changed somewhat, so it's not 100% compatible with TNKernel,
 *    hence the new name: TNeo.
 *
 *    Permission to use, copy, modify, and distribute this software in source
 *    and binary forms and its documentation for any purpose and without fee
 *    is hereby granted, provided that the above copyright notice appear
 *    in all copies and that both that copyright notice and this permission
 *    notice appear in supporting documentation.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE DMITRY FRANK AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DMITRY FRANK OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *    THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/*
 * PIC32 context layout:
 *
 *     SP + 7C EPC
 *     SP + 78 Status
 *     SP + 74 r31/ra
 *     SP + 70 r30/s8/fp
 *     SP + 6C r28/gp
 *     SP + 68 r25/t9
 *     SP + 64 r24/t8
 *     SP + 60 r23/s7
 *     SP + 5C r22/s6
 *     SP + 58 r21/s5
 *     SP + 54 r20/s4
 *     SP + 50 r19/s3
 *     SP + 4C r18/s2
 *     SP + 48 r17/s1
 *     SP + 44 r16/s0
 *     SP + 40 r15/t7
 *     SP + 3C r14/t6
 *     SP + 38 r13/t5
 *     SP + 34 r12/t4
 *     SP + 30 r11/t3
 *     SP + 2C r10/t2
 *     SP + 28 r9/t1
 *     SP + 24 r8/t0
 *     SP + 20 r7/a3
 *     SP + 1C r6/a2
 *     SP + 18 r5/a1
 *     SP + 14 r4/a0
 *     SP + 10 r3/v1
 *     SP + 0C r2/v0
 *     SP + 08 r1/at
 *     SP + 04 hi
 *     SP + 00 lo
 */

/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/

#include "_tn_tasks.h"
#include "_tn_sys.h"
#if !__GCC
#include <xc.h>
#else
#include "p32mx150f128b.h"
#endif


/*******************************************************************************
 *    EXTERNAL DATA
 ******************************************************************************/

//-- gp: provided by linker
extern unsigned long _gp;



/*******************************************************************************
 *    EXTERNAL FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * Self-check for the application that uses TNeo:
 *
 * PIC32 application must include the file 
 * src/arch/pic32/tn_arch_pic32_int_vec1.S to the main project,
 * in order to dispatch vector_1 correctly, but if it is forgotten,
 * no error is generated at the build time: we just get to the 
 * _DefaultInterrupt when we should switch context.
 *
 * Note that we can't include that file to the TNeo library
 * project: it doesn't work.
 *
 * So, dummy function was invented, and if we forgot to 
 * include that file, we got an error at the link time.
 *
 * That function merely returns 0.
 */
extern int 
_you_should_add_file___tn_arch_pic32_int_vec1_S___to_the_project(void);




/*******************************************************************************
 *    PROTECTED DATA
 ******************************************************************************/

/*
 * For comments on these variables, see the file tn_arch_pic32.h
 */

volatile int tn_p32_int_nest_count;
void *tn_p32_user_sp;
void *tn_p32_int_sp;




/*******************************************************************************
 *    IMPLEMENTATION
 ******************************************************************************/

void _tn_arch_sys_start(
      TN_UWord      *int_stack,
      TN_UWord       int_stack_size
      )
{
   //-- reset interrupt nesting count
   tn_p32_int_nest_count = 0;

   //-- set interrupt's top of the stack
   tn_p32_int_sp = int_stack + int_stack_size/*'full desc stack' model*/;


   //-- setup core software 0 interrupt, it should be set to lowest priority
   //   and should not use shadow register set.
   //   (TNeo PIC32 port uses it for switch context routine)

   TN_BFA(TN_BFA_WR, IPC0, CS0IP, 1);  //-- set priority 1
   TN_BFA(TN_BFA_WR, IPC0, CS0IS, 0);  //-- set subpriority 0

   TN_BFA(TN_BFA_WR, IFS0, CS0IF, 0);  //-- clear interrupt flag
   TN_BFA(TN_BFA_WR, IEC0, CS0IE, 1);  //-- enable interrupt

   //-- perform first context switch
   _tn_arch_context_switch_now_nosave();
}





/*
 * See comments in the file `tn_arch.h`
 */
TN_UWord *_tn_arch_stack_init(
      TN_TaskBody   *task_func,
      TN_UWord      *stack_low_addr,
      TN_UWord      *stack_high_addr,
      void          *param
      )
{
   TN_UWord *cur_stack_pt = stack_high_addr + 1/*'full desc stack' model*/;

   //-- if you got "undefined reference" error here, it means that you
   //   forgot to include the file 
   //
   //   <tneo_path>/src/arch/pic32/tn_arch_pic32_int_vec1.S
   //
   //   to the main project.
   //   This requirement is stated in the documentation, here:
   //
   //   https://dfrank.bitbucket.io/tneokernel_api/latest/html/arch_specific.html#pic32_details
   //
   //-- see comments above of the following function for a bit more details:
   _you_should_add_file___tn_arch_pic32_int_vec1_S___to_the_project();



   //-- filling register's position in the stack - for debugging only

   //-- ABI argument area
   *(--cur_stack_pt) = 0;
   *(--cur_stack_pt) = 0;
   *(--cur_stack_pt) = 0;
   *(--cur_stack_pt) = 0;

   //-- EPC: address that PC is set to when context switch is done
   //   and `eret` is executed.
   //   We should set it to task body function address.
   *(--cur_stack_pt) = (TN_UWord)task_func;

   //-- Status register: EXL and IE bits are set
   *(--cur_stack_pt) = 0x00000003L;                       

   //-- Return address that is used when task body function returns:
   //   we set it to _tn_task_exit_nodelete, so that returning from task
   //   body function is equivalent to calling tn_task_exit(0)
   *(--cur_stack_pt) = (TN_UWord)_tn_task_exit_nodelete;

   *(--cur_stack_pt) = 0x30303030L;          //-- fp
   *(--cur_stack_pt) = (TN_UWord)&_gp;       //-- gp - provided by linker
   *(--cur_stack_pt) = 0x25252525L;          //-- t9
   *(--cur_stack_pt) = 0x24242424L;          //-- t8
   *(--cur_stack_pt) = 0x23232323L;          //-- s7
   *(--cur_stack_pt) = 0x22222222L;          //-- s6
   *(--cur_stack_pt) = 0x21212121L;          //-- s5
   *(--cur_stack_pt) = 0x20202020L;          //-- s4
   *(--cur_stack_pt) = 0x19191919L;          //-- s3
   *(--cur_stack_pt) = 0x18181818L;          //-- s2
   *(--cur_stack_pt) = 0x17171717L;          //-- s1
   *(--cur_stack_pt) = 0x16161616L;          //-- s0
   *(--cur_stack_pt) = 0x15151515L;          //-- t7
   *(--cur_stack_pt) = 0x14141414L;          //-- t6
   *(--cur_stack_pt) = 0x13131313L;          //-- t5
   *(--cur_stack_pt) = 0x12121212L;          //-- t4
   *(--cur_stack_pt) = 0x11111111L;          //-- t3
   *(--cur_stack_pt) = 0x10101010L;          //-- t2
   *(--cur_stack_pt) = 0x09090909L;          //-- t1
   *(--cur_stack_pt) = 0x08080808L;          //-- t0
   *(--cur_stack_pt) = 0x07070707L;          //-- a3
   *(--cur_stack_pt) = 0x06060606L;          //-- a2
   *(--cur_stack_pt) = 0x05050505L;          //-- a1
   *(--cur_stack_pt) = (TN_UWord)param;      //-- a0 - task's function argument
   *(--cur_stack_pt) = 0x03030303L;          //-- v1
   *(--cur_stack_pt) = 0x02020202L;          //-- v0
   *(--cur_stack_pt) = 0x01010101L;          //-- at
   *(--cur_stack_pt) = 0x33333333L;          //-- hi
   *(--cur_stack_pt) = 0x32323232L;          //-- lo

   _TN_UNUSED(stack_low_addr);

   return cur_stack_pt;
}


void  tn_arch_int_dis(void){
    __asm__ ("di");    //__builtin_disable_interrupts();
}
/** See comments in the file `tn_arch.h` */
void tn_arch_int_en(void) {
	__asm__ ("ei");   //__builtin_enable_interrupts();
}


/*
 * See comments in the file `tn_arch.h`
 */
TN_UWord tn_arch_sched_dis_save(void)
{
   TN_UWord ret = TN_BFA(TN_BFA_RD, IEC0, CS0IE);
   TN_BFA(TN_BFA_WR, IEC0, CS0IE, 0);
   return ret;
}



/*
 * See comments in the file `tn_arch.h`
 */
void tn_arch_sched_restore(TN_UWord sched_state)
{
   TN_BFA(TN_BFA_WR, IEC0, CS0IE, !!sched_state);
}




