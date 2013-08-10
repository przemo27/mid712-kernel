#include "rk28_debug_uart.h"
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/ptrace.h>
#include <asm/arch/rk28_debug.h>

#if USE_JTAG
static int      rk28_uart_inited = 1;
#else
static int      rk28_uart_inited = 0;
#endif

#define TIMER_GPIO                      GPIOPortF_Pin7
#define TIMER_GPIO_INIT()            do { GPIOSetPinDirection(TIMER_GPIO,GPIO_OUT);\
                                                        GPIOSetPinLevel(TIMER_GPIO,GPIO_HIGH); }while( 0 )
#define TIMER_GPIO_REVERSE()    do { GPIOSetPinLevel(TIMER_GPIO,(!GPIOGetPinLevel(TIMER_GPIO)));\
                                                        } while( 0 )

static pUART_REG g_UART =((pUART_REG)(( UART0_BASE_ADDR)));
static  pGRF_REG g_grfReg=((pGRF_REG)(( REG_FILE_BASE_ADDR)));

void iomux_set_uart0(char type)
{
	unsigned int configa = g_grfReg->IOMUX_A_CON;
	unsigned int configb = g_grfReg->IOMUX_B_CON;

	switch(type)
	{
		case 1:
			configa &= ~0x0f000;
			configa |= 0x5000;
			break;
		case 2:
			configb |= 0x3000;
			break;
		case 0:
			configa &= ~0x0f000;
			configb &= ~0x03000;
			break;
	}
	g_grfReg->IOMUX_A_CON = configa;
	g_grfReg->IOMUX_B_CON = configb;
}


int uart_write_byte(char uartCh, char byte)
{
	int uartTimeOut;

            if( !rk28_uart_inited )
                        return (-1);

             if( byte == '\n' )
	        uart_write_byte(0,0x0d);
             
	uartTimeOut = 0xffff;
	while((g_UART->UART_USR & (1<<1)) != (1<<1))
	{
		if(uartTimeOut == 0)
		{
			return (-1);
		}
		uartTimeOut--;
	}

	g_UART->UART_THR = byte;
             
	return (0);
}


void uart_reset(pUART_REG phead)
{
	pUART_REG	 phwHead = (pUART_REG)phead;
	phwHead->UART_SRR = 1 | (1<<1) | (1<<2);
	phwHead->UART_IER = 0;
   /// phwHead->UART_SRR = 0;
}

int uart_set_iop(unsigned char useIrDA)
{
	pUART_REG	phwHead   = 
                                ((pUART_REG)(IO_PA2VA_APB( UART1_BASE_ADDR))) ;
	if((useIrDA == 0) || (useIrDA == (1<<6)))
	{
		phwHead->UART_MCR = useIrDA;
		return (0);
	}
	return (-1); 
}

unsigned int uart_set_baudrate(pUART_REG phead, unsigned int baudRate)
{
	unsigned int	uartTemp;
	pUART_REG	 phwHead = (pUART_REG)phead;

	if(baudRate>115200)
	{
		return (-1);		
	}
	phwHead->UART_LCR = phwHead->UART_LCR | (1<<7);////| PARITY_DISABLED | ONE_STOP_BIT | LCR_WLS_8;  ///enable DL ,set 8 bit character
	uartTemp = (1000 * 24000) / 16 / baudRate;
	phwHead->UART_DLL = uartTemp & 0xff;
	phwHead->UART_DLH = (uartTemp>>8) & 0xff;
	phwHead->UART_LCR = phwHead->UART_LCR & (~(1<<7));	///PARITY_DISABLED | ONE_STOP_BIT | LCR_WLS_8;

	return (0);
}
void uart_set_fifo_enabled_numb(pUART_REG phead)
{
	pUART_REG	 phwHead = (pUART_REG)phead;
	phwHead->UART_SFE = 1;
	phwHead->UART_SRT = 3;
	phwHead->UART_STET = 1;
}


int uart_set_lcr_reg(pUART_REG phead, char byteSize, char parity, char stopBits )
{
	pUART_REG	 phwHead = (pUART_REG)phead;
	unsigned int lcr;
	int bRet = 0;

	lcr = (unsigned int)(phwHead->UART_LCR);
	lcr &= ~0x03;//lcr &= ~UART_DATABIT_MASK;
	switch ( byteSize )    ///byte set
	{
		case 5:
			lcr |= 0x00;//SERIAL_5_DATA;
			break;
		case 6:
			lcr |= 0x01;//SERIAL_6_DATA;
			break;
		case 7:
			lcr |= 0x02;//SERIAL_7_DATA;
			break;
		case 8:
			lcr |= 0x03;//SERIAL_8_DATA;
			break;
		default:
			bRet = -1;
			break;
	}

	switch ( parity )  ///Parity set
	{
		case 0:
			lcr |= 0;
			break;
		case 1:
			lcr |= (1<<3);
			break;
		default:
			bRet = -1;
			break;
	}

	switch ( stopBits )  ///StopBits set
	{
		case 0:
			lcr |= 0;
			break;
		case 1:
			lcr |= (1<<2);
			break;
		default:
			bRet = -1;
			break;
	}

	if (bRet == 0)
	{
		phwHead->UART_LCR = lcr;
	}
	return(bRet);
}

#if 0
int uart_reinit( void )
{
        rk28_uart_inited = 1; /* after mmu disable ,set to phy addr. */
        g_UART =((pUART_REG)(( UART0_BASE_ADDR)));
        g_grfReg=((pGRF_REG)(( REG_FILE_BASE_ADDR)));
        debug_print("uart reinit\n");
        return (0);
}
#endif

int __init uart_init(char uartCh, unsigned int baudRate)
{

#if 0 /* HSL@RK,20090708, loader already done this well */
	unsigned int  returnTemp;
	pUART_REG puartreg ;
	if(uartCh == 0)
	{
		puartreg = (pUART_REG)(IO_PA2VA_AHB(UART0_BASE_ADDR));
		scu_enable_clk(0x12);
	}
	else
	{
	}

//	uart_reset(puartreg);  

	/*
	returnTemp = uart_set_iop(0);
	if(returnTemp == -1)
	{
		return (-1);
	}
	*/
	returnTemp = uart_set_lcr_reg(puartreg,8,0,0);
	if(returnTemp == -1)
	{
		return (-1);
	}
	returnTemp = uart_set_baudrate(puartreg,baudRate);
	if(returnTemp == -1)
	{
		return (-1);
	}
	uart_set_fifo_enabled_numb(puartreg);
                
   /// UARTSetIntEnabled(puartreg,uartIntNumb);
#endif   
        rk28_uart_inited = 2; /* flag for io already remap to avoid data abort. */
        g_UART =((pUART_REG)(IO_PA2VA_APB( UART0_BASE_ADDR)));
        g_grfReg=((pGRF_REG)(IO_PA2VA_APB( REG_FILE_BASE_ADDR)));
        /*  20100315,HSL@RK,this function will make noise at machine power on 
          *  OPEN ONLY ON DEBUG OR CHANGE the debug GPIO--TIMER_GPIO.
         */
        //TIMER_GPIO_INIT();  
        debug_print("++++uart_init after map board io\n");
        return (0);
}

void uart_write_string(char *s)
{
        while(*s){
                uart_write_byte(0,*s);
                s++;
        }
        return ;
}
void debug_atoi(char *s, int a)
{
	int i=0,t_a = a;
	while((t_a != 0)&&(i < 127)){
		t_a = t_a / 10;
		i++;
	}
	s[i--] = 0;
	for( ; a != 0 ; i--){
		s[i] = a % 10 + 0x30;
		a = a / 10;
	}
	//s[i] = 0;
	return ;
}

void debug_print(const char* format,...)
{
        char s[130];
        va_list args;
        int i;
        va_start(args, format);
        i = vscnprintf(s,128,format,args);
        va_end(args);
        uart_write_string( s );
}

void debug_print_symbol(const char * fmt , unsigned long  addr )
{
        char p0[40];
        char p1[120];
        sprint_symbol( p0 , addr );
        sprintf(p1 , fmt , p0 );
        uart_write_string( p1 );
}


void debug_gpio_reverse( void )
{
        TIMER_GPIO_REVERSE();
}

void debug_at_exception(  int stage ) /* struct pt_regs *regs*/
{
        debug_print("at irq %d\n" , stage );
}

