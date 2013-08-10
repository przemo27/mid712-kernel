/*
 *  linux/drivers/rk28_serial.c
 *
 *  Driver for Atmel AT91 / AT32 Serial ports
 *  Copyright (C) 2003 Rick Bronson
 *
 *  Based on drivers/char/serial_sa1100.c, by Deep Blue Solutions Ltd.
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  DMA support added by Chip Coldwell.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/clk.h>  
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>  
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/compiler.h>
#include <asm/io.h>
#include <linux/irq.h>

#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/arch/hw_common.h>

#include <asm/arch/rk28_serial.h>
#include <asm/param.h>

#if defined(CONFIG_ROCK_UART) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#define pheaduart  ((pUART_REG)(port->membase))

#define NR_PORTS (2)
#define ROCKCHIP_SERIAL_RINGSIZE 1024

#define ROCKCHIP_ISR_PASS_LIMIT  256  

#define UART_GET_LCR(port)	    __raw_readl(&(pheaduart->UART_LCR)) 
#define UART_PUT_LCR(port,v)	__raw_writel(v,&(pheaduart->UART_LCR)) 
#define UART_GET_DLL(port)	    __raw_readl(&(pheaduart->UART_DLL))  
#define UART_GET_DLH(port)	    __raw_readl(&(pheaduart->UART_DLH))  
#define UART_PUT_DLL(port,v)	__raw_writel(v,&(pheaduart->UART_DLL)) 
#define UART_PUT_DLH(port,v)	__raw_writel(v,&(pheaduart->UART_DLH)) 
#define UART_PUT_CHAR(port,v)	__raw_writel(v,&(pheaduart->UART_THR)) 
#define UART_GET_CHAR(port)	    __raw_readl(&(pheaduart->UART_RBR))  
#define UART_GET_USR(port)	    __raw_readl(&(pheaduart->UART_USR))  
#define UART_PUT_IER(port,v)	__raw_writel(v,&(pheaduart->UART_IER)) 
#define UART_GET_IIR(port)	    __raw_readl(&(pheaduart->UART_IIR))  
#define UART_PUT_SRR(port,v)	__raw_writel(v,&(pheaduart->UART_SRR))
#define UART_PUT_SFE(port,v)	__raw_writel(v,&(pheaduart->UART_SFE))

///#define DEBUG_LHH   1

/*
 * We wrap our port structure around the generic uart_port.
 */
struct rockchip_uart_port {
	struct uart_port	uart;		/* uart */
	struct clk		*clk;		/* uart clock */
	unsigned short		suspended;	/* is port suspended? */
};

static struct rockchip_uart_port rockchip_ports[NR_PORTS];

static inline struct rockchip_uart_port * to_rockchip_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct rockchip_uart_port, uart);
}

/* translate a port to the device name */
static inline const char *rockchip_serial_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}


#define ROCKCHIP_SERIAL_NAME	"ttyS"   //驱动名
#define ROCKCHIP_SERIAL_MAJOR	 TTY_MAJOR   //5 //204 //5  ///204        //主设备号
#define ROCKCHIP_SERIAL_MINOR	 64 //1  ///64         //次设备号

#ifdef SUPPORT_SYSRQ
static struct console rockchip_console;
#endif

extern int uartportregs;	
extern int uartportregs1;

/*
 * 判断发送缓冲区是否为空
 *先以FIFO打开做，后面可以做成FIFO关或FIFO开。
 */
static u_int rockchip_serial_tx_empty(struct uart_port *port)
{
   #ifdef DEBUG_LHH
   printk("Enter::%s\n",__FUNCTION__);
   #endif
   while(!(UART_GET_USR(port)&UART_TRANSMIT_FIFO_EMPTY))
        cpu_relax();
	if(UART_GET_USR(port)&UART_TRANSMIT_FIFO_EMPTY)
	{
        return (1);///1：空
	}
	else
	{
        return (0);///0:非空
	}
}

/*
 * Power / Clock management.
 */
static void rockchip_serial_pm(struct uart_port *port, unsigned int state,
			    unsigned int oldstate)
{
	//struct rockchip_uart_port *rockchip_port = to_rockchip_uart_port(port);
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif

	switch (state) {
	case 0:
		/*
		 * Enable the peripheral clock for this serial port.
		 * This is called on uart_open() or a resume event.
		 */
		//clk_enable(rockchip_port->clk);
		break;
	case 3:
		/*
		 * Disable the peripheral clock for this serial port.
		 * This is called on uart_close() or a suspend event.
		 */
	//	clk_disable(rockchip_port->clk);
		break;
	default:
		printk(KERN_ERR "rockchip_serial: unknown pm %d\n", state);
	}
}

/*
 * Return string describing the specified port
 */
static const char *rockchip_serial_type(struct uart_port *port)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
	return (port->type == PORT_ROCKCHIP) ? "ROCKCHIP_SERIAL" : NULL;
}

static void rockchip_serial_enable_ms(struct uart_port *port)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif

}

/* no modem control lines */
static unsigned int rockchip_serial_get_mctrl(struct uart_port *port)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
	return 0;
}

static void rockchip_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
        
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
  
}

/*
 * Stop transmitting.
 */
static void rockchip_serial_stop_tx(struct uart_port *port)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif

}

/*
 * Start transmitting.
 */
static void rockchip_serial_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	while(!(uart_circ_empty(xmit)))
	{
		while (!(UART_GET_USR(port) & UART_TRANSMIT_FIFO_NOT_FULL))
                	cpu_relax();
		UART_PUT_CHAR(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	} 
}

/*
 * Stop receiving - port is in process of being closed.
 */
static void rockchip_serial_stop_rx(struct uart_port *port)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
}

/*
 * Control the transmission of a break signal
 */
static void rockchip_serial_break_ctl(struct uart_port *port, int break_state)
{
  unsigned int temp;
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
    
  temp = UART_GET_LCR(port);
	if (break_state != 0)       
        temp = temp & (~BREAK_CONTROL_BIT);/* start break */
	else
        temp = temp | BREAK_CONTROL_BIT; /* stop break */
    UART_PUT_LCR(port, temp);	
}


/*
 * Characters received (called from interrupt handler)
 */
static void rockchip_rx_chars(struct uart_port *port)
{
	unsigned int ch, flag;
	while((UART_GET_USR(port) & UART_RECEIVE_FIFO_NOT_EMPTY) == UART_RECEIVE_FIFO_NOT_EMPTY)
	{
	    ch = UART_GET_CHAR(port);
	    flag = TTY_NORMAL;
		  port->icount.rx++;
		  if (uart_handle_sysrq_char(port, ch))
		  {
			continue;
		  } 
		 uart_insert_char(port, 0, 0, ch, flag);
	}
	spin_unlock(&port->lock);
	tty_flip_buffer_push(port->info->tty);
	spin_lock(&port->lock);
	
}

/*
 * Interrupt handler
 */
static irqreturn_t rockchip_uart_interrupt(int irq, void *dev_id)
{	
	struct uart_port *port = dev_id;
	unsigned int status, pending;
	
		status = UART_GET_IIR(port); 
		pending = status & 0x0f;
    if((pending == RECEIVER_DATA_AVAILABLE) || (pending == CHARACTER_TIMEOUT))
		rockchip_rx_chars(port);
		
	  return IRQ_HANDLED;	
}

/*
 * Disable the port
 */
static void rockchip_serial_shutdown(struct uart_port *port)
{
   #ifdef DEBUG_LHH
   printk("Enter::%s\n",__FUNCTION__);
   #endif
   UART_PUT_IER(port,0x00);
   free_irq(port->irq, port);
}
/*
 * Perform initialization and enable port for reception
 */
static int rockchip_serial_startup(struct uart_port *port)
{
	//struct rockchip_uart_port *rockchip_port = to_rockchip_uart_port(port);
	struct tty_struct *tty = port->info->tty;	
	int retval;	
		
	retval = request_irq(port->irq,rockchip_uart_interrupt,IRQF_SHARED,
		     tty ? tty->name : "rockchip_serial",port);
	if(retval)
		{
			printk("\nrockchip_serial_startup err \n");	
			rockchip_serial_shutdown(port);
			return 	retval;
		}	
  UART_PUT_IER(port,0x01);  //enable uart recevice IRQ
	return 0;
}

/*
 * Change the port parameters
 */
static void rockchip_serial_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
    unsigned long flags;
    unsigned int mode, baud,dll,dlh;
    
    /* Get current mode register */
    mode = UART_GET_LCR(port) & (BREAK_CONTROL_BIT | EVEN_PARITY_SELECT | PARITY_ENABLED
                       | ONE_HALF_OR_TWO_BIT | UART_DATABIT_MASK);  
    
    baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
    /* byte size */
    switch (termios->c_cflag & CSIZE) {
    case CS5:
        mode |= LCR_WLS_5;
        break;
    case CS6:
        mode |= LCR_WLS_6;
        break;
    case CS7:
        mode |= LCR_WLS_7;
        break;
    default:
        mode |= LCR_WLS_8;
        break;
    }
    
    /* stop bits */
    if (termios->c_cflag & CSTOPB)
        mode |= ONE_STOP_BIT;
    
    /* parity */
    if (termios->c_cflag & PARENB) 
    {
        mode |= PARITY_ENABLED;
        if (termios->c_cflag & PARODD)
            mode |= ODD_PARITY;
        else
            mode |= EVEN_PARITY;
    }
    spin_lock_irqsave(&port->lock, flags);
    
    dlh = (1000 * 24000) / 16 / baud;
    dll = dlh & 0xff;
    dlh = (dlh>>8) & 0xff;
    mode = mode | LCR_DLA_EN;
    while(UART_GET_USR(port) & UART_USR_BUSY)
    cpu_relax(); 
    UART_PUT_LCR(port,mode);
    UART_PUT_DLL(port,dll);   
	  UART_PUT_DLH(port,dlh); 
	  mode = mode & (~LCR_DLA_EN);
    UART_PUT_LCR(port,mode);	
    spin_unlock_irqrestore(&port->lock, flags);
}


static void rockchip_serial_release_port(struct uart_port *port)
{
        
  struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;
	release_mem_region(port->mapbase, size);
	
	#ifdef DEBUG_LHH
	printk("Enter::%s\n",__FUNCTION__);
	#endif
}

static int rockchip_serial_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;
	const char *name = rockchip_serial_portname(port);
	#ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
	return request_mem_region(port->mapbase, size, name) ? 0 : -EBUSY;
}		      


/*
 * Configure/autoconfigure the port.
 */
static void rockchip_serial_config_port(struct uart_port *port, int flags)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_ROCKCHIP;
		rockchip_serial_request_port(port);
	}
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int rockchip_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_ROCKCHIP)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if ((void *)port->mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}


static struct uart_ops rockchip_serial_ops = {
	.pm		= rockchip_serial_pm,
	.tx_empty	= rockchip_serial_tx_empty,
	.get_mctrl	= rockchip_serial_get_mctrl,
	.set_mctrl	= rockchip_serial_set_mctrl,
	.stop_tx	= rockchip_serial_stop_tx,
	.start_tx	= rockchip_serial_start_tx,
	.stop_rx	= rockchip_serial_stop_rx,
	.enable_ms	= rockchip_serial_enable_ms,
	.break_ctl	= rockchip_serial_break_ctl,
	.startup	= rockchip_serial_startup,
	.shutdown	= rockchip_serial_shutdown,
	.set_termios	= rockchip_serial_set_termios,
	.type		= rockchip_serial_type,
	.release_port	= rockchip_serial_release_port,
	.request_port	= rockchip_serial_request_port,
	.config_port	= rockchip_serial_config_port,
	.verify_port	= rockchip_serial_verify_port,
};


/*
 * Configure the port from the platform device resource info.
 */
static void __devinit rockchip_init_port(struct rockchip_uart_port *rockchip_port,
				      struct platform_device *pdev)
{
	struct uart_port *port = &rockchip_port->uart;

  port->mapbase  = pdev->resource[0].start;
	port->irq      = pdev->resource[1].start;
  
	port->uartclk  = 24000000;  
	port->iotype   = UPIO_MEM; 
	port->flags    = UPF_BOOT_AUTOCONF; 
	port->ops      = &rockchip_serial_ops;
	port->fifosize = 16;
	port->line     = pdev->id; 
	port->dev	= &pdev->dev;
	if(pdev->id == 0)
		port->membase	= (void __iomem *)uartportregs;
	else 
		port->membase	= (void __iomem *)uartportregs1;
	UART_PUT_SFE(port,0x01);  ///enable fifo	
}



#ifdef CONFIG_ROCK_UART
static void rockchip_console_putchar(struct uart_port *port, int ch)
{
    while (!(UART_GET_USR(port) & UART_TRANSMIT_FIFO_NOT_FULL))
		cpu_relax();
	  UART_PUT_CHAR(port, ch);
}

/*
 * Interrupts are disabled on entering
 */
static void rockchip_console_write(struct console *co, const char *s, u_int count)
{
	struct uart_port *port = &rockchip_ports[co->index].uart;
	
	uart_console_write(port, s, count, rockchip_console_putchar);	

}

/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init rockchip_console_get_options(struct uart_port *port, int *baud,
					     int *parity, int *bits)
{
   volatile static unsigned int lcr;
	unsigned int dlh, dll;


  while(UART_GET_USR(port) & UART_USR_BUSY)
  cpu_relax();
	lcr = UART_GET_LCR(port) | (LCR_DLA_EN);
	UART_PUT_LCR(port,lcr);	
	dll = UART_GET_DLL(port);   //读DLL和DLH时要置下LCR_DLA_EN。
	dlh = UART_GET_DLH(port);
	lcr = lcr & (~LCR_DLA_EN);
  UART_PUT_LCR(port,lcr);	
    
	*parity = 'n'; //无有寄偶校验
	if (lcr & PARITY_ENABLED) 
	{
		*parity = 'y';//有寄偶校验
	}
	switch (lcr & 0x03) 
	{
		case 0:	*bits = 5; break;
		case 1:	*bits = 6; break;
		case 2:	*bits = 7; break;
		case 3:	*bits = 8; break;
	}
	*baud = (1000 * 24000) / 16 / (((dlh & 0xff)<<8) | (dll & 0xff));
	pr_debug("%s:baud = %d, parity = %c, bits= %d\n", __FUNCTION__, *baud, *parity, *bits);
}


static int __init rockchip_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &rockchip_ports[co->index].uart;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	if (port->membase == NULL) {
		/* Port not initialized yet - delay setup */
		return -ENODEV;
	}  
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		rockchip_console_get_options(port, &baud, &parity, &bits);
	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver rockchip_uart;

static struct console rockchip_console = {
	.name		= ROCKCHIP_SERIAL_NAME,
	.write		= rockchip_console_write,
	.device		= uart_console_device,
	.setup		= rockchip_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= 1,  //1,  //0, 
	.data		= &rockchip_uart,
};

#define ROCKCHIP_SERIAL_CONSOLE &rockchip_console

/*
 * Early console initialization (before VM subsystem initialized).
 */
static int __init rockchip_console_init(void)
{
	if (rockchip_default_console_device) {
		add_preferred_console(ROCKCHIP_SERIAL_NAME,
				      rockchip_default_console_device->id, NULL);
		rockchip_init_port(&rockchip_ports[rockchip_default_console_device->id],
				rockchip_default_console_device);
		register_console(&rockchip_console);
	printk("++++++++++++++++++++++++++++++++++++++++++++++\n");
	printk("%s:%d\n",__FUNCTION__,__LINE__);
	}
	return 0;
}

console_initcall(rockchip_console_init);


static inline bool rockchip_is_console_port(struct uart_port *port)
{
	return port->cons && port->cons->index == port->line;
}

#else
#define ROCKCHIP_SERIAL_CONSOLE NULL
static inline bool rockchip_is_console_port(struct uart_port *port)
{
	return false;
}

#endif

static struct uart_driver rockchip_uart = {
	.owner		= THIS_MODULE,
	.dev_name	= ROCKCHIP_SERIAL_NAME,  //"rockchip_uart0",
	.nr		    = 2,     ///芯片的终端数量
	.cons		= ROCKCHIP_SERIAL_CONSOLE,
	.driver_name	= "rockchip_serial", ///"rockchip_serial", ///uart0",  //ROCKCHIP_SERIAL_NAME,
	.major		= ROCKCHIP_SERIAL_MAJOR,
	.minor		= ROCKCHIP_SERIAL_MINOR,
};

#ifdef CONFIG_PM
static int rockchip_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct rockchip_uart_port *rockchip_port = to_rockchip_uart_port(port);
  
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(port->irq);
	else {
		uart_suspend_port(&rockchip_uart, port);
		rockchip_port->suspended = 1;
	}

	return 0;
}

static int rockchip_serial_resume(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct rockchip_uart_port *rockchip_port = to_rockchip_uart_port(port);
        
  #ifdef DEBUG_LHH
  printk("%s::enter\n",__FUNCTION__);
  #endif
	if (rockchip_port->suspended) {
		uart_resume_port(&rockchip_uart, port);
		rockchip_port->suspended = 0;
	} else
		disable_irq_wake(port->irq);

	return 0;
}
#else
#define rockchip_serial_suspend NULL
#define rockchip_serial_resume NULL
#endif

static int __devinit rockchip_serial_probe(struct platform_device *pdev)
{
	struct rockchip_uart_port *port;
	int ret;
 
	printk("++++++++++++++++++++++++++++++++++++++++++++++\n");
	printk("%s:%d:%d\n",__FUNCTION__,__LINE__,pdev->id);
	BUILD_BUG_ON(!is_power_of_2(ROCKCHIP_SERIAL_RINGSIZE));
  
	port = &rockchip_ports[pdev->id];
	rockchip_init_port(port, pdev);


	ret = uart_add_one_port(&rockchip_uart, &port->uart);
	if (ret)
		goto err_add_port;

	device_init_wakeup(&pdev->dev, 1);
	platform_set_drvdata(pdev, port);
	return 0;

err_add_port:
	if (!rockchip_is_console_port(&port->uart)) {
		//clk_disable(port->clk);
		//clk_put(port->clk);
		port->clk = NULL;
	}
	return ret;
}

static int __devexit rockchip_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	//struct rockchip_uart_port *rockchip_port = to_rockchip_uart_port(port);
	int ret = 0;
	platform_set_drvdata(pdev, NULL);

	ret = uart_remove_one_port(&rockchip_uart, port);

	/* "port" is allocated statically, so we shouldn't free it */

	//clk_disable(rockchip_port->clk);
	//clk_put(rockchip_port->clk);

	return ret;
}

static struct platform_driver rockchip_serial_driver = {
	.probe		= rockchip_serial_probe,
	.remove		= __devexit_p(rockchip_serial_remove),
	.suspend	= rockchip_serial_suspend,
	.resume		= rockchip_serial_resume,
	.driver		= {
		.name	= "rockchip_usart", //"rockchip_uart0",
		.owner	= THIS_MODULE,
	},
};

static int __init rockchip_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&rockchip_uart);
	if (ret)
		return ret;
	ret = platform_driver_register(&rockchip_serial_driver);
	if (ret)
		uart_unregister_driver(&rockchip_uart);

	return ret;
}

static void __exit rockchip_serial_exit(void)
{
	platform_driver_unregister(&rockchip_serial_driver);
	uart_unregister_driver(&rockchip_uart);
}


module_init(rockchip_serial_init);
module_exit(rockchip_serial_exit);

MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("Rockchip RK28XX Serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rockchip_usart");
