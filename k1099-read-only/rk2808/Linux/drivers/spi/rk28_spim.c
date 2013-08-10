/*
 *  linux/drivers/rk28_spim.c
 *
 *  Driver for rockchip spi master ports
 *  Copyright (C) 2009 lhh
 *
 *  Bported on drivers/spi/rockchip_spim.c
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License port published by
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
#include <linux/kernel.h> 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/list.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/hw_common.h>
#include <asm/arch/rk28_spim.h>
#include <asm/arch/iomux.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/rk28_scu.h>

//#define DEBUG_LHH   1
#define DEBUG

#define pheadspim  ((pSPIM_REG)(port->regs))

#define SPIM_GET_CTRLR0(port)     __raw_readl(&(pheadspim->SPIM_CTRLR0)) 
#define SPIM_PUT_CTRLR0(port,v)   __raw_writel(v,&(pheadspim->SPIM_CTRLR0)) 
#define SPIM_GET_CTRLR1(port)     __raw_readl(&(pheadspim->SPIM_CTRLR1)) 
#define SPIM_PUT_CTRLR1(port,v)   __raw_writel(v,&(pheadspim->SPIM_CTRLR1)) 
#define SPIM_GET_SPIENR(port)     __raw_readl(&(pheadspim->SPIM_SPIENR)) 
#define SPIM_PUT_SPIENR(port,v)   __raw_writel(v,&(pheadspim->SPIM_SPIENR)) 
#define SPIM_GET_SER(port)        __raw_readl(&(pheadspim->SPIM_SER)) 
#define SPIM_PUT_SER(port,v)      __raw_writel(v,&(pheadspim->SPIM_SER)) 
#define SPIM_GET_TXFTLR(port)     __raw_readl(&(pheadspim->SPIM_TXFTLR)) 
#define SPIM_PUT_TXFTLR(port,v)   __raw_writel(v,&(pheadspim->SPIM_TXFTLR)) 
#define SPIM_GET_RXFTLR(port)     __raw_readl(&(pheadspim->SPIM_RXFTLR)) 
#define SPIM_PUT_RXFTLR(port,v)   __raw_writel(v,&(pheadspim->SPIM_RXFTLR)) 
#define SPIM_GET_IMR(port)        __raw_readl(&(pheadspim->SPIM_IMR)) 
#define SPIM_PUT_IMR(port,v)      __raw_writel(v,&(pheadspim->SPIM_IMR)) 
#define SPIM_GET_BAUDR(port)      __raw_readl(&(pheadspim->SPIM_BAUDR)) 
#define SPIM_PUT_BAUDR(port,v)    __raw_writel(v,&(pheadspim->SPIM_BAUDR)) 
#define SPIM_GET_DMACR(port)      __raw_readl(&(pheadspim->SPIM_DMACR)) 
#define SPIM_PUT_DMACR(port,v)    __raw_writel(v,&(pheadspim->SPIM_DMACR)) 
#define SPIM_GET_DR0(port)        __raw_readl(&(pheadspim->SPIM_DR0)) 
#define SPIM_PUT_DR0(port,v)      __raw_writel(v,&(pheadspim->SPIM_DR0)) 
#define SPIM_GET_SR(port)         __raw_readl(&(pheadspim->SPIM_SR)) 
#define SPIM_PUT_SR(port,v)       __raw_writel(v,&(pheadspim->SPIM_SR))
#define SPIM_GET_ISR(port)        __raw_readl(&(pheadspim->SPIM_ISR)) 
#define SPIM_PUT_ISR(port,v)      __raw_writel(v,&(pheadspim->SPIM_ISR))

#define SPIM_CS0_OUT               __raw_writel(__raw_readl(GPIO0_BASE_ADDR_VA + 0x10) | 0x10 ,(GPIO0_BASE_ADDR_VA + 0x10))
#define SPIM_CS0_HIGH              __raw_writel(__raw_readl(GPIO0_BASE_ADDR_VA + 0x0c) | 0x10 , (GPIO0_BASE_ADDR_VA + 0x0c))
#define SPIM_CS0_LOW               __raw_writel(__raw_readl(GPIO0_BASE_ADDR_VA + 0x0c) & (~0x10),(GPIO0_BASE_ADDR_VA + 0x0c))
#define SPIM_CS1_OUT               __raw_writel(__raw_readl(GPIO0_BASE_ADDR_VA + 0x10) | 0x01 ,(GPIO0_BASE_ADDR_VA + 0x10))
#define SPIM_CS1_HIGH              __raw_writel(__raw_readl(GPIO0_BASE_ADDR_VA + 0x0c) | 0x01 , (GPIO0_BASE_ADDR_VA + 0x0c))
#define SPIM_CS1_LOW               __raw_writel(__raw_readl(GPIO0_BASE_ADDR_VA + 0x0c) & (~0x01),(GPIO0_BASE_ADDR_VA + 0x0c))

/*
 * The core SPI transfer engine just talks to a register bank to set up
 * DMA transfers; transfer queue progress is driven by IRQs.  The clock
 * framework provides the bporte clock, subdivided for each spi_device.
 *
 * Newer controllers, marked with "new_1" flag, have:
 *  - CR.LportTXFER
 *  - SPI_MR.DIV32 may become FDIV or must-be-zero (here: always zero)
 *  - SPI_SR.TXEMPTY, SPI_SR.NSSR (and corresponding irqs)
 *  - SPI_CSRx.CSAAT
 *  - SPI_CSRx.SBCR allows fportter clocking
 */
struct rockchip_spim {
	spinlock_t		lock;

	void __iomem		*regs;
	int			irq;
	struct clk		*clk;
	struct platform_device	*pdev;
	unsigned		new_1:1;
	struct spi_device	*stay;

	u8			stopping;
	struct list_head	queue;
	struct spi_transfer	*current_transfer;
	unsigned long		current_remaining_bytes;
	struct spi_transfer	*next_transfer;
	unsigned long		next_remaining_bytes;

	unsigned char *rx_ptr;		/* pointer in the Tx buffer */
	const unsigned char *tx_ptr;	/* pointer in the Rx buffer */
};

/*
 * Earlier SPI controllers  have a design bug whereby
 * they portsume that spi slave device state will not change on deselect, so
 * that automagic deselection is OK.  ("NPCSx rises if no data is to be
 * transmitted")  Not so!  Workaround uses nCSx pins port GPIOs; or newer
 * controllers have CSAAT and friends.
 *
 * Since the CSAAT functionality is a bit weird on newer controllers port
 * well, we use GPIO to control nCSx pins on all controllers, updating
 * MR.PCS to avoid confusing the controller.  Using GPIOs also lets us
 * support active-high chipselects despite the controller's belief that
 * only active-low devices/systems exists.
 */

static void cs_activate(struct rockchip_spim *port, struct spi_device *spi)
{
  int cr;
  
  #ifdef DEBUG_LHH
  printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
  #endif
  
  while(SPIM_GET_SR(port)& SPI_BUSY_FLAG)
     cpu_relax();
	SPIM_PUT_SPIENR(port,0);
	/* Set the SPI clock phase and polarity */
	cr = SPIM_GET_CTRLR0(port) & (~(SERIAL_CLOCK_PHASE_START | SERIAL_CLOCK_POLARITY_HIGH));
	if (spi->mode & SPI_CPHA)
		  cr |= SERIAL_CLOCK_PHASE_START;
	if (spi->mode & SPI_CPOL)
		  cr |= SERIAL_CLOCK_POLARITY_HIGH;
	SPIM_PUT_CTRLR0(port, cr); 

	/* Activate the chip select */
	SPIM_PUT_SER(port,1+spi->chip_select);
	SPIM_PUT_SPIENR(port,SPI_ENABLE);
}

static void cs_deactivate(struct rockchip_spim *port, struct spi_device *spi)
{
  #ifdef DEBUG_LHH
  printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
  #endif
   while(SPIM_GET_SR(port)& SPI_BUSY_FLAG)
    cpu_relax();
	SPIM_PUT_SER(port,0);
}

/*
 * spi master transmit to fifo write end out  
 */
static inline void rockchip_spim_fill_tx_fifo(struct rockchip_spim *port,struct spi_transfer *xfer)
{
	unsigned int temp;
	
  #ifdef DEBUG_LHH
  printk("Enter::%s\n",__FUNCTION__);
  #endif
  while(SPIM_GET_SR(port)& SPI_BUSY_FLAG)
    cpu_relax();
  SPIM_PUT_SPIENR(port,0);
  temp = SPIM_GET_CTRLR0(port);
  SPIM_PUT_CTRLR0(port,((temp & (~(SPIM_E2PROM_READ|SERIAL_CLOCK_PHASE_START | SERIAL_CLOCK_POLARITY_HIGH))) | TRANSMIT_ONLY));
  SPIM_PUT_SPIENR(port,SPI_ENABLE);
  while(xfer->len != 0)
  {
    if (SPIM_GET_SR(port) & TRANSMIT_FIFO_NOT_FULL) 
    {
      if (port->tx_ptr)
      {
			   SPIM_PUT_DR0(port, *port->tx_ptr++);
		  } 
		  else 
		  {
			   SPIM_PUT_DR0(port, 0);
		  }
		  xfer->len--;
    }  
  }  
	while(!(SPIM_GET_SR(port)& TRANSMIT_FIFO_EMPTY))
    cpu_relax();
  while(SPIM_GET_SR(port)& SPI_BUSY_FLAG)
    cpu_relax();  
}

/*
 * spi master receive to fifo receive end out  
 */
static inline void rockchip_spim_fill_rx_fifo(struct rockchip_spim *port,struct spi_transfer *xfer)
{
    unsigned int sr;
    int lenght;
    int timeouttest=0;
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    
    port->current_transfer = xfer; 
    lenght = xfer->len;  
    while(SPIM_GET_SR(port)& SPI_BUSY_FLAG)
    cpu_relax(); 
		SPIM_PUT_SPIENR(port,0);	
		sr = SPIM_GET_CTRLR0(port);		
    SPIM_PUT_CTRLR0(port,(sr & (~(SPIM_E2PROM_READ | SERIAL_CLOCK_PHASE_START | SERIAL_CLOCK_POLARITY_HIGH)))| RECEIVE_ONLY);    
    SPIM_PUT_CTRLR1(port,lenght-1);
    SPIM_PUT_SPIENR(port,SPI_ENABLE);	
    port->rx_ptr = xfer->rx_buf;
    
    /*lenght more than one FIFO then disable irq */
    if(xfer->len > 0x16)  
        local_irq_disable();
    
	  SPIM_PUT_DR0(port,0);
    while(lenght != 0)
    {
        if (SPIM_GET_SR(port) & RECEIVE_FIFO_NOT_EMPTY)
        {
            *(port->rx_ptr)++ = SPIM_GET_DR0(port);
            lenght--;
            timeouttest=0;
        }
        timeouttest++;
	      if(timeouttest>0x3fff)
	      {	      
	          #ifdef DEBUG_LHH
	          printk("Enter::%s----%d-----lenght =%x\n",__FUNCTION__,__LINE__,lenght);
	          #endif
	          break;		      
	      }
    }
    
   /*lenght more than one FIFO then recover irq */
   if(xfer->len > 0x16) 
      local_irq_enable();  
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

/*
 * spi master init setup 
 */
static int rockchip_spim_setup(struct spi_device *spi)
{
    struct rockchip_spim	*port;
    unsigned int		bits = spi->bits_per_word;
    int div;
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    port = spi_master_get_devdata(spi->master);
    
    if (port->stopping)
        return -ESHUTDOWN;
    if (spi->chip_select > spi->master->num_chipselect) 
    {
        dev_dbg(&spi->dev,"setup: invalid chipselect %u (%u defined)\n",
          spi->chip_select, spi->master->num_chipselect);
    	  return -EINVAL;
    }
    
    if (bits == 0)
        bits = 8;
    if (bits < 4 || bits > 16) {
    	dev_dbg(&spi->dev,
    			"setup: invalid bits_per_word %u (8 to 16)\n",
    			bits);
    	return -EINVAL;
    }
    SPIM_PUT_SPIENR(port,0);
    SPIM_PUT_CTRLR0(port,(SPIM_GET_CTRLR0(port) & 0xf) | (bits - 1));
    if (spi->mode & ~MODEBITS) {
        dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
    		  spi->mode & ~MODEBITS);
    	  return -EINVAL;
    }
    div = rockchip_clk_get_apb()/(spi->max_speed_hz);  //clk_get_rate(port->clk) / hz;
    dev_info(&spi->dev, "setting pre-scaler to %d (hz %d)\n", div, spi->max_speed_hz);
    SPIM_PUT_BAUDR(port,div);   
    cs_activate(port,spi); 
    if(spi->chip_select == 0)
    SPIM_CS0_OUT;
    else if(spi->chip_select == 1)
    SPIM_CS1_OUT;  
    return 0;
}
/*
 * spi master transfer and receive in function
 */
static int rockchip_spim_transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct rockchip_spim	*port;
    struct spi_transfer	*xfer;
    int temp_len;
    int cnt_write_comm;
    unsigned char *tx_buf_temp;
    struct device		*controller = spi->master->dev.parent;
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    
    port = spi_master_get_devdata(spi->master);    
    dev_dbg(controller, "new message %p submitted for %s\n",
    		msg, spi->dev.bus_id);
    
    if (unlikely(list_empty(&msg->transfers) || !spi->max_speed_hz))
        return -EINVAL;
    SPIM_PUT_SPIENR(port,0);    
    SPIM_PUT_BAUDR(port,rockchip_clk_get_apb()/(spi->max_speed_hz));   
    SPIM_PUT_SPIENR(port,SPI_ENABLE);  
    if (port->stopping)
        return -ESHUTDOWN;
    if (spi->chip_select == 0)
    {
        SPIM_CS1_HIGH;     
        SPIM_CS0_LOW;
    }
    else if(spi->chip_select == 1)
    {
        SPIM_CS0_HIGH;     
        SPIM_CS1_LOW;
    } 
     
    list_for_each_entry(xfer, &msg->transfers, transfer_list)
    {
        if (!(xfer->tx_buf || xfer->rx_buf))
        {
    		    dev_dbg(&spi->dev, "missing rx or tx buf\n");
    		    return -EINVAL;
    	  }    
    	  /* FIXME implement these protocol options!! */
    	  if (xfer->bits_per_word || xfer->speed_hz)
    	  {
    		    dev_dbg(&spi->dev, "no protocol options yet\n");
    		    return -ENOPROTOOPT;
    	  }
    		port->tx_ptr = xfer->tx_buf;
        port->rx_ptr = xfer->rx_buf;
        tx_buf_temp = (unsigned char *)xfer->tx_buf;
        temp_len = 	xfer->len;
    		if(xfer->tx_buf)  
    		{
    		    if(xfer->rx_buf)   
    		    {
    		        for (cnt_write_comm = 0; *tx_buf_temp++ != 0; cnt_write_comm++);
    		        if (xfer->len <= cnt_write_comm)
    		            cnt_write_comm = 1;
    		        xfer->len = cnt_write_comm;
    		        rockchip_spim_fill_tx_fifo(port,xfer);
    		        xfer->len = temp_len -cnt_write_comm;
    		    }
    		    else
    		    {
    		        rockchip_spim_fill_tx_fifo(port,xfer);
    		    }
    		  			  
    		}
    		if(xfer->rx_buf)  
    		    rockchip_spim_fill_rx_fifo(port,xfer);
    }
    if (spi->chip_select == 0)    
    SPIM_CS0_HIGH;
    else if(spi->chip_select == 1)
    SPIM_CS1_HIGH; 
    msg->complete(msg->context);
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    return 0;
}

/*
 * spi master exit and dis inite  
 */
static void rockchip_spim_cleanup(struct spi_device *spi)
{
    struct rockchip_spim	*port = spi_master_get_devdata(spi->master);
    unsigned long		flags;
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    if (!spi->controller_state)
        return;
    
    spin_lock_irqsave(&port->lock, flags);
    if (port->stay == spi)
    {
    	  port->stay = NULL;
    	  cs_deactivate(port, spi);
    }
    spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * spi master register init and system init  
 */
static int __init rockchip_spim_probe(struct platform_device *pdev)
{
    struct resource		*regs;
    int			irq;
    ///struct clk		*clk;
    int			ret;
    struct spi_master	*master;
    struct rockchip_spim	*port;
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    
    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!regs)
    	  return -ENXIO;
    
    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
    	  return irq;
    
    //clk = clk_get(&pdev->dev, "spi_clk");
    //if (IS_ERR(clk))
    //	return PTR_ERR(clk);
    
    /* setup spi core then atmel-specific driver state */
    ret = -ENOMEM;
    master = spi_alloc_master(&pdev->dev, sizeof *port);
    if (!master)
    	  goto out_free;
    
    master->bus_num = pdev->id;
    master->num_chipselect = 2;
    master->setup = rockchip_spim_setup;
    master->transfer = rockchip_spim_transfer;
    master->cleanup = rockchip_spim_cleanup;
    platform_set_drvdata(pdev, master);
    
    port = spi_master_get_devdata(master);   
    spin_lock_init(&port->lock);
    INIT_LIST_HEAD(&port->queue);
    port->pdev = pdev;
    port->regs = ioremap(regs->start, (regs->end - regs->start) + 1);
    if (!port->regs)
    	  goto out_free;
    port->irq = irq;
    port->clk = rockchip_clk_get_apb();  //clk;
    
    /* Initialize the hardware */
    //clk_enable(clk);
    SPIM_PUT_SER(port,0);
    SPIM_PUT_CTRLR0(port,0x07);
    SPIM_PUT_CTRLR1(port,0x03);
    SPIM_PUT_SER(port,1);
    SPIM_PUT_TXFTLR(port,0x0f);
    SPIM_PUT_RXFTLR(port,0x0f);
    SPIM_PUT_IMR(port,0x00);  ///0x10);
    
    /* go! */
    dev_info(&pdev->dev, "rockchip SPI Master Controller at 0x%08lx (irq %d)\n",
    		(unsigned long)regs->start, irq);
    
    ret = spi_register_master(master);
    if (ret)
    	  goto out_reset_hw;
    return 0;

out_reset_hw:
    //clk_disable(clk);
    free_irq(irq, master);
    iounmap(port->regs);
out_free:
    //clk_put(clk);
    spi_master_put(master);
    return ret;
}

static int __exit rockchip_spim_remove(struct platform_device *pdev)
{
    struct spi_master	*master = platform_get_drvdata(pdev);
    struct rockchip_spim	*port = spi_master_get_devdata(master);
    struct spi_message	*msg;
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    /* reset the hardware and block queue progress */
    spin_lock_irq(&port->lock);
    port->stopping = 1;
    spin_unlock_irq(&port->lock);
    
    /* Terminate remaining queued transfers */
    list_for_each_entry(msg, &port->queue, queue)
    {
    	/* REVISIT unmapping the dma is a NOP on ARM and AVR32
    	 * but we shouldn't depend on that...
    	 */
        msg->status = -ESHUTDOWN;
        msg->complete(msg->context);
    }
    
    
    //clk_disable(port->clk);
    //clk_put(port->clk);
    free_irq(port->irq, master);
    iounmap(port->regs);
    
    spi_unregister_master(master);
    
    return 0;
}

#ifdef	CONFIG_PM

static int rockchip_spim_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    //struct spi_master	*master = platform_get_drvdata(pdev);
    //struct rockchip_spim	*port = spi_master_get_devdata(master);
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    //clk_disable(port->clk);
    return 0;
}

static int rockchip_spim_resume(struct platform_device *pdev)
{
    //struct spi_master	*master = platform_get_drvdata(pdev);
    //struct rockchip_spim	*port = spi_master_get_devdata(master);
    
    #ifdef DEBUG_LHH
    printk("Enter::%s----%d\n",__FUNCTION__,__LINE__);
    #endif
    //clk_enable(port->clk);
    return 0;
}

#else
#define	rockchip_spim_suspend	NULL
#define	rockchip_spim_resume	NULL
#endif


static struct platform_driver rockchip_spim_driver = {
	.driver		= {
		.name	= "rockchip_spi_master",
		.owner	= THIS_MODULE,
	},
	.suspend	= rockchip_spim_suspend,
	.resume		= rockchip_spim_resume,
	.remove		= __exit_p(rockchip_spim_remove),
};

static int __init rockchip_spim_init(void)
{
               rockchip_scu_apbunit_register( SCU_IPID_SPI0 , "spioclk", NULL );
	  return platform_driver_probe(&rockchip_spim_driver, rockchip_spim_probe);
}
module_init(rockchip_spim_init);

static void __exit rockchip_spim_exit(void)
{
	  platform_driver_unregister(&rockchip_spim_driver);
}
module_exit(rockchip_spim_exit);

MODULE_DESCRIPTION("Rockchip RK28XX Spi master port driver");
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rockchip_spi master port");

