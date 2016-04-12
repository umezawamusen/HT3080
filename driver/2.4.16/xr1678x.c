/*****************************************************************************/
/*
*      xr1678x.c  -- EXAR multiport serial driver for XR16L78x family of UARTS.
*
*      Copyright (C) 2005 Exar Corporation.
*      Copyright (C) 2011 UMEZAWA MUSEN DENKI Co.,Ltd.
*
*      Based on Linux 2.6 Kernel's  drivers/serial/8250.c
*
*      This program is free software; you can redistribute it and/or modify
*      it under the terms of the GNU General Public License as published by
*      the Free Software Foundation; either version 2 of the License, or
*      (at your option) any later version.
*
*      This program is distributed in the hope that it will be useful,
*      but WITHOUT ANY WARRANTY; without even the implied warranty of
*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*      GNU General Public License for more details.
*
*      You should have received a copy of the GNU General Public License
*      along with this program; if not, write to the Free Software
*      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*
*	   Multiport Serial Driver for EXAR's XR16L78x Family of UARTs
*	   for			: LINUX 2.4.16
*	   date			: Febraury, 2005
*	   version		: 1.0
*
*	Check Release Notes for information on what has changed in the new
*	version.
*/
#include <linux/config.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>

#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_core.h>
#include <linux/time.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "xr1678x.h"
#define _INLINE_ inline

#define SERIALEXAR_SHARE_IRQS 1 
unsigned int share_irqs = SERIALEXAR_SHARE_IRQS;

#define UART_XR788_NR	8 // MAX 8 ports per card for 788:

#define UART_MAX_BOARDS  4 /* MAX 4 boards */

#define XR_788_MAJOR       40
#define XR_788_MINOR       0

#define PASS_LIMIT	256
#define UART_HT_SPR 0x07
#define UART_HT_FCTR 0x08
#define UART_HT_EFR 0x09
#define UART_HT_TXTRG 0x0A
#define UART_HT_RXTRG 0x0B
#define UART_HT_XOFF1 0x0C
#define UART_HT_XOFF2 0x0D
#define UART_HT_XON1 0x0E
#define UART_HT_XON2 0x0F


#define HT_FCTR_EN_AUTO_RS485 0x20
#define EFR_EFBE 0x10
#define MSR_RS485_DELAY 0xF0
#define HT_AUTO485_TIMER_1M_SEC 0x3999 - 1 /* 1ms timer */

#define UART_HT_DCR_OFFSET 0x80
#define UART_HT_INT1	0x01
#define UART_HT_INT2	0x02
#define UART_HT_INT3	0x03
#define UART_HT_TMRCTL	0x04
#define UART_HT_TMRLSB	0x06
#define UART_HT_TMRMSB	0x07

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define HT_SPR_SET_TIMER BIT0
#define HT_SPR_ECHO_FLAG BIT1
#define HT_SPR_LOCAL_ECHO BIT2
#define HT_SPR_HALF_DUPLEX BIT3
#define HT_SPR_MULTI_DROP BIT4

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)

struct irq_info {
	spinlock_t		lock;
	struct list_head	*head;
};
static struct uart_info *IRQ_ports[NR_IRQS];
unsigned int max_ports = UART_XR788_NR * UART_MAX_BOARDS; 
unsigned int auto_485_timer_flag = 0;
unsigned int auto_485_timer_num = HT_AUTO485_TIMER_1M_SEC;

#define port_acr        unused[0]       /* 8bit */
#define port_ier        unused[1]       /* 8bit */
#define port_rev        unused[2]       /* 8bit */
#define port_lcr        unused[3]       /* 8bit */

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
#define PORT_MAX_XR 1 
#define XR788_TYPE 1 // the second entry that is [1] in the array
static const struct serial_uart_config uart_config[PORT_MAX_XR+1] = {
	{ "Unknown",	1,	0 },
	{ "XR78x",		64,	0 },
};

#define ht_outb(v,p) __raw_writeb(v,__io(ISAIO8_BASE+p))
#define ht_inb(p) __raw_readb(__io(ISAIO8_BASE+p))

static _INLINE_ unsigned int serial_in(struct uart_port *port, int offset)
{
	offset <<= port->regshift;
	switch (port->iotype) {
	case SERIAL_IO_HUB6:
		ht_outb(port->hub6 - 1 + offset, port->iobase);
		return ht_inb(port->iobase + 1);
	case SERIAL_IO_MEM:
		return readb(port->membase + offset);

	default:
		return ht_inb(port->iobase + offset);
	}
}

static _INLINE_ void
serial_out(struct uart_port *port, int offset, int value)
{
	offset <<= port->regshift;

	switch (port->iotype) {
	case SERIAL_IO_HUB6:
		ht_outb(port->hub6 - 1 + offset, port->iobase);
		ht_outb(value, port->iobase + 1);
		break;

	case SERIAL_IO_MEM:
		writeb(value, port->membase + offset);
		break;

	default:
		ht_outb(value, port->iobase + offset);
//		printk("Sout %04x %02x\n",port->iobase+offset, value);
	}
}
static _INLINE_ unsigned int dcr_in(struct uart_port *port, int offset)
{
	int iobase = (port->iobase & 0xf00) + UART_HT_DCR_OFFSET;
	return ht_inb(iobase + offset);
}
static _INLINE_ void
dcr_out(struct uart_port *port, int offset, int value)
{
	int iobase = (port->iobase & 0xf00) + UART_HT_DCR_OFFSET;
	ht_outb(value, iobase + offset);
}
/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)
static void set_spr_reg(struct uart_port *port, char data)
{
	char spr = serial_inp(port, UART_HT_SPR);
	spr |= data;
	serial_outp(port,UART_HT_SPR,spr);

}

static void reset_spr_reg(struct uart_port *port, char data)
{
	char spr = serial_inp(port, UART_HT_SPR);
	spr &= ~data;
	serial_outp(port,UART_HT_SPR,spr);

}

static char get_spr_reg(struct uart_port *port)
{
	return serial_inp(port, UART_HT_SPR);
}

static int is_auto_rs485(struct uart_port *port)
{
	unsigned char FCTR = serial_inp(port, UART_HT_FCTR);
	if ( FCTR & HT_FCTR_EN_AUTO_RS485)
		return 1;

	return 0;
}
void set_rx_clear_timer(struct uart_port *port)
{
	if ( is_auto_rs485(port) == 0 )
	{
		return; /* not set timer */
	}
	set_spr_reg(port, HT_SPR_SET_TIMER);
	if ( dcr_in(port, UART_HT_TMRCTL) & 0x20 )
	{
		return;
	}
	dcr_out(port, UART_HT_TMRMSB, ((auto_485_timer_num>>8)&0xff));
	dcr_out(port, UART_HT_TMRLSB, (auto_485_timer_num&0xff));
	dcr_out(port, UART_HT_TMRCTL, 0x03);

}
static void serialxr78x_stop_tx(struct uart_port *port, unsigned int tty_stop)
{
//	printk("stop_tx\n");

	if (port->port_ier & UART_IER_THRI) {
		port->port_ier &= ~UART_IER_THRI;
		serial_out(port, UART_IER, port->port_ier);
		set_rx_clear_timer(port);		
	}
}

static void serialxr78x_start_tx(struct uart_port *port, unsigned int nonempty, unsigned int tty_start)
{
//	printk("start_tx\n");
	if (nonempty && !(port->port_ier & UART_IER_THRI)) {
		port->port_ier |= UART_IER_THRI;
		serial_out(port, UART_IER, port->port_ier);
		if ( is_auto_rs485(port) == 1 && 
		     !(get_spr_reg(port) & HT_SPR_LOCAL_ECHO))
		{
			set_spr_reg(port, HT_SPR_ECHO_FLAG);	
		}
	}
}

static void serialxr78x_stop_rx(struct uart_port *port)
{

//	printk("stop_rx\n");
	port->port_ier &= ~UART_IER_RLSI;
	port->read_status_mask &= ~UART_LSR_DR;
	serial_out(port, UART_IER, port->port_ier);
}

static void serialxr78x_enable_ms(struct uart_port *port)
{

//	printk("enable_ms\n");
	port->port_ier |= UART_IER_MSI;
	serial_out(port, UART_IER, port->port_ier);
}
static _INLINE_ void
receive_chars(struct uart_info *info, int *status, struct pt_regs *regs)
{
	struct tty_struct *tty = info->tty;
	struct uart_port *port = info->port;
	unsigned char ch;
	int max_count = 256;
//	printk("receive_chars\n");

	do {
                if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
	                tty->flip.tqueue.routine((void *)tty);
                        if (tty->flip.count >= TTY_FLIPBUF_SIZE)
                                return; // if TTY_DONT_FLIP is set
                }
		if ( is_auto_rs485(port) == 1 && get_spr_reg(port)&HT_SPR_ECHO_FLAG )
		{
			/* auto RS485 TX from RX */
			ch = serial_inp(port, UART_RX);
			goto ignore_char;
		}
		ch = serial_inp(port, UART_RX);
		*tty->flip.char_buf_ptr = ch;
		*tty->flip.flag_buf_ptr = TTY_NORMAL;
		port->icount.rx++;

		if (*status & (UART_LSR_BI | UART_LSR_PE |
			       UART_LSR_FE | UART_LSR_OE)) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				port->icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
//				if (uart_handle_break(info, &serial78x_console))
//					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				port->icount.parity++;
			else if (*status & UART_LSR_FE)
				port->icount.frame++;
			if (*status & UART_LSR_OE)
				port->icount.overrun++;

			/*
			 * Mask off conditions which should be ingored.
			 */
			*status &= port->read_status_mask;

			if (*status & UART_LSR_BI) {
				*tty->flip.flag_buf_ptr = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				*tty->flip.flag_buf_ptr = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				*tty->flip.flag_buf_ptr = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(info, ch, regs))
			goto ignore_char;
		if ((*status & port->ignore_status_mask) == 0) {
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
		if ((*status & UART_LSR_OE) &&
		    tty->flip.count < TTY_FLIPBUF_SIZE) {
			/*
			 * Overrun is special, since it's reported
			 * immediately, and doesn't affect the current
			 * character.
			 */
			*tty->flip.flag_buf_ptr = TTY_OVERRUN;
			tty->flip.flag_buf_ptr++;
			tty->flip.char_buf_ptr++;
			tty->flip.count++;
		}
	ignore_char:
		*status = serial_inp(port, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
}
static _INLINE_ void transmit_chars(struct uart_info *info)
{
	struct uart_port *port = info->port;
	int count;

//	printk("transmit_chars\n");
	if (port->x_char) {
		serial_outp(port, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
        if (info->xmit.head == info->xmit.tail
             || info->tty->stopped
             || info->tty->hw_stopped) {
		serialxr78x_stop_tx(port, 0);
		return;
	}

	count = port->fifosize;
	do {
		serial_out(port, UART_TX, info->xmit.buf[info->xmit.tail]);
		info->xmit.tail = (info->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (info->xmit.head == info->xmit.tail)
			break;
	} while (--count > 0);
	if (CIRC_CNT(info->xmit.head, info->xmit.tail, UART_XMIT_SIZE) < WAKEUP_CHARS)
		uart_event(info, EVT_WRITE_WAKEUP);		
	
	if (info->xmit.head == info->xmit.tail)
		serialxr78x_stop_tx(port, 0);
}

static _INLINE_ void check_modem_status(struct uart_info *info)
{
	struct uart_port *port = info->port;
	int status;

//	printk("check_modem_status\n");
	status = serial_in(port, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		port->icount.rng++;
	if (status & UART_MSR_DDSR)
		port->icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(info, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(info, status & UART_MSR_CTS);

	wake_up_interruptible(&info->delta_msr_wait);
}

/*
 * This handles the interrupt from one port.
 */
static inline void
serialxr78x_handle_port(struct uart_info *info, struct pt_regs *regs)
{
	struct uart_port *port = info->port;
	unsigned int status = serial_inp(port, UART_LSR);

	if (status & UART_LSR_DR)
		receive_chars(info, &status, regs);
	check_modem_status(info);
	if (status & UART_LSR_THRE)
		transmit_chars(info);

}
int is_clear_timer_flag(struct uart_port *port)
{
	int i;
	int base = port->iobase & 0xf00;
	int pt = port->iobase & 0x0f0;

	for ( i = 0; i <= 0x70; i+=0x10 )
	{
		char spr = ht_inb(base + i + UART_HT_SPR);
		if ( spr & HT_SPR_SET_TIMER )
		{
			if ( i == pt )
			{
				continue;
			}
			return 0;
		}
	}
	return 1;
}
void toggle_gpio(int flag)
{
	unsigned long tmp;
	tmp = inb(0xFF000041); /* PBDDR */
	tmp |= 0x80;
	outb(tmp, 0xFF000041);
	tmp = 0xff;
	if ( flag == 0 )
		tmp &= ~ 0x80;
	else
		tmp |= 0x80;
	outb(tmp,0xFF000001);
}
void device_configuration_interrupt(struct uart_info *info)
{
	struct uart_port *port = info->port;
	char int1 = dcr_in(port, UART_HT_INT1);
	char spr = serial_in(port, UART_HT_SPR);

	if ( int1 & 0x07 )
	{
		auto_485_timer_flag = 1;	
	}
	if( auto_485_timer_flag && (spr & HT_SPR_SET_TIMER) ) 
	{	

		if(!(serial_inp(port, UART_LSR) & 0x40))
		{
			return;
		}
		if(is_clear_timer_flag(port) == 1)	
		{
			char timer_ctrl = dcr_in(port, UART_HT_TMRCTL);
			dcr_out(port,UART_HT_TMRCTL,(timer_ctrl&~0x2));
		}
		reset_spr_reg(port, (HT_SPR_SET_TIMER|HT_SPR_ECHO_FLAG) );
		serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO|UART_FCR_CLEAR_RCVR); /* RX FIFO reset */
		auto_485_timer_flag = 0;
	}
}
#define UART_ISR 0x02 
/*
 * This is the serial driver's interrupt routine.
 *
 * Arjan thinks the old way was overly complex, so it got simplified.
 * Alan disagrees, saying that need the complexity to handle the weird
 * nature of ISA shared interrupts.  (This is a special exception.)
 *
 * In order to handle ISA shared interrupts properly, we need to check
 * that all ports have been serviced, and therefore the ISA interrupt
 * line has been de-asserted.
 *
 * This means we need to loop through all ports. checking that they
 * don't have an interrupt pending.
 */
static void serialxr78x_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct uart_info *info;
	struct uart_info *end_mark = NULL;
	int pass_counter = 0;

	//DEBUG_INTR("serialxr78x_interrupt(%d)...", irq);
//	printk("serialxr78x_interrupt \n");

	info = *(struct uart_info **)dev_id;
	if ( !info )
		return;
	do {
		unsigned int iir;

		iir = serial_in(info->port, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			spin_lock(&info->lock);
			serialxr78x_handle_port(info, regs);
			spin_unlock(&info->lock);
			end_mark = NULL;
		} else if (end_mark == NULL)
			end_mark = info;

		spin_lock(&info->lock);
		device_configuration_interrupt(info);
		spin_unlock(&info->lock);
		info = info->next_info;
		if ( info )
			continue;
		info = *(struct uart_info **)dev_id;	

		if (pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR "serialxr78x: too much work for "
				"irq%d\n", irq);
			break;
		}
	} while (info != end_mark);


	//DEBUG_INTR("end.\n");
	/* FIXME! Was it really ours? */
}
#if 0
/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_port *port)
{
	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &up->list)
			i->head = i->head->next;
		list_del(&up->list);
	} else {
		BUG_ON(i->head != &up->list);
		i->head = NULL;
	}

	spin_unlock_irq(&i->lock);
}

static int serial_link_irq_chain(struct uart_port *port)
{
	struct irq_info *i = irq_lists + port->irq;
	int ret, irq_flags = port->flags & UPF_SHARE_IRQ ? SA_SHIRQ : 0;

	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add(&up->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&up->list);
		i->head = &up->list;
		spin_unlock_irq(&i->lock);

		ret = request_irq(port->irq, serialxr78x_interrupt,
				  irq_flags, "xrserial", i);
		if (ret < 0)
			serial_do_unlink(i, port);
	}

	return ret;
}

static void serial_unlink_irq_chain(struct uart_port *port)
{
	struct irq_info *i = irq_lists + port->irq;

	BUG_ON(i->head == NULL);

	if (list_empty(i->head))
		free_irq(port->irq, i);

	serial_do_unlink(i, port);
}
/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
static void serialxr78x_timeout(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;
	unsigned int timeout;
	unsigned int iir;

//	printk("timeout\n");
	iir = serial_in(port, UART_IIR);
	if (!(iir & UART_IIR_NO_INT)) {
		spin_lock(&port->lock);
		serialxr78x_handle_port(up, NULL);
		spin_unlock(&port->lock);
	}

	timeout = up->port.timeout;
	timeout = timeout > 6 ? (timeout / 2 - 2) : 1;
	mod_timer(&up->timer, jiffies + timeout);
}
#endif
static unsigned int serialxr78x_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret;

//	printk("tx_empty\n");

	save_flags(flags); cli();
	ret = serial_in(port, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	restore_flags(flags);

	return ret;
}

static unsigned int serialxr78x_get_mctrl(struct uart_port *port)
{
	unsigned long flags;
	unsigned char status;
	unsigned int ret;

//	printk("get_mctrl\n");
	save_flags(flags); cli();
	status = serial_in(port, UART_MSR);
	restore_flags(flags);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}
#define MASK (UART_MCR_OUT1 | UART_MCR_OUT2)
static void serialxr78x_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned char mcr = 0;
//	printk("set_mctrl\n");

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & ~MASK);

	serial_out(port, UART_MCR, mcr);
}

static void serialxr78x_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;

//	printk("break_ctl\n");
	save_flags(flags); cli();
	if (break_state == -1)
		port->port_lcr |= UART_LCR_SBC;
	else
		port->port_lcr &= ~UART_LCR_SBC;
	serial_out(port, UART_LCR, port->port_lcr);
	restore_flags(flags);
}

static int serialxr78x_startup(struct uart_port *port, struct uart_info *info)
{
	unsigned long flags;
	int retval;

//	printk("statup\n");

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in set_termios())
	 */
	serial_outp(port, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_outp(port, UART_FCR, 0);
	
	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_inp(port, UART_LSR);
	(void) serial_inp(port, UART_RX);
	(void) serial_inp(port, UART_IIR);
	(void) serial_inp(port, UART_MSR);

	/*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */
        if (port->irq && (!IRQ_ports[port->irq] ||
            !IRQ_ports[port->irq]->next_info)) {
		retval = request_irq(port->irq, serialxr78x_interrupt,
				     SA_SHIRQ,"xrserial",
				     &IRQ_ports[port->irq]); 
		if ( retval )
			return retval;
	}

        /*
         * Insert serial port into IRQ chain.
         */
         info->next_info = IRQ_ports[port->irq];
         IRQ_ports[port->irq] = info;

	/*
	 * Now, initialize the UART
	 */
	serial_outp(port, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&info->lock, flags);
		
	/*
	* Most PC uarts need OUT2 raised to enable interrupts.
	*/
	if (is_real_interrupt(port->irq))
		info->mctrl |= TIOCM_OUT2;

	serialxr78x_set_mctrl(port, info->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	port->port_ier = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;	
	serial_outp(port, UART_IER, port->port_ier);

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_inp(port, UART_LSR);
	(void) serial_inp(port, UART_RX);
	(void) serial_inp(port, UART_IIR);
	(void) serial_inp(port, UART_MSR);

	/* init Scratch pad reg. */
	serial_outp(port, UART_HT_SPR, 0x00);

	return 0;
}

static void serialxr78x_shutdown(struct uart_port *port, struct uart_info *info)
{
	struct uart_info **infop;
	int retval;
	unsigned long flags;
//	printk("shutdown\n");

        /*
         * First unlink the serial port from the IRQ chain...
         */
        for (infop = &IRQ_ports[port->irq]; *infop; infop = &(*infop)->next_info)
	        if (*infop == info)
	                break;
 
        if (*infop == info)
                *infop = info->next_info;

        /*
         * Free the IRQ, if necessary
         */
        if (port->irq && (!IRQ_ports[port->irq] ||
                          !IRQ_ports[port->irq]->next_info)) {
                free_irq(port->irq, &IRQ_ports[port->irq]);
                if (IRQ_ports[port->irq]) {
                        retval = request_irq(port->irq, serialxr78x_interrupt,
                                             SA_SHIRQ, "xrserial", &IRQ_ports[port->irq]);
                        if (retval)
                                printk("serial shutdown: request_irq: error%d"
                                       " couldn't reacquire IRQ.\n", retval);
                }
	}


	/*
	 * Disable interrupts from this port
	 */
	port->port_ier = 0;
	serial_outp(port, UART_IER, 0);

	spin_lock_irqsave(&info->lock, flags);
	
	info->mctrl &= ~TIOCM_OUT2;

	serialxr78x_set_mctrl(port, info->mctrl);
	spin_unlock_irqrestore(&info->lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(port, UART_LCR, serial_inp(port, UART_LCR) & ~UART_LCR_SBC);
	serial_outp(port, UART_FCR, UART_FCR_ENABLE_FIFO |
				  UART_FCR_CLEAR_RCVR |
				  UART_FCR_CLEAR_XMIT);
	serial_outp(port, UART_FCR, 0);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(port, UART_RX);

}
static void
serialxr78x_change_speed(struct uart_port *port, unsigned int cflag, unsigned int iflag, unsigned int quot)
{
	unsigned char cval;
	unsigned long flags;

//	printk("change_speed\n");
	switch (cflag & CSIZE) {
	case CS5:
		cval = 0x00;
		break;
	case CS6:
		cval = 0x01;
		break;
	case CS7:
		cval = 0x02;
		break;
	default:
	case CS8:
		cval = 0x03;
		break;
	}

	if (cflag & CSTOPB)
		cval |= 0x04;
	if (cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	save_flags(flags); cli();

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;
	/*
	 * Characteres to ignore
	 */
	port->ignore_status_mask = 0;
	if (iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;
	/*
	 * CTS flow control flag and modem status interrupts
	 */
	port->port_ier &= ~UART_IER_MSI;
	if (port->flags & ASYNC_HARDPPS_CD || cflag & CRTSCTS ||
	    !(cflag &CLOCAL))
		port->port_ier |= UART_IER_MSI;

	serial_out(port, UART_IER, port->port_ier);

	serial_outp(port, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	
	serial_outp(port, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_outp(port, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_outp(port, UART_LCR, cval);		/* reset DLAB */
	port->port_lcr = cval;					/* Save LCR */
	
	serial_outp(port, UART_FCR, UART_FCR_ENABLE_FIFO);/* set fcr */
	restore_flags(flags);

}
/*
 *      EXAR ioctls
 */
//#define 	FIOQSIZE		0x5460 
#define		EXAR_READ_REG      	(FIOQSIZE + 1)
#define 	EXAR_WRITE_REG     	(FIOQSIZE + 2)

struct xrioctl_rw_reg {
	unsigned char reg;
	unsigned char regvalue;
};

static int ht_set_auto485_dly(struct uart_port *up, unsigned long arg )
{
	char efr;
	char msr = 0x00;

	if ( arg > 0x0f)
		return -EINVAL;
	
	efr = serial_inp(up,UART_HT_EFR);

	efr |= EFR_EFBE;
	serial_outp(up,UART_HT_EFR,efr);
	msr |= (arg << 4);
	msr &= 0xf0;				/* mask */
	serial_outp(up,UART_MSR,msr);
	efr &= ~EFR_EFBE;
	serial_outp(up,UART_HT_EFR,efr);
	return 0;
}
static int ht_set_local_echo(struct uart_port *up, unsigned long arg )
{
	switch( arg )
	{
		case HT_SET_LOCAL_ECHO_ON:
			set_spr_reg(up,HT_SPR_LOCAL_ECHO);
		break;
		case HT_SET_LOCAL_ECHO_OFF:
			reset_spr_reg(up,HT_SPR_LOCAL_ECHO);
		break;
		default:
			return -EINVAL;
		break;
	}

	return 0;
}

#define MCR_DTR BIT0
#define MCR_RTS BIT1

static int ht_set_rs485_mode(struct uart_port *up, unsigned long arg )
{
	char mcr = serial_inp(up,UART_MCR);
	switch( arg )
	{
		case HT_SET_RS485_FULL:
			mcr |= MCR_DTR;
		break;
		case HT_SET_RS485_HALF:
			mcr &= ~MCR_DTR;
		break;
		default:
			return -EINVAL;
		break;
	}
	mcr |= MCR_RTS;
	serial_outp(up,UART_MCR,mcr);

	return 0;
}

#define HT_EFR_AUTO_RTS 0x40
#define HT_EFR_AUTO_CTS 0x80
#define HT_MCR_EN_DTR_DSR 0x04
#define HT_PROG_RXTRG_TXTRG (BIT6|BIT7)
#define HT_RXTRG 42 /* 64 * 2/3 */ 
#define HT_TXTRG 42 /* 64 * 2/3 */ 
#define HT_HYST 0x03 /* +/-8 */

static int ht_set_auto_hardflow(struct uart_port *up, unsigned long arg )
{
	char efr;
	char mcr;
	char fctr;

	/* FIFO clear */
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	
	efr = serial_inp(up,UART_HT_EFR);
	mcr = serial_inp(up,UART_MCR);
	fctr = serial_inp(up,UART_HT_FCTR);
	switch( arg )
	{
		case HT_SET_FLOW_ENABLE:
//			printk("enable auto_hardflow\n");
			fctr |= HT_PROG_RXTRG_TXTRG;
			fctr |= HT_HYST; 
			serial_outp(up,UART_HT_FCTR,fctr); /* default */

			serial_outp(up,UART_HT_RXTRG,HT_RXTRG); /* default */
			serial_outp(up,UART_HT_TXTRG,HT_TXTRG); /* default */
			efr |= 	(HT_EFR_AUTO_RTS | HT_EFR_AUTO_CTS);
			efr |= EFR_EFBE;
			serial_outp(up,UART_HT_EFR,efr);
			mcr &= ~HT_MCR_EN_DTR_DSR;
			mcr |= MCR_DTR | MCR_RTS;
			serial_outp(up,UART_MCR, mcr);
			efr &= ~ EFR_EFBE;
			serial_outp(up,UART_HT_EFR,efr);

		break;
		case HT_SET_FLOW_DISABLE:
//			printk("disable auto_hardflow\n");
			fctr &= ~HT_PROG_RXTRG_TXTRG;
			serial_outp(up,UART_HT_FCTR,fctr); /* default */
			efr &= 	~(HT_EFR_AUTO_RTS | HT_EFR_AUTO_CTS);
			efr |= EFR_EFBE;
			serial_outp(up,UART_HT_EFR,efr);
			mcr &= ~HT_MCR_EN_DTR_DSR; 			
			serial_outp(up,UART_MCR, mcr);
			efr &= ~ EFR_EFBE;
			serial_outp(up,UART_HT_EFR,efr);
		break;
		default:
			return -EINVAL;
		break;
	}
	return 0;
}
#define AUTO_TRANSMIT_XON1_XOFF1 BIT3 
#define AUTO_COMP_XON1_XOFF1 BIT1 
#define HT_XON   0x11
//#define HT_XON   0x31
#define HT_XOFF  0x13
//#define HT_XOFF  0x33
static int ht_set_auto_softflow(struct uart_port *up, unsigned long arg )
{
	char efr;
	char fctr;
	/* FIFO clear */
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	
	efr = serial_inp(up,UART_HT_EFR);
	fctr = serial_inp(up,UART_HT_FCTR);

	switch( arg )
	{
		case HT_SET_FLOW_ENABLE:
//			printk("enable auto_softflow\n");


			fctr |= HT_PROG_RXTRG_TXTRG; /* table D */
			fctr |= HT_HYST; 
			serial_outp(up,UART_HT_FCTR,fctr);

			efr |= (AUTO_TRANSMIT_XON1_XOFF1 | AUTO_COMP_XON1_XOFF1);

			serial_outp(up,UART_HT_EFR,efr);
			serial_outp(up,UART_HT_XON1,HT_XON);
			serial_outp(up,UART_HT_XOFF1,HT_XOFF);

			serial_outp(up,UART_HT_RXTRG,HT_RXTRG); /* default */
			serial_outp(up,UART_HT_TXTRG,HT_TXTRG); /* default */
		break;
	
		case HT_SET_FLOW_DISABLE:
//			printk("disable auto_softflow\n");
			fctr &=  ~HT_PROG_RXTRG_TXTRG;
			serial_outp(up,UART_HT_FCTR,fctr);
			efr &= ~0x0f;
			serial_outp(up,UART_HT_EFR,efr);
		break;
		default:
			return -EINVAL;
		break;
	}
	return 0;
}
static int ht_set_rxtrg(struct uart_port *up, unsigned long arg )
{
	char rxtrg = (char)arg&0xff;
	if ( arg > 0x40 )
		return -EINVAL;
	serial_outp(up,UART_HT_RXTRG,rxtrg);
	return 0;

}

static int ht_set_txtrg(struct uart_port *up, unsigned long arg )
{
	char txtrg = (char)arg&0xff;
	if ( arg > 0x40 )
		return -EINVAL;
	serial_outp(up,UART_HT_TXTRG,txtrg);
	return 0;

}
static int ht_set_hyst(struct uart_port *up, unsigned long arg)
{
	char fctr = serial_inp(up,UART_HT_FCTR);
	if ( arg > 0x0f)
		return -EINVAL;

	fctr &= ~0x0f; /* clear hyst */
	fctr |= (arg&0x0f);  /* set hyst */
	serial_outp(up, UART_HT_FCTR, fctr);
	return 0;
}

static int ht_set_autoRS485(struct uart_port *port, unsigned long flag )
{
	unsigned char FCTR,MSR,EFR;
	int ret = 0;

	FCTR = serial_inp(port,UART_HT_FCTR);
	EFR = serial_inp(port,UART_HT_EFR);
	MSR = serial_inp(port,UART_MSR);

	if ( flag == HT_SET_AUTO_RS485_ENABLE )
	{
		/* enable auto RS485 */
//		printk("enable autoRS485\n");
		
		FCTR |= HT_FCTR_EN_AUTO_RS485;
		serial_outp(port,UART_HT_FCTR,FCTR);
		EFR |= EFR_EFBE;
		serial_outp(port,UART_HT_EFR,EFR);
		MSR |= MSR_RS485_DELAY;
		serial_outp(port,UART_MSR,MSR);
		EFR &= ~EFR_EFBE;
		serial_outp(port,UART_HT_EFR,EFR);
	}
	else if ( flag == HT_SET_AUTO_RS485_DISABLE ) 
	{
		/* disable auto RS485 */
//		printk("disable autRS485\n");
		FCTR &= ~HT_FCTR_EN_AUTO_RS485;
		serial_outp(port,UART_HT_FCTR,FCTR);
		EFR |= EFR_EFBE;
		serial_outp(port,UART_HT_EFR,EFR);
		MSR &= ~MSR_RS485_DELAY;
		serial_outp(port,UART_MSR,MSR);
		EFR &= ~EFR_EFBE;
		serial_outp(port,UART_HT_EFR,EFR);
	}
	else
	{
		ret = -EINVAL;
	}
	return ret;

}

/*
 * This function is used to handle Exar Device specific ioctl calls
 * The user level application should have defined the above ioctl
 * commands with the above values to access these ioctls and the 
 * input parameters for these ioctls should be struct xrioctl_rw_reg
 * The Ioctl functioning is pretty much self explanatory here in the code,
 * and the register values should be between 0 to XR_17X15Y_EXTENDED_RXTRG
 */

static int
serialxr78x_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	struct xrioctl_rw_reg ioctlrwarg;

//	printk("ioctl\n");

	switch (cmd)
	{
		case EXAR_READ_REG:
		if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
			return -EFAULT;
		ioctlrwarg.regvalue = serial_inp(port, ioctlrwarg.reg);
		if (copy_to_user((void *)arg, &ioctlrwarg, sizeof(ioctlrwarg)))
			return -EFAULT;
		ret = 0;
		break;
		
		case EXAR_WRITE_REG:
		if (copy_from_user(&ioctlrwarg, (void *)arg, sizeof(ioctlrwarg)))
			return -EFAULT;
		serial_outp(port, ioctlrwarg.reg, ioctlrwarg.regvalue);
		ret = 0;
		break;

		case HT_SET_AUTO_RS485:
		ret = ht_set_autoRS485(port,arg);
		break;

		case HT_SET_XON_XOFF:
		ret = ht_set_auto_softflow(port,arg);
		break;

		case HT_SET_RTSCTS:
		ret = ht_set_auto_hardflow(port,arg);
		break;

		case HT_SET_RS485_MODE:
		ret = ht_set_rs485_mode(port,arg);
		break;

		case HT_SET_LOCAL_ECHO:
		ret = ht_set_local_echo(port,arg);
		break;

		case HT_SET_HYSTERESIS:
		ret = ht_set_hyst(port,arg);
		break;

		case HT_SET_RXTRG:
		ret = ht_set_rxtrg(port,arg);
		break;
	
		case HT_SET_TXTRG:
		ret = ht_set_txtrg(port,arg);
		break;
	
		case HT_SET_AUTO485_DLY:
		ret = ht_set_auto485_dly(port,arg);
		break;

		default:
		break;
	}
	
	return ret;
}

static void
serialxr78x_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
//	printk("pm\n");
	if (state) {
		/* sleep */
		serial_outp(port, UART_IER, UART_IERX_SLEEP);
	} else {
		/* wake */
		/* Wake up UART */
		serial_outp(port, UART_IER, 0);
	}
}

static void serialxr78x_release_port(struct uart_port *port)
{	
}

static int serialxr78x_request_port(struct uart_port *port)
{
	return 0;
}

static void serialxr78x_config_port(struct uart_port *port, int flags)
{
	
//	printk("config_port\n");
	if (flags & UART_CONFIG_TYPE)
	{	
		port->type = XR788_TYPE;
		port->fifosize = uart_config[port->type].dfl_xmit_fifo_size;
	}
}

static int
serialxr78x_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->irq >= NR_IRQS || ser->irq < 0 ||
	    ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
	    ser->type > PORT_MAX_8250 || ser->type == PORT_CIRRUS ||
	    ser->type == PORT_STARTECH)
		return -EINVAL;
	return 0;
}
static const char *
serialxr78x_type(struct uart_port *port)
{
	int type = port->type;

//	printk("type\n");
	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static struct uart_ops serialxr78x_pops = {
	.tx_empty	= serialxr78x_tx_empty,
	.set_mctrl	= serialxr78x_set_mctrl,
	.get_mctrl	= serialxr78x_get_mctrl,
	.stop_tx	= serialxr78x_stop_tx,
	.start_tx	= serialxr78x_start_tx,
	.stop_rx	= serialxr78x_stop_rx,
	.enable_ms	= serialxr78x_enable_ms,
	.break_ctl	= serialxr78x_break_ctl,
	.startup	= serialxr78x_startup,
	.shutdown	= serialxr78x_shutdown,
	.change_speed   = serialxr78x_change_speed,
	.pm		= serialxr78x_pm,
	.type		= serialxr78x_type,
	.release_port	= serialxr78x_release_port,
	.request_port	= serialxr78x_request_port,
	.config_port	= serialxr78x_config_port,
	.ioctl		= serialxr78x_ioctl,
	.verify_port	= serialxr78x_verify_port,
};

static struct uart_port serialxr78x_ports[UART_XR788_NR*UART_MAX_BOARDS];

#define XR78x_BASE_IO 		0x500
#define XR78x_IRQ 		IRQ_ISA5	
#define XR78x_UART_OFFSET 	0x10

static int irq[UART_MAX_BOARDS];
static int linux_irq[UART_MAX_BOARDS];
static int io[UART_MAX_BOARDS];

static int irq_count = 0;
static int linux_irq_count = 0;
static int io_count = 0;

static int major = XR_788_MAJOR;

static void __init serial78x_init_ports(void)
{
	struct uart_port *port;
	static int first = 1;
	int i;
	int j = 0;

	if (!first)
		return;
	first = 0;
	for (i = 0, port = serialxr78x_ports; i < max_ports;
	     i++, port++) {
		port->iobase   = io[i/UART_XR788_NR] + (j*XR78x_UART_OFFSET);
		port->irq      = irq_cannonicalize(linux_irq[i/UART_XR788_NR]);
		port->uartclk  = 921600 * 16;
		port->flags    = ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST;
		port->hub6     = 0;
		port->membase  = 0;
		port->iotype   = SERIAL_IO_PORT;
		port->regshift = 0;
		port->ops      = &serialxr78x_pops;
		if (share_irqs)
			port->flags |= ASYNC_SHARE_IRQ;
		++j;
		if ( j >= UART_XR788_NR )
			j = 0;
	}

}
#if 0
static void __init serialxr78x_register_ports(struct uart_driver *drv)
{
	int i;
	
	serial78x_init_ports();
	for (i = 0; i < max_ports; i++) {
		struct uart_port *port = &serialxr78x_ports[i];

		port->line = i;
		port->ops = &serialxr78x_pops;
		init_timer(&up->timer);
		timer.function = serialxr78x_timeout;

		uart_add_one_port(drv, port);
	}
}
#endif
#define SERIALXR_CONSOLE	NULL

static struct tty_driver	exvar_sdriver, exver_cdriver;
static struct tty_struct *	exver_tty[UART_XR788_NR*UART_MAX_BOARDS+1];
static struct termios *		exver_termios[UART_XR788_NR*UART_MAX_BOARDS+1];
static struct termios *		exver_termios_locked[UART_XR788_NR*UART_MAX_BOARDS+1];

static struct uart_driver serialxr78x_reg = {
	.owner			= THIS_MODULE,
	.normal_name		= "ttyXR78x",
	.callout_name		= "cua",
	.normal_major		= XR_788_MAJOR,
	.callout_major		= TTYAUX_MAJOR,
	.normal_driver		= &exvar_sdriver,
	.callout_driver		= &exver_cdriver,
	.table			= exver_tty,
	.termios		= exver_termios,
	.termios_locked		= exver_termios_locked, 
	.minor			= XR_788_MINOR,
	.nr			= UART_XR788_NR,
//	.cons			= SERIALXR_CONSOLE,
	.port			= serialxr78x_ports,
};

/*
 * register_serial and unregister_serial allows for 16x50 serial ports to be
 * configured at run-time, to support PCMCIA modems.
 */

static int __register_serial(struct serial_struct *req, int line)
{
	struct uart_port port;

	port.iobase   = req->port;
	port.membase  = req->iomem_base;
	port.irq      = req->irq;
	port.uartclk  = req->baud_base * 16;
	port.fifosize = req->xmit_fifo_size;
	port.regshift = req->iomem_reg_shift;
	port.iotype   = req->io_type;
	port.flags    = req->flags | ASYNC_BOOT_AUTOCONF;
//	port.mapbase  = req->iomap_base;
	port.line     = line;
	
	if (share_irqs)
		port.flags |= ASYNC_SHARE_IRQ;

	/*
	 * to be safer, check and default to the standard clock rate.
	 */
	if (port.uartclk == 0)
		port.uartclk = 921600 * 16; // XR17x15y clock rate

	return uart_register_port(&serialxr78x_reg, &port);
}

/**
 *	register_serial - configure a xr17x15y serial port at runtime
 *	@req: request structure
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use an error is returned. If the port
 *	is not currently in the table it is added.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int register_serial(struct serial_struct *req)
{
	return __register_serial(req, -1);
}

/**
 *	unregister_serial - remove a xr17x15y serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may be called from interrupt
 *	context.
 */
void unregister_serial(int line)
{
	uart_unregister_port(&serialxr78x_reg, line);
}

MODULE_PARM(io, "1-4i");
MODULE_PARM(irq, "1-4i");
MODULE_PARM(major, "i");
static int __init serialxr78x_init(void)
{
	int ret, i;

	printk(KERN_INFO "Exar XR16L78x specific serial driver $Revision: 1.0 $ "
		"%d ports, IRQ sharing %sabled\n", (int) UART_XR788_NR,
		share_irqs ? "en" : "dis");
	/* check IRQ */

	for ( i = 0; i < UART_MAX_BOARDS; ++i )
	{
		if ( irq[i] != 0 )
			++irq_count;
		if ( io[i] != 0 )
			++io_count;
	}
	if ( irq_count == 0 || io_count == 0 )
		return -1;
	if ( irq_count != io_count )
		return -1;
	if ( irq_count > UART_MAX_BOARDS || io_count > UART_MAX_BOARDS )
		return -1;
	max_ports = UART_XR788_NR * io_count;
	printk("max_ports = %d\n",max_ports);
	serialxr78x_reg.nr = max_ports;
	
	for ( i = 0; i < irq_count; ++i)
	{
		if ( irq[i] >= 3 && irq[i] <= 7 )
			linux_irq[i] = convirq_from_isa(irq[i]);
		else if ( irq[i] == 2 )
			linux_irq[i] = convirq_from_isa(9);	/* IRQ2 = IRQ9 */
		else
			return -1;
		printk("irq[%d] = %d\n",i,linux_irq[i]);
	}
	linux_irq_count = irq_count;
	/* check io */
	for ( i = 0; i < io_count; ++i)
	{
		io[i] = io[i] - io[i]%0x100;
		if ( io[i] > 0xff00 )
			return -1;
		printk("io[%d] = %x\n",i,io[i]);
	}

	/* set major no. */
	serialxr78x_reg.normal_major = major;
	printk("MAJOR No = %d\n",serialxr78x_reg.normal_major);
	for (i = 0; i < NR_IRQS; i++)
		spin_lock_init(&irq_lists[i].lock);

	serial78x_init_ports();
	ret = uart_register_driver(&serialxr78x_reg);
	return ret;
}

static void __exit serialxr78x_exit(void)
{
//	int i;
			
//	for (i = 0; i < max_ports; i++)
//		uart_remove_one_port(&serialxr78x_reg, &serialxr78x_ports[i].port);
	uart_unregister_driver(&serialxr78x_reg);
}

module_init(serialxr78x_init);
module_exit(serialxr78x_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exar XR16L78x specific serial driver $Revision: 1.0 $");
