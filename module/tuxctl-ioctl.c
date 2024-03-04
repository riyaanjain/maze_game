/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)
/**/

//global variables
static unsigned char arr[2];	/**/
static bool spam;	//was unsigned int
static unsigned char saved_state[6];	/**/ //6 bytes of LED packet, pre-reset state

int tuxctl_init(struct tty_struct *tty);
int tuxctl_set_led(struct tty_struct *tty, unsigned long argument);
int tuxctl_buttons(struct tty_struct *tty, unsigned long argument);

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

	switch(a){
		case MTCP_BIOC_EVENT:
			arr[0]=b;
			arr[1]=c;
			break;
		case MTCP_ACK:
			spam=0;
			break;
		case MTCP_RESET:
			tuxctl_init(tty);
			tuxctl_ldisc_put(tty,saved_state,sizeof(saved_state));
			//tuxctl_set_led(tty,led);
			break;
	}

    /*printk("packet : %x %x %x\n", a, b, c); */
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int /**/
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	case TUX_INIT:
		spam=0;
		return tuxctl_init(tty);		//should ret 0
	case TUX_BUTTONS:
		return tuxctl_buttons(tty,arg);	//ret 0 or -EINVAL if pointer to arg not valid
	case TUX_SET_LED:
		if(!spam){
			spam=1;
			return tuxctl_set_led(tty,arg);	//ret 0
		}
	case TUX_LED_ACK:
		return 0;
	case TUX_LED_REQUEST:
		return 0;
	case TUX_READ_LED:
		return 0;
	default:
	    return -EINVAL;
    }
}

/**/
int tuxctl_init(struct tty_struct *tty){
	unsigned char cmds[2];
	spam=0;
	cmds[0]=MTCP_BIOC_ON;
	cmds[1]=MTCP_LED_USR;
	tuxctl_ldisc_put(tty,cmds,2);

	return 0;
}

/**/

int tuxctl_set_led(struct tty_struct *tty, unsigned long argument){
	static const unsigned char display_segments[16]={
		0xE7,0x06,0xCB,0x8F,0x2E,0xAD,0xED,0x86,
		0xEF,0xAE,0xEE,0x6D,0xE1,0x4F,0xE9,0xE8
	};

	unsigned char led_cmds[6]={MTCP_LED_SET,0x0F};	//command and all leds on
	unsigned char led_mask=(argument>>16) & 0x0F;
	unsigned char decimal_mask=(argument>>24) & 0x0F;

	unsigned int i;
	for(i=0; i<4; ++i){
		if(led_mask & (1<<i)){
			led_cmds[i+2] = display_segments[argument & 0x0F];
			if(decimal_mask & (1<<i)){
				led_cmds[i+2] = led_cmds[i+2] | 0x10;	//bit 5=1
			}
		}
		else{
			led_cmds[i+2]=0x00;
		}
		argument>>=4;
	}
	if(tuxctl_ldisc_put(tty,led_cmds,sizeof(led_cmds))==0){		//swapped
		memcpy(saved_state,led_cmds,sizeof(led_cmds));
	}
	return 0;
}

/**/
int tuxctl_buttons(struct tty_struct *tty, unsigned long argument){
	if(!argument){
		return -EINVAL;
	}
	else{
		unsigned char button_state=0;
		int ret;

		unsigned char button_input = ~arr[0] & 0x0F;		//0x0F = 0000 1111 (gives lower 4 bits) and gets CBAS
		unsigned char direction_input = ~arr[1] & 0x0F;				//right left down up RLDU

		button_state |= (direction_input & (1<<0)) ? (1<<7) : 0;	//right
		button_state |= (direction_input & (1<<2)) ? (1<<6) : 0;	//left
		button_state |= (direction_input & (1<<1)) ? (1<<5) : 0;	//down
		button_state |= (direction_input & (1<<3)) ? (1<<4) : 0;	//up
		button_state |= (button_input & (1<<0)) ? (1<<3) : 0;	//C
		button_state |= (button_input & (1<<1)) ? (1<<2) : 0;	//B
		button_state |= (button_input & (1<<2)) ? (1<<1) : 0;	//A
		button_state |= (button_input & (1<<3)) ? (1<<0) : 0;	//start

		ret = copy_to_user((unsigned char *)argument,&button_state,sizeof(button_state));
		if(ret){
			return -EINVAL;
		}
		return 0;
	}
}