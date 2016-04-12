/*
 * 3080.c
 * HT3080 Sample program (read thread)
 *		2011/03/22 UMEZAWA MUSEN DENKI CO.,LTD.
 *	
 */
/*******************************************************************************
 * inlucde files.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <sys/ipc.h>
#include <sys/sem.h>

#include "3080.h"
#include "xr1678x.h"

/*******************************************************************************
 * define
 ******************************************************************************/
#define HT3080 "/dev/ttyXR78x%d"
#define BUFF_SIZE	1024*8

/*******************************************************************************
 * global variable
 ******************************************************************************/
char buff[MAX_TTY][BUFF_SIZE];
int fd_ht3080[MAX_TTY];		/* file descriptor(HT3080) */
int read_loop = 1;

char *ht3080_rdpnt[MAX_TTY],*ht3080_wrpnt[MAX_TTY];
int  ht3080_chars_in[MAX_TTY];
int  ht3080_str_in[MAX_TTY];
int  ht3080_port = 0;
int rs485_mode = HT_SET_RS485_HALF;

/*******************************************************************************
 * function
 ******************************************************************************/
void ht3080_init(void);

int ht3080_open(void);
void ht3080_close(void);

int ht3080_read_str(int com, char *str, int size);
int ht3080_read_char(int com);

int ht3080_char_avail(int com);
char ht3080_getchar(int com);
void ht3080_set_ioctl(int baud);
void ht3080_set_rs485_mode(int mode);
void ht3080_set_local_echo(void);

/*******************************************************************************
 * read HT3080(8ports)
 ******************************************************************************/
void ht3080_main(void)
{
	int i;
	int ret = 0;

	if ( ht3080_open() == 0 )
	{
		return;
	}
	ht3080_set_ioctl(B9600);
	ht3080_init();
	ht3080_set_rs485_mode(rs485_mode);
	ht3080_set_local_echo();

	while(read_loop)
	{
		for ( i = 0; i < MAX_TTY; ++i )
		{
			ht3080_read_char(i);
		}
	}

}
/*******************************************************************************
 * open HT3080(8ports)
 ******************************************************************************/
void ht3080_init(void)
{
	int i;

	/* buffer initialization */
	for ( i = 0; i < MAX_TTY; ++i )
	{
		ht3080_rdpnt[i] = buff[i];
		ht3080_wrpnt[i] = buff[i];
		ht3080_chars_in[i] = 0;
		ht3080_str_in[i] = 0;
	}

}
/*******************************************************************************
 * finish HT3080 read theread
 ******************************************************************************/
void ht3080_end_thread(void)
{
	ht3080_close();
	read_loop = 0;
}

/*******************************************************************************
 * open HT3080 8ports
 ******************************************************************************/
int ht3080_open(void)
{
	char tty[32];
	int i = 0;
	int ret = 0;

	for ( i = 0; i < MAX_TTY; ++i )
	{
		memset(tty,'\0',32);
		sprintf(tty,HT3080,i);
		fd_ht3080[i] = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK);   /* デバイスをオープンする */
		if ( fd_ht3080[i] < 0 )
		{
			printf("open err tty%d\r\n",i);
			fd_ht3080[i] = 0;
			continue;
		}
		ret = 1;
	}
	return ret;
}
/*******************************************************************************
 * close HT3080 8ports
 ******************************************************************************/
void ht3080_close(void)
{
	int i;


	for ( i = 0; i < MAX_TTY; ++i )
	{
		close(fd_ht3080[i]);
		fd_ht3080[i] = 0;
	}
}
/*******************************************************************************
 * Initialization
 ******************************************************************************/
void ht3080_set_ioctl(int baud)
{
	struct termios newtio;
	struct termios oldtio;
	int ret = 0; 
	unsigned int m = TIOCM_RTS; 
	int i;
    int modem = 0;

	for ( i = 0; i < MAX_TTY; ++i )
	{
		if ( fd_ht3080[i] == 0 )
		{
			continue;
		}
		ioctl(fd_ht3080[i], TCGETS, &oldtio);					/* save current settings */
		newtio = oldtio;
		newtio.c_cflag = baud  | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;										/* non canonical */

		ret = ioctl(fd_ht3080[i], TCSETS, &newtio);				/* set new settings */
		if ( ret < 0 )
		{
			printf("error set ioctl fd=%d\r\n",i);
			fd_ht3080[i] = 0;
		}
	}

}

/*******************************************************************************
 * send a character
 ******************************************************************************/
void ht3080_write_char(int com, char c)
{
	write(fd_ht3080[com],&c,1);
}

/*******************************************************************************
 * get characters
 ******************************************************************************/
int ht3080_read_char(int com)
{
	char c;
	char tmp[BUFF_SIZE];
	char *ptr_tmp = tmp;
	int size = 0;

	size = read(fd_ht3080[com], ptr_tmp, BUFF_SIZE);

	while(size > 0)
	{
		c = *ptr_tmp;
		++ptr_tmp;
		--size;

		*ht3080_wrpnt[com] = c;
		ht3080_wrpnt[com]++;
		if (ht3080_wrpnt[com] >= (buff[com] + BUFF_SIZE)){
			ht3080_wrpnt[com] = buff[com];
		}
		ht3080_chars_in[com]++;
		if('\r' == c || '\n' == c)
		{
			ht3080_str_in[com]++;
		}
	}
	return 1;
}
/*******************************************************************************
 * get received character length
 ******************************************************************************/
int ht3080_char_avail(int com)
{	int i;
	i = ht3080_chars_in[com];
	return i;
}

/*******************************************************************************
 * get received line count
 ******************************************************************************/
int ht3080_str_avail(int com)
{	int i;
	i = ht3080_str_in[com];
	return i;
}
/*******************************************************************************
 * get a character from buffer
 ******************************************************************************/
char ht3080_getchar(int com)
{
	char c;
	c = *ht3080_rdpnt[com];
	ht3080_rdpnt[com]++;
	if (ht3080_rdpnt[com] >= (buff[com]+BUFF_SIZE)){
		ht3080_rdpnt[com] = buff[com];
	}
	ht3080_chars_in[com]--;
	return c;
}

/*******************************************************************************
 *	get a line from the specified port
 *
 *	use ht3080_str_avail() before calling this function to check the
 *	line is available.
 *
 *	returns received line length
 ******************************************************************************/
int ht3080_get_str(int com, char *text)
{
	int count = 0;
	char rdata;

	while(ht3080_char_avail(com)) {
		rdata = ht3080_getchar(com);
		if('\r' == rdata || '\n' == rdata){						/* CR or LF */
			if ( ht3080_str_in[com] )
				ht3080_str_in[com]--;							/* decrement the line counter */
			break;
		}else if(rdata){
			*(text+count) = rdata;								/* save the character */
			count++;
		}
		if(128 < count){										/* line buffer overflow */
			break;
		}
	}

	*(text+count) = '\0';
	return(count);
}
/*******************************************************************************
 * set RS485 mode(0:Half/1:Full) 
 ******************************************************************************/
void ht3080_set_rs485_mode(int mode)
{
	int i = 0;
	for ( i = 0; i < MAX_TTY; ++i )
	{
		ioctl(fd_ht3080[i], HT_SET_AUTO_RS485, HT_SET_AUTO_RS485_ENABLE);
		ioctl(fd_ht3080[i], HT_SET_RS485_MODE, mode);			
	}
}
/*******************************************************************************
 * local echo off 
 ******************************************************************************/
void ht3080_set_local_echo(void)
{
	int i = 0;
	for ( i = 0; i < MAX_TTY; ++i )
		ioctl(fd_ht3080[i], HT_SET_LOCAL_ECHO, HT_SET_LOCAL_ECHO_OFF);			
}
void set_rs485_mode(int mode)
{
	rs485_mode = mode;
}
