/*
 * test.c
 * HT3080 RS485 Receive Sample
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
#include <pthread.h>
#include "3080.h"
#include "xr1678x.h"

/*******************************************************************************
 * define
 ******************************************************************************/
/*******************************************************************************
 * global variable
 ******************************************************************************/
char buff[128];
pthread_t thrid_read_ht3080;

/*******************************************************************************
 * function
 ******************************************************************************/
int cmd_proc(void);
void send_str(void);
void print_help(void);
int check_parameter(int argc, char *argv[]);

/*******************************************************************************
 * strupr
 ******************************************************************************/
char *strupr (char *s)
{
	char *ret = s;

	while (*s != '\0') {
		if (islower (*s)) *s = toupper(*s);
		++s;
	}
	return ret;
}
/*******************************************************************************
 * main
 ******************************************************************************/
int main(int argc, char *argv[])
{
	int i,j;
	int ret = 0;
	char str[256];
	char c;
	int flag = 0;

	printf("HT3080 RS485 Receive sample\r\n");

	/* check parameter */
	if ( check_parameter(argc, argv) == 0 )
	{
		print_help();
		return 0;
	}
	
	/* create HT3080 receive thread */
	ret = pthread_create(&thrid_read_ht3080, NULL, 
				   (void * (*)(void *))ht3080_main, 0);
	if (0 != ret) {	/* error */
		printf("ERROR: Cound not create HT3080 read thread\n");
		return 0;
	}

	while(1)
	{

		for ( i = 0; i < MAX_TTY; ++i )
		{
			while(ht3080_str_avail(i))
			{
				char tmp[32];
				memset(tmp, '\0', 32);
				memset(str, '\0', 256);
				ret = ht3080_get_str(i,str);
				printf("Received CH%d:%s\n",i+1,str);
				if (strcmp(strupr(str),"STOP")==0) goto END;
			}
		}
	}
END:
	ht3080_end_thread();
}
int check_parameter(int argc, char *argv[])
{
	int i;
	char str[32];
	char *tmp_str = str;

	if ( argc < 0 || argc > 3 )
		return 0;											/* parameter error */	
	if ( argc == 1 )
	{
		set_rs485_mode(HT_SET_RS485_HALF);
		return 1;
	}

	strcpy(tmp_str, argv[1]);

	if ( *tmp_str == '-' )
		++tmp_str;
	else
		return 0;

	if ( strcmp(strupr(tmp_str), "HALF") == 0 )
	{
		set_rs485_mode(HT_SET_RS485_HALF);
	}
	else if ( strcmp(strupr(tmp_str), "FULL") == 0 )
	{
		set_rs485_mode(HT_SET_RS485_FULL);
	}
	else
		return 0;

	return 1;
}

void print_help(void)
{
	printf("Usage : -Option\r\n");
	printf("Options\r\n");
	printf("no option         Auto RS485 Half duplex test\r\n");
	printf("-half             Auto RS485 Half duplex test\r\n");
	printf("-full             Auto RS485 Full duplex test\r\n");
}

