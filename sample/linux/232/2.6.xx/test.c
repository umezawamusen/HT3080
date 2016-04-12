/*
 * test.c
 * HT3080 Send/Receive Sample
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

/*******************************************************************************
 * define
 ******************************************************************************/

/*******************************************************************************
 * global variable
 ******************************************************************************/
pthread_t thread_read_ht3080;

/*******************************************************************************
 * function
 ******************************************************************************/
int check_parameter(int argc, char *argv[]);
void print_help(void);

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

	printf("HT3080 Send/Receive sample\r\n");

	/* create HT3080 receive thread */
	ret = pthread_create(&thread_read_ht3080, NULL, 
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
				ret = ht3080_get_str(i,str);
				printf("Received CH%d:%s\n",i+1,str);
				for ( j = ret; j >= 0; --j)
					ht3080_write_char(i,str[j]);
				ht3080_write_char(i,'\r');
				ht3080_write_char(i,'\n');
				if (strcmp(strupr(str),"STOP")==0) goto END;
			}

		}
	}
END:
	ht3080_end_theread();
}

