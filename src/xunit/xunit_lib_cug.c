#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>      
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#include "xunit_lib.h"

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableMasterMode								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera master mode			    *
  **********************************************************************************************************
*/

BOOL EnableMasterMode()
{
	int ret =0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLEMASTERMODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableTriggerMode								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera trigger mode			    *
  **********************************************************************************************************
*/

BOOL EnableTriggerMode()
{
	int ret =0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLETRIGGERMODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	return TRUE;
}


