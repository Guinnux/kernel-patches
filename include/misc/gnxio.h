/*
 * gnxio.h
 *
 *  Created on: 06 Dec 2012
 *      Author: john
 */

#ifndef GNXIO_H_
#define GNXIO_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

#define GNX_IO_INPUT 0
#define GNX_IO_OUTPUT 1
#define GNX_IO_LOW 0
#define GNX_IO_HIGH 1



struct gnx_io_pin_status
{
	uint16_t pin_id;
	uint16_t pin_level;
};
struct gnx_io_pin_cfg
{
	uint16_t pin_id;
	uint8_t pin_in_out;
	uint8_t def_level;
	char pin_desc[16];
};

#define GNX_IOCTL_MAGIC 'g'

/* Reuest the number of IOs mapped in this driver */
/* ioctl takes no argument and return the either an error code or the number of pins */
#define GNX_IO_IOCTLGNIO _IO(GNX_IOCTL_MAGIC,1)

/* Request the IO Map */
#define GNX_IO_IOCTLGMAP _IOR(GNX_IOCTL_MAGIC,2,int)



#endif /* GNXIO_H_ */
