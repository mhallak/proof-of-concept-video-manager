/*
 * 812 ioctl header
 * Copyright (c) 2013, Sensoray Co., Inc.
 */

#ifndef S812_IOCTL_H
#define S812_IOCTL_H

#include <linux/videodev2.h>

#define S812_CID_BASE			(V4L2_CTRL_CLASS_USER | 0x1000)
#define S812_CID_GPIO	                (S812_CID_BASE + 1)

// for S812_VIDIOC_READ_REG
struct s812_reg {
	int type;  // 0 == DMAERRs, other types TBD
	int val;   // value of register
};


#define S812_VIDIOC_READ_REG (BASE_VIDIOC_PRIVATE + 0)

// if a DMA error occurs, the register will hold the value
// until it is cleared by the command below
// clearing the register does not reset the board.  It only clears
// the internal DMA error tracking register.
#define S812_VIDIOC_CLEAR_REG (BASE_VIDIOC_PRIVATE + 1)

#endif /* S812_IOCTL_H */
