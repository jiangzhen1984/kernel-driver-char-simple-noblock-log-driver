#ifndef __LINUX_NBF_LOGGER_H
#define __LINUX_NBF_LOGGER_H


#include <linux/types.h>
#include <linux/ioctl.h>

#define NBF_LOGGER_IOCTL_MAGIC ('t')

#define NBF_LOGGER_IOCTL_SET_IO \
    _IOWR(NBF_LOGGER_IOCTL_MAGIC, 1, int)

#define NBF_LOGGER_IOCTL_GET_IO \
    _IOWR(NBF_LOGGER_IOCTL_MAGIC, 2, int)




#endif

