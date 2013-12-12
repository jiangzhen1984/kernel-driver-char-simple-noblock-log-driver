
#include <stdio.h>
#include <errno.h>
#include "../nbflogger.h"


int main(int argc, char ** argv)
{
    int fd;
    int val;

    fd = open("/dev/nbflogger",0); 

    if (fd < 0) {
        printf("errorno  %d  \n", errno);
        return -1;
    }

    printf("===%d\n",  NBF_LOGGER_IOCTL_GET_IO);

    ioctl(fd, NBF_LOGGER_IOCTL_GET_IO, &val); 
    ioctl(fd, NBF_LOGGER_IOCTL_SET_IO, val); 
    
    close(fd);
}
