#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "s812ioctl.h"

int main(int argc, char **argv)
{
    int fd;
    int i;
    int rc;
    struct s812_reg reg;
    fd = open("/dev/video0", O_RDWR  | O_NONBLOCK, 0);
    if (fd == -1) {
        printf("failed to open\n");
        return -1;
    }
    reg.type = 0;
    reg.val = 0;
    rc = ioctl(fd, S812_VIDIOC_CLEAR_REG, &reg);
    if (rc != 0) {
        printf("failed to read register %x %d\n", rc, rc);
    }
    close(fd);
    return 0;
}
