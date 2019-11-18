/*************************************************************************************
*                                                                                    *
*   This simple demo program is designated for demonstrating use of the              *
*   Sensoray model 812 GPIO.                                                         *
*                                                                                    *
*   Revision:                                                                        *
*                                                                                    *
*      December, 2013 Initial, for Sensoray Model 812                                *
*                                                                                    *
*   The COPYRIGHT is covered by the GNU General Public License.                      *
*                                                                                    *
*																					 *
* 812 GPIO Register Map																 *
* 																					 *
* --------------------------------- ctrl 31...24 ----------------------------------- *
* --    31         30        29        28        27        26        25         24   *
* --  ctrl31     ctrl30    ctrl29    ctrl28    ctrl27    ctrl26    ctrl25     ctrl24 *
* ---------------------------------------------------------------------------------- *
* --------------------------------- ctrl 23...16 ----------------------------------- *
* --    23         22        21        20        19        18        17         16   *
* --  ctrl23     ctrl22    ctrl21    ctrl20    ctrl19    ctrl18    ctrl17     ctrl16 *
* ---------------------------------------------------------------------------------- *
* --------------------------------- gpio 15...8 ------------------------------------ *
* --    15         14        13        12        11        10         9         8    *
* --   int31      int30     int29     int28     int27     int26     int25     int24  *
* ---------------------------------------------------------------------------------- *
* --------------------------------- gpio 7...0 ------------------------------------- *
* --     7          6         5         4         3         2         1         0    *
* --   int23      int22     int21     int20     int19     int18     int17     int16  *
* ---------------------------------------------------------------------------------- *
* 																					 *
* ctrl - I/O control of gpio pin													 *
* 	0 : pin is output																 *
* 	1 : pin is input																 *
* 																					 *
* Note: Mapping of 812 GPIO[8:1] to internal gpio is dependent on SW1   			 *
* (see hardware manual).  By default 812 gpio[8:1] is mapped to internal			 *
* gpio[23:16].																		 *
* 																					 *
*************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

#include "../driver/s812ioctl.h"

#define GET  0
#define SET  1

void usage(void)
{
    printf("Usage:\nTo get GPIO register val: 'gpio -g'\nTo set GPIO register val: 'gpio -s <val in hex>'\n");
}

int main(int argc, char *argv[])
{
    int fd, cmd;
    int val = 0;
    struct v4l2_control control;

    if (argc < 2)
    {
        usage();
        exit(EXIT_FAILURE);
    }
        
    if ( strcmp(argv[1], "-g") == 0 || strcmp(argv[1], "-G") == 0 )
    {        
        cmd = GET;
    }
    else if ( strcmp(argv[1], "-s") == 0 || strcmp(argv[1], "-S") == 0)
    {
        cmd = SET;
        if ( argc < 3 )
        {
            usage();
            exit(EXIT_FAILURE);
        }
        sscanf(argv[2], "%x", &val); 
        printf("\n\n%x\n\n", val);
    }    
    
    memset (&control, 0, sizeof (control));
    control.id = S812_CID_GPIO;
    fd = open("/dev/video0", O_RDWR);
    if (fd < 0) {
        printf("\nError opening 812\n");
        exit(0);
    }

    switch(cmd)
    {
        case GET:
            if (0 == ioctl (fd, VIDIOC_G_CTRL, &control))
            {
                printf("\nGPIO: 0x%x\n\n", control.value);
            }
            else
            {
                printf("\nFailed to read GPIO\n\n");
            }
            break;
        case SET:
            control.value = val;
            if (0 == ioctl (fd, VIDIOC_S_CTRL, &control))
            {
                printf("\nsuccess\n");            
            }
            else
            {
                printf("Failed to write to GPIO");
            }
            break;
        default:
            usage();        
    }          
                      
    close(fd);

    return 0;
}

