/*********************************************************************************
*                                                                                *
*   This simple demo program is designated for demonstrating launching all       *
*   8 channel previews from Sensoray Model 812, using XawTV and in one program.  *
*   It can be used for Sensoray Model 811/911 4-channel previews as well,        *
*   with minor code change, such as revising 8-channel to 4-channel.             *
*                                                                                *
*   Revision:                                                                    *
*                                                                                *
*      June, 2012   Charlie X. Liu    Initial, for Sensoray Model 812 and SX11   *
*                                                                                *
*   The COPYRIGHT is covered by the GNU General Public License.                  *
*                                                                                *
*********************************************************************************/

#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

int main ()
{
  pid_t child_pid;

  //printf ("the main program process id is %d\n", (int) getpid ());
  child_pid = fork ();
  //printf ("the child's process id is %d\n", (int) child_pid);

  if (child_pid != 0) {
    //printf ("this is the parent process, with id %d\n", (int) getpid ());

	// Preview all 8 video channels, using XawTV:
    system ("xawtv -c /dev/video0 &");
    printf ("launched: xawtv -c /dev/video0 & \n");
    system ("xawtv -c /dev/video1 &");
    printf ("launched: xawtv -c /dev/video1 & \n");
    system ("xawtv -c /dev/video2 &");
    printf ("launched: xawtv -c /dev/video2 & \n");
    system ("xawtv -c /dev/video3 &");
    printf ("launched: xawtv -c /dev/video3 & \n");
    system ("xawtv -c /dev/video4 &");
    printf ("launched: xawtv -c /dev/video4 & \n");
    system ("xawtv -c /dev/video5 &");
    printf ("launched: xawtv -c /dev/video5 & \n");
    system ("xawtv -c /dev/video6 &");
    printf ("launched: xawtv -c /dev/video6 & \n");
    system ("xawtv -c /dev/video7 &");
    printf ("launched: xawtv -c /dev/video7 & \n\n");

  } else {
    //printf ("this is the child process, with id %d\n", (int) getpid ());

	// Listening audio Channel-1, for one hour:
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:1,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:1,0 | aplay & \n");
# if 0
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:2,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:2,0 | aplay & \n");
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:3,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:3,0 | aplay & \n");
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:4,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:4,0 | aplay & \n");
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:5,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:5,0 | aplay & \n");
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:6,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:6,0 | aplay & \n");
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:7,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:7,0 | aplay & \n");
# endif
	// Listening audio Channel-8, for one hour:
    system ("arecord -d 3600 -f s16_le -r 32000 -D hw:8,0 | aplay &");
    printf ("launched: arecord -f s16_le -r 32000 -D hw:8,0 | aplay & \n\n");

  }

  return 0;

}

