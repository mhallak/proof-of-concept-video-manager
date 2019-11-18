/********************************************************************************
*                                                                               *
*   This simple demo program is designated for demonstrating A/V (Video+Audio)  *
*   capturing & recording from 2 pairs of A/V channels on Sensoray Model 812    *
*   and/or Model 811/911, using MEncoder.                                       *
*                                                                               *
*   Revision:                                                                   *
*                                                                               *
*      July, 2012   Charlie X. Liu    Initial, for Sensoray Model 812 and SX11  *
*                                                                               *
*   The COPYRIGHT is covered by the GNU General Public License.                 *
*                                                                               *
********************************************************************************/

#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

//#define DURATION 60	// in seconds
//#define DURATION 3600	// 1 hour
#define DURATION 600	// 10 minutes


int main ()
{
  pid_t main_pid;
  pid_t parent_pid;
  pid_t child_pid;
  int i;

  main_pid = getpid ();
  printf ("the main program process id is %d\n", (int) main_pid );
  child_pid = fork ();
  printf ("the child's process id is %d\n", (int) child_pid );

  if (child_pid != 0) {
    parent_pid = getpid ();
    printf ("this is the parent process, with id %d\n", (int) parent_pid );

    // A/V capturing & recording from channel-1:
	system ("mencoder tv:// -tv driver=v4l2:device=/dev/video0:norm=ntsc:forceaudio:audiorate=32000:alsa:adevice=hw.1,0:amode=1 -ovc raw -oac pcm -o av-raw-byMEncoder-ch1.avi &" );
    //printf ("launched: mencoder tv:// -tv driver=v4l2:device=/dev/video0:norm=ntsc:forceaudio:audiorate=32000:alsa:adevice=hw.1,0:amode=1 -ovc raw -oac pcm -o av-raw-byMEncoder-ch1.avi \n");

    // A/V capturing & recording from channel-2:
    system ("mencoder tv:// -tv driver=v4l2:device=/dev/video1:norm=ntsc:forceaudio:audiorate=32000:alsa:adevice=hw.2,0:amode=1 -ovc raw -oac pcm -o av-raw-byMEncoder-ch2.avi &" );
    //printf ("launched: mencoder tv:// -tv driver=v4l2:device=/dev/video1:norm=ntsc:forceaudio:audiorate=32000:alsa:adevice=hw.2,0:amode=1 -ovc raw -oac pcm -o av-raw-byMEncoder-ch2.avi & \n");

  } else {
    printf ("this is the child process, with id %d\n", (int) getpid ());

    // preview other video channels:
    system ("vlc v4l2:///dev/video2 &");
    //printf ("launched: vlc v4l2:///dev/video2 & \n");
#if 0
    system ("vlc v4l2:///dev/video3 &");
    //printf ("launched: vlc v4l2:///dev/video3 & \n");
    system ("vlc v4l2:///dev/video4 &");
    //printf ("launched: vlc v4l2:///dev/video4 & \n");
    system ("vlc v4l2:///dev/video5 &");
    //printf ("launched: vlc v4l2:///dev/video5 & \n");
    system ("vlc v4l2:///dev/video6 &");
    //printf ("launched: vlc v4l2:///dev/video6 & \n");
    system ("vlc v4l2:///dev/video7 &");
    //printf ("launched: vlc v4l2:///dev/video7 & \n");
# endif

  }

  for (i = 0; i < DURATION; i++) {
    fprintf(stderr, "."); fflush(stderr);
    sleep(1);

  } 

  kill (parent_pid, SIGTERM);
  kill (child_pid, SIGTERM);

  return 0;

}
