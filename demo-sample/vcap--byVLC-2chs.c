/*********************************************************************************
*                                                                                *
*   This simple demo program is designated for demonstrating video               *
*   capturing & recording from 2 video channels on Sensoray Model 812            *
*   and/or Model 811/911, using VLC and with MPEG-4 transcoding.                 *
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

    // video capturing & recording from channel-1:
    system ("cvlc v4l2:///dev/video0:standard=ntsc:width=840:height=480 --sout \"#transcode{vcodec=mpeg4,vb=3000}:standard{access=file,dst=./video-byVLC-ch1-mp4.avi}\" &" );
    //printf ("launched: vlc v4l2:///dev/video0 & \n");
    // video capturing & recording from channel-2:
    system ("cvlc v4l2:///dev/video1:standard=ntsc:width=840:height=480 --sout \"#transcode{vcodec=mpeg4,vb=3000}:standard{access=file,dst=./video-byVLC-ch2-mp4.avi}\" &" );
    //printf ("launched: vlc v4l2:///dev/video1 & \n");

  } else {
    printf ("this is the child process, with id %d\n", (int) getpid ());

    // preview other video channels:
    system ("vlc v4l2:///dev/video2 &");
    //printf ("launched: vlc v4l2:///dev/video2 & \n");
# if 0
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
