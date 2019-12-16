ls
cd Downloads/
ls
cd sdk_812_1.1.18_linux/
ls
./driver.py
ls
cd driver/
ls
make
sudo make install
sudo make load
make load
vi install.txt
ifconfig
LS
ls
gstreamer
gst-launch-1.0 nvcamerasrc ! 'video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12' ! nvvidconv flip-method=2 ! nvegltransform ! nveglglessink -e
v4l2-ctl --list-devices
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
uname
uname -a
sudo apt install terminator
sudo apt install git-all
sudo apt install vim
gst-launch-1.0 
gst-launch-1.0 v4l2src device="/dev/video1" ! "video/x-raw, width=640, height=480, format=(string)I420" ! xvimagesink -e
sudo su
PATH
path
echo $PATH
sl
sudo apt install sl
sl
sudo apt install fortune
exit
path
vi .bashrc
vi .bash_aliases 
vi .bashrc
vi /usr/share/bash-completion/bash_completion
exit
path
exit
path
echo $PATH
vi /usr/share/bash-completion/bash_completion
sudo vi /usr/share/bash-completion/bash_completion
ls /usr/share/doc/bash/examples
ls /usr/share/doc/bash
cd /usr/share/doc/bash
less README.md.bash_completion.gz 
ls
ls -l
cd ../bash-completion/
ls
less changelog.Debian.gz 
cd ../
ls
grep startup *
find . | xargs grep startup
find . -type f | xargs grep startup
vi /etc/profile
vi /etc/profile.d/bash_completion.sh 
exit
cd /etc/profile.d/
ls
grep PATH *
ssh lavi.local -l robil
exit
path
bash
exit
gst-launch-1.0 v4l2src device=/dev/video0 ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! videoparse height=576 width=704 format=UYVY framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 
man v4l2-ctl
v4l2-ctl
v4l2-ctl
v4l2-dbg
v4l2-dbg -D --verbose
v4l2-dbg -D 
v4l2-dbg -D -d=/dev/video0
v4l2-dbg -D --device=/dev/video0
v4l2-dbg -D --device=/dev/video0 --log-status
v4l2-dbg -D --device=/dev/video2 --log-status
ls
./capture-test.sh 
clear
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
export GST_DEBUG=4
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
export GST_DEBUG=3
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src ! jpegdec ! xvimagesink
gst-launch-1.0 v4l2src device=/dev/video0 ! jpegdec ! xvimagesink
gst-launch-1.0 v4l2src device=/dev/video1 ! jpegdec ! xvimagesink
ls
./capture -d /dev/video1 -f -c 100 
./capture -d /dev/video0 -f -c 100 
./capture -d /dev/video1 -f -c 100 -o > input.yuv
./capture -d /dev/video0 -f -c 100 -o > input.yuv
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
export GST_V4L2_USE_LIBV4L2=1
gst-launch-1.0 v4l2src ! xvimagesink
vi ~/.bash_aliases 
h
export GST_DEBUG=3
export GST_DEBUG=2
gst-launch-1.0 v4l2src ! xvimagesink
export GST_DEBUG=1
gst-launch-1.0 v4l2src ! xvimagesink
gst-launch-1.0 v4l2src device=/dev/video0 ! xvimagesink
gst-launch-1.0 v4l2src ! jpegdec ! xvimagesink
gst-launch-1.0 v4l2src ! xvimagesink
gst-launch-1.0 v4l2src 704x576 ! xvimagesink
gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-raw, width=640, height=480, format=(string)UYVY" ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-raw, width=704, height=576, format=(string)UYVY" ! queue ! videoconvert ! xvimagesink sync=false
ls
la
cd ~/Downloads/sdk_812_1.1.18_linux/
mkdir others
cp ~/Documents/install.txt others/
cd others/
ls
vi log-activities.txt
mv sdk_812_1.1.18_linux sdk_812_bad
ls
cd sdk_812_bad
git fcsk --full
git fsck --full
git reset --hard
git reset --keep
git fsck --full
git gc --auto
git log
git prune --exire now
git prune --expire now
git reflog --all
cd ../
rm -rf sdk_812_bad/ 
sudo rm -rf sdk_812_bad/ 
rm -rf sdk_backup/
ls
cd sdk_812_1.1.18_linux/
ls
./driver.py 
find . -name "*.ko"
cd driver
cd vbuf2/
make
lsmod
rmmod
rmmod tw6869
ls
rmmod tw6869.ko
sudo reboot now
gst-launch-1.0 --gst-debug=v4l2src:5 v4l2src device="/dev/video0" ! fakesink 2>&1 | sed -une '/caps of src/ s/[:;] /\n/gp'
mpv av://v4l2:/dev/video0 tv:///0
vlc v4l2:///dev/video0
dmesg
vlc v4l2:///dev/video0
mpv av://v4l2:/dev/video0 tv:///0
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY, interlace-mode=interlace ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY, interlace-mode=interleave ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 -v v4l2src device=/dev/video0 ! video/x-raw,format=UYVY,interlace-mode=interleaved ! videoconvert ! xvimagesink
cd Downloads/sdk_812_1.1.18_linux/
ls
git gui
git fcsk --help
git fsck --help
man 7 git-fsck
git log
cd ../
cp -r sdk_812_1.1.18_linux/ sdk_backup
cd
ls Downloads/sdk_backup/
la Downloads/sdk_backup/
cd Downloads/sdk_812_1.1.18_linux/
git reset --hard
cd ../
cd sdk_812_1.1.18_linux/
ls
git remote -v
cd ../
git clone https://github.com/mhallak/sdk_812_1.1.18_linux.git 
cd sdk_812_1.1.18_linux/
ls
gitk
cd ../sdk_backup/
ls
cd others/
ls
cp * ../../sdk_812_1.1.18_linux/others/
cd ../../sdk_812_1.1.18_linux/
git gui
git push origin master
gitk
mpv av://v4l2:/dev/video0 tv:///0
sensoray-testing/capture -d /dev/video1 -f -c 100 -o > input.yuv
sensoray-testing/capture -d /dev/video0 -f -c 100 -o > input.yuv
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
sensoray-testing/capture -d /dev/video0 -f -c 500 -o > input.yuv
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
sudo apt install vlc
vlc --demux rawvideo --rawvid-fps 25 --rawvid-width 704 --rawvid-height 576 --rawvid-chroma UYVY input.yuv  
h
gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-raw, width=704, height=576, format=(string)UYVY" ! queue ! videoconvert ! xvimagesink sync=false
vi ~/.bash_aliases 
export GST_DEBUG=0
gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-raw, width=704, height=576, format=(string)UYVY" ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device="/dev/video0" ! video/x-raw, width=704, height=576, format=(string)UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device="/dev/video0" ! video/x-raw, width=704, height=576, format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=uyvy ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576 ! queue ! videoconvert ! xvimagesink sync=false
h
gst-launch-1.0 filesrc location=./input.yuv ! videoparse height=576 width=704 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576 format=uyvy framerate=1/25 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576 format=uyvy ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576 format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=uyvy ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY 
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=YUY2 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=RGB16 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=RGB15 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=UYVY, framerate=25/1 ! queue ! videoconvert ! xvimagesink sync=false
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, width=704, height=576, format=YUY2, framerate=25/1 ! queue ! videoconvert ! xvimagesink sync=false
dmesg
tailf var/log/syslog
tail -f var/log/syslog
tail -f /var/log/syslog
grep 32 *
grep 28 *
cd Videos/
gst-launch-1.0 videomixer name=mixer sink_1::alpha=0.5 sink_1::xpos=50 sink_1::ypos=50 ! videoconvert ! ximagesink videotestsrc pattern=snow timestamp-offset=3000000000 ! "video/x-raw,format=AYUV,framerate=(fraction)30/1,width=640,height=480" !   timeoverlay ! queue2 ! mixer. videotestsrc pattern=smpte !  "video/x-raw,format=AYUV,framerate=(fraction)10/1,width=800,height=600" ! queue2 ! mixer.
nvgstplayer-1.0 --help
head -n 1 /etc/nv_tegra_release
which nvgstplayer
ls -l /usr/bin/nvgst*
h
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! queue ! h264parse ! nvv4l2decoder ! nv3dsink -e
systemctl stop gdm
loginctl terminate-seat seat0
h
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink -e
h
which gst-launch-1.0 
cd Videos/
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! queue ! h264parse ! nvv4l2decoder ! nv3drmvideosink -e
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink -e
h
bash
cd Videos/
gst-launch-1.0 filesrc location=pedestrian.mp4 ! qtdemux ! h264parse ! nv4l2decoder ! nvdrmvideosink -e
gst-launch-1.0 filesrc location=pedestrian.mp4 ! qtdemux ! h264parse ! nvv4l2decoder ! nvdrmvideosink -e
ls
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! h264parse ! nvv4l2decoder ! nvdrmvideosink -e
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! h264parse ! nvv4l2decoder ! nvdrmvideosink conn_id=0 plane_id=1 set_mode=0 -e
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! h264parse ! nvv4l2decoder ! nv3dsink -e
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! queue ! h264parse ! nvv4l2decoder ! nv3dsink -e
reboot
bash
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink -e
cd Videos/
cd Downloads/sdk_812_1.1.18_linux/
ls
ls others
cd demo-sample/
cd PyGTK/
ls
./demoSX12.py 
cd
cd Videos/
gst-launch-1.0 filesrc location=jellyfish-400-mbps-4k-uhd-hevc-10bit.mkv ! matroskademux ! queue ! h265parse ! nvv4l2decoder ! nvvidconv ! 'video/x-raw(memory:NVMM), format=(string)NV12' ! nv3dsink -e
gst-launch-1.0 videomixer name=mixer sink_1::alpha=0.5 sink_1::xpos=50 sink_1::ypos=50 ! videoconvert ! ximagesink videotestsrc pattern=snow timestamp-offset=3000000000 ! "video/x-raw,format=AYUV,framerate=(fraction)30/1,width=640,height=480" !   timeoverlay ! queue2 ! mixer. videotestsrc pattern=smpte !  "video/x-raw,format=AYUV,framerate=(fraction)10/1,width=800,height=600" ! queue2 ! mixer.
ls
gst-launch-1.0 filesrc location=jellyfish-400-mbps-4k-uhd-hevc-10bit.mkv ! matroskademux ! queue ! h265parse ! nvv4l2decoder ! nvvidconv ! 'video/x-raw(memory:NVMM), format=(string)NV12' ! nv3dsink -e
ls /
cd /boot
ls
ls Image 
sudo cp Image Image.backup
ls
ls I*
ll I*
env | grep args
env | grep boot
vi extlinux/extlinux.conf 
sudo vi extlinux/extlinux.conf 
dmesg
tail -f /var/log/syslog
ls /dev/v*
./demoSX12.py 
cd
cd /boot/extlinux/
ls
ls -la
rm .extlinux.conf.swp 
sudo vi extlinux.conf 
rm .extlinux.conf.swp 
sudo rm .extlinux.conf.swp 
sudo vi extlinux.conf 
reboot
h | grep clone
exit
h | grep clone
bash
exit
env | grep DIS
g gst-launch or nvgstplayer.
export DISPLAY=:0
env | grep DIS
cd Videos/
ls
ls *.mp4
gst-launch-1.0 filesrc location=cars.mp4 name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
gst-launch-1.0 filesrc location=cars.mp4 ! name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
gst-launch-1.0 filesrc location=cars.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
ls *.mp4
gst-launch-1.0 filesrc location=ChID-BLITS-EBU-Narration.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
ls *.mp4
gst-launch-1.0 filesrc location=pedestrians.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
ls *.mp4
gst-launch-1.0 filesrc location=parking_sfm.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
gst-launch-1.0 filesrc location=ChID-BLITS-EBU-Narration.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_aac ! audioconvert ! alsasink -e
gst-launch-1.0 filesrc location=ChID-BLITS-EBU-Narration.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_amrwb ! audioconvert ! alsasink -e
gst-launch-1.0 filesrc location=ChID-BLITS-EBU-Narration.mp4 ! qtdemux name=demux demux.audio_0 ! queue ! avdec_amrnb ! audioconvert ! alsasink -e
find / -name "*.mp3" 2>/dev/null
mv ~/Downloads/Vivaldi_Sonata_eminor_.mp3 ./
gst-launch-1.0 filesrc location=Vivaldi_Sonata_eminor_.mp3 ! mpegaudioparse ! avdec_mp3 ! audioconvert ! alsasink -e
h
gst-launch-1.0 -v fakesrc silent=false num-buffers=3 ! fakesink silent=false
env | grep GST
a
san
env | grep DIS
export DISPLAY:=1.0
export DISPLAY=:1.0
san
vi scripts/camera_sanity.sh 
san
nvidia-smi
nvidia-detector 
lspci
sha1sum -c /etc/nv_tegra_release
glxinfo | egrep -i '(version|nvidia)'
ls /usr/local/cuda-10.0/lib64/lib*
nvidia-settings
ls /usr/local/cuda-10.0/lib64/lib* | grep gl
ls /usr/local/cuda-10.0/lib64/lib* | grep -i gl
find / -type f -name "*[Gg][Ll]*.so.*"
find / -type f -name "*[Gg][Ll]*.so.*" 2>/dev/null
exit
cat /proc/quadd/version 
cat Documents/install.txt 
cat /proc/quadd/version 
sudo apt install netcat-openbsd
sudo apt autoremove
ip addr show
a
alias
bash
h
exit
san
cd Videos/
gst-launch-1.0 nvcompositor name=comp sink_0::xpos=0 sink_0::ypos=0 sink_0::width=1920 sink_0::height=1080 sink_1::xpos=0 sink_1::ypos=0 sink_1::width=1600 sink_1::height=1024 sink_2::xpos=0  sink_2::ypos=0 sink_2::width=1366 sink_2::height=768 sink_3::xpos=0 sink_3::ypos=0 sink_3::width=1024 sink_3::height=576 ! nvoverlaysink display-id=1 filesrc location=TextInMotion-VideoSample-1080p_h264.mp4 ! qtdemux ! h264parse ! omxh264dec ! comp. filesrc location=sample_1080p_h265.mp4 ! qtdemux ! h265parse ! omxh265dec ! comp. filesrc  location= tears_of_steel_1080p_vp8.webm matroskademux ! omxvp8dec ! comp. filesrc location=tears_of_steel_1080p_vp9.webm ! matroskademux ! omxvp9dec ! comp. -e
gst-launch-1.0 nvcompositor name=comp sink_0::xpos=0 sink_0::ypos=0 sink_0::width=1920 sink_0::height=1080 ! autovideoconvert ! nvoverlaysink sync=0 videotestsrc ! comp.
exit
