#
# tw6869, Intersil|Twchwell TW6869 video driver with V4L/2 support
#
# Makefile - driver build system
#

# Requires GNU Make

# take version info from last module build if available
#KERNELRELEASE := $(shell cat $(obj)/.version 2>/dev/null || uname -r)

# Usual paths to the kernel source tree
KDIR := /lib/modules/`uname -r`/build

# Standard path to install the driver
MODULE_INSTALLDIR = /lib/modules/`uname -r`/kernel/drivers/media/video/tw686x
KDRV_INSTALLDIR = /lib/modules/`uname -r`/kernel/drivers/media/pci/tw686x

ifeq ($(shell id -u),0)
  SUDO =
  pulseaudio = su -c "pulseaudio $(1)" $(shell ps -o user -C pulseaudio | tail -1)
else
  SUDO = sudo 
  pulseaudio = pulseaudio $(1)
endif

ifneq ($(KERNELRELEASE),)   # We were called by build system.

ifeq (${audio}, n)
tw6869-objs := TW68-core.o TW68-video.o
else
tw6869-objs := TW68-core.o TW68-video.o TW68-ALSA.o
EXTRA_CFLAGS += -DAUDIO_BUILD
endif

ifeq (${jetson}, y)			# for use with jetson tk1 board 
EXTRA_CFLAGS += -DJETSON_TK1
endif

ifeq (${ntsc_op}, y)
EXTRA_CFLAGS += -DNTSC_STANDARD_NODETECT
endif

ifeq (${pal_op}, y)
EXTRA_CFLAGS += -DPAL_STANDARD_NODETECT
endif



# detect presence of include files
ifneq ($(wildcard $(CURDIR)/include/media/v4l2-common.h),)
EXTRA_CFLAGS += -DHAVE_V4L2_COMMON
endif
ifneq ($(wildcard $(CURDIR)/include/media/v4l2-dev.h),)
EXTRA_CFLAGS += -DHAVE_V4L2_DEV
ifneq ($(shell grep video_device_node_name $(CURDIR)/include/media/v4l2-dev.h),)
EXTRA_CFLAGS += -DHAVE_VIDEO_DEVICE_NODE_NAME
endif
endif
ifneq ($(wildcard $(CURDIR)/include/media/v4l2-device.h),)
EXTRA_CFLAGS += -DHAVE_V4L2_DEVICE
endif
ifneq ($(wildcard $(CURDIR)/include/media/v4l2-event.h),)
EXTRA_CFLAGS += -DHAVE_V4L2_EVENT
endif
ifneq ($(wildcard $(CURDIR)/include/media/v4l2-ioctl.h),)
EXTRA_CFLAGS += -DHAVE_V4L2_IOCTL
endif
ifneq ($(wildcard $(CURDIR)/include/media/videobuf-vmalloc.h),)
ifneq ($(shell grep ext_lock $(CURDIR)/include/media/videobuf-vmalloc.h),)
EXTRA_CFLAGS += -DHAVE_VIDEOBUF_VMALLOC_EXT_LOCK
endif
endif
ifneq ($(shell grep "gfp_t gfp" $(CURDIR)/include/linux/firmware.h),)
EXTRA_CFLAGS += -DHAVE_REQUEST_FIRMWARE_GFP
endif

obj-m += tw6869.o



EXTRA_CFLAGS += -Idrivers/media/video

else   # We were called from command line

PWD := "$(shell pwd)"

Debug: default
Release: default


audio: modules

default: modules

modules:
	@echo '**************************************************************************'
	@echo '* Building Intersil|Techwell TW6869 driver...                            *'
	@echo '* Type "make help" for a list of available targets.                      *'
	@echo '**************************************************************************'
	@echo
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	rm *.o *.ko *.mod.o
	rm -rf .tmp_versions
	rm -rf Module.symvers

endif

install: modules_install

modules_install:
	mkdir -p kernel_backup
	find /lib/modules/$(shell uname -r)/ -name 'tw6869.ko' | xargs -I{} cp --parent {} kernel_backup
	find /lib/modules/$(shell uname -r)/ -name 'tw686x.ko' | xargs -I{} cp --parent {} kernel_backup
	find /lib/modules/$(shell uname -r)/ -name 'tw686x.ko.xz' | xargs -I{} cp --parent {} kernel_backup
	find /lib/modules/$(shell uname -r)/ -name 'tw6869.ko' | sudo xargs rm -f
	find /lib/modules/$(shell uname -r)/ -name 'tw686x.ko' | sudo xargs rm -f
	find /lib/modules/$(shell uname -r)/ -name 'tw686x.ko.*' | sudo xargs rm -f
	find /lib/modules/$(shell uname -r)/ -name 'tw6869.ko' | sudo xargs rm -f
	/bin/mkdir -p $(MODULE_INSTALLDIR)
	/bin/cp ./tw6869.ko $(MODULE_INSTALLDIR)
	/bin/chmod 644 $(MODULE_INSTALLDIR)/tw6869.ko
	/sbin/depmod -ae
	@echo
	@echo '**************************************************************************'
	@echo '* Driver installed successfully. Type "make load" to load it.            *'
	@echo '**************************************************************************'
	@echo

rl:
	sudo make modules_install;sudo make unload;sudo /sbin/modprobe tw6869 TW68_debug_alsa=1


load:
	/sbin/modprobe tw6869;
	pulseaudio --start
	echo "Please note: Pulse audio may take some time to configure.\n";\
	echo "If audio device doesn't appear right away, this is why.\n" ;\

unload:
	echo "Unloading 812 module."		;\
	rmmod tw6869;\
	echo "Stopping pulseaudio."		;\
	sudo /usr/bin/killall -9 pulseaudio	;\
	sudo /usr/bin/killall -9 pulseaudio	;\
	sudo /usr/bin/killall -9 alsactl	;\
	echo "Retrying unload 812 module."	;\
	rmmod tw6869		;\
	echo "Unloaded";\
	rmmod tw686x		;

reload:
	/sbin/modprobe -r tw6869
	/sbin/modprobe tw6869

uninstall: modules_uninstall

modules_uninstall:
	/bin/rm -rf $(MODULE_INSTALLDIR)/tw6869.ko
	/bin/rm -rf $(MODULE_INSTALLDIR)
	/sbin/depmod -ae
	@echo
	@echo '**************************************************************************'
	@echo '* Driver uninstalled successfully.                                       *'
	@echo '**************************************************************************'
	@echo

help:
	@echo 'List of available targets. Type:'
	@echo '- "make modules" to build the modules'
	@echo '- "make modules_install" to install the built modules'
	@echo '- "make reload" to reload the modules'
	@echo '- "make load" to load the installed modules'
	@echo '- "make modules_uninstall" to uninstall the installed modules'
	@echo '- "make clean" to remove all generated files in the current directory'
	@echo '- "make tar" to create an source code archive of the current directory in $(TARFILE)'
	@echo '- "make help" to print the list of available targets'


RELEASE_VERSION := $(shell date +%Y%m%d)
TARFILE=../tw6869-$(RELEASE_VERSION).tar.gz
DIR=../
FILE=tw6869

tar:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	/bin/rm -rf *~
	tar -choz --verbose -C $(DIR) --file=$(TARFILE) $(FILE)
	@echo
	@echo '**************************************************************************'
	@echo '* Source package created in the parent folder ../                        *'
	@echo '**************************************************************************'
	@echo

