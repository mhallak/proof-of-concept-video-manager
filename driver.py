#!/usr/bin/python

# 812 Driver Install Helper
# Copyright (c) 2017 Sensoray Co., Inc.


import pygtk
pygtk.require('2.0')
import gtk, os, ctypes, glob, threading, subprocess, time, select, string, sys, socket
import platform
import os
from fcntl import *
from mmap import *


debug = False
for arg in sys.argv:
	if arg == "-d":
		debug = True


def dprint(*args):
	if debug:
		for x in args:
			print x,
		print

def make_menu_item(name, callback, data=None, value=None):
	item = gtk.MenuItem(name)
	item.value = value
	item.connect("activate", callback, data)
	item.show()
	return item

def format_k(scale, value):
	if value >= 1000000:
		return "%.0d,%03.0dK" % (value / 1000000, (value / 1000) % 1000)
	if value >= 100000:
		return "%.0dK" % (value / 1000)
	return "%d" % value

class Demo:
	def delete_event(self, widget, event, data=None):
		return False

	def destroy(self, widget, data=None):
		gtk.main_quit()
		if self.vd != None:
			self.vd.close()

	def set_dev(self, widget, data):
		if self.vd != None:
			self.vd.close()
		try: self.vd = open(data, "rw")
		except: self.vd = None
		
		# clear the window and reenumerate the widgets
		self.window.remove(self.window.get_child())
		self.window.add(self.setup())

	def set_dim(self, widget, data):
		self.width = data[0]
		self.height = data[1]
		#dprint("set_dim", self.width, self.height)
		format = v4l2_format()
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		try: ioctl(self.vd, VIDIOC_G_FMT, format)
		except IOError: return
		format.fmt.pix.width = self.width
		format.fmt.pix.height = self.height
		format.fmt.pix.bytesperline = 0
		dprint("new dim is", self.width, self.height)
		try: 
			ioctl(self.vd, VIDIOC_S_FMT, format)
		except IOError:
			print "Unable to set format"

	def set_format(self, widget, data=None):
		format = v4l2_format()
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		try: ioctl(self.vd, VIDIOC_G_FMT, format)
		except IOError: return
		format.fmt.pix.pixelformat = data
		format.fmt.pix.field = V4L2_FIELD_ANY
		format.fmt.pix.bytesperline = 0
		dprint("new format is", data)
		try: 
			ioctl(self.vd, VIDIOC_S_FMT, format)
			self.outfmt = data
		except IOError:
			print "Unable to set format"
		# clear the window and reenumerate the widgets
		self.window.remove(self.window.get_child())
		self.window.add(self.setup())

	def set_input(self, widget, data=None):
		dprint("new input is", data)
		input = ctypes.c_int(data)
		self.input = int(data)
		try: ioctl(self.vd, VIDIOC_S_INPUT, input)
		except IOError: return

	def set_output(self, widget, data=None):
		dprint("new output is", data)
		output = ctypes.c_int(data)
		self.output = int(data)
		try: ioctl(self.vd, VIDIOC_S_OUTPUT, output)
		except IOError: return

	def set_audio_input(self, widget, data=None):
		dprint("new audio input is", data)
		audio = v4l2_audio()
		try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
		except IOError: return
		audio.index = data
		try: ioctl(self.vd, VIDIOC_S_AUDIO, audio)
		except IOError: return

	def set_audio_mode(self, widget, data=None):
		dprint("new audio mode is", data)
		audio = v4l2_audio()
		try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
		except IOError: return
		audio.mode = data
		try: ioctl(self.vd, VIDIOC_S_AUDIO, audio)
		except IOError: return

	def set_audio_output(self, widget, data=None):
		dprint("new audio input is", data)
		audio = v4l2_audioout()
		audio.index = data
		try: ioctl(self.vd, VIDIOC_S_AUDOUT, audio)
		except IOError: return

	def set_frame_interval(self, widget, index):
                dprint("test")
	def set_std(self, widget, data=None):
	        dprint("test")
	def set_ctrl(self, widget, id):
                dprint("test")
	def get_ctrl(self, cid):
                dprint("test")
	def set_str_ctrl(self, widget, id, entry=None):
                dprint("test")
	
	def set_toggle(self, widget, id):
		widget.value = widget.get_active()
		self.set_ctrl(widget, id)
		
	def set_audmode(self, widget, id):
		audio = v4l2_audio()
		audio.index = 0
		try: ioctl(self.vd, VIDIOC_G_AUDIO, audio)
		except IOError: return
		if widget.get_active():
			audio.mode |= V4L2_AUDMODE_AVL
		else:
			audio.mode &= ~V4L2_AUDMODE_AVL
		try: ioctl(self.vd, VIDIOC_S_AUDIO, audio)
		except IOError: print "Unable to set audio mode"


	def set_jpegcomp(self, widget, adj):
		jpegcomp = v4l2_jpegcompression()
		jpegcomp.quality = int(widget.value)
		dprint("new jpeg quality is", jpegcomp.quality)
		try: ioctl(self.vd, VIDIOC_S_JPEGCOMP, jpegcomp)
		except IOError:
			print "unable to set jpeg quality"
			try: 
				ioctl(self.vd, VIDIOC_G_JPEGCOMP, jpegcomp)
				adj.value = jpegcomp.quality
			except IOError:
				print "unable to get jpeg quality"
	def set_crop(self, widget, val):
		crop = v4l2_crop()
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
		crop.c.left = val[0]
		crop.c.top = val[1]
		crop.c.width = val[2]
		crop.c.height = val[3]
		try: ioctl(self.vd, VIDIOC_S_CROP, crop)
		except IOError:
			print "unable to set crop"
		
	def message_box(self, msg):
		msg = gtk.MessageDialog(self.window,
			gtk.DIALOG_MODAL,
			gtk.MESSAGE_INFO,
			gtk.BUTTONS_CLOSE,
			msg)
		msg.show()
		msg.run()
		msg.destroy()
		
	def message_box_yes_no(self, msg):
		msg = gtk.MessageDialog(self.window,
			gtk.DIALOG_MODAL,
			gtk.MESSAGE_INFO,
			gtk.BUTTONS_YES_NO,
			msg)
		msg.show()
		result = msg.run()
		msg.destroy()
		return result == gtk.RESPONSE_YES
		
	
	def is_device_busy(self):
		for thread in self.threads:
			if thread.devname == self.vd.name:
				if thread.is_alive():
					self.message_box("The device is busy. ("+thread.action+")")
					return True
				self.threads.remove(thread)
				break
		return False
	
	def take_snapshot(self, widget):
		if self.is_device_busy():
			return

		# Generate a unique filename
		i = 0
		while True:
			name = "out_%d.jpg" % i
			try: os.stat(name)
			except: break
			i += 1
		thread = CaptureThread(self.vd.name, name, V4L2_PIX_FMT_JPEG, -1, self.width, self.height, self.norm, self.input, True, self)
		self.threads.append(thread)
		thread.start()

	def launch_mplayer(self, widget):
		if self.is_device_busy():
			return
		if self.outfmt == V4L2_PIX_FMT_MP4V \
		or self.outfmt == V4L2_PIX_FMT_MPEG4 \
		or self.outfmt == V4L2_PIX_FMT_MP2V \
		or self.outfmt == V4L2_PIX_FMT_MPEG2 \
		or self.outfmt == V4L2_PIX_FMT_H264 \
		or self.outfmt == V4L2_PIX_FMT_MP42 \
		or self.outfmt == V4L2_PIX_FMT_MPEG:
			if not self.message_box_yes_no("The format %s is not recommended for preview. Preview anyway?" % (self.outfmt_name)):
				return
		thread = MplayerThread(self.vd.name, self.outfmt, self.width, self.height, self.norm, self.input, self)
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Previewing")
		return

	def get_videnc(self):
		ctrl = v4l2_ext_control()
		ctrl.id = V4L2_CID_MPEG_VIDEO_ENCODING
		ctrls = v4l2_ext_controls()
		ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl.id)
		ctrls.count = 1
		ctrls.controls = ctypes.pointer(ctrl)
		try: 
			ioctl(self.vd, VIDIOC_G_EXT_CTRLS, ctrls)
		except IOError: print "unable to get ext control"
		return ctrl.value

	def launch_capture(self, widget):
		if self.is_device_busy():
			return
		name = "output"
		videnc = self.get_videnc()
		if self.outfmt == V4L2_PIX_FMT_NV12:
			name += ".nv12"
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			name += ".yuyv"
		elif self.outfmt == V4L2_PIX_FMT_UYVY:
			name += ".uyvy"
		elif self.outfmt == V4L2_PIX_FMT_GREY:
			name += ".grey"
		elif self.outfmt == V4L2_PIX_FMT_MP4V or self.outfmt == V4L2_PIX_FMT_MPEG4:
			name += ".m4v"
		elif self.outfmt == V4L2_PIX_FMT_H264:
			name += ".h264"
		elif self.outfmt == V4L2_PIX_FMT_MPEG:
			name += ".ts"
		elif self.outfmt == V4L2_PIX_FMT_MP2V or self.outfmt == V4L2_PIX_FMT_MPEG2:
			name += ".m2v"
		elif self.outfmt == V4L2_PIX_FMT_MJPEG or self.outfmt == V4L2_PIX_FMT_JPEG:
			name += ".mjpg"
		elif self.outfmt == V4L2_PIX_FMT_MP42:
			name += ".mp4"
		else:
			name += ".dat"
		save = gtk.FileChooserDialog("Save Capture As...", self.window, gtk.FILE_CHOOSER_ACTION_SAVE, (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT, gtk.STOCK_SAVE, gtk.RESPONSE_ACCEPT))
		save.set_default_response(1)
		save.set_do_overwrite_confirmation(True)
		save.set_current_name(name)
		if self.default_folder:
			save.set_current_folder(self.default_folder)
		save.show()
		if save.run() != gtk.RESPONSE_ACCEPT:
			save.destroy()
			return
		self.default_folder = save.get_current_folder()
		thread = CaptureThread(self.vd.name, save.get_filename(), self.outfmt, videnc, self.width, self.height, self.norm, self.input, False, self)
		save.destroy()
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Capturing")
		return

	def launch_stream(self, widget):
		if self.is_device_busy():
			return
		videnc = self.get_videnc()
		dialog = gtk.Dialog("UDP Stream...", self.window, gtk.DIALOG_MODAL | gtk.DIALOG_DESTROY_WITH_PARENT, (gtk.STOCK_CANCEL, gtk.RESPONSE_REJECT, gtk.STOCK_OK, gtk.RESPONSE_ACCEPT))
		label = gtk.Label("Address")
		label.show()
		dialog.vbox.pack_start(label)
		ip = gtk.Entry()
		if self.default_stream_ip:
			ip.set_text(self.default_stream_ip)
		else:
			ip.set_text('127.0.0.1')
		ip.show()
		dialog.vbox.pack_start(ip)
		label = gtk.Label("UDP Port")
		label.show()
		dialog.vbox.pack_start(label)
		port = gtk.Entry()
		if self.default_stream_port:
			port.set_text(self.default_stream_port)
		else:
			port.set_text('1234')
		port.show()
		dialog.vbox.pack_start(port)
		dialog.show()
		if dialog.run() != gtk.RESPONSE_ACCEPT:
			dialog.destroy()
			return
		self.default_stream_ip = ip.get_text()
		self.default_stream_port = port.get_text()
		thread = CaptureThread(self.vd.name, '-', self.outfmt, videnc, self.width, self.height, self.norm, self.input, False, self, ip.get_text(), port.get_text())
		dialog.destroy()
		self.threads.append(thread)
		self.status.set_text("Status: Streaming")
		thread.start()
		return

	def launch_capture_avi(self, widget):
		if self.is_device_busy():
			return
		name = "output"
		if self.outfmt == V4L2_PIX_FMT_NV12:
			name += "_nv12.avi"
		elif self.outfmt == V4L2_PIX_FMT_YUYV:
			name += "_yuyv.avi"
		elif self.outfmt == V4L2_PIX_FMT_UYVY:
			name += "_uyvy.avi"
		elif self.outfmt == V4L2_PIX_FMT_GREY:
			name += "_grey.avi"
		elif self.outfmt == V4L2_PIX_FMT_MP4V or self.outfmt == V4L2_PIX_FMT_MPEG4:
			name += "_mpeg4.avi"
		elif self.outfmt == V4L2_PIX_FMT_MP2V or self.outfmt == V4L2_PIX_FMT_MPEG2:
			name += "_mpeg2.avi"
		elif self.outfmt == V4L2_PIX_FMT_H264:
			name += "_h264.avi"
		elif self.outfmt == V4L2_PIX_FMT_MJPEG or self.outfmt == V4L2_PIX_FMT_JPEG:
			name += "_mjpeg.avi"
		elif self.outfmt == V4L2_PIX_FMT_MP42:
			name += ".avi"
			if not self.message_box_yes_no("The MP4 fragmented format is not recommended for capturing to AVI. Capture anyway?"):
				return
		else:
			name += ".avi"
		save = gtk.FileChooserDialog("Save Capture As...", self.window, gtk.FILE_CHOOSER_ACTION_SAVE, ("Cancel", 0, "Save", 1))
		save.set_default_response(1)
		save.set_do_overwrite_confirmation(True)
		save.set_current_name(name)
		save.show()
		if save.run() != 1:
			save.destroy()
			return
		thread = ffmpegThread(self.vd.name, save.get_filename(), self.outfmt, self.width, self.height, self.norm, self.input, self)
		save.destroy()
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Capturing")
		return

	def launch_playback(self, widget):
		if self.is_device_busy():
			return
		load = gtk.FileChooserDialog("Choose a file to playback:", self.window, gtk.FILE_CHOOSER_ACTION_OPEN, ("Cancel", 0, "Open", 1))
		filter = gtk.FileFilter()
		filter.add_pattern("*.h264")
		filter.add_pattern("*.264")
		filter.add_pattern("*.m4v")
		filter.add_pattern("*.m2v")
		filter.add_pattern("*.mpeg4")
		filter.add_pattern("*.mp4")
		filter.add_pattern("*.mp4f")
		filter.add_pattern("*.ts")
		#filter.add_pattern("*.mjpg")
		#filter.add_pattern("*.mjpeg")
		load.set_filter(filter)
		if self.default_folder:
			load.set_current_folder(self.default_folder)
		load.show()
		if load.run() != 1:
			load.destroy()
			return
		self.default_folder = load.get_current_folder()
		thread = PlaybackThread(self.vd.name, self.norm, load.get_filename(), self)
		load.destroy()
		self.threads.append(thread)
		thread.start()
		self.status.set_text("Status: Playing")
		return
		

	def stop_thread(self, widget):
		for thread in self.threads:
			if thread.devname == self.vd.name:
				if thread.is_alive():
					thread.stop()
				else:
					self.threads.remove(thread)
				break
		self.status.set_text("Status: Stopped")

	def stop_all_threads(self):
		for thread in self.threads:
			if (thread.is_alive()):
				thread.stop()
	def get_status_string(self):
		for thread in self.threads:
			if thread.devname == self.vd.name:
				if (thread.is_alive()):
					return "Status: "+thread.action
		return "Status: Stopped"
	def update_status(self, name, msg):
		if name == self.vd.name:
			self.status.set_text(msg)
		
	def __init__(self):
		self.threads = []
		self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
		self.window.connect("delete_event", self.delete_event)
		self.window.connect("destroy", self.destroy)
		self.window.set_border_width(10)
		self.window.set_title("Sensoray 812 Driver Check")
		self.default_stream_ip = None
		self.default_stream_port = None
		self.default_folder = os.getcwd()

		# open the device
		self.vd = None
		matches = glob.glob("/dev/video[0-9]*")
		if matches:
			matches.sort()
		for dev in matches:
			try:
				self.vd = open(dev, "rw")
				break
			except:
				continue
		
		self.window.add(self.setup())
		
		self.window.show()

		
	def setup(self):
		vbox = gtk.VBox(False, 10)
		
		
		# get the driver name
		
		hbox = gtk.HBox(False, 10)
		label = gtk.Label("Kernel Version:")
		hbox.pack_start(label, False, False, 0)
		label.show()
                
                lbl2string = platform.uname()[2];
                label = gtk.Label(lbl2string)
                hbox.pack_start(label, False, False, 0)
		label.show()

		vbox.pack_start(hbox, False, False, 0)
		hbox.show()

                sdir = "/lib/modules/" + lbl2string
                dprint(sdir)
                lbl2string = "Install driver in driver or driver/vbuf2"
                for root, dirs, files in os.walk(sdir):
                        for file in files:
                                if file.startswith("tw686"):
                                # Michele if file.startswith("tw686x."):
                                        dprint(os.path.join(root, file))
                                        lbl2string = "TW68 kernel driver exists. See readme"

                dprint(lbl2string)

                #next line
                
		hbox = gtk.HBox(False, 10)                


                label = gtk.Label(lbl2string)
                hbox.pack_start(label, False, False, 0)
		label.show()

		vbox.pack_start(hbox, False, False, 0)
		hbox.show()

		quitbox = gtk.HBox(False, 10)

		button = gtk.Button("Quit")
		button.connect_object("clicked", gtk.Widget.destroy, self.window)
		button.show()
		quitbox.pack_start(button, False, False, 0)
		
		vbox.pack_start(quitbox, False, True, 0)
		quitbox.show()
		
		vbox.show()
		return vbox

	def main(self):
		gtk.gdk.threads_init()
		gtk.main()
		self.stop_all_threads()

if __name__ == "__main__":
	demo = Demo()
	demo.main()
