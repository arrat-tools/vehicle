#!/usr/bin/env python3

from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtGui import QPixmap, QPainter
import sys, os, json, cv2, time, math, subprocess, glob, signal, shutil, re
from datetime import datetime
import rospkg, rospy
rospack = rospkg.RosPack()
import numpy as np
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import pyqtgraph as pg
import copy
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

from sensor_msgs.msg import Image, CompressedImage
from lib_sub_window import ImageWindow

CLRNET_CUTOFF = 1018
CLRNET_H = 1242
CLRNET_W = 2208
IMG_WIN_H = 800
SHOULDER = 3.0

def kill_subprocess(name):
	# name = input("Enter process Name: ")
	try:
		# iterating through each instance of the process
		for line in os.popen("ps ax | grep " + name + " | grep -v grep"): 
			fields = line.split()
			# extracting Process ID from the output
			pid = fields[0] 
			# terminating process 
			os.kill(int(pid), signal.SIGKILL) 
		print("Process Successfully terminated")
	except:
		print("Error Encountered while running script")

def img_msg_raw2img(raw, info):
    if 'jpeg' in info['type']:
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    
    elif info['type'] == 'bgr8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 3)
    
    elif info['type'] == 'rgb8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 3)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    elif info['type'] == 'rgba8':
        # img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 4)[:,:,0:3] # old method
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 4)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        
    elif info['type'] == 'bgra8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(info['height'], info['width'], 4)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    else:
        img = None
    return img

def process_cal(config_dict, shoulder=2.0):
	h, w = config_dict['h'], config_dict['w']
	um, vm = config_dict['um'], config_dict['vm']
	u1, v1, u2, v2 = config_dict['u1'], config_dict['v1'], config_dict['u2'], config_dict['v2']
	u3, v3, u4, v4 = config_dict['u3'], config_dict['v3'], config_dict['u4'], config_dict['v4']
	vscale = h/config_dict['lane_marker_span']
	lane_width = config_dict['left_offset'] - config_dict['right_offset'] 
	uscale = w/(2*shoulder+lane_width)
	um_ = uscale * shoulder
	# uscale_ = (w - 2*um_)/lane_width
	dst_u_left = w/2 - uscale*config_dict['left_offset']
	dst_u_right = w/2 - uscale*config_dict['right_offset']
	dst = np.float32([[dst_u_left, h-vm], [dst_u_right, h-vm], [dst_u_left, vm], [dst_u_right, vm]])
	src = np.float32([[u1,v1], [u2,v2], [u3,v3], [u4,v4]]) # first warp set
	M = cv2.getPerspectiveTransform(src, dst)
	return src, dst, M, um_

class MainWindow(QtWidgets.QMainWindow):

	def __init__(self, *args, **kwargs):
		super(MainWindow, self).__init__(*args, **kwargs)
		rospy.init_node('camera_calibration', anonymous=True)
		node_name = rospy.get_name()
		package_name = 'ra-daq-gui'
		pkgpath = rospack.get_path(package_name)
		uic.loadUi(os.path.join(pkgpath, 'scripts', 'cam_cal.ui'), self)

		""" Params """
		self.syncdir = os.path.join(os.path.expanduser('~'), rospy.get_param("sync_dir"))
		self.data_dir = os.path.join(os.path.expanduser('~'), rospy.get_param("data_dir"))
		self.inputs_file = os.path.join(pkgpath, rospy.get_param("audit_inputs_pkgpath"))
		self.cam_age_tol = rospy.get_param("cam_age_tol")
		self.cam_image_compressed = rospy.get_param("cam_image_compressed")
		self.cam_topic = rospy.get_param("cam_topic")
		self.cam_required = rospy.get_param("cam_required") 
		cam_launch_command = rospy.get_param("cam_launch_command")

		""" Subscribers"""
		if self.cam_image_compressed:
			rospy.Subscriber(self.cam_topic, CompressedImage, self.rec_cam)
		else:
			rospy.Subscriber(self.cam_topic, Image, self.rec_cam)

		self.cam_launch_command = cam_launch_command

		""" Cal Data """
		self.config_dict = json.load(open(self.inputs_file))['cal_data']
		self.src, self.dst, self.calMatrix, um = process_cal(self.config_dict, shoulder=SHOULDER)
		# Set user inputs from loaded file
		self.uiLeft.setText('%.2f' % self.config_dict['left_offset'])
		self.uiRight.setText('%.2f' % (-1.0*self.config_dict['right_offset']))
		self.uiSpan.setText('%.2f' % self.config_dict['lane_marker_span'])
		self.uiBumper.setText('%.2f' % self.config_dict['near_marker_long'])
		self.uiCamLong.setText('%.2f' % self.config_dict['cam_gps_long'])
		self.uiCamLat.setText('%.2f' % self.config_dict['cam_gps_lat'])

		""" Init Timer """
		self.timer = QTimer()
		self.timer.timeout.connect(self.update)
		self.timer.start(100) # ms
		rospy.on_shutdown(self.myhook)

		""" Init Variables """
		self.t0 = time.time()
		self.tLast = self.t0
		self.tNow = 0
		self.n = 0
		self.fps = 0
		self.tlast_cam = 0.0
		self.camok = False
		self.cam_on = False
		self.waiting_for_cam = False
		self.tlast_clicked_cam = 0.0
		self.firstCall = True
		self.camOpen = False
		self.cal_window = None
		self.warp = False
		self.img = None
		# self.cam_process = subprocess.Popen(
        #     ['bash', '-c', f'{self.cam_launch_command}']
        # 	) # stdout=subprocess.PIPE, stderr=subprocess.PIPE

		""" Initialize """
		self.initUI()

	def rec_cam(self, msg):
		self.tlast_cam = time.time()
		if msg._type == "sensor_msgs/Image":
			info = {}
			info['type'] = msg.encoding
			info['height'] = msg.height
			info['width'] = msg.width
		elif msg._type == "sensor_msgs/CompressedImage":
			info = {}
			info['type'] = msg.format
		else:
			info = {}
			info['type'] = ""
		raw = np.frombuffer(msg.data, dtype=np.uint8)
		img = img_msg_raw2img(raw, info)
		
		if img is not None:
			self.img = img
			if self.firstCall:
				self.imgWidth = img.shape[1]
				self.imgHeight = img.shape[0]
				self.imgU1 = 0
				self.imgV1 = 0
				self.imgU2 = self.imgWidth
				self.imgV2 = self.imgHeight
				self.firstCall = False
				self.camOpen = True

			if self.camOpen:
				img = cv2.resize(img, (self.imgLabelWidth, int(self.imgLabelWidth/self.imgWidth*self.imgHeight)))
				img = cv2.cvtColor(img[self.imgV1:self.imgV2, self.imgU1:self.imgU2], cv2.COLOR_BGR2RGB)
				img = QImage(img, img.shape[1], img.shape[0], 
						img.strides[0], QImage.Format_RGB888)
				self.imgLabel.setPixmap(QPixmap.fromImage(img))

	def myhook(self):
		# self.timer.stop()
		self.camOpen = False
		rospy.loginfo(f"[RA GUI] Closed - ending processes...")
		try:
			# Kill gps process by pid
			subprocess.run(["kill", str(self.gps_process.pid)])
			# Wait for the process to be killed
			self.gps_process.wait()
			print(f"Process with PID {self.gps_process.pid} is no longer alive. Return code: {self.gps_process.returncode}")
			self.gps_on = False
		except:
			pass
		try:
			# Kill cam process by pid
			subprocess.run(["kill", str(self.cam_process.pid)])
			# Wait for the process to be killed
			self.cam_process.wait()
			print(f"Process with PID {self.cam_process.pid} is no longer alive. Return code: {self.cam_process.returncode}")
			self.cam_on = False
		except:
			pass
		time.sleep(0.25)

	def clickedStart(self):
		pass

	def clickedStop(self):
		pass
		
	def update(self):
		""" Update internal variables """
		self.tNow = time.time() - self.t0 # update Time
		self.camok = time.time() - self.tlast_cam < self.cam_age_tol

		# execution rate
		if time.time() - self.tLast >= 1.0:
			self.fps = self.n
			self.n = 0
			self.tLast = time.time()          
		self.n = self.n + 1 
		# StatusString = "Time: "+"{:.2f}".format(self.tNow) + " s, FPS: "+"{:.0f}".format(self.fps)
		StatusString = "Version 1.0"
		self.statusBar().showMessage(StatusString) 

		""" Update Labels """
		#
		if self.camok:
			self.labelCamOk.setStyleSheet("background-color: lightgreen")
			self.labelCamOk.setText('On')
		else:
			self.labelCamOk.setStyleSheet("background-color: red") 
			self.labelCamOk.setText('Off')

		# user activity
		# self.waiting_for_cam = time.time() - self.tlast_clicked_cam < 30.0
		if self.waiting_for_cam and self.camok:
			self.btnCam.setEnabled(True)
			self.waiting_for_cam = False

		if self.cal_window is not None and self.img is not None:
			img_cal = copy.deepcopy(self.img)
			img_warp = cv2.warpPerspective(img_cal, self.calMatrix, (img_cal.shape[1], img_cal.shape[0])) 
			if self.warp:
				self.cal_window.set_img(img_warp)
			else:
				hood_v = int(CLRNET_CUTOFF/CLRNET_H*img_cal.shape[0])
				cv2.line(img_cal, (0, hood_v), (img_cal.shape[1], hood_v), (0,0,255), 2)
				self.cal_window.set_img(img_cal)

	def clickedCalibrate(self):
		if not self.firstCall:
			self.cal_window = ImageWindow(self.src, x=self.geometry().width()+100, y=self.geometry().y(), height=IMG_WIN_H, aspect=self.imgWidth/self.imgHeight)

	def clickedVerify(self):
		# self.config_dict['left_offset'] = float(self.uiLeft.text())
		# self.config_dict['right_offset'] = -float(self.uiLeft.text())
		ui_same = ['%.2f' % self.config_dict['left_offset'] == '%.2f' % float(self.uiLeft.text()),
			 '%.2f' % self.config_dict['right_offset'] == '%.2f' % (-1.0*float(self.uiRight.text())),
			 '%.2f' % self.config_dict['lane_marker_span'] == '%.2f' % float(self.uiSpan.text()),
			 '%.2f' % self.config_dict['near_marker_long'] == '%.2f' % float(self.uiBumper.text()),
			 '%.2f' % self.config_dict['cam_gps_long'] == '%.2f' % float(self.uiCamLong.text()),
			 '%.2f' % self.config_dict['cam_gps_lat'] == '%.2f' % float(self.uiCamLat.text())]
		self.warp = not self.warp
		src_same = self.src == self.cal_window.src
		if not src_same.all() or not all(ui_same):
			self.labelChange.setText('Modified')
			self.labelChange.setStyleSheet("background-color: red") 
		self.config_dict['u1'], self.config_dict['v1'] = int(self.cal_window.src[0][0]), int(self.cal_window.src[0][1])
		self.config_dict['u2'], self.config_dict['v2'] = int(self.cal_window.src[1][0]), int(self.cal_window.src[1][1])
		self.config_dict['u3'], self.config_dict['v3'] = int(self.cal_window.src[2][0]), int(self.cal_window.src[2][1])
		self.config_dict['u4'], self.config_dict['v4'] = int(self.cal_window.src[3][0]), int(self.cal_window.src[3][1])
		self.config_dict['left_offset'] = float(self.uiLeft.text())
		self.config_dict['right_offset'] = -1.0*float(self.uiRight.text())
		self.config_dict['lane_marker_span'] = float(self.uiSpan.text())
		self.config_dict['near_marker_long'] = float(self.uiBumper.text())
		self.config_dict['cam_gps_long'] = float(self.uiCamLong.text())
		self.config_dict['cam_gps_lat'] = float(self.uiCamLat.text())
		self.src, self.dst, self.calMatrix, um = process_cal(self.config_dict, shoulder=SHOULDER)
		self.config_dict['um'] = int(um)

	def clickedSave(self):
		if self.labelChange.text() == 'Modified':
			datetime_object = datetime.fromtimestamp(time.time())
			datetime_string = datetime_object.strftime("%Y-%m-%d %H:%M:%S")
			self.labelChange.setText('--')
			self.labelChange.setStyleSheet("background-color: lightgray") 
			all_config_dict = json.load(open(self.inputs_file))
			all_config_dict['cal_data'] = self.config_dict
			all_config_dict['cal_data']['timestamp'] = datetime_string
			json_object = json.dumps(all_config_dict, indent=4)
			with open(self.inputs_file, "w") as outfile:
				outfile.write(json_object)
				outfile.close()

	# def btnCamClicked(self):
	# def start_cam(self):
	# 	self.tlast_clicked_cam = time.time()
	# 	if not self.waiting_for_cam and not self.camok:
	# 		self.cam_process = subprocess.Popen(
    #         ['bash', '-c', f'{self.cam_launch_command}']
    #     	) # stdout=subprocess.PIPE, stderr=subprocess.PIPE
	# 		self.cam_on = True
	# 		self.camOpen = True
	# 		self.btnCam.setEnabled(False)
	# 		self.waiting_for_cam = True
	# 		rospy.logwarn(f'[RA GUI] Launched Camera')
	# 	else:
	# 		# Kill process by pid
	# 		subprocess.run(["kill", str(self.cam_process.pid)])
	# 		# Wait for the process to be killed
	# 		self.cam_process.wait()
	# 		print(f"Cam process with PID {self.cam_process.pid} is no longer alive. Return code: {self.cam_process.returncode}")
	# 		self.cam_on = False
	# 		self.camOpen = False

	def initUI(self):
		""" Buttons and Clicks """
		# self.btnCam.clicked.connect(self.btnCamClicked)
		self.btnCal.clicked.connect(self.clickedCalibrate) 
		self.btnVerify.clicked.connect(self.clickedVerify) 
		self.btnSave.clicked.connect(self.clickedSave) 
		
		""" Labels """
		self.labelCamOk.setText('Off')
		self.labelCamOk.setAlignment(QtCore.Qt.AlignCenter) 
		self.labelCamOk.setStyleSheet("background-color: lightgray") 

		self.labelChange.setText('--')
		self.labelChange.setAlignment(QtCore.Qt.AlignCenter) 
		self.labelChange.setStyleSheet("background-color: lightgray") 

		# self.latLabel.setStyleSheet("background-color: lightgray") 
		# self.lonLabel.setStyleSheet("background-color: lightgray") 
		# self.latLabel.setFont(QFont('Arial', 20))
		# self.lonLabel.setFont(QFont('Arial', 20))
		# self.labelLatname.setFont(QFont('Arial', 12))
		# self.labelLonname.setFont(QFont('Arial', 12))

		""" Window and Layout """
		self.setWindowTitle("Road Audit Camera Calibration")
		self.setFixedSize(self.size()) # 1174, 1128
		self.move(50, 50)
		# self.setGeometry(0, 0, 1174, 900) # 1174, 1128
		self.layout = pg.GraphicsLayout(border=(0,0,0)) # layout central widget
		# self.graphView.setFixedSize(self.graphView.width(), self.graphView.height())
		# self.graphView.setCentralItem(self.layout)

		self.imgLabelWidth = self.imgLabel.width()

	# def keyPressEvent(self, event):
	# 	print(event.key())
		# if event.key() == 16777236: # right arraw
		# 	self.update(self.index + 1)
		# if event.key() == 16777234: # left arrow
		# 	self.update(self.index - 1)

		# if event.key() == 44: # '<' key press


def main():
	app = QtWidgets.QApplication(sys.argv)
	mainwindow = MainWindow()
	mainwindow.show()
	sys.exit(app.exec_())

if __name__ == '__main__':         
	main()