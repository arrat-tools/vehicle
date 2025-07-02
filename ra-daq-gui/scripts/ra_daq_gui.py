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
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

from sensor_msgs.msg import NavSatFix, Image, CompressedImage
from standardize_units import make_standard_unit_data

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

# def get_pid_by_name(process_name):
#     try:
#         output = subprocess.check_output(["pidof", process_name])
#         return int(output.strip())
#     except subprocess.CalledProcessError:
#         return None

def fix_route_characters(input_string):
	# replace all special characters
	# desired_route_name = re.sub(r"[^a-zA-Z0-9\s]", "", desired_route_name).upper().strip()
	invalid_chars = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '{', '}', '[', ']', ',', '.', '/', '~', '`', '|']
	output_string = input_string.replace(" ","")
	output_string = output_string.replace("-","_")
	for c_ in invalid_chars:
		output_string = output_string.replace(c_,"")
	return output_string.upper().strip()

class NamedPopen(subprocess.Popen):
     """
     Like subprocess.Popen, but returns an object with a .name member
     """
     def __init__(self, *args, name=None, **kwargs):
         self.name = name
         super().__init__(*args, **kwargs)

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

def get_next_unit_dir(data_dir):
	unit_nums = []  
	for unit_dir in glob.glob(data_dir + '/unit*'):
		unit_nums.append(int(unit_dir.split('unit')[1]))
	if len(unit_nums) > 0:
		next_unit_num = max(unit_nums) + 1
	else:
		next_unit_num = 1  
	next_unit_dir = os.path.join(os.path.join(data_dir, 'unit'+str(next_unit_num)))           
	os.mkdir(next_unit_dir)
	rospy.loginfo(f"['RA GUI']New unit directory created: {next_unit_dir}")
	return next_unit_dir, next_unit_num

def topic_list_from_file(topics_file_path, use_depth, depth_topic):
	# topics_list = ['/gps/fix', '/image/compressed']
	topics_file = open(topics_file_path)
	content = topics_file.read()
	topics_file.close()
	topics_list = content.split("\n")
	topics_list = [i for i in topics_list if i]
	if not use_depth:
		index_depth_topic = topics_list.index(depth_topic)
		topics_list.pop(index_depth_topic)
	return topics_list

def make_record_command(record_command_list, topics_list): # record_command_list[3] = self.unit_dir+"/raw.bag"
	record_command_list.extend(topics_list)
	record_command_list.append("__name:=recbag")
	return record_command_list

class MainWindow(QtWidgets.QMainWindow):

	def __init__(self, *args, **kwargs):
		super(MainWindow, self).__init__(*args, **kwargs)
		rospy.init_node('road_audit_gui', anonymous=True)
		node_name = rospy.get_name()
		# pkgpath = os.path.join(os.path.expanduser('~'), 'tutorial_ws/src/ra-daq-gui')
		package_name = 'ra-daq-gui'
		pkgpath = rospack.get_path(package_name)
		uic.loadUi(os.path.join(pkgpath, 'scripts', 'ra_daq_gui.ui'), self)

		""" Params """
		self.extract_images, self.create_hood_blank = True, False
		
		# data_dir = 'data/i70'
		# max_bag_size = 200
		# topics_file_pkgpath = 'config/record_topics.txt'
		# imgdirname = 'images'
		# depthdirname = 'depths'
		# gps_age_tol = 1.0
		# cam_age_tol = 1.0
		# cam_image_compressed = True
		# gps_topic = '/gps/fix' # '/ublox/fix'
		# cam_topic = 'lanecam' # '/cam_usb/compressed'
		# cam_launch_command = 'roslaunch cam_usb cam.launch'
		# gps_launch_command = 'roslaunch am_ublox ublox.launch'
		
		self.syncdir = os.path.join(os.path.expanduser('~'), rospy.get_param("sync_dir"))
		self.data_dir = os.path.join(os.path.expanduser('~'), rospy.get_param("data_dir"))
		self.max_bag_size = rospy.get_param("max_bag_size")
		self.topics_file_path = os.path.join(pkgpath, rospy.get_param("topics_file_pkgpath"))
		self.audit_inputs_pkgpath = os.path.join(pkgpath, rospy.get_param("audit_inputs_pkgpath"))
		self.imgdirname = rospy.get_param("imgdirname")
		self.gps_filename = rospy.get_param("gpsfilename")
		self.depthdirname = rospy.get_param("depthdirname")
		self.gps_age_tol = rospy.get_param("gps_age_tol")
		self.cam_age_tol = rospy.get_param("cam_age_tol")
		self.cam_image_compressed = rospy.get_param("cam_image_compressed")
		self.gps_topic = rospy.get_param("gps_topic")
		self.cam_topic = rospy.get_param("cam_topic")
		self.depth_topic = rospy.get_param("depth_topic")
		self.cam_required = rospy.get_param("cam_required") 
		self.gps_required = rospy.get_param("gps_required") 
		gps_launch_command = rospy.get_param("gps_launch_command")
		cam_launch_command = rospy.get_param("cam_launch_command")
		if self.depthdirname != '':
			self.depth_dirname = None
		self.next_unit_num = 0

		""" Subscribers"""
		rospy.Subscriber(self.gps_topic, NavSatFix, self.rec_gps_fix)
		if self.cam_image_compressed:
			rospy.Subscriber(self.cam_topic, CompressedImage, self.rec_cam)
		else:
			rospy.Subscriber(self.cam_topic, Image, self.rec_cam)

		# Recording commands
		self.record_command_list = ["rosbag", "record",  "-O", "filepath", "--split", "--size="+str(self.max_bag_size)]

		# Launch commands
		# self.gps_launch_command = ["roslaunch", "am_ublox",  "ublox.launch"]
		#self.cam_launch_command = ["source", "~/trial_ws/devel/setup.bash", "&&", "roslaunch", "zed_tools",  "zed2.launch"]
		# self.cam_launch_command = ["source ~/.bashrc &&", "roslaunch", "zed_tools",  "zed2.launch"]
		# self.cam_launch_command = 'roslaunch zed_tools zed2.launch'
		# self.cam_launch_command = ["roslaunch", "cam_usb",  "cam.launch"]
		self.gps_launch_command = gps_launch_command
		self.cam_launch_command = cam_launch_command

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
		self.latitude, self.longitude = 0.0, 0.0
		self.tlast_gps = 0.0
		self.gps_update_interval = 0.1
		self.tlast_cam = 0.0
		self.gpsok, self.camok = False, False
		self.gps_on = False
		self.cam_on = False
		self.waiting_for_cam = False
		self.waiting_for_gps = False
		self.tlast_clicked_cam = 0.0
		self.daq_on = False
		self.firstCall = True
		self.camOpen = False
		self.show_plots = False
		self.plotTime = np.linspace(-1, 0, num=200)
		self.plotRed = 0*self.plotTime
		self.plotGreen = 0*self.plotTime
		self.plotBlue = 0*self.plotTime

		# vehicle and system id
		inputs_file = self.inputs_file = os.path.join(pkgpath, rospy.get_param("audit_inputs_pkgpath"))
		inputs_dict = json.load(open(inputs_file))
		self.veh_id, self.sys_id = inputs_dict['veh_id'], inputs_dict['sys_id']
		self.software_version = '1.0'

		""" Initialize """
		self.initUI()
		if self.show_plots:
			self.initChart()
		# self.initCam()

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
			if self.firstCall:
				self.imgWidth = img.shape[1]
				self.imgHeight = img.shape[0]
				self.imgU1 = 0
				self.imgV1 = 0
				self.imgU2 = self.imgWidth
				self.imgV2 = self.imgHeight
				self.firstCall = False
			if self.camOpen:
				img = cv2.resize(img, (self.imgLabelWidth, int(self.imgLabelWidth/self.imgWidth*self.imgHeight)))
				if self.show_plots:
					# RGB content
					meanB = np.mean(img[self.imgU1:self.imgU2,self.imgV1:self.imgV2,0])
					meanG = np.mean(img[self.imgU1:self.imgU2,self.imgV1:self.imgV2,1])
					meanR = np.mean(img[self.imgU1:self.imgU2,self.imgV1:self.imgV2,2])
					self.plotTime[:-1] = self.plotTime[1:]
					self.plotTime[-1] = self.tNow
					self.plotRed[:-1] = self.plotRed[1:]
					self.plotRed[-1] = meanR
					self.plotGreen[:-1] = self.plotGreen[1:]
					self.plotGreen[-1] = meanG
					self.plotBlue[:-1] = self.plotBlue[1:]
					self.plotBlue[-1] = meanB
					self.plotData()
				img = cv2.cvtColor(img[self.imgV1:self.imgV2, self.imgU1:self.imgU2], cv2.COLOR_BGR2RGB)
				img = QImage(img, img.shape[1], img.shape[0], 
						img.strides[0], QImage.Format_RGB888)
				self.imgLabel.setPixmap(QPixmap.fromImage(img))

	def plotData(self):
		self.p1.plot1.setData(self.plotTime, self.plotRed)
		self.p1.plot2.setData(self.plotTime, self.plotGreen)
		self.p1.plot3.setData(self.plotTime, self.plotBlue)

	def rec_gps_fix(self, msg):
		if time.time() - self.tlast_gps >= self.gps_update_interval:
			self.latitude, self.longitude = msg.latitude, msg.longitude
			self.tlast_gps = time.time()
			try:
				self.gpsLabel.setText('%.5f' % self.latitude + ', ' + '%.5f' % self.longitude)
				# self.lonLabel.setText('%.5f' % self.longitude)
			except:
				pass

	def prepare_daq_start(self):
		if not all([self.camok if self.cam_required else True, self.gpsok if self.gps_required else True]):
		# if not self.camok or not self.gpsok:
			rospy.logerr(f'[RA GUI] DAQ cannot be started without required sensors')
		else:
			# if self.route_name is not None and self.route_tracking_ok and self.route_valid:
			# desired_route_name = self.comboRoutes.currentText()
			desired_route_name = self.uiRoute.text()
			desired_direction = self.uiDirection.text()
			desired_mile_start = self.uiMileStart.text()
			desired_mile_end = self.uiMileEnd.text()
			desired_lane = self.uiLane.text()
			use_depth = self.checkDepth.isChecked()
			if desired_route_name != "":
				desired_route_name = fix_route_characters(desired_route_name)
				self.uiRoute.setText(desired_route_name)
				route_data_dir = os.path.join(self.data_dir, desired_route_name)
				all_route_dirs = []
				for route_dir in glob.glob(self.data_dir + '/*'):
					all_route_dirs.append(os.path.basename(route_dir))
				if not desired_route_name in all_route_dirs:
					os.mkdir(route_data_dir) # make route data dir
				# make next unit dir
				self.unit_dir, self.next_unit_num = get_next_unit_dir(route_data_dir)
				self.unitnumLabel.setText('%.0f' % self.next_unit_num  + ' Units')
				# make unit metadata and write to json
				datetime_object = datetime.fromtimestamp(time.time())
				datetime_string = datetime_object.strftime("%Y-%m-%d %H:%M:%S")
				unit_metadata_dict = {'unit_datetime': datetime_string, 
						  'route':desired_route_name, 
						  'direction':desired_direction, 
						  'lane':desired_lane, 
						  'start_mile_marker':desired_mile_start, 
						  'end_mile_marker':desired_mile_end, 
						  'end_mile_marker':desired_mile_end, 
						  'depth_map':use_depth}
				json_object = json.dumps(unit_metadata_dict, indent=4)
				with open(os.path.join(self.unit_dir, 'unit_metadata.json'), "w") as outfile:
					outfile.write(json_object)
				outfile.close()
				rospy.loginfo(f'[RA GUI] New route directory created at {self.unit_dir}')
				# add topics to record command list
				topics_list = topic_list_from_file(self.topics_file_path, use_depth, self.depth_topic)
				# make recording command
				record_command_list = make_record_command(self.record_command_list, topics_list)
				record_command_list[3] = self.unit_dir+"/raw.bag" # replace bag path in record command list
				subprocess.Popen(record_command_list)
				self.daq_on = True
			else:
				# route_data_dir = self.data_dir # temp route is the root data dir
				rospy.logerr(f'[RA GUI] Route name cannot be empty string')

	# def clickedClear(self):
	# 	desired_route_name = self.uiRoute.text()
	# 	if desired_route_name != "":
	# 		route_data_dir = os.path.join(self.data_dir, desired_route_name)
	# 		if os.path.exists(route_data_dir):
	# 			rospy.logwarn(f'[RA GUI] Deleting route data')
	# 			shutil.rmtree(route_data_dir)
	# 			rospy.loginfo(f'[RA GUI] Route data deleted')
	# 		else:
	# 			rospy.logerr(f'[RA GUI] No route data directory to delete')
	# 	else:
	# 		rospy.logerr(f'[RA GUI] Route name cannot be empty string')

	def clickedPreprocess(self):
		if not any([self.daq_on, self.camok, self.gpsok, self.gps_on, self.cam_on]):
			desired_route_name = self.uiRoute.text()
			if desired_route_name != "":
				rospy.logwarn(f'[RA GUI] Preprocess raw route data to make standard unit data')
				route_data_dir = os.path.join(self.data_dir, desired_route_name)
				# self.btnPreprocess.setEnabled(False)
				# self.btnPreprocess.setText("Processing raw data ....")
				make_standard_unit_data(route_data_dir, self.syncdir, self.audit_inputs_pkgpath, 
							self.gps_topic, self.cam_topic, self.depth_topic,
							self.gps_filename, self.imgdirname, self.depthdirname, 
							extract_images=self.extract_images, create_hood_blank=self.create_hood_blank)
				# self.btnPreprocess.setEnabled(True)
				# self.btnPreprocess.setText("Make Units")
			else:
				rospy.logerr(f'[RA GUI] Route name cannot be empty string')
		else:
			rospy.logerr(f'[RA GUI] Please turn off sensors and data acquisition tasks before preprocessing data')

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
		if self.daq_on:
			rospy.logwarn(f'[RA GUI] DAQ is already running')
		else:
			self.prepare_daq_start()

	def clickedStop(self):
		if self.daq_on:
			self.daq_on = False
			subprocess.run(["rosnode", "kill", "recbag"])
		else:
			rospy.logwarn(f'[RA GUI] DAQ is not running')
		
	def update(self):
		""" Update internal variables """
		self.tNow = time.time() - self.t0 # update Time
		self.gpsok = time.time() - self.tlast_gps < self.gps_age_tol
		self.camok = time.time() - self.tlast_cam < self.cam_age_tol

		# execution rate
		if time.time() - self.tLast >= 1.0:
			self.fps = self.n
			self.n = 0
			self.tLast = time.time()          
		self.n = self.n + 1 
		# StatusString = "Time: "+"{:.2f}".format(self.tNow) + " s, FPS: "+"{:.0f}".format(self.fps)
		# StatusString = "Version 1.0"
		StatusString = "Vehicle ID: "+self.veh_id + ", " + "System ID: "+self.sys_id + ", " + "Version: "+self.software_version 
		self.statusBar().showMessage(StatusString) 

		""" Update Labels """
		#
		if self.camok:
			self.labelCamOk.setStyleSheet("background-color: lightgreen")
			self.labelCamOk.setText('On')
		else:
			self.labelCamOk.setStyleSheet("background-color: red") 
			self.labelCamOk.setText('Off')
		#
		if self.gpsok:
			self.labelGpsOk.setStyleSheet("background-color: lightgreen")
			self.labelGpsOk.setText('On')
		else:
			self.labelGpsOk.setStyleSheet("background-color: red") 
			self.labelGpsOk.setText('Off')
		#
		if self.daq_on:
			self.labelDaqOn.setStyleSheet("background-color: lightgreen")
			self.labelDaqOn.setText('DAQ On')
		else:
			self.labelDaqOn.setStyleSheet("background-color: lightgray") 
			self.labelDaqOn.setText('DAQ Off')

		# user activity
		# self.waiting_for_cam = time.time() - self.tlast_clicked_cam < 30.0
		if self.waiting_for_cam and self.camok:
			self.btnCam.setEnabled(True)
			self.waiting_for_cam = False
		if self.waiting_for_gps and self.gpsok:
			self.btnGps.setEnabled(True)
			self.waiting_for_gps = False

		# if self.firstCall:
		# 	self.firstCall = False

	def btnCamClicked(self):
		self.tlast_clicked_cam = time.time()
		# self.camOpen = not(self.camOpen)
		if not self.waiting_for_cam and not self.camok:
			self.cam_process = subprocess.Popen(
            ['bash', '-c', f'{self.cam_launch_command}']
        	) # stdout=subprocess.PIPE, stderr=subprocess.PIPE
			self.cam_on = True
			self.camOpen = True
			self.btnCam.setEnabled(False)
			self.waiting_for_cam = True
			rospy.logwarn(f'[RA GUI] Launched Camera')
		else:
			# Kill process by pid
			subprocess.run(["kill", str(self.cam_process.pid)])
			# Wait for the process to be killed
			self.cam_process.wait()
			print(f"Cam process with PID {self.cam_process.pid} is no longer alive. Return code: {self.cam_process.returncode}")
			self.cam_on = False
			self.camOpen = False


	def btnGpsClicked(self):
		if not self.waiting_for_gps and not self.gpsok:
			# self.gps_process = subprocess.Popen(self.gps_launch_command)
			self.gps_process = subprocess.Popen(
            ['bash', '-c', f'{self.gps_launch_command}']
        	) #  # stdout=subprocess.PIPE, stderr=subprocess.PIPE
			self.gps_on = True
			self.btnGps.setEnabled(False)
			self.waiting_for_gps = True
			rospy.logwarn(f'[RA GUI] Launched GPS')
			# self.ublox_process = NamedPopen('roslaunch am_ublox ublox.launch', shell=True, name="ublox_launch")
		else:
			# subprocess.run(["rosnode", "kill", "/ublox/reader"]) # kill ros node
			# Kill process by pid
			subprocess.run(["kill", str(self.gps_process.pid)])
			# Wait for the process to be killed
			self.gps_process.wait()
			print(f"GPS process with PID {self.gps_process.pid} is no longer alive. Return code: {self.gps_process.returncode}")
			self.gps_on = False

	def clickedSwapmile(self):
		start_mi = self.uiMileStart.text()
		end_mi = self.uiMileEnd.text()
		self.uiMileStart.setText(end_mi)
		self.uiMileEnd.setText(start_mi)

	def initUI(self):
		""" Buttons and Clicks """
		self.btnCam.clicked.connect(self.btnCamClicked)
		self.btnGps.clicked.connect(self.btnGpsClicked)
		self.btnStart.clicked.connect(self.clickedStart) 
		self.btnStop.clicked.connect(self.clickedStop)
		self.btnPreprocess.clicked.connect(self.clickedPreprocess) 
		self.btnSwapmile.clicked.connect(self.clickedSwapmile) 
		self.btnPreprocess.setText("Make Units")
		self.uiRoute.setText("")
		self.uiLane.setText("1")
		self.uiDirection.setText("East")
		self.labelMileStart.setText("Start (mile)")
		self.labelMileEnd.setText("End (mile)")
		
		""" Labels """
		self.labelCamOk.setText('Off')
		self.labelCamOk.setAlignment(QtCore.Qt.AlignCenter) 
		self.labelCamOk.setStyleSheet("background-color: lightgray") 

		self.labelGpsOk.setText('Off')
		self.labelGpsOk.setAlignment(QtCore.Qt.AlignCenter) 
		self.labelGpsOk.setStyleSheet("background-color: lightgray") 

		self.labelDaqOn.setText('DAQ Off')
		self.labelDaqOn.setAlignment(QtCore.Qt.AlignCenter) 
		self.labelDaqOn.setStyleSheet("background-color: lightgray") 

		self.gpsLabel.setStyleSheet("background-color: lightgray") 
		# self.lonLabel.setStyleSheet("background-color: lightgray") 
		self.gpsLabel.setFont(QFont('Arial', 20))
		# self.lonLabel.setFont(QFont('Arial', 20))
		self.labelGpsname.setFont(QFont('Arial', 12))
		# self.labelLonname.setFont(QFont('Arial', 12))

		self.unitnumLabel.setStyleSheet("background-color: lightgray") 
		self.unitnumLabel.setFont(QFont('Arial', 30))

		""" Window and Layout """
		self.setWindowTitle("Road Audit DAQ")
		self.setFixedSize(self.size()) # 1174, 1128
		self.move(50, 50)
		# self.setGeometry(0, 0, 1174, 900) # 1174, 1128
		self.layout = pg.GraphicsLayout(border=(0,0,0)) # layout central widget
		# self.graphView.setFixedSize(self.graphView.width(), self.graphView.height())
		# self.graphView.setCentralItem(self.layout)

		self.imgLabelWidth = self.imgLabel.width()

	# def initCam(self):
		# self.vs = WebcamVideoStream(src=0).start()
		# self.imgLabelWidth = self.imgLabel.width()
		# self.vs = cv2.VideoCapture(0)

	def initChart(self):
		self.p1 = self.layout.addPlot(title="RGB")
		self.p1.plot1 = self.p1.plot([], [], pen=pg.mkPen(color=(255,0,0),width=2), name="Red")
		self.p1.plot2 = self.p1.plot([], [], pen=pg.mkPen(color=(0,255,0),width=2), name="Green")
		self.p1.plot3 = self.p1.plot([], [], pen=pg.mkPen(color=(0,0,255),width=2), name="Blue")
		#self.p1.setRange(xRange={-100,100})
		#self.p1.setRange(yRange={-1,11})
		self.p1.setLabel('left', "RGB Mean")
		self.p1.setLabel('bottom', "Time [s]")
		self.p1.showGrid(x=True, y=True)
      	
def main():
	app = QtWidgets.QApplication(sys.argv)
	mainwindow = MainWindow()
	mainwindow.show()
	sys.exit(app.exec_())

if __name__ == '__main__':         
	main()