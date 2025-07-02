#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import time, os, json
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMainWindow, QLabel, QFileDialog
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import cv2

class ImageWindow(QMainWindow):
	def __init__(self, src, x=100, y=100, title="Image Window", height=600, aspect=4/3):
		super(ImageWindow, self).__init__()
		self.setWindowTitle(title)
		cam_height = height
		cam_width = int(aspect*cam_height)
		self.setGeometry(x, y, cam_width, cam_height) # 2*cam_width
		self.setFixedSize(self.geometry().width(), self.geometry().height())
		self.label1 = QLabel("main label", self)
		self.label1.setGeometry(0, 0, cam_width, cam_height)
		self.init_first_call(False, cam_width, cam_height) # width, height
		self.src = src

		""" Variables """
		# self.t0 = time.time()
		# self.tLast = time.time() 
		# self.n = 0
		# self.fps = 0
		# self.cam_init = False
		self.zoom_step = 0.1
		self.zoom_min = 0.1
		self.mouse_is_down = False
		self.mouse_anchor = [0, 0]
		self.roi_anchor = [0, 1, 0, 1]
		self.first_call = True
		self.control_key = False
		self.usel, self.vsel = None, None
		self.collected = []

		self.show()

	def pan(self, dx, dy):
		du = int(dx/self.labelWidth*(self.roi_anchor[2]-self.roi_anchor[0]))
		dv = int(dy/self.labelWidth*(self.roi_anchor[3]-self.roi_anchor[1]))
		u1 = self.roi_anchor[0] - du
		u2 = self.roi_anchor[2] - du
		v1 = self.roi_anchor[1] - dv
		v2 = self.roi_anchor[3] - dv
		self.roi = self.saturate_pan([u1, v1, u2, v2], self.scale)
		self.paint_img()

	def zoom_in(self, x, y):
		self.scale = max(self.scale*(1-self.zoom_step), self.zoom_min)
		u = int(x/self.labelWidth*(self.roi[2]-self.roi[0]) + self.roi[0])
		v = int(y/self.labelHeight*(self.roi[3]-self.roi[1]) + self.roi[1])
		self.roi = self.saturate_zoom(u, v, self.scale)
		self.paint_img()

	def zoom_out(self, x, y):
		self.scale = min(self.scale*(1+self.zoom_step), 1.0)
		u = int(x/self.labelWidth*(self.roi[2]-self.roi[0]) + self.roi[0])
		v = int(y/self.labelHeight*(self.roi[3]-self.roi[1]) + self.roi[1])
		self.roi = self.saturate_zoom(u, v, self.scale)
		self.paint_img()

	def saturate_zoom(self, u, v, scale):
		# horizontal
		u1 = u - int(scale*self.imgWidth/2)
		u2 = u1 + int(scale*self.imgWidth)
		# hit left?
		if u1 <= 0:
			u1 = 0
			u2 = u1 + int(scale*self.imgWidth)
		# hit right?
		if u2 >= int(self.imgWidth):
			u2 = int(self.imgWidth)
			u1 = int(u2 - scale*self.imgWidth)
		# vertical
		v1 = v - int(scale*self.imgHeight/2)
		v2 = v1 + int(scale*self.imgHeight)
		# hit top?
		if v1 <= 0:
			v1 = 0
			v2 = v1 + int(scale*self.imgHeight)
		# hit bottom?
		if v2 >= int(self.imgHeight):
			v2 = int(self.imgHeight)
			v1 = v2 - int(scale*self.imgHeight)
		return [u1, v1, u2, v2]

	def saturate_pan(self, roi, scale):
		u1, v1, u2, v2 = roi[0], roi[1], roi[2], roi[3]
		# hit left?
		if u1 <= 0:
			u1 = 0
			u2 = u1 + int(scale*self.imgWidth)
		# hit right?
		if u2 >= int(self.imgWidth):
			u2 = int(self.imgWidth)
			u1 = int(u2 - scale*self.imgWidth)
		# vertical
		# hit top?
		if v1 <= 0:
			v1 = 0
			v2 = v1 + int(scale*self.imgHeight)
		# hit bottom?
		if v2 >= int(self.imgHeight):
			v2 = int(self.imgHeight)
			v1 = v2 - int(scale*self.imgHeight)
		return [u1, v1, u2, v2]

	def mouse_wheel(self, event):
		if event.angleDelta().y() > 0: # zoom in
			if self.scale > self.zoom_min: # zoom allowed, scale not minimum yet
				self.zoom_in(event.x(), event.y())

		if event.angleDelta().y() < 0: # zoom out
			if self.scale < 1.0: # zoom allowed, scale not maximum yet
				self.zoom_out(event.x(), event.y())

	#def wheelEvent(self, event):
		#print(event.source(), event.pos(), event.position(), event.angleDelta(), event.buttons(), event.x(), event.y())

	def mouse_down(self, event):
		self.mouse_is_down = True
		self.mouse_anchor = [event.x(), event.y()]
		self.roi_anchor[0:] = self.roi[0:]
		u = int(event.x()/self.labelWidth*(self.roi_anchor[2]-self.roi_anchor[0]) + self.roi_anchor[0])
		v = int(event.y()/self.labelHeight*(self.roi_anchor[3]-self.roi_anchor[1]) + self.roi_anchor[1])
		self.usel, self.vsel = u, v
		print("Image coordinates (u, v)", u, v)

	def mouse_up(self, event):
		self.mouse_is_down = False

	def mouse_move(self, event):
		if self.mouse_is_down and self.scale < 1.0: # panning possible
			x = min(max(event.x(), 0), self.labelWidth)
			y = min(max(event.y(), 0), self.labelHeight)
			dx = x-self.mouse_anchor[0]
			dy = y-self.mouse_anchor[1]
			self.pan(dx, dy)

	def init_first_call(self, first_call, img_width, img_height):
		self.imgWidth = img_width # set image size
		self.imgHeight = img_height
		self.labelWidth = self.label1.width() # set label width variable
		scale = self.labelWidth/self.imgWidth # compute scale between image and label
		labelHeight = int(scale * self.imgHeight) # compute desired label height
		self.labelHeight = labelHeight # set label height variable
		self.label1.resize(self.labelWidth, self.labelHeight) # adjust label size
		self.scale = 1.0
		self.roi = [0, 0, self.imgWidth, self.imgHeight]
		if first_call:
			self.first_call = False
			self.label1.mousePressEvent = self.mouse_down
			self.label1.mouseReleaseEvent = self.mouse_up
			self.label1.mouseMoveEvent = self.mouse_move
			self.label1.wheelEvent = self.mouse_wheel

	def set_img(self, img):
		self.img = img
		if self.first_call:
			self.init_first_call(True, img.shape[1], img.shape[0]) # width, height
		self.paint_img()

	def paint_img(self):
		#img = imutils.resize(self.img[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2], :], width=self.labelWidth)
		img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
		img = cv2.resize(img[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2], :], (self.labelWidth, self.labelHeight), interpolation = cv2.INTER_AREA)
		img = QImage(img, img.shape[1], img.shape[0], img.strides[0], QImage.Format.Format_RGB888)
		self.label1.setPixmap(QPixmap.fromImage(img)) 
		#self.label2.setPixmap(QPixmap.fromImage(img))

	def move_window(self, x, y):
		self.setGeometry(x, y, self.geometry().width(), self.geometry().height())

	def keyPressEvent(self, event):
		if self.usel is not None and self.vsel is not None:
			if event.key() == 49:
				self.src[0] = [self.usel, self.vsel]
			if event.key() == 50:
				self.src[1] = [self.usel, self.vsel]
			if event.key() == 51:
				self.src[2] = [self.usel, self.vsel]
			if event.key() == 52:
				self.src[3] = [self.usel, self.vsel]
			
	# 	if event.key() == Qt.Key_Space:
	# 		self.collected.append((self.usel, self.vsel))
	# 		print("Point appended to collected")

	# 	if event.key() == Qt.Key_Escape:
	# 		self.clear_collected()

	# def clear_collected(self):
	# 	self.collected = []
	# 	print("Collected points cleared")

	def keyReleaseEvent(self, event):
		if event.key() == Qt.Key_Control:
			self.control_key = False

	def closeEvent(self, event):
		print("User Exit Sub Cam")
		self.close()

if __name__ == '__main__':
	import sys
	from PyQt5.QtWidgets import QApplication

	app = QApplication(sys.argv)
	main_win = ImageWindow()
	#main_win.show()
	sys.exit(app.exec_())
