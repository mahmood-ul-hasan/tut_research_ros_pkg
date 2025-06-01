#!/usr/bin/python
import os
import math
import numpy
import pyquaternion
import scipy.optimize

import rospy
import rospkg
import message_filters
from sensor_msgs.msg import *


class CalibrationNode:
	def __init__(self):
		subs = [
			message_filters.Subscriber('/scan', LaserScan),
			message_filters.Subscriber('/ptu/joint_states', JointState)
		]
		self.sync = message_filters.ApproximateTimeSynchronizer(subs, 100, 0.1)
		self.sync.registerCallback(self.callback)

		self.scans = []

	def callback(self, scan_msg, joints_msg):
		self.scans.append((scan_msg, joints_msg))
		print 'joint_angles', joints_msg.position

	def save(self):
		numpy.save('/tmp/scans.npy', self.scans)
		print 'scan data saved to /tmp/scans.npy'

	# convert range data into 3D points
	def scan2points(self, scan_msg):
		ranges = scan_msg.ranges
		angles = [scan_msg.angle_min + scan_msg.angle_increment * i for i in range(len(scan_msg.ranges))]

		SCAN_DISTANCE_MIN = 0.5
		SCAN_DISTANCE_MAX = 50.0
		ranges_angles = [(r, a) for r, a in zip(ranges, angles) if r > SCAN_DISTANCE_MIN and r < SCAN_DISTANCE_MAX]

		ranges = numpy.float64([r for r, a in ranges_angles])
		angles = numpy.float64([a for r, a in ranges_angles])
		points = numpy.vstack([numpy.zeros(ranges.shape), numpy.sin(angles) * ranges, numpy.cos(angles) * ranges])

		return points

	# rotate a scan with pan & tilt angles
	def rotate(self, points, pan, tilt):
		pan_rot = pyquaternion.Quaternion(axis=[0, 0, 1], angle=pan)
		tilt_rot = pyquaternion.Quaternion(axis=[0, 1, 0], angle=tilt)
		rotation = tilt_rot * pan_rot

		return rotation.rotation_matrix.dot(points)

	# find the tilt angle which minimizes ICP error
	def estimate_tilt(self, xyz0, pan0, xyz1, pan1):
		def error(x):
			pts0 = self.rotate(xyz0, pan0, 0.0)
			pts1 = self.rotate(xyz1, pan1, x[0])

			err = 0.0

			for pt in pts0.T:
				diff = numpy.linalg.norm(pts1.T - pt, axis=1)
				err += numpy.min(diff)

			return err

		x0 = [0]

		def callback(x):
			print 'tilt:', x[0], 'err:', error(x)

		result = scipy.optimize.minimize(error, x0, method='Nelder-Mead', callback=callback)

		return result.x[0]

	# peform tilt angle calibration
	def calibrate(self):
		self.scans = numpy.load('/tmp/scans.npy')

		idx_angles = [(i, x[1].position[0]) for i, x in enumerate(self.scans)]
		begin_idx = [i for i, x in idx_angles if math.degrees(x) > 90.1][0]

		idx_angles = idx_angles[begin_idx:]
		end_idx = [i for i, x in idx_angles if math.degrees(x) < -90.1][0]

		self.scans = [x for x in self.scans[begin_idx:end_idx] if math.degrees(x[1].position[0]) < 90.0 and math.degrees(x[1].position[0]) > -90.0]

		xyz0 = self.scan2points(self.scans[0][0])
		pan0 = self.scans[0][1].position[0]

		xyz1 = self.scan2points(self.scans[-1][0])
		pan1 = self.scans[-1][1].position[0]

		tilt1 = self.estimate_tilt(xyz0, pan0, xyz1, pan1)

		# save calibration data
		package_path = rospkg.RosPack().get_path('ridar_scan')
		if not os.path.exists(package_path + '/data'):
			os.mkdir(package_path + '/data')

		with open(package_path + '/data/calibration.csv', 'w') as f:
			print 'pan0', pan0
			print 'pan1', pan1
			print 'tilt', -tilt1
			print >> f, 'pan0', pan0
			print >> f, 'pan1', pan1
			print >> f, 'tilt', -tilt1

		print 'saved to', package_path + '/data/calibration.csv'

		# visualization
		pts0 = self.rotate(xyz0, pan0, 0)
		pts1_orig = self.rotate(xyz1, pan1, 0)
		pts1_calib = self.rotate(xyz1, pan1, tilt1)

		try:
			import plotly
		except:
			print 'failed to import plotly'
			return

		plotly.offline.init_notebook_mode(connected=True)

		data = [
			plotly.graph_objs.Scatter3d(x=pts0[0, :], y=pts0[1, :], z=pts0[2, :], mode='markers', marker=dict(size=3), name='scan0'),
			plotly.graph_objs.Scatter3d(x=pts1_orig[0, :], y=pts1_orig[1, :], z=pts1_orig[2, :], mode='markers', marker=dict(size=3), name='pts1_orig'),
			plotly.graph_objs.Scatter3d(x=pts1_calib[0, :], y=pts1_calib[1, :], z=pts1_calib[2, :], mode='markers', marker=dict(size=3), name='pts1_calib')
		]
		layout = plotly.graph_objs.Layout(
			scene=dict(
				xaxis=dict(range=[-50, 50]),
				yaxis=dict(range=[-50, 50]),
				zaxis=dict(range=[-50, 50])
			)
		)

		fig = plotly.graph_objs.Figure(data=data, layout=layout)

		plotly.offline.plot(fig)


def main():
	rospy.init_node('calibration_node')

	node = CalibrationNode()

	print 'feed range & joints data, then press ctrl+C'

	# collect data
	while not rospy.is_shutdown():
		pass

	# estimate the tilt angle
	node.save()
	node.calibrate()


if __name__ == '__main__':
	main()
