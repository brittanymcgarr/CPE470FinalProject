################################################################################
## capture.py                                                                 ##
##                                                                            ##
## Small routine for detecting and capturing the trained target (banana) and  ##
## reporting the result. Returns an integer of how many of the target were    ##
## found in the image.                                                        ##
##                                                                            ##
## Based on Coding Robin's tutorial at:                                       ##
## http://coding-robin.de/2013/07/22/train-your-own-opencv-haar-classifier.html#
## for the banana.xml training data. Document used with permission of author. ##
##                                                                            ##
## Brittany McGarr                                                            ##
## CPE 470 Autonomous Mobile Robots                                           ##
## Fall 2016                                                                  ##
################################################################################

# Import the packages

import datetime
import cv2
import io
import os
import picamera
import numpy

# Capture class item
# Should be SINGLETON! Only one camera allowed in module
class Capture:
	camera = picamera.PiCamera()
	
	# Constructor
	def __init__(self):
		self.camera.resolution = (320, 240)

	# findTarget(string target)
	# Uses the camera (PiCamera) module to capture an image and search for the 
	# trained image with a default for the faces, but currently supports bananas,too

	def findTarget(self, target='faces'):
		# Use a memory stream for individual image
		stream = io.BytesIO()

		# Load the cascade file of data
		# Get the current directory
		path = os.getcwd()

		# Check for which data set to use
		if target == 'banana':
			path += "/banana.xml"
		else:
			path += "/faces.xml"

		print path

		# Load the cascade file xml
		cascade = cv2.CascadeClassifier(path)

		# Finding the image through a sample of images (up to 100 attempts)
		# The number of images detected
		found = 0
		detected = []
		count = 1

		# Search 50 times or until found has been populated
		while found < 1 and count < 50:
			# Capture an image
			self.camera.capture(stream, format='jpeg')
		
			# Setup the buffer to NumPy array
			buffer = numpy.fromstring(stream.getvalue(), dtype=numpy.uint8)

			# Create the OpenCV image format
			image = cv2.imdecode(buffer, 1)

			# Convert the image to grayscale
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

			# Use the cascade to detect the training images
			detected = cascade.detectMultiScale(gray, 1.1, 5)

			# Update how much is in found
			found = len(detected)

			# Update the count
			count += 1

		# Now, draw rectangles around the found objects if any were found
		if found < 1:
			return found

		# Draw a rectangle around where each item was detected
		for (x, y, width, height) in detected:
			cv2.rectangle(image, (x, y), (x+width, y+height), (255, 255, 0), 2)

		# Save the image as a jpeg with date information
		date = datetime.datetime.now().strftime("%I%M%p%B%d")
		cv2.imwrite("result_" + date + ".jpg", image)

		# Return the number of found targets
		return found

