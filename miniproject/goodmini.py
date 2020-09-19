"""
Created on Wed Apr 18 15:50:22 2020

@author: Valdas Druskinis

Miniproject hand-in on autonomous objects recognition and classification of fruits.

P4 Robotics.
Robotic Perception
"""

import cv2
import numpy as np

def colorRates():
	#Using HSV colour pallette from https://i.stack.imgur.com/gyuw4.png
	#Looking for specific colours in range, targeting desired objects
	#1st entry in the matrix- Hue, 2nd - Saturation, 3rd - Value
	lower_darkgreen = np.array([40, 36,0])
	upper_darkgreen = np.array([68, 255,255])
	lower_yellow = np.array([20, 100, 20])
	upper_yellow = np.array([33, 255, 255])
	lower_ry = np.array([0, 100, 20])
	upper_ry = np.array([30, 250, 255])
	return lower_darkgreen, upper_darkgreen, lower_yellow, upper_yellow, lower_ry, upper_ry

def picture(picture): #Scaling original picture to 20%
	scale_percent = 20
	width = int(picture.shape[1]* scale_percent/100) #
	height = int(picture.shape[0]* scale_percent/100) #
	dim = (width, height)
	origimg = cv2.resize(picture, dim, interpolation = cv2.INTER_AREA)
	img = cv2.resize(picture, dim, interpolation = cv2.INTER_AREA)
	return img

def means(img):
	image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #Converting BGR image to HSV
	lower_darkgreen, upper_darkgreen, lower_yellow, upper_yellow, lower_ry, upper_ry = colorRates() #using color pallette.

	#making masks for all objects and then merging them together into one mask.
	#every mask is thresholding for desired color range
	mask1 = cv2.inRange(image_hsv, lower_darkgreen, upper_darkgreen)
	mask2 = cv2.inRange(image_hsv, lower_ry, upper_ry)
	mask3 = cv2.inRange(image_hsv, lower_yellow, upper_yellow)
	masks = mask1 | mask2 | mask3

	#Finding mean value for color green
	mean1 = cv2.mean(image_hsv, mask1)
	mean1 = np.array(mean1)
	mean1 = mean1.take(-4) #my hue colour for green
	return mean1, masks
 
def object_recognition(img): 
	lower_darkgreen, upper_darkgreen, lower_yellow, upper_yellow, lower_ry, upper_ry = colorRates()
	mean1, masks = means(img)
	#Kernel filters
	kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (60, 60))
	kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))

	#Using morphology to open (remove noise) and close(fill objects)
	open_img = cv2.morphologyEx(masks, cv2.MORPH_OPEN, kernel1)
	close_img = cv2.morphologyEx(open_img, cv2.MORPH_CLOSE, kernel)
	close = cv2.morphologyEx(masks, cv2.MORPH_CLOSE, kernel2) #filling lemon at different filter

	#merging both filled lemon and the rest of the objects
	final = close | masks
	#cv2.imshow('g', final)
	return final

def edges(final,resized):
	#applying Canny function to find the lines of the objects
	edge = cv2.Canny(final,55,35)

	mean1, masks = means(img) #using mean1 and masks values from means function
	lower_darkgreen, upper_darkgreen, lower_yellow, upper_yellow, lower_ry, upper_ry = colorRates()

	dictionary = {} #introducting dictionary to collect data values (in this case strings)
	contours, hierarchy = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:] #finding how many different contours are there
	print("Number of Contours found in the picture = " + str(len(contours)))

	for cnt in range(len(contours)): #ranging for all found contours
		area = cv2.contourArea(contours[cnt]) #checking if contour area of the objects are similar
		#print(area)

 		#we are only interested in contours of specific size
		if 30000 < cv2.contourArea(contours[cnt]) < 36000: #Banana have the largest contour area of all chosen objects.
			#Using moments to find object's coordinates.
			M = cv2.moments(contours[cnt])				
			x = int(M['m10']/M['m00']-30)
			y = int(M['m01']/M['m00']-30)
			cord = (x,y)
			cv2.putText(img, "banana",cord, cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2) #writing down banana at (x,y) object's coordinates.
			dictionary.setdefault(cnt, "banana")
		#Since lemon and apple have almost the same area size and they both share yellow color,
		#one can define which one is which is by looking at their perimeter. Apple have larger perimeter than lemon, thus:
		if 0 < cv2.arcLength(contours[cnt], True) < 1000:
			if 540 < cv2.arcLength(contours[cnt], True) < 555:
				M = cv2.moments(contours[cnt])
				x = int(M['m10']/M['m00']-30)
				y = int(M['m01']/M['m00']-30)
				cord = (x,y)
				cv2.putText(img, "apple",cord, cv2.FONT_HERSHEY_PLAIN, 1.5,(0, 0, 255), 2)
				dictionary.setdefault(cnt, "apple")
			elif 530 < cv2.arcLength(contours[cnt], True) < 539:
				M = cv2.moments(contours[cnt])
				x = int(M['m10']/M['m00']-30)
				y = int(M['m01']/M['m00']-30)
				cord = (x,y)
				cv2.putText(img, "lemon",cord, cv2.FONT_HERSHEY_PLAIN, 1.5,(155,55,255), 2)
				dictionary.setdefault(cnt, "lemon")
			elif lower_darkgreen[0] < mean1 < upper_darkgreen[0]: #Since avocado is the only object that does not contain yellow colour, we can use green mean value to find it
				M = cv2.moments(contours[cnt])
				x = int(M['m10']/M['m00']-30)
				y = int(M['m01']/M['m00']-30)
				cord = (x,y)
				cv2.putText(img, "avocado",cord, cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 140, 255), 2)
				dictionary.setdefault(cnt, "avocado")
	return dictionary, contours #returning defined entries and found contours

def printContours(img, dictionary, contours):
	#Printing contours for every found object.
	#Dictionary relates to the index of an object.
	for entry in dictionary:
		if dictionary.get(entry) == "avocado":
			colour = (0, 140, 255)
		elif dictionary.get(entry) == "banana":
			colour = (0, 255, 0)
		elif dictionary.get(entry) == "apple":
			colour = (0, 0, 255)
		elif dictionary.get(entry) == "lemon":
			colour = (155,55,255)
		cv2.drawContours(img, contours, entry, colour, 5)

def show(n,m):
	#Show function to visualize images
	cv2.imshow("Original Image",n)
	cv2.imshow("Objects in the Image", m)
	cv2.waitKey(0)

	#Use function below if needed
	#cv2.destroyAllWindows(0)

#The program starts here
if __name__ == '__main__':
	original = cv2.imread('/home/valdas/myobjects.png',1) #Original picture with 4 objects
	#original = cv2.imread('/home/valdas/Desktop/RoboticP/figs/edited1.png') #Picture with 6 objects for testing purposes
	resized = picture(original)
	img = picture(original)

	segmentation = object_recognition(img)
	d, c = edges(segmentation, resized)

	printContours(img, d, c) #printing contours for image, with specified entries and contours
	show(resized,img) #showing original resized image and edited one.
