import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from sensor_msgs.msg import LaserScan

class image_converter:

    def __init__(self):        
        cv2.namedWindow("RGB", 1) #create a new window for the rgb output 
        cv2.startWindowThread() #start the window thread
        
        self.objectBegin = 0 #contains the starting location of the detected object
        self.objectDetected = False #contains a boolean value stating whether or not an object has been detected
        self.blueLocated = False #boolean for logging when the blue value has been located
        self.greenLocated = False #boolean for logging when the green value has been located
        self.yellowLocated = False #boolean for logging when the yellow value has been located
        self.redLocated = False #boolean for logging when the red value has been located  
        self.objectLocatedLock = False   #boolean to prevent the search/recovery from running if an object has been found
        self.objectLocatedCounter = 0 #int to kepe track of the ammount of time that the the system has been idling
        self.objectLock = 999 #this value is used to lock the threshold if a colour is detected
        self.bridge = CvBridge() #load the cvBridge to convert openCV images to ROS
        self.twist_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist) #creates a twist publisher
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image, self.callback) #subscribes to the rgb camera feed
        self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.lasercallback) #subscribes to the laser scanner
        
    def lasercallback(self, data):
        self.laser = Laserdata #return the laser data

    def callback(self, data):
        try:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8") #converts openCV to ROS
        except CvBridgeError as e:
            print "dam sun u got an error" #informative error message
            print e #print the real error message
            
        ranges = self.laser.ranges #get the range data from the scanner
        center = int(round((len(ranges)-1)/2)) #get the ranges for the center of the image
        leftRanges = ranges[center-10:center] #get the ranges for the right side of the image
        rightRanges = ranges[center:center+10] #get the ranges for the left side of the image
        
        hsv = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV) #creates the hsv image
        blueThresh = cv2.inRange(hsv, numpy.array([ 110,  50,  50]), numpy.array([130, 255, 250])) #creates blue thresholded       
        greenThresh = cv2.inRange(hsv, numpy.array([ 50,  30,  30]), numpy.array([70, 255, 255])) #creates green thresholded
        yellowThresh = cv2.inRange(hsv, numpy.array([ 30,  100,  100]), numpy.array([30, 255, 255])) #creates yellow thresholded
        redThresh = cv2.inRange(hsv, numpy.array([ 0,  100,  100]), numpy.array([5, 255, 255])) #creates red thresholded      
        floorThresh = cv2.inRange(hsv, numpy.array([ 0,  0,  154]), numpy.array([0, 0, 155])) #creates floor thresholded       

        height, width, channels = cvImage.shape #get the dimensions of the image
        cvFloor = floorThresh[height/1.5:height,0:width] #splits the floor threshold into a smaller section
        cvArray = (np.array(cvFloor)) #create an array for the floor
        cvLeft = cvFloor[0:height,0:width/2] #splits that in half so we have the bottom left
        cvLeftAr = (np.array(cvLeft)) #transforms the bottom left image into an array
        cvRight = cvFloor[0:height,width/2:width] #splits the floor thresholded image so that we have the bottom right
        cvRightAr = (np.array(cvRight)) #transforms the bottom right image into an array
        
        cvFloorCount = np.count_nonzero(cvArray == 255) #count the number of floor pixels that are available in total
        cvLeftFloorCount = np.count_nonzero(cvLeftAr == 255) #count the number of instances of 255(floor) in the floor left image
        cvRightFloorCount = np.count_nonzero(cvRightAr == 255) #the same but for the floor right image
        
        cv2.imshow("RGB", cvImage) #shows the RGB camera feed in a pop out box

        if(self.objectLocatedLock == True): #if the object lock has been enabled
                self.objectLocatedCounter = self.objectLocatedCounter + 1 #increment value
                
                if(self.objectLocatedCounter == 1): 
                    if(self.redLocated == True and self.yellowLocated == True and self.greenLocated == True and self.blueLocated == True):
                        print "I JUST FOUND THE FINAL OBJECT" #if all object have been found, print this
                    else:
                        print "I JUST FOUND AN OBJECT" #if an object has been found, print this
                    
                else:
                    if(self.objectLocatedCounter > 50 and self.objectLocatedCounter < 150):
                        twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.5)) #rotate the turtlebot to show that it has found the object
                        self.twist_pub.publish(twist_msg) #publish the twist
                    
                    if(self.objectLocatedCounter == 200):
                        self.objectLocatedCounter = 0 #reset the located counter
                        self.objectLocatedLock = False #disengage the lock
                               
        elif numpy.nanmin(ranges) < 0.75 or cvFloorCount < 53500: #if the laser ranges dip below 0.75 or there is less than 53500 pixels of floor visible       
            twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)) #turn the robot to the left
            self.twist_pub.publish(twist_msg) #publish the twist
            
        else:       
            for xx in range (0, 4): #establishes a for loop consisting of four iterations (one for each colour we are searching for)
                searchCriteria = 0 #sets this value to its default (0)
                continueSearching = True #sets the boolean to allow for searching of the input feed
                proceed = True #sets the boolean to initate the searching
                
                if self.redLocated == True and self.yellowLocated == True and self.blueLocated == True and self.greenLocated == True: #if all objects have been found
                    print "all objects found, idling"
                    self.objectLock = 000 #set the object lock to a number that can not be confused
                    
                else: #otherwise if one or more objects have still not been found...          
                    if self.objectLock == 999: #if the object lock is at its default (not been overided)              
                        if xx == 0 and self.redLocated == False: # initiate searching for red
                            print "Search Mode: RED :: Object Begin: %s" % (self.objectBegin)
                            redArray = np.array(redThresh) #generate a search array based upon the threshold
                            searchArray = redArray #set the search array to the colour we want to search for
                            searchCriteria = 0 #set the search criteria to 0 (aka Red)
                        elif xx == 0 and self.redLocated == True: #otherewise if this colour has been already located
                            proceed = False #don't allow the program to proceed
                            
                        if xx == 1 and self.yellowLocated == False: #initiate searching for yellow
                            print "Search Mode: YELLOW :: Object Begin: %s" % (self.objectBegin)
                            yellowArray = np.array(yellowThresh) #generate a search array based upon the threshold
                            searchArray = yellowArray #set the search array to the colour we want to search for
                            searchCriteria = 1 #set the search criteria to 1 (aka Yellow)
                        elif xx == 1 and self.yellowLocated == True: #otherewise if this colour has been already located
                            proceed = False #don't allow the program to proceed
                            
                        if xx == 2 and self.blueLocated == False: #initiate searching for blue
                            print "Search Mode: BLUE :: Object Begin: %s" % (self.objectBegin)
                            blueArray = np.array(blueThresh) #generate a search array based upon the threshold
                            searchArray = blueArray #set the search array to the colour we want to search for
                            searchCriteria = 2 #set the search criteria to 2 (aka Blue)
                        elif xx == 2 and self.blueLocated == True: #otherewise if this colour has been already located
                            proceed = False #don't allow the program to proceed
                            
                        if xx == 3 and self.greenLocated == False: #initiate searching for green
                            print "Search Mode: GREEN :: Object Begin: %s" % (self.objectBegin)
                            greenArray = np.array(greenThresh) #generate a search array based upon the threshold
                            searchArray = greenArray #set the search array to the colour we want to search for
                            searchCriteria = 3 #set the search criteria to 3 (aka Green)
                        elif xx == 3 and self.greenLocated == True: #otherewise if this colour has been already located
                            proceed = False #don't allow the program to proceed
                            
                    else: #if the object lock has been engaged
                        if self.objectLock == 0: #if red
                            print "Search Mode: RED :: Object Begin: %s" % (self.objectBegin)
                            redArray = np.array(redThresh) #create an array based upon the threshold
                            searchArray = redArray #copies the new array into the search array
                            searchCriteria = 0 #sets the search criteria to the variable representing this colour
                        elif self.objectLock == 1: #if yellow
                            print "Search Mode: YELLOW :: Object Begin: %s" % (self.objectBegin)
                            yellowArray = np.array(yellowThresh) #create an array based upon the threshold
                            searchArray = yellowArray #copies the new array into the search array
                            searchCriteria = 1 #sets the search criteria to the variable representing this colour
                        elif self.objectLock == 2: #if blue
                            print "Search Mode: BLUE :: Object Begin: %s" % (self.objectBegin)
                            blueArray = np.array(blueThresh) #create an array based upon the threshold
                            searchArray = blueArray #copies the new array into the search array
                            searchCriteria = 2 #sets the search criteria to the variable representing this colour
                        elif self.objectLock == 3: #if green
                            print "Search Mode: GREEN :: Object Begin: %s" % (self.objectBegin)
                            greenArray = np.array(greenThresh) #create an array based upon the threshold
                            searchArray = greenArray #copies the new array into the search array
                            searchCriteria = 3 #sets the search criteria to the variable representing this colour
                            
                    if proceed == True: #if nothing has stopped the program proceeding
                        collisionArray = searchArray #create the collision array by copying the search array
                        collisionArray = collisionArray[380] #remove all but the bottom 120pixels
                    
                        if 255 in searchArray: #if 255 (white) is detected in the thresholded image
                            for s in searchArray: #initiate a deeper search
                                if continueSearching == True: #set the boolean value to continue searching until the white value has been found
                                    if 255 in s: #if 255 (white) is found on a specific line of the array...
                                        value = 0 #create a value which will be used to identifty the target pixel number
                                        for ss in s: #initiate a deeper search (pixel by pixel)
                                            value = value + 1 #increment the value on each iteration
                                            if ss == 255: #if a white pixel is found
                                                try:
                                                    if s[value + 10] == 255: #check that it's not random noise by checking if the value 10 pixels along is still white   
                                                        self.objectBegin = value #copy the pixel position to the objectBegin location             
                                                        
                                                        if searchCriteria == 0: #if red
                                                            self.objectLock = 0 #set the appropriate object lock value
                                                        elif searchCriteria == 1: #if yellow
                                                            self.objectLock = 1 #set the appropriate object lock value
                                                        elif searchCriteria == 2: #if blue
                                                            self.objectLock = 2 #set the appropriate object lock value
                                                        elif searchCriteria == 3: #if green
                                                            self.objectLock = 3 #set the appropriate object lock value
                    
                                                    else: #otherwise if it fails to find an object that isn't noise
                                                        continueSearching = False #stop any subsequent searches
                                                        
                                                except: #if it tries to log an object that is out of bounds (off the right side of the screen)
                                                    twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.1)) #start rotating left
                                                    self.twist_pub.publish(twist_msg) #publish the twist
                                                    continueSearching = False #stop any future searches
                                                           
                                                if self.objectBegin > 310 and self.objectBegin < 365: #if the object is roughly in the middle of the camera feed
                                                    continueMoving = True #permit movement towards the object
                                                    if 255 in collisionArray: #if a value of 255 (white) is detected in the collision array...
                                                        continueMoving = False #prevent any movement
                                                        if searchCriteria == 0: #if red
                                                            self.objectDetected = False #remove object detection boolean
                                                            self.redLocated = True #mark the object as located
                                                            self.objectBegin = 0 #remove the object begin value
                                                            self.objectLock = 999 #remove the object lock and set it to default
                                                            continueSearching = False #stop future searches in the current iteration
                                                            self.objectLocatedLock = True
                                                        elif searchCriteria == 1: #if yellow
                                                            self.objectDetected = False #remove object detection boolean
                                                            self.yellowLocated = True #mark the object as located                                        
                                                            self.objectBegin = 0 #remove the object begin value
                                                            self.objectLock = 999 #remove the object lock and set it to default
                                                            continueSearching = False #stop future searches in the current iteration
                                                            self.objectLocatedLock = True
                                                        elif searchCriteria == 2: #if blue
                                                            self.objectDetected = False #remove object detection boolean
                                                            self.blueLocated = True #mark the object as located
                                                            self.objectBegin = 0 #remove the object begin value
                                                            self.objectLock = 999 #remove the object lock and set it to default
                                                            continueSearching = False #stop future searches in the current iteration
                                                            self.objectLocatedLock = True
                                                        elif searchCriteria == 3: #if green
                                                            self.objectDetected = False #remove object detection boolean
                                                            self.greenLocated = True #mark the object as located
                                                            self.objectBegin = 0 #remove the object begin value
                                                            self.objectLock = 999 #remove the object lock and set it to default
                                                            continueSearching = False #stop future searches in the current iteration
                                                            self.objectLocatedLock = True
                                                        
                                                    if continueMoving == True: #if movement is still allowed, this means that the object has not been marked as located yet
                                                        twist_msg = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)) #move forwards towards the target
                                                        self.twist_pub.publish(twist_msg) #publish the twist
                                                        self.objectDetected = True #indicate that an object has been detected
                                                    
                                                else: #otherwise if the object is not in the center of the screen                           
                                                    if self.objectDetected == True: #...and the object has been previous marked as detected
                                                        twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.5)) #rotate to the left
                                                        self.twist_pub.publish(twist_msg) #publish the twist
                                                        self.objectDetected = False #mark the object as not detected
                                                    
                                                    else: #otherwise if the object has not been previously marked as detected                                                        
                                                        if(self.objectBegin < 320): #if the object is on the left hand side of the stream
                                                            twist_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)) #rotate to the right
                                                            self.twist_pub.publish(twist_msg) #publish thw twist
                                                        elif(self.objectBegin > 320): #else if the object is on the right hand side of the stream
                                                            twist_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, -0.1)) #rotate to the left
                                                            self.twist_pub.publish(twist_msg) #publish the twist
                        
                        else: #if no value of 255 is detected within the stream
                            if self.objectLock != 999: #and if the object lock has been previously set
                                self.objectLock = 999 #remove it and set to default
                        
            if self.objectLock == 999: #if the object lock has not been set (no values have been detected)
                if numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) > 2.0: #if the area in front of the robot is ok...
                    if(cvLeftFloorCount < cvRightFloorCount): #if there is more free space to the right of the robot
                        twist_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, -0.2)) #turn right
                    elif(cvLeftFloorCount > cvRightFloorCount): #else if there is more free space to the left of the robot
                        twist_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)) #turn right
                    else: #otherwise if neither side has more free space
                        twist_msg = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)) #proceed forward
                    self.twist_pub.publish(twist_msg) #publish the twist
                    
                elif numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) < 2.0: #if the area in front of the robot is not ok and the left side is ok
                    if(cvLeftFloorCount < cvRightFloorCount): #if the right side has more free space
                        twist_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, -0.5)) #turn right
                    elif(cvLeftFloorCount > cvRightFloorCount): #else if the left side has more free space
                        twist_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.5)) #turn left
                    else: #otherwise if neither side has more free space
                        twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5)) #proceed to rotate                    
                    self.twist_pub.publish(twist_msg) #publish the twist
                    
                elif numpy.nanmin(leftRanges) < 2.0 and numpy.nanmin(rightRanges) > 2.0: #else if the area in front of the robot is not ok and the right side is ok
                    twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.5)) #rotate to the left
                    self.twist_pub.publish(twist_msg) #publish the twist
                    
                else: #if no direction is appropriate
                    if(cvLeftFloorCount < cvRightFloorCount): #if there is more floor space on the right side of the image
                        twist_msg = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, -0.5)) #move and rotate to the right
                    elif(cvLeftFloorCount > cvRightFloorCount): #else if there is more floor space on the left side of the image
                        twist_msg = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.5)) #move and rotate to the left
                    elif(cvLeftFloorCount == cvRightFloorCount): #else if both sides have an equal ammount of floor space
                        twist_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5)) #rotate to the left and look for a more appropriate route
                    self.twist_pub.publish(twist_msg) #publish the twist                   
            
image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()