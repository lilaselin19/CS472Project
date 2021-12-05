"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *

from struct import *
import numpy as np
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

BEHAVIORLIST = []
  
class Behavior:
  def activate(){
    raise NotImplemented
  }
  
class avoidZombieBehavior(Behavior):
  def activate(){
    #actual content
	}
  
def avoidObstacleBehavior(TIMECOUNTER): 
  # we define obstacles as walls, strumps, and trees 
  # we want to avoid these within the meter or half meter
  # at every timestep 
  for i in (0, len(array_of_objects))
    if array_of_objects[4] == 'obstacle':
      # if the obstacle is in the CENTER (determined by camera) 
      # turn left for one time step
    elif array_of_objects[4] == 'wall':
      # if the obstacle is in the CENTER (determined by camera) 
     		# if camera determines wall to be on left 
        # turn right
        # elif camera determines wall to be right
        # turn left
        # elif (wall is on both sides)
        # turn left 
      
    
  # check array_of_objects for any 'obstacle' or 'wall' 
  # we may specify WALL because you likely have to turn more 
  # DISTANCE MATTERS ONLY IF IT'S IN YOUR CENTER OF YOUR CAMERA OR LIDAR (same calcuation as the berry) .5 
  # if not a wall, we pick left or right and it turns
  # if a wall, we see what direction wall continues in; we turn in the opposite direction
  
  # should be called in every behavior; should not be its own seperate thing 
  

# we may not need this anymore because we are seeking only one berry at a time
class seekBerryBehavior(Behavior):
  
class exploreBehavior(Behavior):

  
def readLidar(lidarDevice){}

def testCloser(cloudArray, array_of_objects, counter):
    print("----------in testCloser")
    if np.isinf(cloudArray[counter][0]):
        return False
  	# is our x value closer than the previous object's x value? 
    
    if len(array_of_objects) == 0:
        #print("len 0")
        return False
    elif array_of_objects[len(array_of_objects)-1][0] - cloudArray[counter][0] > 5: 
      # this is a new object
        print("test closer is true!")
        return True
    print("neither")
    return False
                      
def testFurther(cloudArray, array_of_objects, counter):
    print("----------in testFurther")
    if np.isinf(cloudArray[counter][0]):
        return False
  	# is our x value closer than the previous object's x value? 
   #if cloudArray[i][0] < array_of_objects[len(array_of_objects)-1][0]:
    print(array_of_objects[len(array_of_objects)-1][0])
    #print(cloudArray[i][0])
    if array_of_objects[len(array_of_objects)-1][0] - cloudArray[counter][0] < 5: 
      # this is a new object
        print("this is a new object!")
        return True
    print("returning false")
    return False
                      
def newObjectFunction(cloudArray, array_of_objects, counter, objectDepth):
    print("- new object function -")
     # we need to keep track of this across functions
    objectDepth += 1
     # create a new element in the array for new object1
    
     # loop through subsequent cloud points
    for j in range(counter, len(cloudArray)):
        print("in for loop")                  
        # base case 1: return to original object : ie if cloud point at j is non zero and is FUTHER than the prior object
        #print(cloudArray)
        print(len(array_of_objects))
        #print(j)
        testCloserResult = testCloser(cloudArray, array_of_objects, j)
        print(testCloserResult)
        testFurtherResult = testFurther(cloudArray, array_of_objects, j)
        print(testFurtherResult)
        #print("NOF before if")
        if not np.isinf(cloudArray[j][0]) and testFurtherResult:
            print("NOF in if base case 1")
          # save new object1 / update the end points 
            array_of_objects[len(array_of_objects)-1][2] = cloudArray[j-1][0]
            array_of_objects[len(array_of_objects)-1][3] = cloudArray[j-1][2]
          # return objectDepth and counter and go back to the original function
            return [objectDepth, j]
                          
        # base case 2: chugging along the new object 
        elif not np.isinf(cloudArray[j][0]) and not testFurtherResult and testCloser(cloudArray, array_of_objects,j) == False:
            print("NOF base case 2")
            # continue
            continue
                          
        # recursive case: you sense a NEW object atop the new one. Let's call it new object2
        elif not np.isinf(cloudArray[j][0]) and testCloserResult:
            print("NOF recursive case")
            # call function again
            #print(cloudArray)
            #print(array_of_objects)
            #print(j)
            #print(objectDepth)
            print("before recursing")
            print(len(array_of_objects))
            array_of_objects.append([cloudArray[j][0], cloudArray[j][2],0,0])
            returnList = newObjectFunction(cloudArray, array_of_objects, j+1, objectDepth)
            print("after recursive")
            print(returnList)
            objectDepth = returnList[0]
            counter = returnList[1]
            j = counter
            
        else:
            print("in else")


def readLidar(cloudArray):
    # need the pointCloud
    # state if we are in an object. 0 means we are not; 1 means we are
    objectFound = 0
    # each element in array has four values: xstart, zstart, xend, zend
    array_of_objects = []
    # if the x element of the point Cloud is not infinity i.e something is there AND we are not in an object already
    for i in range(0, len(cloudArray)):
        #print("in for loop")
        testCloserResult = testCloser(cloudArray, array_of_objects, i)
        print("elif conditions")
        #print(testCloserResult)
        #print(objectFound)
        #print(cloudArray[i][0])
        
        
        # case 1: you are at the start of an object 
        if not np.isinf(cloudArray[i][0]) and objectFound == 0: # and not np.isnan(cloudArray[i][0]):
            print("1st not inf")
            if objectFound == 0:
                objectFound = 1
                # add an element to the array of objects  
                array_of_objects.append([cloudArray[i][0], cloudArray[i][2],0,0])
                # return, so it doesn't go into the third case? 
        #print("after if") 
        
        	# case 3: you are at the end of an object 
        elif np.isinf(cloudArray[i][0]) and objectFound == 1: # and np.isnan(cloudArray[i][0]):
            print("2nd is inf")
            if objectFound == 1:
                objectFound = 0
                array_of_objects[len(array_of_objects)-1][2] = cloudArray[i-1][0]
                array_of_objects[len(array_of_objects)-1][3] = cloudArray[i-1][2]  
        
        # case 2 : an object is in front of another one
        # if we are using color, the third condition will be TestDifferentColor
        
        
        elif (not np.isinf(cloudArray[i][0])) and objectFound == 1 and testCloserResult:
            print("3rd overlap")
            objectDepth = 0
            array_of_objects.append([cloudArray[i][0], cloudArray[i][2],0,0])
            returnList = newObjectFunction(cloudArray, array_of_objects, i, objectDepth)
            objectDepth = returnList[0]
            counter = returnList[1]
            # update the counter in the cloud array after going through new objects
            i = counter 
            
        elif not np.isinf(cloudArray[i][0]) and objectFound == 1:
            print("4th chugging along")
            continue
        
             
                
        
    	
                
    print(array_of_objects)
    return array_of_objects
          
          
                      
                      
    
                      
        
        
      
    
    
    

def readCamera(cameraDevice){}

def createAvoidZombieBehavior(){}

def createAvoidObstacleBehavior(){}

def createSeekBerryBehavior(){}

def exploreBehavior(){}


def exploreBehaviour(fr, fl, br, bl, TIMECOUNTER):
    print(TIMECOUNTER)
    if TIMECOUNTER % 20 == 0:
        #fr.setVelocity(0.0)
        #br.setVelocity(0.0)
        fr.setPosition(float(0))
        #fl.setPosition(float('inf'))
        br.setPosition(float(0))
        #bl.setPosition(float('inf'))
        
    elif TIMECOUNTER % 2 == 0:
        fr.setPosition(float('inf'))
        br.setPosition(float('inf'))

        
def cropImage(img):
    imgShape = img.shape
    #print(imgShape)
    print("size of image is", imgShape)
    crop = img[23:64,0:128]
    print("size of crop image is", crop.shape)
    cv.imwrite('origImg.jpg',img)
    cv.imwrite('croppedImg.jpg',crop)
    return crop

# Use contour function to determine outline of all objecst with that color
def findObjectOnCamera(img, mask):
    print("findObjectOnCamera")
    #print(img.shape)
    #print(mask.shape)
    coordsList = []
    # red is going into the for loop
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    for c in contours:
        # calculate moments for each contour
        M = cv.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv.circle(mask, (cX, cY), 5, (70, 115, 3), -1)
        #cv.putText(mask, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv.imwrite('contoursImg.jpg',mask)
        coordsList.append([cX, cY])
    return coordsList

# This function converts our thresholded image into a black and white one
# We do this to ensure that when the image is contured, each object is percieved only once (shadows interfere with that)
def convertToBinaryImage(img):
    bgrImg = cv.cvtColor(img, cv.COLOR_HSV2BGR)
    grayImg = cv.cvtColor(bgrImg, cv.COLOR_BGR2GRAY)
    (thresh, blackAndWhiteImage) = cv.threshold(grayImg, 2, 255, cv.THRESH_BINARY)
    cv.imwrite('BWImg.jpg',blackAndWhiteImage)
    return blackAndWhiteImage
    

# Recieves input of color limits
# Converts img to hsv
# Creates and applies mask
# Returns [img, mask]
def createMaskImage(img, lower_limit, upper_limit):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_limit, upper_limit)
    result = cv.bitwise_and(img, img, mask= mask)
    binaryResult = convertToBinaryImage(result)
    return [binaryResult, mask]

# Note that the mask is now a black and white image; this was created for debugging
# colorObjectList stores all colors and their coords [coords, color]
# Function loops through result from FindObject (uses contouring) and adds color; adds to list IF object exists
# We plan to use this list to loop through and determine what color each lidar object has 
def findCoords(colorObjectList, src, mask, color):
    coordsList = findObjectOnCamera(src, mask)
    for c in range(0, len(coordsList)):
        coords = [coordsList[c], color]
        #coordsList = [findObjectOnCamera(src, mask), color]
        print(color, " coords is", coords)
        if coords[0] != None:
            colorObjectList.append(coords)
        print("color object list", colorObjectList)
        

#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    TIMECOUNTER = 0
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
   	
		camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    
    print("resolution: "+str(lidar.getHorizontalResolution()))
    print("layer number: "+str(lidar.getNumberOfLayers()))
    print(dir(lidar))
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))
    
    
    i=0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
        # called every timestep
        # call the lidar first 
        # Call the camera every second
        # Crude pairing of the lidar (and color info once a second)
        # Pass info to behaviors
        # each behavior outputs a speed for each wheel
				# some central system weights the importance of each behavior and sums the speads
        lidar.enablePointCloud()
        # pointCloud = lidar.getPointCloud(data_type = 'buffer')
        # print("points cloud is ", pointCloud)
        # sz = sys.getsizeof(pointCloud[0])
        # print("size of one point cloud is",sz)
        # print("point cloud elt at 70 is ", pointCloud[70])
        # array_sz = shape.pointCloud()
        # print("we calculated array size!")
        # print(array_sz)
        # print("Before for loop")
        """
        num_points = lidar.getNumberOfPoints()
        point_buf = lidar.getPointCloud(data_type="buffer")
        # need to put full size of the 'format'; can't just asssume you can break it into 5
        points = struct.unpack("fffif" * num_points, point_buf)
        """
        # ------------------------------------
        
        #TODO: CHANGE NAMES
        num_points = lidar.getNumberOfPoints()
        point_buf = lidar.getPointCloud(data_type="buffer")
        points = unpack("fffif" * num_points, point_buf)
        #print(len(points))
        
        cloudArray = []
        pointCloud = []
        for i in range(0, len(points)):
            if i % 5 == 0 and i != 0:
                if not np.isnan(pointCloud[0]):
                    cloudArray.append(pointCloud)
                pointCloud = []
            pointCloud.append(points[i])
        print(cloudArray)
        
        
        # PLAN OUT MAIN ACTIONS
        array_of_objects = readLidar(cloudArray)
        # iterate through the objects (assume they are already paired with the correct object labeling) to look at the fifth element
        # they're in order
        # question one: is there a zombie within 5 m of you?
        # if yes, trigger avoid zombie behavior (can adjust this potentially based on the kind of zombie)
        
        # question two: no zombie near you. is there a berry?
        # search for berry. at the first one, see if there's a zombie near it (3 m). 
        # Nick advice: don't look for zombie bc you are evaluating every time step 
        # if so, search for other berry and repeat same thing. if not, go towards berry
        
        # have an avoidObstacle function we call every time we start moving. would check any obstacles within 1 meter. 
        
        
        # if you are in low health, does anything change? 
        
        # also option: once you learn which berry causes a negative effect, change its name to 'taintedBerry' 
        # might need to count 
        # can just test it twice and see (make a quick guestimate)
        # or smth like that so your berry function runs as normal
        
        
        # look at first object. if it's a berry, check objects on either side
        # and see if there's a zombie. we search for zombies in the ret of the array and if they're less than 5 m away, go to berry
        # ought we to create a dictionary? search for berries 
        
        TIMECOUNTER += 1
        if len(array_of_objects) == 0:
        		
        		exploreBehaviour(fr, fl, br, bl, TIMECOUNTER)
            
            
        # processing image     
        image = camera1.getImageArray()
        imageArray = np.array(image)
        imgFileResult = camera1.saveImage('imgFile.jpg', 100)
        print(imgFileResult)
        
        src = cv.imread('imgFile.jpg', cv.IMREAD_UNCHANGED)
        hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        
        
        # CREATE WHITE AND BLACK
        gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        lightWall = cv.cvtColor(src, cv.COLOR_BGR2HLS)
        darkWall = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        #white = 255 - white
        gray = 255 - gray
        ret_black, thresh_black = cv.threshold(gray, 225, 255, cv.THRESH_BINARY_INV)
        #ret_white, thresh_white = cv.threshold(white, 225, 255, 0)
        Lchannel = lightWall[:,:,1]
        #change 250 to lower numbers to include more values as "white"
        #mask = cv.inRange(Lchannel, 250, 255)
        mask = cv.inRange(lightWall, np.array([0,210,0]), np.array([255,255,255]))
        res = cv.bitwise_and(src,src, mask= mask)
        #mask = cv.inRange(imgHLS, np.array([0,250,0]), np.array([255,255,255]))
        #darkWall = 255 - darkWall
        ret_darkWall, thresh_darkWall = cv.threshold(darkWall, 100, 255, cv.THRESH_BINARY_INV)
        
        
 				# CREATE BLUE IMAGE
        #hsvCrop = cv.cvtColor(cropImg, cv.COLOR_BGR2HSV)
        #lower_blue = np.array([100, 50, 50])
        #upper_blue = np.array([130, 255, 255])
        #mask_blue = cv.inRange(hsvCrop, lower_blue, upper_blue)
        #result_blue = cv.bitwise_and(cropImg, cropImg, mask=mask_blue)
        
        cropSrc = cropImage(src)
        result_blue = createMaskImage(cropSrc, np.array([100, 100, 50]), np.array([130,255,255]))
        findCoords(colorObjectList, cropSrc, result_blue[0], 'blue')
        #findObjectOnCamera(cropImg, mask_blue)
        
        
        
        # CREATE AQUA 
        result_aqua = createMaskImage(src, np.array([70, 45, 45]), np.array([100,255,255]))
        #lower_aqua = np.array([70, 45, 45])
        #upper_aqua = np.array([100,255,255])
        #mask_aqua = cv.inRange(hsv, lower_aqua, upper_aqua)
        #result_aqua = cv.bitwise_and(src, src, mask=mask_aqua)
        
        #findCoords(colorObjectList, src, result_aqua[0], 'aqua')
        
        #coords_aqua = [findObjectOnCamera(src, mask_aqua), 'aqua']
        #print("Aqua coords is", coords_aqua)
        #if coords_aqua[0] != None:
        #    colorObjectList.append(coords_aqua)
        #print("color object list", colorObjectList)
        #colorObjectList.append([findObjectOnCamera(src, mask_aqua), 'aqua'])
        #print("aqua's coordinates are ",coordinates)
        
        # CREATE GREEN
        result_green = createMaskImage(src, np.array([40,40,40]), np.array([70,255,255]))
        #lower_green = np.array([40,40,40])
        #upper_green = np.array( [70,255,255])
        #mask_green = cv.inRange(hsv, lower_green, upper_green)
        #result_green = cv.bitwise_and(src, src, mask=mask_green)
        
        #findCoords(colorObjectList, src, result_green[0], 'green')
        
        # coords_green = [findObjectOnCamera(src, mask_green), 'green']
        # print("Green coords is", coords_green)
        # if coords_green[0] != None:
            # colorObjectList.append(coords_green)
        # print("color object list", colorObjectList)
        #findObjectOnCamera(src, mask_green)
        
        # CREATE PURPLE - Work on range
        result_purple = createMaskImage(src, np.array([130, 50, 50]), np.array([140, 255, 255]))
        #lower_purple = np.array([130, 0, 50])
        #upper_purple = np.array([160, 255, 255])
        #mask_purple = cv.inRange(hsv, lower_purple, upper_purple)
        #result_purple = cv.bitwise_and(src, src, mask=mask_purple)
        
        #findCoords(colorObjectList, src, result_purple[0], 'purple')
        
        #findObjectOnCamera(src, mask_purple)
        #findCoords(colorObjectList, src, mask_purple, 'purple')
        # coords_purple = [findObjectOnCamera(src, mask_purple), 'purple']
        # print("Green coords is", coords_green)
        # if coords_purple[0] != None:
            # colorObjectList.append(coords_purple)
        # print("color object list", colorObjectList)
        #findObjectOnCamera(src, mask_purple)
        
        # CREATE RED
        result_red = createMaskImage(src, np.array([0, 100, 50]), np.array([10, 255, 255]))
        #lower_red = np.array([0, 80, 50])
        #upper_red = np.array([10, 255, 255])
        #mask_red = cv.inRange(hsv, lower_red, upper_red)
        #result_red = cv.bitwise_and(src, src, mask=mask_red)
        
        #findCoords(colorObjectList, src, result_red[0], 'red')
        
        #findCoords(colorObjectList, src, result_green[0], 'green')
        #findObjectOnCamera(src, mask_red)
        
        # CREATE ORANGE
        result_orange = createMaskImage(src, np.array([5, 140, 0]), np.array([25, 255, 255]))
        #lower_orange = np.array([10, 50, 20])
        #upper_orange = np.array([25, 255, 255])
        #mask_orange = cv.inRange(hsv, lower_orange, upper_orange)
        #result_orange = cv.bitwise_and(src, src, mask=mask_orange)
        
        #findCoords(colorObjectList, src, result_orange[0], 'orange')
        
        #findObjectOnCamera(src, mask_orange)
        
        # CREATE YELLOW
        result_yellow = createMaskImage(src, np.array([25, 93, 0]), np.array([45, 255, 255]))
        #lower_yellow = np.array([25, 93, 0])
        #upper_yellow = np.array([45, 255, 255])
        #mask_yellow = cv.inRange(hsv, lower_yellow, upper_yellow)
        #result_yellow = cv.bitwise_and(src, src, mask=mask_yellow)
        
        #findCoords(colorObjectList, src, result_yellow[0], 'yellow')
        
        #findObjectOnCamera(src, mask_yellow)
        
        #CREATE PINK
        result_pink = createMaskImage(src, np.array([160, 0, 50]), np.array([180, 255, 255]))
        #lower_pink = np.array([160, 0, 50])
        #upper_pink = np.array([180, 255, 255])
        #mask_pink = cv.inRange(hsv, lower_pink, upper_pink)
        #result_pink = cv.bitwise_and(src, src, mask=mask_pink)
        
        #findCoords(colorObjectList, src, result_pink[0], 'pink')
        
        #findObjectOnCamera(src, mask_pink)
        
        #lower_stump = np.array([0, 0, 0])
        #upper_stump = np.array([360, 255, 50])
        #mask_stump = cv.inRange(hsv, lower_stump, upper_stump)
        #result_stump = cv.bitwise_and(src, src, mask=mask_stump)
        
        
        #lower_wall = np.array([0, 0, 230])
        #upper_wall = np.array([180, 25, 255])
        #mask_wall = cv.inRange(hsv, lower_wall, upper_wall)
        #result_wall = cv.bitwise_and(src, src, mask=mask_wall)
        
        #lower_tree = np.array([224, 30, 20])
        #upper_tree = np.array([330, 13, 6])
        #mask_tree = cv.inRange(hsv, lower_tree, upper_tree)
        #result_tree = cv.bitwise_and(src, src, mask=mask_tree)
        
        cv.imwrite('resultImgBlue.jpg',result_blue)
        cv.imwrite('resultImgAqua.jpg',result_aqua)
        cv.imwrite('resultImgGreen.jpg',result_green)
        cv.imwrite('resultImgPurple.jpg',result_purple)
        cv.imwrite('resultImgRed.jpg',result_red)
        cv.imwrite('resultImgOrange.jpg',result_orange)
        cv.imwrite('resultImgYellow.jpg',result_yellow)
        cv.imwrite('resultImgPink.jpg',result_pink)
        #cv.imwrite('resultImgStump.jpg',result_stump)
        cv.imwrite('resultImgWall.jpg',result_wall)
        #cv.imwrite('resultImgTree.jpg',result_tree)
        cv.imwrite('retBlack.jpg',thresh_black)
        cv.imwrite('retWhite.jpg',res)
        cv.imwrite('retdarkWall.jpg',thresh_darkWall)
            
        
        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to complete the turn
                 
        #if i==300
            # i = 0
        
        #i+=1
        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
