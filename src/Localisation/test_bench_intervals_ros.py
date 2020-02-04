#!/usr/bin/env python2
############################################################
# Localisation simulation12: 
#     mobile: tank style
#     landmarks: any
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#         speed
#     noise: yes
#     limited range: yes
# Remarks: Relies on (noisy) measure of speed. Would be better to get rid of it if possible.
#          No measure of distance to different landmarks
#          Real time version, not a simulation
############################################################     

import rospy

from pyibex import *
from pyibex.geometry import SepPolarXY
from vibes import *

from roblib import *

from numpy import pi, array, asarray, zeros, ones, uint8, arange
from math import factorial
import random
from itertools import permutations
from time import time, sleep
import cv2

from std_msgs.msg import Float32, String

from gps_converter import get_local_coordinates

##################################################################################################
#########################   IA computation functions     #########################################


def compute_all_positions(X_boxes, azimuths, pos_wanted_accuracy, speed, heading, lateral_speed, field, landmarks, range_of_vision, dt):
    if len(azimuths) != 0:            
        # when at least one mark is detected, improve position estimation
        anonymised_marks, anonymised_angles, anonymised_distances = anonymise_measures(landmarks, azimuths, range_of_vision)        
        inner_boxes, outer_boxes, frontier_boxes = compute_boxes(field, anonymised_marks, anonymised_distances, anonymised_angles, pos_wanted_accuracy)
        static_estimations = inner_boxes + frontier_boxes
    else:
        static_estimations = [field]
    
    lat_moves = Interval(0.).inflate(lateral_speed)

    f_evol = Function("x[2]", "(x[0] + %f*(%s*cos(%s) + %s*sin(%s)), x[1] + %f*(%s*sin(%s) + %s*cos(%s)))"%(2*(dt, str(speed), str(heading), str(lat_moves), str(heading))))
    dynamic_estimations = [f_evol.eval_vector(X_box) for X_box in X_boxes]
    
    # possible positions = field & static estimation & dynamic estimation
    X_boxes = []
    
    print "Static boxes: ", len(static_estimations)
    print "Dynamic boxes: ", len(dynamic_estimations)
    print
    
    if len(static_estimations) > 1: # meaning at least one mark is detected
        dynamic_union = dynamic_estimations[0]  # then we can afford to loose some precision on dynamics given
        for dynamic_box in dynamic_estimations[1:]: # that static estimations are better, in order to reduce computing time
            dynamic_union |= dynamic_box
    
        for static_box in static_estimations:
            intersection_box = field & dynamic_union & static_box
            if not intersection_box.is_empty():
                X_boxes.append(intersection_box)
                
    else: # In case no marks are detected, all estimations are computed based on system dynamics
        for dynamic_box in dynamic_estimations:
            intersection_box = field & dynamic_box
            if not intersection_box.is_empty():
                X_boxes.append(intersection_box)
    return X_boxes



def compute_boxes(field, anonymised_marks, anonymised_distances, anonymised_angles, pos_wanted_accuracy):
    # Set constraints 
    separators = []
    for i in range(len(anonymised_marks)):
        seps =[]
        n = len(anonymised_marks[i])
        for m,d,alpha in zip(anonymised_marks[i], anonymised_distances[n*i:n*(i+1)], anonymised_angles[n*i:n*(i+1)]):
            sep = SepPolarXY(d, alpha)
            fforw = Function("v1", "v2", "(%f-v1;%f-v2)" %(m[0], m[1]))
            fback = Function("p1", "p2", "(%f-p1;%f-p2)" %(m[0], m[1]))
            sep = SepTransform(sep, fback, fforw)
            seps.append(sep)
            
        sep = SepQInterProjF(seps)
        sep.q = 0      # How many measures can be considered as outliers. Here we want 3 correct measures.
        separators.append(sep)
    
    inner_boxes, outer_boxes, frontier_boxes = [], [], []
    for sep in separators:
        # Compute all possible positions. Factor 4 in accuracy is due to bissections effects.
        inner, outer, frontier = pySIVIA(field, sep, 4*pos_wanted_accuracy, draw_boxes = False)
        inner_boxes += inner
        outer_boxes += outer
        frontier_boxes += frontier    
    return inner_boxes, outer_boxes, frontier_boxes 



def compute_gathered_positions(inner_boxes, field, pos_wanted_accuracy):    
    # Create a binary map of the positions where the boat can be
    field_x_low, field_y_low = field[0][0], field[1][0]
    field_x_high, field_y_high = field[0][1], field[1][1]
    binary_map = zeros((int((field_y_high-field_y_low)/pos_wanted_accuracy), int((field_x_high-field_x_low)/pos_wanted_accuracy)), uint8)
    for box in inner_boxes :#+frontier_boxes:
        x_low = int((box[0][0]-field_x_low)/pos_wanted_accuracy)   # Translate to begin index at 0 with positive values. 1 pixel = pos_wanted_accuracy m2.
        x_high = int((box[0][1]-field_x_low)/pos_wanted_accuracy)        
        y_low = int((box[1][0]-field_y_low)/pos_wanted_accuracy)
        y_high =  int((box[1][1]-field_y_low)/pos_wanted_accuracy)
        
        binary_map[y_low:y_high, x_low:x_high] = 255*ones((y_high-y_low, x_high-x_low), uint8)    
    im,contours,hierarchy = cv2.findContours(binary_map, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Fit the different areas into rectangles
    possible_positions = []
    for cnt in contours:
        x,y,w,h  = cv2.boundingRect(cnt)
        x, y, w, h = int(x), int(y), int(w), int(h)            
#        cv2.rectangle(binary_map, (x,y),(x+w,y+h),(255,0,0),1)        
        centre_real, shape_real = ((x + w/2)*pos_wanted_accuracy, (y + h/2)*pos_wanted_accuracy), (w*pos_wanted_accuracy, h*pos_wanted_accuracy)
        possible_positions.append([centre_real, shape_real])    
    return binary_map, possible_positions



def anonymise_measures(landmarks, azimuths, range_of_vision):
    anonymised_marks = list(permutations(landmarks, len(azimuths)))
    random.shuffle(anonymised_marks)
    anonymised_angles = azimuths*len(anonymised_marks)
    anonymised_distances = [Interval(0,range_of_vision)]*len(azimuths) *len(anonymised_marks)
    return anonymised_marks, anonymised_angles, anonymised_distances 




##################################################################################################
#############################   ROS callblacks     ###############################################
def sub_heading(data):
    global heading, heading_accuracy
    heading = Interval(data.data).inflate(heading_accuracy)


def sub_speed(data):
    global speed, speed_accuracy
    speed = Interval(data.data).inflate(speed_accuracy)


def sub_directions(data):
    global marks_directions, azimuth_accuracy
    directions = eval(data.data)
    marks_directions = map(lambda x: Interval(x).inflate(direction_accuracy), directions)



def sub_state(data):
    global real_boat_state_vector
    real_boat_state_vector = asarray(eval(data.data))

##################################################################################################
##################################   Main Loop    ################################################
##################################################################################################



def run():
    rospy.init_node("interval_localiser")
    global marks_directions, speed, heading, heading_accuracy, direction_accuracy, speed_accuracy, real_boat_state_vector
##################################################################################################
#########################         Setup section          #########################################

    # integration step
    dt = rospy.get_param('integration_step', 0.2)

    # Field of research
    base_map, field_limits, landmarks = get_local_coordinates()
    field_x_low, field_x_high, field_y_low, field_y_high  = field_limits

    search_field = IntervalVector([[field_x_low, field_x_high], [field_y_low, field_y_high]])
    
    
    # Estimated specifications of sensors
    heading_accuracy = rospy.get_param('heading_accuracy', 5) *pi/180        # Compass accuracy
    direction_accuracy = rospy.get_param('direction_accuracy', 5) *pi/180    # Camera accuracy
    range_of_vision = rospy.get_param('range_of_vision', 15)                 # maximum distance for mark detection
    angle_of_perception = rospy.get_param('angle_of_perception', 50) *pi/180 # angle of perception of the camera
    speed_accuracy = rospy.get_param('speed_accuracy', 0.3)                  # accuracy on speed measurement.
    
    # To adapt the Dubins' model, we add a term that represents lateral variations of position
    max_lateral_speed = rospy.get_param('max_lateral_speed', 0.5)
    
    # Wanted accuracy on position
    pos_wanted_accuracy = rospy.get_param('pos_wanted_accuracy', 0.3)
    
       

    print "######## INIT ##############"
    print "Integration step = ", dt
    print "Search field = ", search_field
    print "Heading acuracy = ", heading_accuracy
    print "Marks direction accuracy = ", direction_accuracy
    print "Range of vision = ", range_of_vision
    print "Angle of perception = ", angle_of_perception
    print "Speed accuracy = ", speed_accuracy
    print "Max lateral speed = ", max_lateral_speed
    print "Wanted accuracy = ", pos_wanted_accuracy
    print
    print "############################"


    # Initialising variables
    boat_possible_positions = [search_field]
    cv2.namedWindow("Possible positions", cv2.WINDOW_NORMAL)
    marks_directions = []
    speed = Interval(0,100)
    heading = Interval(-pi,pi)
    real_boat_state_vector = array([[0,0,0,0]]).T
    sleep(2)
##################################################################################################
#########################      ROS initialisation        #########################################


    pub_positions = rospy.Publisher("boat_possible_positions", String, queue_size = 2)
    pub_local_landmarks = rospy.Publisher("local_landmarks_coordinates", String, queue_size = 2)
    rospy.Subscriber("buoys_directions", String, sub_directions)
    rospy.Subscriber("heading", Float32, sub_heading)
    rospy.Subscriber("speed", Float32, sub_speed)


    rate = rospy.Rate(1/dt)
    rospy.loginfo("Initiated localisation")


##################################################################################################
#################################      Drawing        ############################################

    ## Drawing
    vibes.beginDrawing()
    vibes.newFigure("Localization")
    vibes.setFigureProperties({'x':800, 'y':100, 'width':800, 'height': 800})        
    vibes.axisLimits(field_x_low, field_x_high, field_y_low, field_y_high)   
    rospy.Subscriber("state_truth", String, sub_state)


    while not rospy.is_shutdown():
        
        # 1st interval function
        boat_possible_positions = compute_all_positions(boat_possible_positions, 
                                                        marks_directions, 
                                                        pos_wanted_accuracy,
                                                        speed, 
                                                        heading, 
                                                        max_lateral_speed, 
                                                        search_field, 
                                                        landmarks, 
                                                        range_of_vision, dt)
        

        #2nd interval function
        binary_map, possible_positions = compute_gathered_positions(boat_possible_positions, 
                                                             search_field , 
                                                             pos_wanted_accuracy)

       
        pub_positions.publish(String(data=str(possible_positions)))
        pub_local_landmarks.publish(String(data=str(landmarks)))

##################################################################################################
############################    Drawing     ############################################    

        ## Drawing   
        X = real_boat_state_vector


        print '\n'
        print "Pos truth: ", X[0,0], X[1,0]
        print "Heading truth: ", X[2,0]
        print "Heading interval: ", heading
        print "Speed truth: ", X[3,0]
        print "Speed interval: ", speed
        print '\n'

        vibes.clearFigure()        
        scale = (field_y_high-field_y_low)/100. 
        for X_box in boat_possible_positions:
            vibes.drawBox(X_box[0][0], X_box[0][1], X_box[1][0], X_box[1][1], color='[blue]')
        vibes.drawCircle(X[0,0], X[1,0], 1*scale, 'blue[black]') 
        for b in marks_directions:
            vibes.drawPie((X[0,0],X[1,0]), (0.,range_of_vision), (b[0],b[1]), color='black',use_radian=True)             
        for m in landmarks:
            vibes.drawCircle(m[0], m[1], 1*scale, 'yellow[black]')  


        ## Display binary map
        cv2.imshow("Possible positions", cv2.flip(binary_map, 0))
        cv2.waitKey(1)
        rate.sleep()



if __name__ == "__main__":
    run()