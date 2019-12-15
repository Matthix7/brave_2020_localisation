#!/usr/bin/env python2
############################################################
# Localisation simulation7: 
#     mobile: tank style
#     landmarks: 3 exactly
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#     noise: yes
#     limited range: no
# Remarks: Solved issue mentioned in previous version
#          Static computing: does not consider knowledge from t(k-1) to compute position at t(k).
############################################################     

from vibes import *
from pyibex import *
from pyibex.geometry import SepPolarXY

from roblib import *

from numpy import pi, array, asarray, zeros, ones, uint8, arange
from math import factorial
import random
from itertools import permutations
from time import time
import cv2

from simu6_intervals import compute_gathered_positions

if __name__ == "__main__":
##################################################################################################
#########################         Setup section          #########################################

    # Field of research
    field_x_low, field_x_high = 0, 150
    field_y_low, field_y_high = 0, 150
    field = [field_x_low, field_x_high, field_y_low, field_y_high]
    
    range_of_vision = oo
    
    # Wanted accuracy on position
    pos_wanted_accuracy = 0.3
    
    # Time step
    dt = 0.7              # Integration step
    slowing_ratio = 3
    
    
    # Landmarks
    landmarks = [[35, 70], [75,95], [25,120]]
    
    
    # Initial state of the boat
    Xinit = array([[30], [30], [1], [5]])



##################################################################################################
#########################   IA computation functions     #########################################

def anonymise_measures(landmarks, bearings, detection_range):
    anonymised_distances = [Interval(0,detection_range)] * len(landmarks)*factorial(len(landmarks))
    anonymised_angles = bearings*factorial(len(landmarks))
    anonymised_marks = list(permutations(landmarks))
    random.shuffle(anonymised_marks)
    return anonymised_marks, anonymised_angles, anonymised_distances 


##################################################################################################
################################  SIMULATION FUNCTIONS   #########################################

# Evolution equation
def evolX(X, u):
    """Returns the derivative of X."""
    X, u = X.flatten(), u.flatten()
    x, y, theta, v = X[0], X[1], X[2], X[3]
    
    dX = array([[v*cos(theta)], [v*sin(theta)], [u[1]], [u[0]]])
    
    return dX


def getBearings(X, landmarks, accuracy = pi/180*5):
    """Returns the bearing for the designated landmark."""
    x, y = X[0, 0], X[1, 0]
    bearings = []
    for landmark in landmarks:
        xl, yl = landmark[0], landmark[1]        
        camera_measure = sawtooth(atan2(yl-y, xl-x) - X[2,0])
        compass_measure = X[2,0]
        bearings.append(Interval(camera_measure+compass_measure-accuracy, camera_measure+compass_measure+accuracy))
    return bearings


def inRange(X, landmarks):
    """Returns the list of the landmarks that the boat is really able to see."""
    perceptionAngle = 45  #degrees
    spotted = []
    for i in range(len(landmarks)):
        dist = norm(X[:2]-landmarks[i])
        if dist < 15 and abs(getBearing(X, landmarks[i])) < perceptionAngle*pi/180:
            spotted.append(landmarks[i])
    return spotted



##################################################################################################
##################################  Simulation    ################################################


# Main loop
if __name__ == "__main__":

    # Initialising variables
    X = Xinit
    cv2.namedWindow("Test", cv2.WINDOW_NORMAL)

    for t in arange(0, 30, dt):

##################################################################################################
############################  Simulate the measure    ############################################
        bearings = getBearings(X, landmarks)        
        anonymised_marks, anonymised_angles, anonymised_distances = anonymise_measures(landmarks, bearings, range_of_vision)        
        
        
        ## Performances assessment
        t0 = time()
        
        ## Drawing
        vibes.beginDrawing()
        vibes.newFigure("Localization")
        vibes.setFigureProperties({'x':800, 'y':100, 'width':800, 'height': 800})        
        vibes.axisLimits(field_x_low, field_x_high, field_y_low, field_y_high)   
        
##################################################################################################
######################### Compute all possible positions #########################################        
        # Set constraints        
        P = IntervalVector([[field_x_low, field_x_high], [field_y_low, field_y_high]])
        separators = []
        for i in range(len(anonymised_marks)):
            seps =[]
            for m,d,alpha in zip(anonymised_marks[i], anonymised_distances[3*i:3*(i+1)], anonymised_angles[3*i:3*(i+1)]):
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
            inner, outer, frontier = pySIVIA(P, sep, 4*pos_wanted_accuracy, draw_boxes = False)
            inner_boxes += inner
            outer_boxes += outer
            frontier_boxes += frontier


        ## Performances assessment
        t1 = time()

        ## Drawing
            
        for box in frontier_boxes:
            vibes.drawBox(box[0][0], box[0][1], box[1][0], box[1][1], color='black[yellow]')
        for box in inner_boxes:
            vibes.drawBox(box[0][0], box[0][1], box[1][0], box[1][1], color='black[red]')
            
        vibes.drawCircle(X[0,0], X[1,0], 0.2, 'blue[black]') 
        for m, b in zip(landmarks, bearings):
            vibes.drawCircle(m[0], m[1], 1, 'yellow[black]')  
            vibes.drawPie((X[0,0],X[1,0]), (0.,35.), (b[0],b[1]), color='black',use_radian=True) 
        vibes.endDrawing()
           
##################################################################################################
############################     Gather in areas      ############################################

        
        map = zeros((int((field_y_high-field_y_low)/pos_wanted_accuracy), int((field_x_high-field_x_low)/pos_wanted_accuracy)), uint8)
        map, possible_positions =  compute_gathered_positions(map, inner_boxes, field, pos_wanted_accuracy)
        
        
        ## Performances assessment
        print
        print "########   "+str(t)+"  ########" 
        print "Temps de calcul: "
        print t1-t0
        print "Temps de rassemblement: "
        print time()-t1
        
        ## Display
        cv2.imshow("Test", cv2.flip(map, 0))
        
        key = cv2.waitKey(int(1000*slowing_ratio*dt))
        if key & 0xFF == 27:    
            cv2.destroyAllWindows()
            break
        elif  key & 0xFF == 32: 
            pause(0.5)
            key = cv2.waitKey(1)
            while key & 0xFF != 32:    
                pause(0.2)
                key = cv2.waitKey(1)



##################################################################################################
################################     Evolution      ##############################################

        # command
        u = array([[0.], [0.0]])  #(speed_command, rotation_command)

        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        
        # end display
#        pause(slowing_ratio* dt)





