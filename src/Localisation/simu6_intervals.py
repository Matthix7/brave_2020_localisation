#!/usr/bin/env python2
############################################################
# Localisation simulation6: 
#     mobile: immobile
#     landmarks: 3
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#     noise: yes
#     limited range: no
############################################################     

from vibes import *
from pyibex import *
from pyibex.geometry import SepPolarXY

from numpy import pi, array, asarray
from math import factorial
from itertools import permutations

#### Creating observation

Marks = [[35, 70], [75,95], [25,120]]

D = [Interval(0,oo), Interval(0, oo), Interval(0,oo)]

heading = pi/180*60
accuracy = pi/180*3

Alpha = [Interval(heading+0.44-accuracy, heading+0.44+accuracy), 
         Interval(heading+0.-accuracy, heading+0.+accuracy), 
         Interval(heading+0.61-accuracy, heading+0.61+accuracy)]


### Anonymising measures to simulate the fact that buoys are not recognisable
anonymised_marks = Marks + [Marks[1], Marks[2], Marks[0]] + [Marks[2], Marks[0], Marks[1]]

anonymised_distances = D*3

anonymised_angles = Alpha*3




vibes.beginDrawing()
vibes.newFigure("Localization")
vibes.setFigureProperties({'x':130, 'y':130, 'width':800, 'height': 800})



P = IntervalVector([[0,80], [0, 130]])
seps = []
for m,d,alpha in zip(anonymised_marks, anonymised_distances, anonymised_angles):
	sep = SepPolarXY(d, alpha)
	fforw = Function("v1", "v2", "(%f-v1;%f-v2)" %(m[0], m[1]))
	fback = Function("p1", "p2", "(%f-p1;%f-p2)" %(m[0], m[1]))
	sep = SepTransform(sep, fback, fforw)
	seps.append(sep)

sep = SepQInterProjF(seps)
sep.q = len(anonymised_marks) - 3

pySIVIA(P, sep, 0.5)


for m in Marks:
	vibes.drawCircle(m[0], m[1], 1, 'yellow[black]')

vibes.axisEqual()
vibes.endDrawing()
 
