#!/usr/bin/env python2
#available at https://www.ensta-bretagne.fr/jaulin/roblib.py 
# For help : https://www.ensta-bretagne.fr/jaulin/python.html  
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/kalmooc.html
# used in RobMOOC :  https://www.ensta-bretagne.fr/jaulin/robmooc.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/inmooc.html


import numpy as np
import matplotlib.pyplot as plt
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round, matmul
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag

from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial, atan2
from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection


def init_figure(xmin,xmax,ymin,ymax): 
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')   
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def draw_sailboat(x,delta_s,delta_r,psi,awind):
    x=x.flatten()
    theta=x[2]
    hull=array([[-0.25,1.25,1.75,1.75,1.25,-0.25,-0.25,-0.25],[-0.5,-0.5,-0.25,0.25,0.5,0.5,-0.5,-0.5],[1,1,1,1,1,1,1,1]])
    sail=np.array([[-1.75,0],[0,0],[1,1]])
    rudder=np.array([[-0.25,0.25],[0,0],[1,1]])
    R=np.array([[cos(theta),-sin(theta),x[0]],[sin(theta),cos(theta),x[1]],[0,0,1]])
    Rs=np.array([[cos(delta_s),-sin(delta_s),0.25*3],[sin(delta_s),cos(delta_s),0],[0,0,1]])
    Rr=np.array([[cos(delta_r),-sin(delta_r),0.25*(-1)],[sin(delta_r),cos(delta_r),0],[0,0,1]])
    R1 = np.matmul(R, hull)
    R2 = np.matmul(np.matmul(R, Rs), sail)
    R3 = np.matmul(np.matmul(R, Rr), rudder)
    plot2D(R1,'black')
    plot2D(R2,'red',2)
    plot2D(R3,'red',2)
    draw_arrow(x[0]+5,x[1],psi,5*awind,'blue')

def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w)

def draw_arrow(x,y,theta,L,col):
    e=0.2
    M1=L*np.array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M=np.append(M1,[[1,1,1,1,1]],axis=0)
    R=np.array([[cos(theta),-sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    plot2D(np.matmul(R, M),col)

def draw_disk(c,r,ax,col): 
    #draw_disk(array([[1],[2]]),0.5,ax,"blue")
    e = Ellipse(xy=c, width=2*r, height=2*r, angle=0)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.7)
    e.set_facecolor(col)

def sawtooth(x):
    return (x+pi)%(2*pi)-pi   # or equivalently   2*arctan(tan(x/2))
