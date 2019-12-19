# -*- coding: utf-8 -*-
"""
Fourier series script for Actuators, Drivers and Electronic Modules exam.
Made my Valdas Druskinis ROB P3 Group 367
"""

from sympy import *
from numpy import pi
import matplotlib.pyplot as plt

def Fourier_Series(n_max):
    t = Symbol('t')
    FS = 0 #Initial FS values is 0
    a0 = 1/pi * integrate(-2*(t**2), (t,-pi,pi)) #Calculating a0 entry
    for N in range(1,n_max+1): #Running the loop for N values from 1 to desired value
        an = 1/pi * integrate(-2*(t**2)*cos(N*t), (t, -pi, pi)) 
        bn = 1/pi * integrate(-2*(t**2)*sin(N*t), (t, -pi, pi))
        FS = (an*cos(N*t)+bn*sin(N*t)) + FS #Summation of all entries for bn and an following the formula
    FS1 = a0/2 + FS # Finished formula, of y(t) = -2t^2. Symbolic values of t
    plot(FS1, (t, -pi, pi), xlabel='N={}'.format(n_max)) #range of t = from -pi to pi
    #print(FS1)
def original():
    t = Symbol('t')
    y = -2*(t**2)
    plot(y,(t, -pi, pi), xlabel= 'y = -2t^2') #original plot of the function y(t)
    
original()
Fourier_Series(1)
Fourier_Series(5)
Fourier_Series(10)




