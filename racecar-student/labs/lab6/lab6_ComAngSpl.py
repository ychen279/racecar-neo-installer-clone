"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab6.py

Title: Lab 6 - Wall Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: This script provides the RACECAR with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the RACECAR should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The RACECAR may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a 
Python file in the RACECAR sim. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: Test your code using the level "Neo Labs > Lab 6: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../../library')
import racecar_core
import numpy as np
import scipy.signal as signal

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
frontHalfAngle = 125. #degree to sample lidar
errBuf = []
bufLen = 10
Kp=3.0
Ki=0
Kd=0
speed = 1.0
peakWidThres = 5 #Minimum three degree width to be classify as a peak
devCount = 30 #Deviation Counter or +- 22.5 deg deviation 

# Create a angle buffer
angBufLen = 10 #Store the past five stamps' angles
angBuf = [0.0]*angBufLen

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    rc.drive.set_speed_angle(0, 0)
    # samples = rc.lidar.get_samples()
    # FindFarDistAngle(samples)


def FindFarDistAngle(lidarSample,frontHalfAngle=125., peakWidThres=5, devCount=30, angBuf=[0.0]*10):
    #Empty up the buffer angles
    angBuf.pop(0)
    #Expect this input lidarSample = rc.lidar.get_samples()
    samples = np.array(lidarSample)
    angles = np.linspace(0, 360, len(samples)+1)[0:-1]
    if len(samples[samples==0])>0:
        samples[samples==0] = np.max(samples) #Set undetective region to maximum distance
    samplesFront = np.concatenate((samples[angles>=(360-frontHalfAngle)],samples[angles<=(frontHalfAngle)]))
    anglesFront = np.concatenate((angles[angles>=(360-frontHalfAngle)]-360,angles[angles<=(frontHalfAngle)]))
    # Create a buffer for peak identification
    samplesFront[0] *= 0.98 #Create two notches at the edges for peak identification
    samplesFront[-1] *= 0.98
    # Find the max distance angle
    idxPeaks, _ = signal.find_peaks(samplesFront,width=peakWidThres) #int List of indices for peak locations
    if len(idxPeaks)<=0: #Extrapolation Inertial Guidance
        angBuf.append(np.mean(angBuf))
        return np.mean(angBuf)
    widPeaks = signal.peak_widths(samplesFront,idxPeaks)[0] #List of size of these peaks in degrees
    idxPeaks = np.array(idxPeaks)
    widPeaks = np.array(widPeaks) #This is actually effectively the index range (Not the angle)
    disPeaks = samplesFront[idxPeaks]
    angPeaks = anglesFront[idxPeaks]
    # Integrate arcLength of each peak
    idxRanPeaksLow = np.clip((idxPeaks - 0.5*widPeaks).astype(int),0,len(anglesFront)-1)
    idxRanPeaksHig = np.clip((idxPeaks + 0.5*widPeaks).astype(int),0,len(anglesFront)-1)
    arcLenPeaks = []
    for i in range(len(idxRanPeaksLow)):
        arcLenPeaks.append(np.sum(samplesFront[idxRanPeaksLow[i]:idxRanPeaksHig[i]+1])*(2*np.pi/len(angles))) #cm
    arcLenPeaks = np.array(arcLenPeaks)
    # Determine a good angle to go with
    idxMaxArc =  idxPeaks[np.argmax(arcLenPeaks)] #index of maximum arc length
    widMaxArc =  int(widPeaks[np.argmax(arcLenPeaks)]) #number of indices across maximum arc length
    # Determine discontinuity around the max arc
    idxRanMaxArcLow = np.clip(idxMaxArc-devCount,0,len(anglesFront)-1)
    idxRanMaxArcHig = np.clip(idxMaxArc+devCount,0,len(anglesFront)-1)
    disMaxArcLow = np.mean(samplesFront[idxRanMaxArcLow:idxMaxArc]) #Average Left Peak Distance
    disMaxArcHig = np.mean(samplesFront[idxMaxArc+1:idxRanMaxArcHig+1])#Average Right Peak Distance
    # Compute percentage discontinuity
    if np.max([disMaxArcLow,disMaxArcHig]) > 0.0:
        difDisMaxArc = np.abs(disMaxArcLow-disMaxArcHig)/np.max([disMaxArcLow,disMaxArcHig]) #(0,1) if 1 for very stiff cliff
    else:
        difDisMaxArc = 0.0 #Default no difference
    # Compute final angle requirement
    angMaxArcTune = anglesFront[idxMaxArc]
    devAngle = anglesFront[idxRanMaxArcHig]-anglesFront[idxRanMaxArcLow]
    if disMaxArcHig>disMaxArcLow: #Apply cliffing correction
        angMaxArcTune += 0.5*devAngle*difDisMaxArc
    else:
        angMaxArcTune -= 0.5*devAngle*difDisMaxArc
    # Update the angle buffer
    angBuf.append(angMaxArcTune)
    # printing
    print("angle buffer",angBuf)
    print("idxPeaks",idxPeaks)
    print("widPeaks",widPeaks)
    print("disPeaks",disPeaks)
    print("angPeaks",angPeaks)
    print("arcLenPeaks",arcLenPeaks)
    print("angMaxArc Untuned",anglesFront[idxMaxArc])
    print("Cliff Difference",difDisMaxArc)
    print("angMaxArc Tuned",angMaxArcTune)
    return np.mean(angBuf)


def PID(errN,errBuf,Kp=0,Ki=0,Kd=0,bufLen=10):
    #Positive Error output Positive Control
    #Create Error Buffer for I controller
    if (bufLen < 5):
        bufLen = 5 #Minimum Buffer for the damping
    if (len(errBuf)>=bufLen):
        errBuf.pop(0)
        errBuf.append(errN)
    else:
        errBuf.append(errN)
        
    # P Control
    P = Kp * errN
    # I Control
    I = Ki * sum(errBuf) 
    # D Control
    if (len(errBuf)>=5):
        D = Kd * (1/12) * (-errBuf[-1]+8*errBuf[-2]-8*errBuf[-4]+errBuf[-5])
    else:
        D = 0.0
    
    return P+I+D


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    #Read gloabl parameters
    global frontHalfAngle, errBuf, bufLen, Kp, Ki, Kd, speed, peakWidThres, devCount, angBuf
    #Read Lidar Data
    samples = rc.lidar.get_samples()
    #Find Target Angle
    angTar = FindFarDistAngle(samples,frontHalfAngle=frontHalfAngle,peakWidThres=peakWidThres, devCount=devCount, angBuf=angBuf)
    #Feed into PID Angle Decision
    errAngN = angTar/(frontHalfAngle) #normalized error(-1,1) -(left) and +(right)
    contAng = PID(errAngN,errBuf,Kp=Kp,Ki=Ki,Kd=Kd,bufLen=bufLen)
    contAng = np.clip(contAng,-1,1)
    #Implement Action
    rc.drive.set_speed_angle(speed, contAng)
    return 0

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
