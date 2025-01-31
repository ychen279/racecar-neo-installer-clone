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
frontHalfAngle = 100 #degree to sample lidar
errBuf = []
bufLen = 10
Kp=1.5
Ki=0.0
Kd=10
speed = 1.0
peakWidThres = 5 #Minimum three degree width to be classify as a peak
devSearchCount = 20 #20 #Deviation Counter or +- 22.5 deg deviation 
devAngleMax = 40 #degree

# Create a angle buffer
angBufLen = 10 #Store the past five stamps' angles
angBuf = [0.0]*angBufLen
angTurnLook = 40 #10. #degree(Maximum turning for searching)

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global frontHalfAngle, peakWidThres, devSearchCount, devAngleMax, angBuf, angTurnLook
    rc.drive.set_speed_angle(0, 0)
    ## Test Print below ##
    # samples = rc.lidar.get_samples()
    # FindFarDistAngle(samples,frontHalfAngle=frontHalfAngle, peakWidThres=peakWidThres, devSearchCount=devSearchCount, devAngleMax=devAngleMax, angBuf=angBuf, angTurnLook=angTurnLook)


def FindFarDistAngle(lidarSample,frontHalfAngle=100., peakWidThres=5, devSearchCount=20, devAngleMax=40, angBuf=[0.0]*10, angTurnLook=40.):
    #Prediction of next angle through extrapolation
    angGusNext = np.clip(np.mean(angBuf),-frontHalfAngle-angTurnLook,frontHalfAngle+angTurnLook) #Use mean to estimate a likely angle
    #Compute turning of the searching angle
    angTurnLook_Cur = angTurnLook*(angGusNext/(frontHalfAngle+angTurnLook)) #[deg]
    #Read in lidar data
    samples = np.array(lidarSample)
    angles = np.linspace(0, 360, len(samples)+1)[0:-1]
    #Clean up the no data region
    if len(np.sum(samples==0))>0:
        isnan_mask = np.isnan(np.where(samples != 0, samples, np.nan))  # Replace zeros with NaN
        samples[isnan_mask] = np.interp(angles[isnan_mask], angles[~isnan_mask], samples[~isnan_mask])
    #Cut out the frontal region
    samplesFront = np.concatenate((samples[angles>=(360-frontHalfAngle+angTurnLook_Cur)],samples[angles<=(frontHalfAngle+angTurnLook_Cur)]))
    anglesFront = np.concatenate((angles[angles>=(360-frontHalfAngle+angTurnLook_Cur)]-360,angles[angles<=(frontHalfAngle+angTurnLook_Cur)]))
    # Create a buffer for peak identification
    samplesFront[0] *= 0.98 #Create two notches at the edges for peak identification
    samplesFront[-1] *= 0.98
    # Find the max distance angle
    idxPeaks, _ = signal.find_peaks(samplesFront,width=peakWidThres) #int List of indices for peak locations
    if len(idxPeaks)<=0: #Extrapolation Inertial Guidance
        angBuf.pop(0) #Empty up a space for the current time step
        angBuf.append(angGusNext)
        return angGusNext
    widPeaks = signal.peak_widths(samplesFront,idxPeaks)[0] #List of size of these peaks in degrees
    idxPeaks = np.array(idxPeaks)
    widPeaks = np.array(widPeaks) #This is actually effectively the index range (Not the angle)
    disPeaks = samplesFront[idxPeaks]
    angPeaks = anglesFront[idxPeaks]
    # Integrate arcLength of each peak
    arcLenPeaks = widPeaks*disPeaks
    # Compute angle weighting
    angWeis = 1-np.abs(angGusNext-angPeaks)/(2*(frontHalfAngle+angTurnLook))
    arcLenPeaksWeighted = arcLenPeaks*(1+angWeis)
    # Determine a good angle to go with
    idxMaxArc =  idxPeaks[np.argmax(arcLenPeaksWeighted)] #index of maximum arc length
    widMaxArc =  int(widPeaks[np.argmax(arcLenPeaksWeighted)]) #number of indices across maximum arc length
    # Determine discontinuity around the max arc
    idxRanMaxArcLow = np.clip(idxMaxArc-devSearchCount,0,len(anglesFront)-1)
    idxRanMaxArcHig = np.clip(idxMaxArc+devSearchCount,0,len(anglesFront)-1)
    disMaxArcLow = np.mean(samplesFront[idxRanMaxArcLow:idxMaxArc]) #Average Left Peak Distance
    disMaxArcHig = np.mean(samplesFront[idxMaxArc+1:idxRanMaxArcHig+1])#Average Right Peak Distance
    # Compute percentage discontinuity
    if np.max([disMaxArcLow,disMaxArcHig]) > 0.0:
        difDisMaxArc = np.abs(disMaxArcLow-disMaxArcHig)/np.max([disMaxArcLow,disMaxArcHig]) #(0,1) if 1 for very stiff cliff
    else:
        difDisMaxArc = 0.0 #Default no difference
    # Compute final angle requirement
    angMaxArcTune = anglesFront[idxMaxArc]
    if disMaxArcHig>disMaxArcLow: #Apply cliffing correction
        angMaxArcTune += devAngleMax*difDisMaxArc
    else:
        angMaxArcTune -= devAngleMax*difDisMaxArc
    # Output
    angBuf.pop(0) #Empty up a space for the current time step
    angBuf.append(angMaxArcTune)
    # printing
    print("angle buffer",angBuf)
    print("guessed angle next",angGusNext)
    print("angTurnLook_Cur\n",angTurnLook_Cur)
    print("idxPeaks",idxPeaks)
    print("widPeaks",widPeaks)
    print("disPeaks",disPeaks)
    print("angPeaks\n",angPeaks)
    print("arcLenPeaks",arcLenPeaks)
    print("angWeis",angWeis)
    print("arcLenPeaksWeighted\n",arcLenPeaksWeighted)
    print("angMaxArc Untuned",anglesFront[idxMaxArc])
    print("Cliff Difference",difDisMaxArc)
    print("Cliff Left and Right",disMaxArcLow," and ",disMaxArcHig)
    print("angMaxArc Tuned",angMaxArcTune)
    return angMaxArcTune


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
    # #Read gloabl parameters
    global errBuf, bufLen, Kp, Ki, Kd, speed
    global frontHalfAngle, peakWidThres, devSearchCount, devAngleMax, angBuf, angTurnLook
    #Read Lidar Data
    samples = rc.lidar.get_samples()
    #Find Target Angle
    angTar = FindFarDistAngle(samples,frontHalfAngle=frontHalfAngle, peakWidThres=peakWidThres, devSearchCount=devSearchCount, devAngleMax=devAngleMax, angBuf=angBuf, angTurnLook=angTurnLook)
    #Feed into PID Angle Decision
    errAngN = angTar/(frontHalfAngle+angTurnLook) #normalized error(-1,1) -(left) and +(right)
    contAng = PID(errAngN,errBuf,Kp=Kp,Ki=Ki,Kd=Kd,bufLen=bufLen)
    contAng = np.clip(contAng,-1,1)
    #Implement Action
    rc.drive.set_speed_angle(speed, contAng)
    return 0
    # pass

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
