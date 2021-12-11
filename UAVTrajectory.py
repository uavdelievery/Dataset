""" BlueSky plugin template. The text you put here will be visible
    in BlueSky as the description of your plugin. """

import pandas as pd
import math
import numpy as np
from bluesky import stack, settings, navdb, traf, sim, scr, tools
from bluesky.core import Entity, timed_function
from bluesky.traffic.windsim import WindSim

from bluesky.tools.aero import ft
from bluesky.tools import geo, areafilter
from bluesky.traffic.performance.openap import coeff, thrust, phase


### Initialization function of your plugin. Do not change the name of this
### function, as it is the way BlueSky recognises this file as a plugin.
def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'P_TEST',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        # Update interval in seconds. By default, your plugin's update function(s)
        # are called every timestep of the simulation. If your plugin needs less
        # frequent updates provide an update interval.
        #'update_interval': 0.0

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        #'update':          update

        # If your plugin has a state, you will probably need a reset function to
        # clear the state in between simulations.
        #'reset':         reset
        }

    # init_plugin() should always return these two dicts.
    return config#, stackfunctions



class myPos:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon

    def get_lat_str(self):
        return str(self.lat)

    def get_lon_str(self):
        return str(self.lon)


class myTrajectory:
    def __init__(self,takeoff,landing):
        self.takeoff=takeoff
        self.landing=landing


### Update function. Stop simulation when 0 UAVs are running or if the time exceed TOT seconds
@timed_function(name='QUITALL', dt=1.0)
def update():
    activeUAVs = traf.ntraf
    if activeUAVs == 0:
        stack.stack("QUIT")
    if sim.simt > 8000:
        stack.stack("QUIT")

### Specify log files components, with interval of 1 second
def myLog():
    stack.stack("ECHO Creating log file...")
    logFile="FULL---SIMOUTPUTLOG"+"_"+str(UAVspeed)+"_"+str(myWindDir)+"-"+str(myWindSpd)
    stack.stack("CRELOG "+logFile+" 1")
    variableList=["id","type","lat" ,"lon" ,"alt","tas","cas","vs","gs","distflown","Temp","trk","hdg","p","rho","perf.thrust","perf.drag","perf.phase","perf.fuelflow"]#, "g_test.myWindDir"]
    stack.stack(logFile+" add "+" ".join(variableList) ) #add variables to be printedin output
    stack.stack(logFile+ " ON")
    stack.stack("ECHO log file create!")

### Wind converter and setter with speed in m/s
def wind(mathWindDir, speed):
    stack.stack("ECHO wind initialization...")
    div,mod=divmod(-mathWindDir+270,360)
    speedKn=speed/0.514444
    print(speedKn)
    wind = WindSim()
    wind.add(52,4,150,mod,speedKn)

### Format the string to create an UAV
def strFormatter(index, UAVt, direction, altitude, speed, trajectory):
    speedKn=speed/0.514444
    #id=str(index)+"-"+UAVt
    string="CRE " + str(index) + " " + UAVt + " " + trajectory.takeoff.get_lat_str() + " " + trajectory.takeoff.get_lon_str() \
           + " " + str(direction) + " " + str(altitude) +" " + str(speedKn)
    return string

### New trajectory
def newTrajectory(tlat,tlon,llat,llon):
    takeoff=myPos(tlat,tlon)
    landing=myPos(llat,llon)
    return myTrajectory(takeoff,landing)

### Load more external plugins
def extPlugin(name):
    stack.stack("ECHO loading required plugins:" + name)
    stack.stack("plugin load "+name)
    stack.stack("ECHO plugins loaded...")

### Find initial direction for an UAV
def findDirection(latA,lonA,latB,lonB):
    dL = lonB - lonA
    X = np.cos(latB) * np.sin(dL)
    Y = np.cos(latA) * np.sin(latB) - np.sin(latA) * np.cos(latB) * np.cos(dL)
    bearing = np.arctan2(X, Y)
    return 360 - ((np.degrees(bearing) + 360) % 360)

### Find initial direction for an UAV
def findDirection1(latA,lonA,latB,lonB):
    X = np.abs(lonA - lonB)

    Y = np.log(np.tan(latB/2 + np.pi/4)/np.tan(latA/2+np.pi/4))
    bearing = np.arctan2(X, Y)
    return 360 - ((np.degrees(bearing) + 360) % 360)


def loadSimulation():
    stack.stack("PAN 4.666473 -74.120218")
    stack.stack("ZOOM 50")
    stack.stack("HOLD")
    for row in shortDf.itertuples():
        trajectory=newTrajectory(row.Latitude,row.Longitude,row.ClientLatitude,row.ClientLongitude)
        hdg=findDirection(row.Latitude,row.Longitude,row.ClientLatitude,row.ClientLongitude)
        #hdg = 90.67
        strFlight=strFormatter(row.Index,UAVtype, hdg, UAVAlt, UAVspeed, trajectory) # 30 knots= 15,43 m/s -- 38,8769knots= 20 m/s
        stack.stack(strFlight)
        stack.stack("HOLD")
        stack.stack("ALT, " + str(row.Index) + ", 100,  10")
        stack.stack(str(row.Index) +" DEST," + trajectory.landing.get_lat_str() + " " + trajectory.landing.get_lon_str())
        stack.stack(str(row.Index) +" ATDIST," + trajectory.landing.get_lat_str() + " " + trajectory.landing.get_lon_str() + ",0.06, ALT " + str(row.Index) + ", 0, 10") # NB: 0.005 NM = 10meters
        stack.stack(str(row.Index) +" ATALT, 0 , DEL, " + str(row.Index) )
        #stack.stack("LINE l"+str(row.Index)+" "+str(row.Latitude)+ " " +str(row.Longitude)+ " " +str(row.ClientLatitude) + " " +str(row.ClientLongitude))  # Creates a line source -- dest
        #stack.stack("ECHO " + str(row.Index))
        break
    stack.stack("ECHO finish creating ALL UAVs")
    stack.stack("OP")



### Body of the file
dataFrame=pd.read_csv("C:\\Users\\Bhumika\\Documents\\restdata.csv") # your file folder
#print(dataFrame["Distance_mts"])
shortDf = dataFrame.drop(dataFrame[dataFrame["Distance_mts"] > 5000].index) # Drop all row too distance for a UAV
#print(shortDf)
shortDf.drop(shortDf.tail(6800).index, inplace=True) # drop last n rows
stack.stack("ECHO Starting custom simulation...")


### Global variables
myWindDir=270
myWindSpd=10 # m/s
UAVspeed=20       # m/ s ->30 knots= 15,43 m/s -- 38,8769knots= 20 m/s
UAVAlt=10        # old was 20
UAVtype="M600"

wind(myWindDir,myWindSpd)
myLog()
if shortDf.empty:
    print("---------- EMPTY DF ----------")
    stack.stack("quit")
loadSimulation()
