import numpy as np
import matplotlib.pyplot as plt

class Line(object):
    def __init__(self,start:tuple,end:tuple):
        self.startX=start[0]
        self.startY=start[1]
        
        self.endX=end[0]
        self.endY=end[1]
        
        self.slope=(self.endY-self.startY)/(self.endX-self.startX)
        #print(self.slope)
        self.length=np.sqrt((self.startX-self.endX)**2+(self.startY-self.endY)**2)
        self.theta=np.arctan2(self.endY-self.startY,self.endX-self.startX)
        
    def contains(self,sensor_loc,thresh=10):
        """
        determine whether the sensor array will "see" this array
        sensor array consists of a section of points, X and Y
        
        sensor_loc is a dict with 4 values
        x1,y1,x2,y2 which define the size and location of the sensor array
        
        if it does, it returns the location along the array,
        otherwise returns None
        """
        #sensor is a line, calculate its slope
        sensorSlope=(sensor_loc["y2"]-sensor_loc["y1"])/(sensor_loc["x2"]-sensor_loc["x1"])
                                                          
        #find the x intercept between these two lines, 
        #we have sensorSlope*(x-sensor_loc["x1"])+sensor_loc["y1"]=lineSlope(x-startX)+startY
                                                         
                                                         
        x_intercept=(self.startY-sensor_loc["y1"]-self.startX*self.slope+sensor_loc["x1"]*sensorSlope)/\
                                                       (sensorSlope-self.slope)
#         print(x_intercept)
#         print(sensorSlope*(x_intercept-sensor_loc["x1"])+sensor_loc["y1"])
#         print(self.slope*(x_intercept-self.startX)+self.startY)
        #if x_intercept is in the correct range
        #check if the x_intercept is in the range of this line
        if x_intercept<max(self.endX,self.startX) and x_intercept>min(self.endX,self.startX):
            if x_intercept<max(sensor_loc["x2"],sensor_loc["x1"]) and x_intercept>min(sensor_loc["x2"],sensor_loc["x1"]):
                return x_intercept, sensorSlope*(x_intercept-sensor_loc["x1"])+sensor_loc["y1"]
        return None
    
    def distance(self,x,y):
        """
        distance from the line to this point
        """
        
        distances=abs((self.endX-self.startX)*(self.startY-y)-(self.startX-x)*(self.endY-self.startY))/self.length
        intercept_x=x+distances*np.sin(self.theta)
        intercept_y=y-distances*np.cos(self.theta)
        
        in_rangeX=np.logical_and(intercept_x<max(self.endX,self.startX),intercept_x>min(self.endX,self.startX))
        in_rangeY=np.logical_and(intercept_y<max(self.endY,self.startY),intercept_y>min(self.endY,self.startY))
        in_range=np.logical_and(in_rangeX,in_rangeY)
        
        dist_endpoints=np.min(np.array([np.sqrt((x-self.endX)**2+(y-self.endY)**2),
                                 np.sqrt((x-self.startX)**2+(y-self.startY)**2)]),axis=0)
        
        distances=distances*in_range+dist_endpoints*np.logical_not(in_range)
        
        return distances
    
    
    
    def draw(self,ax):
        #print([self.startX,self.endX])
        plt.plot([self.startX,self.endX],[self.startY,self.endY],color="black")
        
def make_curve(degrees, r, start_point,n_lines,left=True,start_degree=0):
    """
    draws the curve as a bunch of lines
    """
    if left:
        d=np.linspace(np.radians(start_degree),np.radians(degrees+start_degree),n_lines+1)
        X=r*(np.cos(d)-np.cos(np.radians(start_degree)))
        Y=r*(np.sin(d)-np.sin(np.radians(start_degree)))
    else:
        #therefore curve right
        d=np.linspace(np.pi+np.radians(start_degree),np.radians(180-degrees+start_degree),n_lines+1)
        X=r*(np.cos(d)-np.cos(np.radians(start_degree+180)))
        Y=r*(np.sin(d)-np.sin(np.radians(start_degree+180)))
    #bias to start point
    X+=start_point[0]
    Y+=start_point[1]
    #print(X)
    
    #make lines
    lines=[]
    for i in range(n_lines):
        lines.append(Line([X[i],Y[i]],
                     [X[i+1],Y[i+1]]))
    return lines
    