import numpy as np
import matplotlib.pyplot as plt

class Sensor:
    """
    one line following sensor, for the purpose of this implementation we are using 
    basic sensor that will return the square of distance from this sensor to the line
    """
    def __self__(self,model=None
                 ,add_noise=0.05 #add 5% noise to the simulated measurments
                ):
        
        self.model=model
        self.noise=add_noise
        
    def value(self,dist):
        if model:
            return self.model*(1+(1-np.random.random())*2*self.noise)
        else:
            return dist**2*(1+(1-np.random.random())*2*self.noise)
        
    def fit(self):
        """
        for fitting, for this I would need data arrrrgh
        """
        pass
    
    
class Sensor_Array:
    def __init__(self,sensors,start_X,start_Y,length,start_orientation=0):
        """
        start_X, start_Y is the loc of the center of the array
        start_orientation, the inital starting orientation
        """
        
        self.X=start_X
        self.Y=start_Y
        self.orientation=np.radians(start_orientation)
        #print(self.orientation)
        self.sensors=sensors
        self.length=length
        self.calculateSensorLocs()
        
        
    def calculateSensorLocs(self):
        #print(np.cos(self.orientation))
        self.sensorLocs=np.array([self.X+np.linspace(-self.length/2,self.length/2,len(self.sensors))*np.sin(self.orientation),
                                self.Y-np.linspace(-self.length/2,self.length/2,len(self.sensors))*np.cos(self.orientation)]).T
        
    def get_values(self,line_loc):
        distances=np.linalg.norm(self.sensorLocs-line_loc,axis=1)
        
        sensor_values=np.empty(len(self.sensors))
        
        for i,d in enumerate(distances):
            sensors_values[i]=self.sensors[i].value(d)
        return sensor_values
    
    def update_loc(self,newX,newY,new_orientation):
#         print(self.X)
#         print(self.Y)
        self.X=newX
        self.Y=newY
        self.orientation=new_orientation
        self.calculateSensorLocs()
        
    def plot(self,ax,line_color="black",sensor_colors="blue"):
        #print(orientation)
        for i,sensor_loc in enumerate(self.sensorLocs):
            #print(sensor_loc)
            ax.plot(sensor_loc[0],sensor_loc[1],"o",
                    label=f"sensor{i}",color=sensor_colors)
        ax.plot(self.sensorLocs[:,0],self.sensorLocs[:,1],color=line_color)
        #ax.legend()
        
        
class Car:
    def __init__(self,startX,startY,startOrientation,sensor_array,
                motor_dist=10, #distance from the center of the car to the motor, this is a random value rn
                mPerToSpeed=1, #motor percentage to speed, this is a random value rn
                sensor_array_offest=10 #the offest from the motor centerline to the sensor array in cm, this is a random value rn
                ):
        
        self.X=startX
        self.Y=startY ## the X,Y coords of the center of the motor centerline
        self.orientation=np.radians(startOrientation)
        self.sensor_array=sensor_array
        self.sensor_array_offest=sensor_array_offest
        self.mPerToSpeed=mPerToSpeed
        self.motor_dist=motor_dist
        self.calculate_sensorLoc()
        
    def calculate_sensorLoc(self):
        sensorX=self.X+np.cos(self.orientation)*self.sensor_array_offest
        sensorY=self.Y+np.sin(self.orientation)*self.sensor_array_offest
        self.sensor_array.update_loc(sensorX,sensorY,self.orientation)
        
        
    def move_car(self,leftMotorPercent, 
                 rightMotorPercent,
                 time_step=0.005 #5ms
                ):
        
        #calculate left motor position
        leftMotorX=self.X-self.motor_dist*np.sin(self.orientation)
        leftMotorY=self.Y+self.motor_dist*np.cos(self.orientation)
        #print(f"old Left Motor, x={leftMotorX}, y={leftMotorY} leftMotorPercent={leftMotorPercent}")
        #update left motor position (this is an lazy estimation)
        leftMotorX+=leftMotorPercent/100*self.mPerToSpeed*np.cos(self.orientation)*time_step
        leftMotorY+=leftMotorPercent/100*self.mPerToSpeed*np.sin(self.orientation)*time_step
        #print(f"new Left Motor, x={leftMotorX}, y={leftMotorY} leftMotorPercent={leftMotorPercent}")
        
        #calculate right motor position
        rightMotorX=self.X+self.motor_dist*np.sin(self.orientation)
        rightMotorY=self.Y-self.motor_dist*np.cos(self.orientation)
        #print(f"old Right Motor, x={rightMotorX}, y={rightMotorY} rightMotorPercent={rightMotorPercent}")
        #update motor position
        rightMotorX+=rightMotorPercent/100*self.mPerToSpeed*np.cos(self.orientation)*time_step
        rightMotorY+=rightMotorPercent/100*self.mPerToSpeed*np.sin(self.orientation)*time_step
        #print(f"Right Motor, x={rightMotorX}, y={rightMotorY} rightMotorPercent={rightMotorPercent}")
        #calculate new centerpoint
        self.X=(leftMotorX+rightMotorX)/2
        self.Y=(leftMotorY+rightMotorY)/2
        
        #calculate new orientation
        self.orientation=np.pi/2+np.arctan2(rightMotorY-leftMotorY,rightMotorX-leftMotorX)
        #print(rightMotorY-leftMotorY)
        #print(rightMotorX-leftMotorX)
        #print(f"orientation={self.orientation}")
        #calcluate new sensor array location
        self.calculate_sensorLoc()
        
    def plot_wheel(self,axs,wheel_radius,wheel_loc,wheel_color):
        
        axs.plot([wheel_loc[0]-wheel_radius*np.cos(self.orientation),
                 wheel_loc[0]+wheel_radius*np.cos(self.orientation)],
                 [wheel_loc[1]-wheel_radius*np.sin(self.orientation),
                 wheel_loc[1]+wheel_radius*np.sin(self.orientation)],color=wheel_color)
        
    def plot(self,axs,radius=None,wheel_radius=None,car_color="black", 
             sensor_array_colors={"line_color":"black","sensor_colors":"blue"}):
        
        if radius==None:
            radius=self.motor_dist*1.25
        if wheel_radius==None:
            wheel_radius=self.motor_dist*0.5
        plt.sca(axs)
        #print(radius)
        axs.add_patch(plt.Circle((self.X, self.Y), radius, edgecolor=car_color))
        #print(self.orientation)
        #calculate left motor position
        leftMotorX=self.X-self.motor_dist*np.sin(self.orientation)
        leftMotorY=self.Y+self.motor_dist*np.cos(self.orientation)
        #plot it
        self.plot_wheel(axs,wheel_radius,[leftMotorX,leftMotorY],car_color)
        
        #calculate right motor position
        rightMotorX=self.X+self.motor_dist*np.sin(self.orientation)
        rightMotorY=self.Y-self.motor_dist*np.cos(self.orientation)
        #plot it
        #print([rightMotorX,rightMotorY])
        self.plot_wheel(axs,wheel_radius,[rightMotorX,rightMotorY],car_color)
        
        #plot linear array
        self.sensor_array.plot(axs,**sensor_array_colors)