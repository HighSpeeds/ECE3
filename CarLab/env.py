from carUtils import *
from linesUtils import *
import scipy.stats
import glob
import sklearn.svm


Noise_specs={"sensor noise": scipy.stats.norm(0,1).rvs, #we use a uniform distribution rn (maybe fun to change)
             "motor noise": scipy.stats.norm(1,0.1).rvs, #% error between what the input of the motor percentage to the output of the motor percentage
            }


class SensorValueModel:
    def __init__(self):
        self.model=sklearn.svm.SVR()
        
    def fit(self,distance,values):
        self.model.fit(distance.reshape(-1, 1),np.log(values))
        self.max_distance=np.max(distance)
        self.far_distvalue=np.mean(values[distance==self.max_distance])
        return self
    
    def get_value(self,distance):
        if type(distance)!=np.ndarray:
            distance=np.array(distance)
        #print(distance)
        value=np.exp(self.model.predict(distance.reshape(-1, 1)))
        value[distance>self.max_distance]=self.far_distvalue
        return value
    
    
class Env:
    def __init__(self,Car_specs=Car_specs,save_folder="Runs",Noise_specs=Noise_specs,line_generator=None):
        """
        Initializes the car according to the Car_specs, and noise_specs
        """
        #this may or may not be necessary since maybe the trainer will make the file
        #also assume the trainer will seed
#         if not os.path.exists(save_folder):
#             os.makedirs(save_folder)
        #car specs
        self.Car_specs=Car_specs
        #init car specs according to the car_specs
        #init sensor models
        sensor_models=self.fit_sensor_models()
        #init sensors
        self.sensors=[Sensor(model=sensor_models[i],
                        noise_generator=Noise_specs["sensor noise"]) for i in range(8)]
        #init sensor_array
        sensor_array=Sensor_Array(self.sensors,0,0,self.Car_specs['sensor_spacing']*7)
        #init car
        self.car=Car(0,0,0,sensor_array,
                     motor_dist=Car_specs["wheel_distance"],
                     mPerToSpeed=1, #motor percentage to speed, this is a random value rn
                    sensor_array_offest=Car_specs["sensor_array_distance"],
                car_radius=Car_specs["diameter"]/2,
                wheel_radius=Car_specs["wheel_diameter"]/2)
        
        self.line_generator=line_generator
        self.reset()
        self.Noise_specs=Noise_specs
        self.sensor_values=[[],[],[],[],[],[],[],[]]
        
    def reset(self):
        self.sensor_values=[[],[],[],[],[],[],[],[]]
        if self.line_generator:
            self.lines,self.startX,self.startY,orientation,self.endX,self.endY=self.line_generator()
        else:
            self.lines=[Line((0,0),(100,0))]
            self.startX=0
            self.startY=0.953
            self.endX=100
            self.endY=0
            orientation=0
            
        self.car.set_loc(self.startX,self.startY,orientation)
        
    def distance_from_start(self):
        return np.sqrt((self.startX-self.car.X)**2+(self.startY-self.car.Y)**2)
    
    def distance_from_end(self):
        return np.sqrt((self.endX-self.car.X)**2+(self.endY-self.car.Y)**2)
    
    def fit_sensor_models(self):
        
        sensor_locs=self.Car_specs['sensor_spacing']*3.5-\
                    np.arange(self.Car_specs['num_sensors'])*self.Car_specs['sensor_spacing']
        
        #load sensor data
        T=[]
        X=[[],[],[],[],[],[],[],[]]
        for file in glob.glob("measurements/*.npy"):
            t=int(file[len("measurements\\measurment"):-4])
            raw_data=np.load(file)
            T+=[t]*raw_data.shape[0]
            for i in range(8):
                X[i]+=list(raw_data[:,i])
                #print(raw_data[:,i].shape)

        X=np.array(X)
        T=np.array(T)
        X=X[:,np.all(X>0,axis=0)]
        T=T[np.all(X>0,axis=0)]
        T=T[np.all(X<=2500,axis=0)]
        X=X[:,np.all(X<=2500,axis=0)]
        line_locs=-4+T*0.2
        sensor_models=[]
        for i in range(8):
            x=np.abs(line_locs-sensor_locs[i])
    
            sensor_models.append(SensorValueModel().fit(x,X[i,:]).get_value)
        
        return sensor_models
    
    
    def move_car(self,leftMotorPWM,rightMotorPWM,
                 time_step=0.05 #5 ms
                ):
        
        for i in range(int(time_step//0.005)):
            self.car.move_car(leftMotorPWM/255*Noise_specs["motor noise"]()*100, 
                 rightMotorPWM/255*Noise_specs["motor noise"]()*100)
        
        sensor_values=self.car.sensor_array.get_values(self.lines)
        for i in range(8):
            self.sensor_values[i].append(sensor_values[i])
        return sensor_values
    
    def off_track(self,threshold=None):
        if threshold is None:
            threshold=0.5*self.Car_specs["diameter"]
        
        distances=np.empty(len(self.lines))
        for i,line in enumerate(self.lines):
            distances[i]=line.distance(np.array([self.car.X]),np.array([self.car.Y]))
            
        return np.min(distances)>threshold
    
    def distance_from_track(self):
        distances=np.empty(len(self.lines))
        for i,line in enumerate(self.lines):
            distances[i]=line.distance(np.array([self.car.X]),np.array([self.car.Y]))
            
        return np.min(distances)
        
    
    def plot(self,save_path=None,image_name=None):
        figure, [ax1,ax2] = plt.subplots(2,figsize=(10,5))
        self.plot_car(ax1)
        self.plot_sensor_readings(ax2)
        if save_path and image_name:
            plt.savefig(f"{save_path}/{image_name}")
            plt.close()
        
    def plot_car(self,ax):
        ax.set_aspect('equal', adjustable='box')
        self.car.plot(ax)
        for line in self.lines:
            line.draw(ax)
            
    def plot_sensor_readings(self,ax):
        
        for i in range(8):
            ax.plot(self.sensor_values[i],label=f"sensor{i}")
        ax.legend()