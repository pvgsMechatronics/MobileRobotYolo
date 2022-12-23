from asyncio.windows_events import INFINITE
import time
import numpy as np
from numpy import linalg as al
from shapely import geometry as geo
import requests
import matplotlib.pyplot as plt
import math as m

def main(ids:list[int], confs:list[float], boxes:list[int], df:float, id_robot:int, id_target:int, safe_distance:float):
    
    map = Map(id_target, id_robot, safe_distance)

    for i in range(len(ids)):
        map.add_object(Object(boxes[i], ids[i], confs[i], df))

    map.remake_map()

    d, path = map.make_path()

    if (d == 0):
        print("Robot or target not found!")
        return   
    try:
        print("Trying to connect to the robot...")
        msg = np.array2string(map.robot_pos, precision=5, separator=',')

        while(requests.get('192.168.2.200/calibrate', args={"robot_pose":msg}) != "OK"):
            print("Requesting calibrate...")

        msg = np.array2string(path, precision=5, separator=',')
        while(requests.get('192.168.2.200/path', args={"path":msg}) != "OK"):
            print("Sending path...")
        
        while(msg == "Robot busy!"):
            msg = requests.get('192.168.2.200/log')
            time.sleep(1)
    except:
        print("Robot offline!")
        return

    print_log(msg)
            
def next_point(i:int, length:int):
    if (i == length - 2): return 0    
    else: return (i + 1)
def prev_point(i:int, length:int):
    if (i == 0): return (length - 2)    
    else: return (i - 1)

class Object:
    def __init__(self, pos:list[float], id:int, conf:float, fd:float): #fd = camera focal distance / distance of the camera to the ground
        self.id = id #id class of the object
        self.conf = conf #confidence of the object
        self.pos = geo.Polygon(fd*np.array(pos)) #4 points that delimit the object in space (left-lower, left-high, right-high, right-lower) points  

class Map:
    def __init__(self, id_robot:int, id_target:int, ds:float): 
        self.map_objects = np.array([], dtype=Object) #array of objects in the map
        self.map_zones = np.array([], dtype=geo.Polygon) #array of polygons that represents the influence zones in the map
        self.id_robot = id_robot #robot id in yolo custom dataset
        self.id_target = id_target #target id in yolo custom dataset
        self.ds = ds #range of influence in the objects on the map
        self.robot_pos = np.array([-1,-1], dtype=float) #position of the robot in the map
        self.target_pos = np.array([-1,-1], dtype=float) #position of the target in the map

    def remake_map(self):
        #if two influence zones share the same space on the map, their zones are merged into one
        i=0     
        while (i < len(self.map_zones)):
            j=0
            while (j < len(self.map_zones)):
                if (i!=j) and (self.map_zones[i].intersects(self.map_zones[j])):
                    self.map_zones[i] = self.map_zones[i].union(self.map_zones[j])
                    self.map_zones = np.delete(self.map_zones, j, axis=0)
                    j=0
                    i=0
                else:
                    j=j+1
            i=i+1
        for i in range(len(self.map_zones)):
            #convex_hull function remove local minima of the polygon and soften the influence zone 
            poly = self.map_zones[i].convex_hull
            self.map_zones[i] = poly
            
    def add_object(self, obj:Object):
        if (obj.id == self.id_target):
            self.target_pos = np.array(obj.pos.centroid.coords[0])
        elif (obj.id == self.id_robot):
            self.robot_pos = np.array(obj.pos.centroid.coords[0])
        else:
            self.map_objects = np.append(self.map_objects, obj) #add object to the map
            dist_array = np.array([[-self.ds,-self.ds], [-self.ds,self.ds], [self.ds,self.ds], [self.ds,-self.ds], [-self.ds,-self.ds]]) #array to expand the coords of the object 
            obj_safe_coords = np.array(list(obj.pos.exterior.coords)) + dist_array  #objct influence zone coords
            self.map_zones = np.append(self.map_zones, geo.Polygon(obj_safe_coords)) #polygon  that represents the influence zone of the object    

    def free_path(self, start:list[float], end:list[float], idx_robot:list[int]):
        line_path = geo.LineString([start, end]) 
        for i in range(len(self.map_zones)):
            if (i==idx_robot[0]): #if robot are in a point on the limit of a influence zone, the next and prev point of this zone has free path
                zone_coords = np.array(list(self.map_zones[i].exterior.coords))
                if ((al.norm(end-zone_coords[next_point(idx_robot[1],len(zone_coords))])==0) or
                    (al.norm(end-zone_coords[prev_point(idx_robot[1],len(zone_coords))])==0)):
                    return True
            if (line_path.intersects(self.map_zones[i])): #checks if the line_path enters in the influence zone
                intersect_type = line_path.intersection(self.map_zones[i]).geom_type
                if not ((intersect_type == 'Point') or (intersect_type == 'MultiPoint')):
                    return False
        return True

    def make_path(self): #returns an array of points that the robot must follow to reach the target avoiding obstacles
        if (len(self.robot_pos) == 0):
            return 0
        if (len(self.target_pos) == 0):
            return 0       
        path = np.array([self.robot_pos]) #vector with the points of the path   
        pos = self.robot_pos #actual position of the robot on simulation
        end = self.target_pos #target position and end of the  path
        idx_pos = np.array([-1,-1]) #[index of obstacle of the robot are avoiding, index of the point in this obstacle that robot are located] on simulation
        while not (self.free_path(pos, end, idx_pos)):
            new_dist = INFINITE 
            for i in range(len(self.map_zones)):
                zone = np.array(list(self.map_zones[i].exterior.coords)) #coords of this influenze zone in the map
                for j in range(len(zone)-1):  
                    unvisited = True   
                    for k in range(len(path)):
                        if (path[k][0]==zone[j][0]) and (path[k][1]==zone[j][1]):
                            unvisited = False
                            break
                    if (unvisited) and (self.free_path(pos, zone[j], idx_pos)) and ((al.norm(end-zone[j])+al.norm(pos-zone[j]))<(new_dist)): 
                        new_idx_pos = np.array([i,j])
                        new_pos = zone[j]
                        new_dist = al.norm(pos-zone[j])+al.norm(end-zone[j])
            idx_pos = new_idx_pos
            pos = new_pos
            path = np.append(path, [pos], axis=0)
        path = np.append(path, [end], axis=0)
        return path

    
def print_log(log):
    aux = np.fromstring(log, dtype=float, sep=',')
    x = np.array([], dtype=float)
    y = np.array([], dtype=float)
    theta = np.array([], dtype=float)
    v = np.array([], dtype=float)
    w = np.array([], dtype=float)
    u1 = np.array([], dtype=float)
    u2 = np.array([], dtype=float)
    for i in range(0,len(aux),7):
        x = np.append(x, aux[i])
        y = np.append(y, aux[i+1])
        theta = np.append(theta, aux[i+2])
        v = np.append(v, aux[i+3])
        w = np.append(w, aux[i+4])
        u1 = np.append(u1, aux[i+5])
        u2 = np.append(u2, aux[i+6])
    plt.plot(x,y, color='red')     #print the robot path       
    for i in range(len(map.map_objects)):
        x,y = map.map_objects[i].exterior.xy
        plt.plot(x,y, color='red') #print the obstacle
    plt.title("Robot Path")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    fig.subplots_adjust(hspace=0.5)
    T = 0.01
    t = np.arange(0, len(v), T)
    ax1.plot(t, v)
    ax2.plot(t, w)
    ax3.plot(t, u1, t, u2)
    ax1.grid(True)
    ax2.grid(True)
    ax3.grid(True)
    ax3.set_xlabel('Time')
    ax1.set_ylabel('v (m/s)')
    ax2.set_ylabel('w (rad/s)')
    ax3.set_ylabel('u1 and u2 (V)')
    plt.title("Robot control dynamics")
    plt.show()       
