
# GITHUB LINK - https://github.com/aashrita-chemakura/ENPM661_Project3_Phase2

# DONE BY 
# Jayasuriya Suresh
# Aashrita Chemakura

import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq as hq
import cv2 


# Init Class 
class node_object:
    def __init__(self,pt,theta,parent_node,c2c,cost_to_goal,left_speed,right_speed,costofcurve,total_cost) :
        self.pt=pt
        self.c2c=c2c
        self.parent_node=parent_node
        self.theta=theta
        self.cost_to_goal=cost_to_goal
        self.left_speed=left_speed
        self.right_speed=right_speed
        self.costofcurve=costofcurve
        self.total_cost= total_cost

    def __lt__(self,other):
        return self.total_cost < other.total_cost
    
#Generating Map 
def generate_image(clr):
    global image
    #draw obstacles and bloated map
    cv2.rectangle(image,
                  (int(0+clr),int(0+clr)),
                  (int(image.shape[1]-clr),int(image.shape[0]-clr)),
                  (200,30,70),
                  -1
                  )
    for x in range(image.shape[1]):
        for y in range(image.shape[0]):
            if check_obstacle((x,y),clr):
                image[y][x]=(200,10,50)

    #non bloated obstacles 
    rect_1=np.array([[150,75],[150,200],[165,200],[165,75],[150,75]])
    rect_2=np.array([[250,0],[250,125],[265,125],[265,0],[250,0]])
    cir=(400,110)
    cv2.fillPoly(image,[rect_1],(255,255,255))
    # cv2.polylines(canvas,[rect_1],True,color=(0,0,0),thickness=clearence)
    cv2.fillPoly(image,[rect_2],(255,255,255))
    # cv2.polylines(canvas,[rect_2],True,color=(0,0,0),thickness=clearence)
    cv2.circle(image,cir,(50),(255,255,255),cv2.FILLED)
    # cv2.circle(canvas,cir,(50+clearence),(0,0,0),cv2.FILLED)
    
    image=cv2.flip(image,0)
    return image

#Checking if Valid Point 
def check_valid_point(point,clr):
    x,y=point
    if x<0+clr or x>600-clr or y<0+clr or y>200-clr:
        not_valid_flag=True
        # print("pt outside region")
        return not_valid_flag
    elif check_obstacle(point,clr):
        not_valid_flag=True
        # print("pt in obstruction")
        return not_valid_flag
    else:
        not_valid_flag=False 
        return not_valid_flag
    

def check_obstacle(point,clr):
    x,y=point
    if x>150-clr and x < 165+clr and y>75-clr and y<200-clr:
        obs_flag=True 
    elif  x>250-clr and x < 265+clr and y>0+clr and y<125+clr:
        obs_flag=True 
    elif (((x-400)**2+(y-110)**2) <((50+clr)**2)):
        obs_flag=True 
    else:
        obs_flag=False 
    return obs_flag

#cost function
def plot_curve(point,theta, UL, UR, clr,  cost_parent, list_of_points):
    global image
    global list_of_all_gen_nodes
    list_of_points=[] #list of points alog curve for plotting afterwards 
    end_of_curve= None # get final x,y for child node
    
    x,y=point #start point
    t = 0
    # r = 0.038
    # L = 0.354
    r = 3.3
    L = 18
    dt = 0.1
    Xn = x
    Yn = y
    Xn=0
    Yn=0
    Thetan = 3.14 * theta / 180
    cost=0

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    while t < 1:
        t = t + dt
        # Xs = Xn
        # Ys = Yn
        Xn += r * 0.5 * (UL + UR) * math.cos(Thetan) * dt
        Yn += r * 0.5 * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        # cv2.line(image,(int(Xs),int(image.shape[0]-Ys)),(int(Xn),int(image.shape[0]-Yn)),color=(200,30,40),thickness=1)
        if not check_valid_point((Xn+x,(Yn+y)),clr):
            # cv2.circle(image,(int(Xn+x),int(image.shape[0]-(Yn+y))),1,(20,220,30),1)
            list_of_all_gen_nodes.append((int(Xn+x),int(image.shape[0]-(Yn+y))))
        # cv2.imshow("image",image)
        # cv2.waitKey(1)
        
    # cv2.destroyAllWindows()
      
    curve_cost=np.sqrt(((Xn)**2 + (Yn)**2))
    cost=cost+curve_cost
   
    end_of_curve=(Xn+x,Yn+y)   
    Thetan = 180 * (Thetan) / 3.14
    list_of_points.append(end_of_curve)
    cost=cost_parent+cost
    return list_of_points,end_of_curve,Thetan,cost

#Generate Actions 
def actions(node,clr,end_node,rpm1,rpm2,list_of_path_nodes):
    action_set=[[0,rpm1], 
                [rpm1,0],
                [rpm1,rpm1],
                [0,rpm2],
                [rpm2,0],
                [rpm2,rpm2],
                [rpm1,rpm2],
                [rpm2,rpm1]]
    
    list_of_actions=[]
    for action in action_set:
        list_of_curve_points,new_point,theta,cost_of_child = plot_curve(node.pt,node.theta, action[0],action[1],clr,node.c2c,list_of_path_nodes) 
        if (not check_valid_point(new_point,clr) ):
            list_of_actions.append((new_point,theta,cost_of_child,list_of_curve_points,action[0],action[1]))
    return list_of_actions


#get distance of points
def euclidiean_distance(point1,point2):
    x1,y1=point1
    x2,y2=point2
    p1=(x2-x1)**2+(y2-y1)**2
    p2=np.sqrt(p1)
    # print((x2-x1)," ",(y2-y1),p1,p2)
    distance=np.sqrt((x2-x1)**2+(y2-y1)**2)
    return distance

#checking if node has been explored 
def check_if_visited_node(pt):
    global visited_nodes
    x,y=pt
    x=int(np.round(x)/(1))
    y=int(np.round(y)/(1))
    if visited_nodes[y][x]==0:
        # print("NODE JUST VISITED",visited_nodes[y][x])
        visited_nodes[y][x]=1
        visited_flag=False
        return visited_flag
    
    # print("NODE ALREADY VISITED",visited_nodes[y][x])
    visited_flag=True
    return visited_flag

#Astar Main Code 
def astar(start_node, goal_node, rpm1, rpm2, clr):
    start = start_node
 
    end = goal_node

    node_list = {}
    path_followed_list = []
    goal_dist_thresh = 1.5
    nodes_to_explore = {str(start.pt): start}
    nodes_explored = {}
    queue_of_nodes = [(start.total_cost, start)]
    hq.heapify(queue_of_nodes)

    list_of_path_nodes={}
    
    queue_of_nodes = []

    hq.heappush(queue_of_nodes, [start_node.total_cost, start_node])

    while (len(queue_of_nodes) != 0):

        current_node = (hq.heappop(queue_of_nodes))[1]
        node_list[str(current_node.pt)]=current_node

        dt = euclidiean_distance(current_node.pt, end.pt)
        if dt < goal_dist_thresh:
            goal_node.parent_node = current_node
            goal_node.total_cost = current_node.total_cost
            print("Goal Node found")
            return 1, node_list, path_followed_list

        if str(current_node.pt) in nodes_explored:
            continue
        else:
            nodes_explored[str(current_node.pt)] = current_node

        del nodes_to_explore[str(current_node.pt)]

        action_set=actions(current_node,clr,goal_node,rpm1,rpm2,list_of_path_nodes)
        # cv2.circle(image,(int(current_node.pt[0]),int(image.shape[0]-current_node.pt[1])),1,(0,255,0))
        for idx in action_set:
            x,y=idx[0]
            th=idx[1]

            c2g = euclidiean_distance((x, y), (goal_node.pt[0],goal_node.pt[1]))
            new_node = node_object((x,y),th,
                                   current_node, 
                                    idx[2], 
                                   c2g,idx[4],idx[5],idx[2],
                                   idx[2]+c2g)

    
            if not check_if_visited_node(new_node.pt):
         
          
                if str(new_node.pt) in nodes_to_explore:
                    # print("Updated Cost")
                    if new_node.total_cost < nodes_to_explore[str(new_node.pt)].total_cost:
                        nodes_to_explore[str(new_node.pt)].total_cost = new_node.total_cost
                        nodes_to_explore[str(new_node.pt)].parent_node = new_node.parent_node
                else:
                    nodes_to_explore[str(new_node.pt)] = new_node
                    hq.heappush(queue_of_nodes,
                                [new_node.total_cost,new_node])
            # else:
            #     print("NODE VISITED")
    return 0, node_list, path_followed_list

#backtrack function
def backtrack_path(goal_node,list_of_all_nodes):
    
    x_path = []
    y_path = []
    theta_path = []
    x,y=goal_node.pt
    x_path.append(x)
    y_path.append(y)
    theta_path.append(goal_node.theta)
    parent_node = goal_node.parent_node
    rpml=goal_node.left_speed
    rpm2=goal_node.right_speed
    rpm_list=[]
    rpm_list.append((rpml,rpm2))

    while parent_node != -1:
        x_path.append(parent_node.pt[0])
        y_path.append(parent_node.pt[1])
        theta_path.append(parent_node.theta)
        rpm_list.append([parent_node.left_speed,parent_node.right_speed])
        parent_node = parent_node.parent_node

    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()

    x_path = np.asarray(x_path)
    y_path = np.asarray(y_path)
    theta_path = np.array(theta_path)
    rpm_list=np.array(rpm_list)
    return x_path, y_path, theta_path,rpm_list


#animation function for visualizing all possible points 
def plotting_points(x_list,y_list,theta_list,rpm_list):
    global image
    global list_of_all_gen_nodes
    cv2.imshow('Map', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    counter=0
    for i in list_of_all_gen_nodes:
        cv2.circle(image,i,1,(20,220,30),1)
        counter=counter+1
        if counter==1000:
            counter=0
            cv2.imshow('Explored Nodes', image)
            cv2.waitKey(1)
            

        
    for i in range(len(x_list)):
        x=x_list[i]
        y=y_list[i]
        # x1=x_list[i+1]
        # y1=y_list[i+1]
        # theta=theta_list[i]
        UL=rpm_list[i][0]
        UR=rpm_list[i][1]
        cv2.circle(image,(int(x),image.shape[0]-int(y)),1,(0,0,255),1)
        # cv2.line(image,(int(x),image.shape[0]-int(y)),(int(x1),image.shape[0]-int(y1)),(0,255,0),1)
        counter=counter+1
        if counter==1:
            counter=0
            cv2.imshow('Backtracked Path', image)
            cv2.waitKey(1)
            
        cv2.imshow("Image",image)
        
    cv2.waitKey(0)

    # cv2.destroyAllWindows()

#Main Function 
if __name__ == '__main__':

    #init image and visitednodes matrix 
    global image,visited_nodes
    global list_of_all_gen_nodes
    list_of_all_gen_nodes=[]
    image=np.zeros((200,600,3),dtype="uint8") 
    visited_nodes=np.zeros((400,1200),dtype="uint8") 

    #defining constant 
    robot_radius =5

    #getting inputs
    clr = input("Enter clearance of robot ")
    clr = float(clr)

    Rpms = input("Enter left and right RPMs")
    rpm1, rpm2 = Rpms.split()
    rpm1 = int(rpm1)
    rpm2 = int(rpm2)

    start_coordinates = input("Enter start coordinates: ")
    start_pt_x, start_pt_y = start_coordinates.split()
    start_pt_x = int(start_pt_x)+50
    start_pt_y = int(start_pt_y)+100

    start_theta = input("Enter Orientation of the robot at start node: ")
    start_theta = int(start_theta)

    goal_coordinates = input("Enter goal coordinates: ")
    end_pt_x, end_pt_y = goal_coordinates.split()
    end_pt_x = int(end_pt_x)+50
    end_pt_y = int(end_pt_y)+100

    clr=robot_radius+clr
    
    if check_valid_point((start_pt_x, start_pt_y), clr):
        print("Invalid start node")
        exit(0)

    if  check_valid_point((end_pt_x, end_pt_y), clr):
        print("Invalid goal node")
        exit(0)

    print("#"*20)

    #starting Astar
    start_time = time.time()
    print("Starting Astar")
    c2g = euclidiean_distance((start_pt_x, start_pt_y), (end_pt_x, end_pt_y))
    total_cost = c2g
    start_node = node_object((start_pt_x, start_pt_y),  start_theta, -1, 0, 0,0,0,0,0)
    goal_node = node_object((end_pt_x, end_pt_y),0, -1, 0,  0, 0, 0, 0,0)

    canvas=generate_image(clr)
    flag, node_list, path_followed_list = astar(start_node, goal_node, rpm1, rpm2, clr)

    #backtracking
    if (flag) == 1:
        print("#"*20)
        print("Backtracking Started")
        x_path, y_path, theta_path,rpm_list = backtrack_path(goal_node,node_list)
    else:
        print("Path not found")
        exit()
    end_time=time.time()-start_time
    #plotting and animation 
    print("#"*20)
    print("Plotting Points")
    plotting_points(x_path,y_path,theta_path,rpm_list)
    # cv2.imshow("Image",image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    print("#"*20)
    print("Total Time Taken to find Path: ",end_time)
    print("#"*40)







