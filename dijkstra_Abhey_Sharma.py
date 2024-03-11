#Import important libraries 
import time
import cv2
import heapq as hq
import numpy as np
import matplotlib.pyplot as plt
import copy


def obstacle_map(canvas):

    #Creating rectangle 1
    cv2.rectangle(canvas,pt1=(100,0),pt2=(175,400),color=(130,0,75),thickness=-1)

    #Creating rectangle 2
    cv2.rectangle(canvas,pt1=(275,100),pt2=(350,500),color=(130,0,75),thickness=-1)

    # Draw hexagon
    cv2.fillPoly(canvas, [np.array([(650, 400), (775, 325), (775, 175), (650, 100), (525, 175), (525, 325)])], color=(130,0,75))

    # Draw polygon

    cv2.fillPoly(canvas, [np.array([(900, 50), (900, 125), (1020, 125), (1020, 375), (900, 375), (900, 450), (1100, 450), (1100, 50)])], color=(130,0,75))

    return canvas

def input_coordinates():

    #check if start and goal nodes are valid
    while True:
        start_node_str = input("Enter the coordinates of starting node (x,y):")
        goal_node_str = input("Enter the coordinates of Goal node (x,y):")
        
        start_node = tuple(map(int, start_node_str.split(',')))
        goal_node = tuple(map(int, goal_node_str.split(',')))


        #Check if the start and goal node are valid
        if is_valid(start_node[0],start_node[1]):

            if is_valid(goal_node[0],goal_node[1]):
                break
            else:
                print("Invalid goal node. Please enter valid coordinates.")
                continue
        else:
            print("Invalid start node. Please enter valid coordinates.")
            continue

    return start_node,goal_node


#checking if the robot is at a valid positon
def is_valid(x,y):

    #check if the coordinates are in bounds of canvas
    if (0 <= x <= width and 0 <= y <= height):
        pass
    else:
        # print("Invalid position: out of bounds of canvas")
        return False

    #Here (x-5) and (y-5) is used to account for 5 mm clearance
    #check if the coordinates are within bounds of rectangle 1
    if ((100-5) <= x <= (175+5)) and (0 <= y <= (400+5)):
        # print("Invalid position: Inside rectangle 1")
        return False
       
    #check if the coordinates are within bounds of rectangle 2
    if ((275-5) <= x <= (350+5)) and ((100-5) <= y <= 500):
        # print("Invalid position: Inside rectangle2")
        return False

    #Here (x-3.53) and (y-3.53) is used to account for 5 mm clearance in diagonal dirn
    #check if the coordinates are within bounds of hexagon

    if (((y+3.53)+(3/5)*(x+3.53)-490 >=0) and ((y+3.53)-(3/5)*(x-3.53)+290>=0) and (y)<=175) or ((525-5) <= (x) <= (775+5) and (175 <= (y) <= 325)) or (((y-3.53)-(3/5)*(x+3.53)-10<=0) and ((y-3.53)+(3/5)*(x-3.53) - 790 <=0) and (y)>=325):
        # print("Invalid position: Inside hexagon")
        return False

    #check if the coordinates are within bounds of polygon
    if ((900-5) <= (x) <= (1100+5) and (50-5) <= (y) <= (125+5)) or ((1020-5) <= (x) <= (1100+5) and 125 <= (y) <= 375) or ((900-5) <= (x) <= (1100+5) and (375-5) <= (y) <= (450+5)):
        # print("Invalid position: Inside polygon")
        return False
    
    return True

# Define possible movements (8-connected space)
movements = {
    "right":(1, 0),     
    "left":(-1, 0),  
    "up":(0, 1),   
    "down":(0, -1),    
    "up_right":(1, 1),   
    "up_left":(-1, 1),    
    "down_right":(1, -1),  
    "down_left":(-1, -1)    
}

# moving nodes
def move_node(present_node,move):

    next_node = []

    #x and y coordinates of next_node
    new_x = present_node[0] + movements[move][0]
    new_y = present_node[1] + movements[move][1]

    if is_valid(new_x, new_y):
        next_node = (new_x, new_y)

        return next_node
    
    else:
        return None


#backtracking 
def get_path(start_position, goal_position,closed_list):
    
    path = []
    current_node = goal_position
    
    # Backtrack from goal node to start node
    while current_node != start_position:
        path.append(current_node)
        current_node = tuple(closed_list[current_node])
    
    # Add the start node to the path
    path.append(start_position)
    
    # Reverse the path to get it in the correct order (from start to goal)
    path.reverse()
    
    return path,closed_list

def visualization(path,closed_list,canvas,start_position,goal_position):
    
    #Create video
    output_video = cv2.VideoWriter('visualization.avi', cv2.VideoWriter_fourcc(*'XVID'), 800, (canvas.shape[1], canvas.shape[0]))

    #Draw start and goal node
    cv2.circle(canvas,start_position, 5, (0, 255, 0), -1)
    cv2.circle(canvas,goal_position, 5, (0, 0, 255), -1)

    for visited_node in closed_list:
        canvas[visited_node[1]-1][visited_node[0]-1] = [0,128,139]

        vid = cv2.flip(canvas,0) 
        output_video.write(vid)
    

    optimal_path = copy.deepcopy(path)
    optimal_path.reverse()
    for i in range(len(optimal_path)):

        node = optimal_path.pop()

        canvas[node[1]-1][node[0]-1] = [0, 0, 0]       
        a = cv2.flip(canvas, 0)

        output_video.write(a)
        i+=1

    cv2.imwrite('path.png', a)
    output_video.release() 

#dijkstra algorithm
def dijkstra(start_position, goal_position, canvas):

    # List of nodes to be explored
    open_list = []
    
    # Dictionary stores explored and its parent node
    closed_list = {}

    # heap to store the nodes based on their cost value
    hq.heapify(open_list)

    # Inserting the initial node with its [cost,parent node,present node]
    hq.heappush(open_list, [0, None, start_position])

    #while open list is not empty
    while open_list:

        cost, parent_node, present_node = hq.heappop(open_list)

        # Adding the present node to closed list with its parent node -{present_node:parent_node}
        closed_list[(present_node[0], present_node[1])] = parent_node

        #if goal reached
        if list(present_node) == list(goal_position):
            
            print("goal reached")
            return get_path(start_position, goal_position,closed_list)
        
        #Add neighbouring nodes to the open_list
        for direction in ["up","right","down","left","up_right","up_left","down_right","down_left"]:

            next_node = move_node(present_node,direction)

            #token for direction
            if direction in ["up","right","down","left"]:
                token = True
            else:
                token = False

            if (next_node is not None) and (next_node not in closed_list):

                #neighbours of first node
                if present_node == start_position:

                     if token == True:

                        #adding neighbouring node to open list with cost@1
                        hq.heappush(open_list, [cost + 1, present_node, list(next_node)])
                        hq.heapify(open_list)

                     else:

                        #adding neighbouring node to open list with cost@1.4
                        hq.heappush(open_list, [cost + 1.4, present_node, list(next_node)])
                        hq.heapify(open_list)

                else:
                    flag = False

                    for index,(cost_n,parent_n,present_n) in enumerate(open_list):

                        # if neighbouring node already present in openlist
                        if (present_n == list(next_node)):

                            flag = True
                            
                            if token == True:

                                if (cost+1) < cost_n:

                                    #update cost@1 and parent of node
                                    open_list[index] = [cost + 1, present_node, list(next_node)]
                                    hq.heapify(open_list)

                            else:
                                if (cost+1.4) < cost_n:

                                    #update cost@1.4 and parent of node
                                    open_list[index] = [cost + 1.4, present_node, list(next_node)]
                                    hq.heapify(open_list)
                            break
                    
                    if flag == False:

                        if token == True:

                            #adding neighbouring node to open list with cost@1
                            hq.heappush(open_list, [cost + 1, present_node, list(next_node)])
                            hq.heapify(open_list)
                        else:

                            #adding neighbouring node to open list with cost@1.4
                            hq.heappush(open_list, [cost + 1.4, present_node, list(next_node)])
                            hq.heapify(open_list)

    return "Solution does not exist"

                        

if __name__=="__main__":

    start_time = time.time() 

    #create obstacle map
    # create blank  canvas
    width = 1200
    height = 500
    # canvas = np.ones((height,width,3))
    canvas = np.ones((height,width,3), dtype=np.uint8) * 255

    # draw the obstacle map
    canvas = obstacle_map(canvas)

    # input start and goal node coordinates
    start_position,goal_position = input_coordinates()

    # print(start_position,goal_position)

    #dijkstra algorithm
    path,closed_list = dijkstra(start_position, goal_position, canvas)
    # print(path)

    #Display Node exploration and Optimal path
    visualization(path,closed_list,canvas,start_position,goal_position)

    end_time = time.time()

    print("Total time taken to execute the code: ",end_time-start_time) 