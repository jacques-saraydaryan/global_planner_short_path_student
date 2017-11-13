import pandas as pd
import scipy as sc
import scipy.stats
import numpy as np

def distance_to_obstacle(x,y,grid,width,height,scale):
    try:
        distance=distance_to_obstacle_compute(x,y,grid,width,height,scale)[0]
        return distance
    except TypeError:
        print("no value available")
        return 0


def distance_to_obstacle_coord(x,y,grid,width,height,scale):
    return distance_to_obstacle_compute(x,y,grid,width,height,scale)


def distance_to_obstacle_compute(x,y,grid,width,height,scale):
    distance=0
    cell_x=0
    cell_y=0

    if width / scale > x / scale and height / scale > y / scale:
        x_to_grid =int(round(x / scale))
        y_to_grid = int(round(y / scale))
    else:
        # FIXME in case of outboung value
        return 10000
    targeted_y =y_to_grid
    while targeted_y < height / scale :
        if grid[targeted_y][x_to_grid]== 100:
            break
        distance+=1
        targeted_y+=1
        cell_x=x_to_grid*scale
        cell_y=targeted_y*scale
    # print "---> "+str(distance)
    return distance,cell_x,cell_y



def update_coord_according_scale(x,y,scale):
    return int(round(x/scale)),int(round(y/scale))


# Input a pandas series 
def entropy(particuleslist):
    index=[]
    serie_part=[]
    for i in range(0,len(particuleslist)):
        serie_part.append(round(particuleslist[i].x/10,1))
        index.append(i)
    p_particules=pd.Series(serie_part,index=index)
    p_particules= p_particules.value_counts()/len(particuleslist) # calculates the probabilities
    entropy=sc.stats.entropy(p_particules)  # input probabilities to get the entropy 
    return entropy


    # Input a pandas series 
def std(particuleslist):
    index=[]
    serie_part=[]
    for i in range(0,len(particuleslist)):
        serie_part.append(round(particuleslist[i].x/10,1))
        index.append(i)
    p_particules = np.array(serie_part)
    compute_std=np.std(p_particules)
    return compute_std