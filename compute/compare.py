import Fred as fred
from tqdm import tqdm

FILE_PATH  = "traj-sim-data/input/fbqs/"

OUTPUT_PATH = "traj-sim-data/output/"
NUM_FILES = 100
ERROR_BOUNDS = (1,2,5,10,20,40,60,80,100,1000,10000)

SIM_ALGS = ('sim','fbqs')


#with open("results.csv", "w") as output_f:


outputline="ID,original_length"
for error_bound in ERROR_BOUNDS:
    for sim_alg in SIM_ALGS:
        outputline+=',rate_'+sim_alg+'_'+str(error_bound)
        outputline+=',dist_'+sim_alg+'_'+str(error_bound)
print(outputline)
for i in range(1,NUM_FILES+1):
    outputline=str(i)
    with open(FILE_PATH+str(i)+".txt", "r") as f:
        # read the points as a linestring, with each line in the file containing a point
        points = f.readlines()
        # convert the points to a list of tuples
        points = [tuple(map(float, point.split())) for point in points]
        # create a linestring from the points
        traj = fred.Curve(points)
    outputline+=','+str(traj.complexity)
    print(outputline,end='')
    outputline=''
    

    for error_bound in ERROR_BOUNDS:
        for sim_alg in SIM_ALGS:
            with open(OUTPUT_PATH+sim_alg+'/'+str(error_bound)+'/'+str(i)+".txt", "r") as f:
                segs = f.readlines()
                points_sim = [(None,None)]
                
                for seg in segs:
                    start_x, start_y, end_x, end_y = map(float, seg.split())
                    if (start_x, start_y) != points_sim[-1]:
                        points_sim.append((start_x, start_y))
                    if (end_x, end_y) != points_sim[-1]:
                        points_sim.append((end_x, end_y))
                points_sim=points_sim[1:]
                comp_ratio = -1
                dist = -1
                # print(distance(Point(points[0]),Point(points_sim[0])))
                # if (len(points_sim) > 1):
                sim_traj = fred.Curve(points_sim)
                # print(sim_line)
                # print(line)
                
                comp_ratio = sim_traj.complexity/traj.complexity
                dist= fred.continuous_frechet(traj,sim_traj)
                    
                # print(dist)
                    
                print( ','+str(comp_ratio)+','+str(dist), end='')
    print()
                