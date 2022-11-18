from astar import Astar
import numpy as np
import cv2 

while(1):
    path = None
    try:
        planner = Astar([20,30],[65,84])
        # current_env = planner.env.environment
        path = planner.search()
        planner.car.theta = path[-1][-1]
        planner.start_position = path[-1][0]
        planner.goal_position = (25,96)
        path = planner.search()


        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # dt_list = [0.01,0.05,0.1,0.2,0.3,0.4,0.6]
        # for dt in dt_list:
        #     planner = Astar([10,10],[240,240])
        #     planner.simulation_dt = dt 
        #     planner.env.environment = current_env
        #     try: 
        #         path = planner.search()
        #         print(f"Path for {dt}s dt  found in {planner.n_iters} iters.")
        #         cv2.imwrite(f"search pattern at {dt} dt.jpg", planner.search_pattern)
        #     except KeyError:
        #         print(f"Couldnt find the path for {dt}s dt.")


    except KeyError:
        print("No path exists.. Trying Again")
    # print(path)
    if path is not None:
        break

# prim = planner.get_primitive([(19,34),0.38,0],[-3,-0.52])
# print([round(prim[-1][0][0]),round(prim[-1][0][1])])
# state_motions = planner.get_valid_neigh([[100,100],0,0])
# for state,motion in state_motions:
#     print(state,"\t", motion)
# print(planner.distance((94, 86),(100,100)))
# print(np.array((10,20))*5)
# print([100,100]>np.all(np.array([[101,1],[1,1],[1,2]])>=[0,0]))



