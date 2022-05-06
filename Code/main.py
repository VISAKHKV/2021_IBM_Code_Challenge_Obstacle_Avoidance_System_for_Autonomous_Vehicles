from rrt import RRT

if __name__ == '__main__':
    rrt = RRT((50,50),(930, 510), "ObstacleMap2.png", "DDR.png")
    running = True
    while running:
        try:
            path_coords, total_dist, total_time = rrt.main()
            running = False
        except:
            running = True

    print("coordinates:",path_coords)
    print("total time taken:", total_time)
    print("total distance covered:", total_dist)