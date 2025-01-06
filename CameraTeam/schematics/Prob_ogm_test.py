import numpy as np 
import math
import matplotlib.pyplot as plt
grid_size = 4
GREY = 4
prior_occ = 0.2
grid = np.ones((grid_size, grid_size), dtype=float) * 0.5 
grid_2 = np.ones((grid_size, grid_size), dtype=float)
grayscale = np.ones((grid_size, grid_size), dtype=float)
#print(grid)


sensor_occ = 0.7
sensor_empty = 0.7
grid_val = grid[2][2]
hit =  0 





def probabilistic_update(prior_occ, sensor_occ, sensor_empty, grid_val, hit):
    """
    Probabilistic update step for a singuler grid space
    """
    # Calculate log odds: Odds(x) = p(x)/(1-p(x))

    if hit == 1:
        #print(1)
        sensor_term = math.log(sensor_occ/(1-sensor_occ))
        prior_term = math.log(prior_occ/ (1-prior_occ))
    else:
        #print(0)
        sensor_term = math.log(sensor_empty/(1-sensor_empty))
        prior_term = math.log((1-prior_occ)/prior_occ)
    #print("grid VAL: " + str(grid_val))
    if grid_val == 0:
        recursive_term = 0
    else:
        #print(grid_val)
        recursive_term = math.log(grid_val)
    
    log_odds = sensor_term + recursive_term - prior_term
    # Log probability from Log Odds: p(x) = 1/(1+1/odds(x))
    #Prob from log odds: p(x) = 1-1/(1+exp(logOdds))
    #print(log_odds)
    grid_prob = 1- 1/(1+np.exp(log_odds))
    return grid_prob
    
for i in range(5):
  
        
    #print(grid)
    if grid[2][2] == 4:
        grid[2][2] = prior_occ
        grid[2][2] = probabilistic_update(prior_occ, sensor_occ, sensor_empty,  grid[2][2], 1)
    else:
        grid[2][2] = probabilistic_update(prior_occ, sensor_occ, sensor_empty,  grid[2][2], 1) 
        grid[1][2] = probabilistic_update(prior_occ, sensor_occ, sensor_empty,  grid[1][2], 0) 
    
    print(grid)
#grid[1][2] = 0
grayscale = 1 - grid  # Darker as value approaches 1
print (grayscale)

plt.imshow(grayscale, cmap="gray", vmin=0, vmax=1)
plt.colorbar(label="Grayscale Intensity (Darker as values approach 1)")
plt.title("Bitmap Visualization of Grid")
plt.axis("off")  # Optional: Remove axes for cleaner visualization
plt.show()

