include "include/floorplan.inc"
include "include/box.inc"
include "include/erratic.inc" 
include "include/segway.inc" 

# set the resolution of the underlying raytrace model in meters
resolution 0.02

interval_sim 100  # simulation timestep in milliseconds

window ( 
  size   [ 600 500 1 ]         
  rotate [ 0.000 0.000 ]    
  center [ 0.0 0.0 ]   
  scale 20
  show_data 0
)

floorplan (
  size [35.0 35.0  1] 
  pose [0.0 0.0 0 0]
  bitmap "Rockin2014.pgm"
)

# boxes
box( pose [ 6.0 6.0 0 0 ] color "red")





