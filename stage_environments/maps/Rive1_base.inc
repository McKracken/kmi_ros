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
  center [ 25.0 20.0 ]   
  scale 10
  show_data 0
)

floorplan (
  size [108.75 50.75 1]
  pose [54.375 25.375  0 0]
  bitmap "Rive1.png"
)

# boxes
box( pose [ 15.0 19.0 0.1 0 ] color "blue")
box( pose [ 17.0 19.0 0 45 ] color "red")
box( pose [ 16.0 16.0 0.1 0 ] color "pink")

#circle( pose [ 15.0 17.5 0.1 0 ] color "blue")
#circle( pose [ 14.0 18.0 0.1 0 ] color "blue")





