include "include/floorplan.inc"
include "include/box.inc"
include "include/erratic.inc" 
include "include/segway.inc" 

# set the resolution of the underlying raytrace model in meters
resolution 0.02

interval_sim 100  # simulation timestep in milliseconds

window
( 
  size [ 917.000 688.000 ] 
  center [ -0.169 -0.592 ]
  rotate [ 0 0 ]
  scale 19.757 
)

# load an environment bitmap
floorplan
( 
  name "dis-B1"
  bitmap "dis-B1-2011-09-27.png"
  size [ 50.5 53.5 0.500 ]
  pose [ 12.8 14.5 0 0 ]
)

# boxes
box( pose [ 6.0 6.0 0 0 ] color "red")




