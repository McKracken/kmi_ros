#!/usr/bin/python

import sys

def usage():
    print "  Use: ",sys.argv[0],' <map_name> <robot_type> <robot_name> <initial pose: x, y, theta[DEG]> <output_world_file>'
    print "  e.g. ",sys.argv[0],' dis_B1 diago diago_0 0.0 1.8 0 dis_B1_diago.world'

if __name__ == '__main__':
  if (len(sys.argv)<8):
    usage()
    sys.exit()
  map_name = sys.argv[1]
  robot_type = sys.argv[2]
  robot_name = sys.argv[3]
  init_x = sys.argv[4]
  init_y = sys.argv[5]
  init_th = sys.argv[6]
  outworld = sys.argv[7]
  
  f = open(outworld,'w')
  
  f.write('include "%s_base.inc" \n' % map_name )
  
  f.write('%s( pose [ %s %s 0 %s ] name "%s" color "blue")\n'% (robot_type, init_x, init_y, init_th, robot_name));

  f.close()
  
  print("File %s written.\n" % outworld)
  
  
