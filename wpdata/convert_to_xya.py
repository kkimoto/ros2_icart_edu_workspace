import os
import json
import math

def load_waypoints_from_file(file_path):
    """
    Reads waypoints from a text file, assuming the file contains a 
    single JSON-formatted Python list of waypoints.
    
    File format expected: [ [[x, y, z], [qx, qy, qz, qw]], ... ]
    """
    waypoints = []
    if not os.path.exists(file_path):
        print(f"ERROR: Waypoint file not found at: {file_path}")
        return waypoints

    print(f"Loading waypoints from: {file_path} using JSON parser...")
    try:
        with open(file_path, 'r') as f:
            # Load the entire list structure using the json library
            waypoints = json.load(f)
            
        # Basic structural check (optional but good for robustness)
        if not all(isinstance(wp, list) and len(wp) == 2 for wp in waypoints):
            print("WARNING: Loaded data structure might not match expected waypoint format.")

    except json.JSONDecodeError as e:
        print(f"ERROR: Failed to parse file as JSON. Check file syntax.")
        print(f"Error details: {e}")
        waypoints = []
    except Exception as e:
        print(f"An unexpected error occurred during file loading: {e}")
        waypoints = []

    print(f"Successfully loaded {len(waypoints)} valid waypoints.")

    xya_list = list()
    for wp in waypoints:
      x = wp[0][0]
      y = wp[0][1]

      q = wp[1]
      z = q[2]
      w = q[3]

      th = 2 * math.atan2(z, w)
      ang = math.degrees(th)

      qq = ( math.sin(th/2.0), math.cos(th/2.0) )

      #print(q, qq, f"{x:3.3f} {y:3.3f} {ang:3.3f}")
      #print(f"{x:3.3f} {y:3.3f} {ang:3.3f}")
      xya_list.append( [x,y,ang] )

    return xya_list


if __name__=='__main__':

  xya_list = load_waypoints_from_file( '/home/kimoto/ros2_workspace/edu_ws/wpdata/wp.json' )

  fout = open('wp_xya.txt', 'w')

  for xya in xya_list:
    x = xya[0]
    y = xya[1]
    a = xya[2]
    print(f"{x:3.3f} {y:3.3f} {a:3.3f}")
    fout.write( f"{x:3.3f} {y:3.3f} {a:3.3f}\n" )

  fout.close()
