import math

def normalize_zw( z: float, w: float):
    n = math.hypot(z, w)
    if n == 0.0:
        # default to identity if bad input
        return 0.0, 1.0
    return z / n, w / n

def rad2q( rad ):
  return [0.0, 0.0, math.sin(rad/2.0), math.cos(rad/2.0)]

def qzw2rad( qz, qw):
  return 2 * math.atan2(qz, qw)

def rad2qzw( rad):
  z = math.sin( rad/2 )
  w = math.cos( rad/2 )
  return z,w

def get_fs_pose_in_map( wp_fs, wp_map ):
  x = wp_fs[0][0]
  y = wp_fs[0][1]
  qz = wp_fs[1][2]
  qw = wp_fs[1][3]
  qz,qw = normalize_zw( qz, qw )

  print('wp_fs ', x,y,qz,qw, 'ang', math.degrees(qzw2rad(qz,qw)) )

  pose = wp_map
  qz_org, qw_org = normalize_zw( pose[1][2], pose[1][3] )
  x_org = pose[0][0]
  y_org = pose[0][1]

  print('wp_map', x_org,y_org,qz_org,qw_org, 'ang', math.degrees(qzw2rad(qz_org,qw_org)) )

  c = qw_org * qw_org - qz_org * qz_org   # cos(th)
  s = 2.0 * qw_org * qz_org               # sin(th)
  print('cos', c, 'acos', math.degrees(math.acos(c)), 'sin', s, 'asin', math.degrees(math.asin(s)))
  global_x = x_org + c * x - s * y
  global_y = y_org + s * x + c * y

  print('x', x, 'y', y)
  print('x_org,y_org', x_org, y_org)
  print('c*x', c*x, '-s*y', -s*y)
  print('delta_x', c * x - s * y, 'delta_y', s * x + c * y)
  print('global_x', global_x, 'global_y', global_y)

  global_z = qw * qz_org + qz * qw_org
  global_w = qw * qw_org - qz * qz_org

  global_z, global_w = normalize_zw( global_z, global_w )

  print('calc fs', x,y,qz,qw)
  print('calc org', x_org, y_org, qz_org, qw_org)
  print('calc add', global_x, global_y, global_z, global_w, 'deg', math.degrees(qzw2rad( global_z, global_w )) )
  return [ [global_x,global_y, 0.0], [0.0, 0.0, global_z,global_w] ]

if __name__=='__main__':

  th1 = math.radians(0.0)
  p1 = [ [1.5, 0., 0.0], rad2q(th1) ]
  th2 = math.radians(-90.0)
  p2 = [ [0, 1.5, 0.0], rad2q(th2) ]
  p3 = get_fs_pose_in_map( p2, p1 )
  print(p1, math.degrees( qzw2rad( p1[1][2], p1[1][3] ) ) )
  print(p2, math.degrees( qzw2rad( p2[1][2], p2[1][3] ) ) )
  print(p3, math.degrees( qzw2rad( p3[1][2], p3[1][3] ) ) )
