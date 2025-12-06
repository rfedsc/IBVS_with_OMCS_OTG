import numpy as np
from mycobot_init import MyCobot_Init
from compare_T import actual_to_transformation_matrix,degrees_to_radians


def test():
  mycobot = MyCobot_Init()
  coords = mycobot.get_coords()
  angles = mycobot.get_angles()
  angles_rad = np.zeros(len(angles))
  for i in range(len(angles)):
    angles_rad[i] = degrees_to_radians(angles[i])

  target_T = actual_to_transformation_matrix(coords)
  print(f'target_T:{target_T}')
  print(f'angles:{angles}')
  print(f'angles_rad:{angles_rad}')

if __name__ == "__main__":
  test()
  
