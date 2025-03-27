import numpy as np

POD_WIDTH = 4
MIN_Y = 4
MAX_Y = 38

num_group = int(40/POD_WIDTH)
agent_per_group = 2




def move_next(x, y):
  if x % 2 == 1:
    if y == MIN_Y:
        return [x + 1, y]
    else:
      return [x, y - 1]
  else:
    if y == MAX_Y:
        return [x - 1, y]
    else:
      return [x, y + 1]

def move_next_rotate(x, y):
  if x % 8 == 0:
    if y == MIN_Y-1:
        return [x + 1, y]
    else:
      return [x, y - 1]
  elif x % 8 == 7:
    if y == MAX_Y + 1:
        return [x - 1, y]
    else:
      return [x, y + 1]
  else:
      if y == MAX_Y+1:
        return [x - 1, y]
      elif y == MIN_Y-1:
        return [x + 1, y]
      else:
         print("impossible case")
         exit(-1)


if __name__ == '__main__':
  # Define the parameters
  all_path = []
  print("std::vector<std::vector<std::pair<int, int>>> read_picker_path = {")
  for i in range(num_group):
    start_y = np.random.randint(MIN_Y, MAX_Y, agent_per_group)
    start_y = np.sort(start_y)
    group_path = []
    for j in range(agent_per_group):
      print("{", end=" ")
      tmp_agent_traj = []
      if (j % 2 == 0):
        s_x = POD_WIDTH * i
      else:
        s_x = POD_WIDTH * i + 3
      s_y = start_y[j]
      tmp_agent_traj.append([s_x, s_y])
      print("{{ {0}, {1} }}".format(s_x, s_y), end=", ")
      tmp_x, tmp_y = s_x, s_y
      if ((i+j) % 2 == 1):
        while True:
          tmp_x, tmp_y = move_next(tmp_x, tmp_y)
          if tmp_y < MIN_Y or tmp_y > MAX_Y:
              exit(-1)
          if tmp_x == s_x and tmp_y == s_y:
              break
          print("{{ {0}, {1} }}".format(tmp_x, tmp_y), end=", ")
          tmp_agent_traj.append([tmp_x, tmp_y])
      else:
        while True:
          tmp_x, tmp_y = move_next_rotate(tmp_x, tmp_y)
          if tmp_x == s_x and tmp_y == s_y:
              break
          print("{{ {0}, {1} }}".format(tmp_x, tmp_y), end=", ")
          tmp_agent_traj.append([tmp_x, tmp_y])   
      print("},")
      group_path.append(tmp_agent_traj)
    all_path.append(group_path)
    # print(all_path)
  print("};")
  