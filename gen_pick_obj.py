import numpy as np

POD_WIDTH = 4
MIN_Y = 4
MAX_Y = 102

num_group = int(40/POD_WIDTH)
agent_per_group = 2




def move_next(x, y):
  if x % 8 == 2:
    if y == MIN_Y - 1:
        return [x + 1, y]
    else:
      return [x, y - 1]
  elif x % 8 == 5:
    if y == MAX_Y + 1:
        # if y is at the max, move left
        return [x - 1, y]
    else:
      # otherwise move down
      return [x, y + 1]
  else:
    if y == MAX_Y+1:
        return [x - 1, y]
    elif y == MIN_Y-1:
        # if y is at the min, move right
        return [x + 1, y]
    else:
      return [x, y + 1]

def move_next_rotate(x, y):
  if x % 8 == 1:
    if y == MIN_Y-1:
        return [x + 1, y]
    else:
      return [x, y - 1]
  elif x % 8 == 6:
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
  print("std::vector<std::vector<std::pair<int, int>>> read_picker_obj = {")
  np.random.seed(0)  # For reproducibility, you can remove this in production
  for i in range(num_group):
    start_y = np.random.randint(54, 55, agent_per_group)
    start_y = np.sort(start_y)
    group_path = []
    if (i % 2 == 1):
      for j in range(agent_per_group):
        print("{", end=" ")
        tmp_agent_traj = []
        if (j % 2 == 0):
          s_x = POD_WIDTH * i - 2
          flag = 1
        else:
          s_x = POD_WIDTH * i + 1
          flag = -1
        s_y = start_y[j]
        tmp_agent_traj.append([s_x, s_y])
        print("{{ {0}, {1} }}".format(s_y, s_x+flag), end=", ")
        tmp_x, tmp_y = s_x, s_y
        while True:
          tmp_x, tmp_y = move_next(tmp_x, tmp_y)
          # if tmp_y < MIN_Y or tmp_y > MAX_Y:
          #     exit(-1)
          if tmp_x == s_x and tmp_y == s_y:
              break
          print("{{ {0}, {1} }}".format(tmp_y, tmp_x+flag), end=", ")
          tmp_agent_traj.append([tmp_x, tmp_y])
        print("},")
    else:
      for j in range(agent_per_group):
        print("{", end=" ")
        tmp_agent_traj = []
        if (j % 2 == 0):
          s_x = POD_WIDTH * i + 1
          flag = -1
        else:
          s_x = POD_WIDTH * i + 6
          flag = 1
        s_y = start_y[j]
        tmp_agent_traj.append([s_x, s_y])
        print("{{ {0}, {1} }}".format(s_y, s_x+flag), end=", ")
        tmp_x, tmp_y = s_x, s_y
        while True:
          tmp_x, tmp_y = move_next_rotate(tmp_x, tmp_y)
          if tmp_x == s_x and tmp_y == s_y:
              break
          print("{{ {0}, {1} }}".format(tmp_y, tmp_x+flag), end=", ")
          tmp_agent_traj.append([tmp_x, tmp_y])   
        print("},")
        # group_path.append(tmp_agent_traj)
    # all_path.append(group_path)
    # print(all_path)
  print("};")
  