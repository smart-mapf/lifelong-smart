import random
import math

def read_map(filename):
    """
    Reads an octile-format map file and returns the grid as a list of strings,
    along with its height and width.
    """
    with open(filename, 'r') as f:
        lines = [line.rstrip() for line in f if line.strip()]
    
    # Expecting header lines such as:
    #   type octile
    #   height <number>
    #   width <number>
    #   map
    map_type = lines[0].split()[1]  # e.g., "octile"
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    
    # The grid starts after the line "map" (assumed at index 3)
    grid = lines[4:4 + height]
    return grid, height, width

def find_free_cells(grid):
    """
    Returns a list of (row, col) tuples for each free cell ('.') in the grid.
    """
    free_cells = []
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            if cell == '.':
                free_cells.append((i, j))
    return free_cells

def assign_start_goal(free_cells, num_robots):
    """
    Randomly selects start and goal positions for each robot.
    Each robot gets two distinct cells, and all positions are unique.
    """
    if len(free_cells) < 2 * num_robots:
        raise ValueError("Not enough free cells for the number of robots requested!")
    
    # Randomly sample 2*num_robots distinct free cells.
    chosen_cells = random.sample(free_cells, 2 * num_robots)
    assignments = []
    for i in range(num_robots):
        start = chosen_cells[2 * i]
        goal = chosen_cells[2 * i + 1]
        assignments.append((start, goal))
    return assignments

def compute_octile_distance(start, goal):
    """
    Computes the octile distance between two cells.
    start and goal are (row, col) tuples.
    """
    dx = abs(start[1] - goal[1])
    dy = abs(start[0] - goal[0])
    # Octile distance: max(dx,dy) + (sqrt2 - 1) * min(dx,dy)
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

def write_scen_file(scen_filename, map_filename, map_width, map_height, assignments):
    """
    Writes a scenario file with a header and one line per robot.
    
    The format of each line is:
    <scenario_id>	<map_filename>	<map_width>	<map_height>	<start_x>	<start_y>	<goal_x>	<goal_y>	<optimal_cost>
    
    Coordinates are written as integers and the cost as a floating point with 8 decimals.
    """
    with open(scen_filename, 'w') as f:
        # Write header
        f.write("version 1\n")
        
        # For each assignment, assign a scenario id.
        # Here we simply use a counter starting at 1 (or you can use any scheme).
        for scen_id, (start, goal) in enumerate(assignments, start=1):
            # Our convention:
            # start: (row, col)  --> write as (col, row)
            # goal: (row, col)   --> write as (col, row)
            # (This is common if the map uses x=column and y=row.)
            start_x, start_y = start[1], start[0]
            goal_x, goal_y = goal[1], goal[0]
            cost = compute_octile_distance(start, goal)
            # Build a tab-separated line.
            line = (
                f"{scen_id}\t"
                f"{map_filename}\t"
                f"{map_width}\t"
                f"{map_height}\t"
                f"{start_x}\t"
                f"{start_y}\t"
                f"{goal_x}\t"
                f"{goal_y}\t"
                f"{cost:.8f}\n"
            )
            f.write(line)

if __name__ == "__main__":
    # Input map file (octile-format) and number of robots
    octile_map_file = "supermarket.map"    # Replace with your map file
    scen_file = "supermarket.scen"     # Output scenario file name
    map_filename_for_scen = "supermarket.map"  # This is the map name to be printed in the scen file.
    
    num_robots = 80  # Change this to the desired number of robots/scenarios

    # Read the map and extract grid info.
    grid, height, width = read_map(octile_map_file)
    free_cells = find_free_cells(grid)
    
    # Assign start and goal positions for each robot.
    assignments = assign_start_goal(free_cells, num_robots)
    
    # Write out the scen file.
    write_scen_file(scen_file, map_filename_for_scen, width, height, assignments)
    
    print(f"Scenario file '{scen_file}' created with {num_robots} scenarios.")