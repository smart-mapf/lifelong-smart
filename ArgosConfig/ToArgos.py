import os
import json
import pathlib
import xml.etree.ElementTree as ET

from xml.dom import minidom
from typing import List, Tuple

# def prettify(elem, level=0):
#     """Return a pretty-printed XML string for the Element."""
#     indent = "  "
#     newline = "\n"
#     if len(elem):
#         if not elem.text or not elem.text.strip():
#             elem.text = newline + indent * (level + 1)
#         if not elem.tail or not elem.tail.strip():
#             elem.tail = newline + indent * level
#         for child in elem:
#             prettify(child, level + 1)
#         if not elem.tail or not elem.tail.strip():
#             elem.tail = newline + indent * level
#     else:
#         if level and (not elem.tail or not elem.tail.strip()):
#             elem.tail = newline + indent * level
obstacles = ['@', 'P']
stations = ['S', 'T']


def prettify(elem):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def parse_map_file(map_file_path: str):
    map_file_path: pathlib.Path = pathlib.Path(map_file_path)
    if map_file_path.suffix == ".map":
        map_data = []
        with open(map_file_path, 'r') as file:
            lines = file.readlines()
            height = int(lines[1].split()[1])
            width = int(lines[2].split()[1])
            # Skip the first 4 lines (type, height, width, map)
            for line in lines[4:]:
                map_data.append(line.strip())
    elif map_file_path.suffix == ".json":
        with open(map_file_path, 'r') as f:
            data = json.load(f)
            map_data = data["layout"]
            width = data["n_col"]
            height = data["n_row"]
    print(f"Width: {width}, Height: {height}")
    return map_data, width, height


def read_scen(file_path):
    column_data = []
    num_agent = 0
    with open(file_path, 'r') as file:
        lines = file.readlines()
        num_agent = len(lines) - 1
        for line in lines[1:]:
            columns = line.split('\t')
            column_5 = columns[4]
            column_6 = columns[5]
            column_data.append((column_5, column_6))
    return column_data, num_agent


def create_Argos(map_data: List[str],
                 output_file_path: str,
                 width: int,
                 height: int,
                 robot_init_pos: List[Tuple[str, str]],
                 curr_num_agent: int,
                 port_num: int,
                 n_threads: int,
                 visualization: bool = False,
                 sim_duration: int = 1200):
    """Create an Argos configuration file based on the provided map data and robot initial positions.

    Args:
        map_data (list): grid map in list of str format, where each string
            represents a row of the map.
        output_file_path (str): filepath to store the argos config file.
        width (float): width of the map.
        height (float): height of the map.
        robot_init_pos (list): initial position of the robots in the map.
        curr_num_agent (int): number of robots.
        port_num (int): port number for the server/client communication.
        n_threads (int): number of threads for the simulation. 0 for all
            threads.
        visualization (bool, optional): whether run with visualization.
            Defaults to False.
        sim_duration (int, optional): duration of the simulation in number of
            ticks. Defaults to 1200. With a tick rate of 10 (ticks per second),
            this is 120 seconds.
    """
    # Create the root element
    argos_config = ET.Element("argos-configuration")

    # General configuration section
    framework = ET.SubElement(argos_config, "framework")

    # System configuration
    system = ET.SubElement(framework, "system", threads=str(n_threads))

    # Experiment configuration
    if visualization:
        experiment = ET.SubElement(
            framework,
            "experiment",
            length="0",
            ticks_per_second="10",  # 10 update per simulation second
            random_seed="124")
    else:
        experiment = ET.SubElement(framework,
                                   "experiment",
                                   length="0",
                                   ticks_per_second="10",
                                   random_seed="124",
                                   visualization="none")

    # Controllers section
    controllers = ET.SubElement(argos_config, "controllers")

    # Footbot controller
    footbot_controller = ET.SubElement(
        controllers,
        "footbot_diffusion_controller",
        id="fdc",
        library="build/controllers/footbot_diffusion/libfootbot_diffusion")

    # Actuators
    actuators = ET.SubElement(footbot_controller, "actuators")
    differential_steering = ET.SubElement(actuators,
                                          "differential_steering",
                                          implementation="default")

    # Sensors
    sensors = ET.SubElement(footbot_controller, "sensors")
    footbot_proximity = ET.SubElement(sensors,
                                      "footbot_proximity",
                                      implementation="default",
                                      show_rays="true")
    positioning = ET.SubElement(sensors,
                                "positioning",
                                implementation="default")

    # Parameters
    params = ET.SubElement(
        footbot_controller,
        "params",
        alpha="45.0",
        omega="45.0",
        #  alpha="7.5",
        #  omega="3.0",
        velocity="200",
        acceleration="200.0",
        portNumber=f"{port_num}",
        outputDir=f"metaData{port_num}/",
        simDuration=f"{sim_duration}")
    # params = ET.SubElement(footbot_controller,
    #                        "params",
    #                        alpha="7.5",
    #                        omega="3.0",
    #                        velocity="100",
    #                        acceleration="2.5",
    #                        portNumber=f"{port_num}",
    #                        outputDir=f"metaData{port_num}/")

    # Loop functions
    loop_functions = ET.SubElement(
        argos_config,
        "loop_functions",
        library=
        "build/loop_functions/trajectory_loop_functions/libtrajectory_loop_functions",
        label="trajectory_loop_functions")

    station_count = 0
    for y, row in enumerate(map_data):
        for x, cell in enumerate(row):
            if cell in stations:
                # Creating four walls for each box
                station = ET.SubElement(loop_functions,
                                        f"station{station_count}",
                                        x=f"{-y}",
                                        y=f"{-x}",
                                        z="0")
                station_count += 1

    ET.SubElement(loop_functions, f"num_stations", value=f"{station_count}")

    ET.SubElement(loop_functions, f"port_number", value=f"{port_num}")

    map_center_x = -height / 2 + 0.5
    map_center_y = -width / 2 + 0.5
    arena = ET.SubElement(argos_config,
                          "arena",
                          size=f"{height},{width},1",
                          center=f"{map_center_x},{map_center_y},0")

    wall_thick = 0.05
    wall_height = 0.5
    y_offset = 0.0

    box = ET.SubElement(arena,
                        "box",
                        id=f"wall_north",
                        size=f"{wall_thick},{width},{wall_height}",
                        movable="false")
    body = ET.SubElement(box,
                         "body",
                         position=f"0.5,{map_center_y},{y_offset}",
                         orientation="0,0,0")

    box = ET.SubElement(arena,
                        "box",
                        id=f"wall_west",
                        size=f"{height},{wall_thick},{wall_height}",
                        movable="false")
    body = ET.SubElement(box,
                         "body",
                         position=f"{map_center_x},{-width+0.5},{y_offset}",
                         orientation="0,0,0")

    box = ET.SubElement(arena,
                        "box",
                        id=f"wall_south",
                        size=f"{wall_thick},{width},{wall_height}",
                        movable="false")
    body = ET.SubElement(box,
                         "body",
                         position=f"{-height+0.5},{map_center_y},{y_offset}",
                         orientation="0,0,0")

    box = ET.SubElement(arena,
                        "box",
                        id=f"wall_east",
                        size=f"{height},{wall_thick},{wall_height}",
                        movable="false")
    body = ET.SubElement(box,
                         "body",
                         position=f"{map_center_x},0.5,{y_offset}",
                         orientation="0,0,0")

    for y, row in enumerate(map_data):
        for x, cell in enumerate(row):
            if cell in obstacles:
                # Creating four walls for each box
                box = ET.SubElement(arena,
                                    "box",
                                    id=f"box_{x}_{y}",
                                    size="0.8,0.8,0.5",
                                    movable="false")
                body = ET.SubElement(box,
                                     "body",
                                     position=f"{-y},{-x},0",
                                     orientation="0,0,0")

    # tree = ET.ElementTree(arena)

    agent_count = 0
    for x, y in robot_init_pos:
        foot_bot = ET.SubElement(arena, "foot-bot", id=f"fb_{x}_{y}")
        x, y = -int(y), -int(x)
        body = ET.SubElement(foot_bot,
                             "body",
                             position=f"{x},{y},0",
                             orientation="0,0,0")
        controller = ET.SubElement(foot_bot, "controller", config="fdc")
        agent_count += 1
        if agent_count >= curr_num_agent:
            break

    # Physics engines
    physics_engines = ET.SubElement(argos_config, "physics_engines")
    dynamics2d = ET.SubElement(physics_engines, "dynamics2d", id="dyn2d")

    # Media
    media = ET.SubElement(argos_config, "media")
    if visualization:
        # Visualization
        visualization = ET.SubElement(argos_config, "visualization")
        # qt_opengl = ET.SubElement(visualization, "qt-opengl", autoplay="true")
        qt_opengl = ET.SubElement(visualization, "qt-opengl")

        goal_loc = ET.SubElement(
            qt_opengl,
            "user_functions",
            library=
            "build/loop_functions/trajectory_loop_functions/libtrajectory_loop_functions",
            label="trajectory_qtuser_functions")

        # autoplay = ET.SubElement(qt_opengl, "autoplay",
        #                           autoplay="true")
        # Camera
        camera = ET.SubElement(qt_opengl, "camera")
        placements = ET.SubElement(camera, "placements")
        # TODO@jingitan: Modify this visualize to fit large maps
        placement = ET.SubElement(
            placements,
            "placement",
            index="0",
            position=f"{map_center_x},{map_center_y},{max(width,height)/2.35}",
            look_at=f"{map_center_x},{map_center_y},0",
            up="1,0,0",
            lens_focal_length="15")
    xml_str = prettify(argos_config)
    with open(output_file_path, "w") as f:
        f.write(xml_str)


def create_folder(folder: str):
    if not os.path.exists(folder):
        os.makedirs(folder)


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description="Parser for input map and scen arguments.")
    # # parser.add_argument("-parent", '-p', type=str, default="./data/empty-8-8", help="parent folder")
    # # parser.add_argument('-scen_filename', '-s', type=str, default="empty-8-8-random-1.scen", help='name of scen file')
    # # parser.add_argument('-map_filename', '-m', type=str, default="empty-8-8.map", help='name of map file')
    # # parser.add_argument('-output_filename', '-o', type=str, default="empty-8-8-1.argos", help='name of output file')
    # # parser.add_argument("-parent", '-p', type=str, default="./data/maze-32-32-4", help="parent folder")
    # # parser.add_argument('-scen_filename', '-s', type=str, default="maze-32-32-4-random-1.scen", help='name of scen file')
    # # parser.add_argument('-map_filename', '-m', type=str, default="maze-32-32-4.map", help='name of map file')
    # # parser.add_argument('-output_filename', '-o', type=str, default="maze-32-32-4-random-1.argos", help='name of output file')
    # parser.add_argument("-parent", '-p', type=str, default="./data/warehouse-10-20-10-2-1", help="parent folder")
    # parser.add_argument('-scen_filename', '-s', type=str, default="scen-random/warehouse-10-20-10-2-1-random-1.scen", help='name of scen file')
    # parser.add_argument('-map_filename', '-m', type=str, default="warehouse-10-20-10-2-1.map", help='name of map file')
    # parser.add_argument('-output_filename', '-o', type=str, default="warehouse-10-20-10-2-1-random-1.argos", help='name of output file')
    # args = parser.parse_args()
    # scen_file_path = os.path.join(args.parent, args.scen_filename)
    # map_file_path = os.path.join(args.parent, args.map_filename)
    # output_file_path = os.path.join(args.parent, args.output_filename)
    # robot_init_pos = read_scen(scen_file_path)
    # map_data, width, height = parse_map_file(map_file_path)
    # # create_xml(map_data, output_file_path, width, height, robot_init_pos)
    # create_Argos(map_data, output_file_path, width, height, robot_init_pos)

    map_list = [
        "empty-32-32", "Boston_0_256", "room-64-64-16", "maze-32-32-4",
        "random-64-64-10", "warehouse-20-40-10-2-2", "den520d"
    ]
    NUM_AGENT_STEP = 20
    for map in map_list:
        map_filename = os.path.join("./data", map, map + ".map")
        argos_folder = os.path.join("output", map)
        map_data, width, height = parse_map_file(map_filename)
        create_folder(argos_folder)
        all_agents_solved = True
        curr_num_agent = 0
        while all_agents_solved:
            curr_num_agent += NUM_AGENT_STEP
            output_num_agent_path = os.path.join(argos_folder,
                                                 str(curr_num_agent))
            create_folder(output_num_agent_path)
            for scen_idx in range(1, 26):
                scen_filename = os.path.join("./data", map, "scen-random",
                                             map + f"-random-{scen_idx}.scen")
                output_config_filename = os.path.join(
                    output_num_agent_path, map + f"-random-{scen_idx}.argos")
                robot_init_pos, scen_num_agent = read_scen(scen_filename)
                # create_xml(map_data, output_file_path, width, height, robot_init_pos)
                if scen_num_agent < curr_num_agent:
                    all_agents_solved = False
                    break
                port_num = 12000 + scen_idx
                create_Argos(map_data, output_config_filename, width, height,
                             robot_init_pos, curr_num_agent, port_num)
