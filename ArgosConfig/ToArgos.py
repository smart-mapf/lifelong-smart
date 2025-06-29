import os
import json
import pathlib
import xml.etree.ElementTree as ET

from xml.dom import minidom
from typing import List, Tuple
from lifelong_mapf_argos.ArgosConfig import (PROJECT_ROOT,
                                             CONTAINER_PROJECT_ROOT,
                                             FOOTBOT_DIFFUSION_CONTROLLER_LIB,
                                             TRAJECTORY_LOOP_FUNCTIONS_LIB)

obstacles = ['@', 'T']


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
    return map_data, width, height


def create_Argos(map_data: List[str],
                 output_file_path: str,
                 width: int,
                 height: int,
                 robot_init_pos: List[Tuple[str, str]],
                 curr_num_agent: int,
                 port_num: int,
                 n_threads: int,
                 visualization: bool = False,
                 sim_duration: int = 1200,
                 ticks_per_second: int = 10,
                 screen: int = 0,
                 velocity: float = 200.0,
                 container: bool = False,
                 seed: int = 42):
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
        ticks_per_second (int, optional): number of updates (ticks) per
            simulation second
        screen (int, optional): screen number for logging. Defaults to 0.
        velocity (float, optional): velocity of the robots in cm/s. Defaults to
            200.0 cm/s.
        container (bool, optional): whether to run in a container. Defaults to
            False.
    """
    # Process the library paths of client
    if container:
        footbot_diffusion_controller_lib = pathlib.Path(
            CONTAINER_PROJECT_ROOT) / FOOTBOT_DIFFUSION_CONTROLLER_LIB
        trajectory_loop_functions_lib = pathlib.Path(
            CONTAINER_PROJECT_ROOT) / TRAJECTORY_LOOP_FUNCTIONS_LIB
    else:
        footbot_diffusion_controller_lib = pathlib.Path(
            PROJECT_ROOT) / FOOTBOT_DIFFUSION_CONTROLLER_LIB
        trajectory_loop_functions_lib = pathlib.Path(
            PROJECT_ROOT) / TRAJECTORY_LOOP_FUNCTIONS_LIB

    # Create the root element
    argos_config = ET.Element("argos-configuration")

    # General configuration section
    framework = ET.SubElement(argos_config, "framework")

    # System configuration
    system = ET.SubElement(framework, "system", threads=str(n_threads - 1))

    # Experiment configuration
    if visualization:
        experiment = ET.SubElement(
            framework,
            "experiment",
            length="0",
            ticks_per_second=f"{ticks_per_second}",
            random_seed=f"{seed}",
        )
    else:
        experiment = ET.SubElement(
            framework,
            "experiment",
            length="0",
            ticks_per_second=f"{ticks_per_second}",
            random_seed=f"{seed}",
            visualization="none",
        )

    # Controllers section
    controllers = ET.SubElement(argos_config, "controllers")

    # Footbot controller
    footbot_controller = ET.SubElement(
        controllers,
        "footbot_diffusion_controller",
        id="fdc",
        library=str(footbot_diffusion_controller_lib))

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
        velocity=f"{velocity}",  # in cm/s
        acceleration="200.0",
        portNumber=f"{port_num}",
        outputDir=f"metaData{port_num}/",
        simDuration=f"{sim_duration}",
        screen=f"{screen}",
    )
    # Loop functions
    loop_functions = ET.SubElement(argos_config,
                                   "loop_functions",
                                   library=str(trajectory_loop_functions_lib),
                                   label="trajectory_loop_functions")

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

        goal_loc = ET.SubElement(qt_opengl,
                                 "user_functions",
                                 library=str(trajectory_loop_functions_lib),
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
