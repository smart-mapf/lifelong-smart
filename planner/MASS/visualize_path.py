import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import re
import fire


def visualize_path(file_path):
    # --- Load and parse data ---

    with open(file_path, 'r') as f:
        lines = f.readlines()

        agent_paths = {}
        max_time, max_row, max_col = 0, 0, 0

        for line in lines:
            agent_id = int(re.search(r"Agent (\d+):", line).group(1))
            path = re.findall(
                r"\((\d+),(\d+),([\deE\.\+\-]+),([\deE\.\+\-]+)\)", line)
            # (row, col, t_min, t_max)
            parsed_path = []
            for x, y, t_min, t_max in path:
                t_min = float(t_min)
                t_max = float(t_max)
                if t_max > 1e8:
                    t = t_min + 2
                else:
                    t = (t_min + t_max) / 2
                if t < 10:
                    parsed_path.append((int(x), int(y), t))
            # parsed_path = [(int(x), int(y), float(t_min), float(t_max))
            #                for x, y, t_min, t_max in path]
            agent_paths[agent_id] = parsed_path
            if parsed_path:
                max_row = max(max_row, max(p[0] for p in parsed_path))
                max_col = max(max_col, max(p[1] for p in parsed_path))
                max_time = max(max_time, max(p[2] for p in parsed_path))

        # --- Interpolate positions at each time step ---
        time_resolution = 0.1
        time_steps = int(max_time // time_resolution) + 1
        agent_positions = [{} for _ in range(time_steps)]

        for agent, path in agent_paths.items():
            i = 0
            while i < len(path) - 1:
                r1, c1, t1 = path[i]
                r2, c2, t2 = path[i + 1]
                if t2 <= t1:
                    i += 1
                    continue
                step_start = int(t1 // time_resolution)
                step_end = int(t2 // time_resolution)
                for t in range(step_start, step_end):
                    interp_time = t * time_resolution
                    ratio = (interp_time - t1) / (t2 - t1)
                    rt = r1 + ratio * (r2 - r1)
                    ct = c1 + ratio * (c2 - c1)
                    agent_positions[t][agent] = (ct, rt
                                                 )  # flip to (x=col, y=row)
                i += 1

        # --- Propagate last known positions ---
        latest_known = {}
        for t in range(len(agent_positions)):
            for agent in agent_paths:
                if agent in agent_positions[t]:
                    latest_known[agent] = agent_positions[t][agent]
                elif agent in latest_known:
                    agent_positions[t][agent] = latest_known[agent]

        # --- Initialize plot ---
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(-1, max_col + 2)
        ax.set_ylim(-1, max_row + 2)
        ax.set_aspect('equal')
        ax.set_xticks(range(max_col + 2))
        ax.set_yticks(range(max_row + 2))
        ax.invert_yaxis()  # Top-left as origin
        ax.grid(True)
        ax.set_title("Multi-Agent Trajectories")

        agent_circles = [
            Circle((0, 0), 0.1, label=f"Agent {i}") for i in agent_paths
        ]
        for circle in agent_circles:
            ax.add_patch(circle)

        # --- Animation update function ---
        def update(frame):
            positions = agent_positions[frame]
            for i, circle in enumerate(agent_circles):
                if i in positions:
                    circle.set_center(positions[i])
                    circle.set_visible(True)
                else:
                    circle.set_visible(False)
            ax.set_title(f"Time = {frame * time_resolution:.1f}")
            return agent_circles

        ani = animation.FuncAnimation(fig,
                                      update,
                                      frames=time_steps,
                                      blit=True)

        # --- Save animation ---
        ani.save("agent_trajectories.mp4", writer="ffmpeg", fps=24)
        print("Saved animation to agent_trajectories.mp4")


if __name__ == "__main__":
    fire.Fire(visualize_path)
