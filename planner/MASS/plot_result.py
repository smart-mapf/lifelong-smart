import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_runtime_vs_agents(csv_path, label=None):
    # Load CSV
    df = pd.read_csv(csv_path)

    # Group by number of agents and compute mean and std
    grouped = df.groupby("#agents")[" runtime"]
    means = grouped.mean()
    stds = grouped.std()
    agents = means.index

    # Plot
    plt.plot(agents, means, label=label or csv_path, linewidth=2)
    plt.fill_between(agents, means - stds, means + stds, alpha=0.3)

    plt.xlabel("#agents")
    plt.ylabel("Runtime (s)")
    plt.title("Runtime vs. #Agents")
    plt.legend()
    plt.grid(True)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_files", nargs='+', help="Paths to CSV files")
    parser.add_argument("--labels", nargs='*', help="Optional labels for each CSV file")
    args = parser.parse_args()

    for i, csv_file in enumerate(args.csv_files):
        label = args.labels[i] if args.labels and i < len(args.labels) else None
        plot_runtime_vs_agents(csv_file, label)

    plt.savefig("result.png")
