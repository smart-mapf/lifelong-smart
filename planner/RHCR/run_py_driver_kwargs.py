import sys

sys.path.append('build')
sys.path.append('scripts')
import fire
import json
import numpy as np
import warehouse_sim  # type: ignore # ignore pylance warning
import json

from env_search.graph.g_graph import GuidanceGraph
from env_search.utils import read_in_map_by_domain, block_idxs_from_domain


def main(map_filepath, kwargs_file, domain="kiva", seed=0):

    # np.random.seed(seed)
    with open(kwargs_file, "r") as f:
        kwargs = json.load(f)

    _, map_np, _ = read_in_map_by_domain(map_filepath, domain)
    block_idxs = block_idxs_from_domain(domain)
    map_json = json.loads(kwargs["map"])
    weights_matrix = map_json["weights_matrix"]
    g_graph = GuidanceGraph.from_weights_matrix(map_np, block_idxs,
                                                weights_matrix)
    assert g_graph.is_strongly_connected()
    ret = warehouse_sim.run(**kwargs)

    analysis = json.loads(ret)
    # print(analysis.keys())
    # print(np.array(analysis["tile_usage"]).shape)
    # print(np.array(analysis["vertex_wait_matrix"]).shape)
    # # print(np.array(analysis["edge_pair_usage"]).shape)

    # # print(analysis["throughput"], analysis["edge_pair_usage_mean"],
    # #   analysis["edge_pair_usage_std"])
    # print("throughput", analysis["throughput"])
    # print("avg_rotations", analysis["avg_rotations"])


if __name__ == "__main__":
    fire.Fire(main)
