import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Rectangle

fig_dir = Path.home() / "dev/data/exp/womapf/follow/figures"
fig_dir.mkdir(parents=True, exist_ok=True)

lacam_init_dir = Path.home() / "dev/data/exp/womapf/follow/2024-11-04T20-58-33.820"
lacam_no_following_dir = (
    Path.home() / "dev/data/exp/womapf/follow/2024-11-04T21-24-52.874"
)


def reformat_result(result):
    keys = result[0]
    result = result[1:]

    foo = []
    for i in range(len(result)):
        bar = {}
        for j in range(len(keys)):
            val = result[i][j]
            try:
                val = int(val)
            except ValueError:
                pass
            bar[keys[j]] = val
        foo.append(bar)
    return foo


def read_yaml(file_path):
    with open(file_path, mode="r") as file:
        data = yaml.safe_load(file)
    return data


def read_csv(file_path):
    with open(file_path, mode="r", newline="") as file:
        reader = csv.reader(file)
        data = [row for row in reader]
    return data


# Read and parse result.csv from each directory
init_config = read_yaml(lacam_init_dir / "config.yaml")
init_result = read_csv(lacam_init_dir / "result.csv")
no_follow_config = read_yaml(lacam_no_following_dir / "config.yaml")
no_following_result = read_csv(lacam_no_following_dir / "result.csv")

assert init_config["maps"] == no_follow_config["maps"]
maps = init_config["maps"]
n_agents = list(
    range(
        init_config["num_min_agents"],
        init_config["num_max_agents"] + 1,
        init_config["num_interval_agents"],
    )
)

init_result = reformat_result(init_result)
no_following_result = reformat_result(no_following_result)

# Organize the data by map, then by (scen, num_agents) tuple
organized_data_init = {}
organized_data_no_following = {}

for map_name in maps:
    organized_data_init[map_name] = {}
    for result in init_result:
        if map_name in result["map_name"]:
            key = (result["scen"], result["num_agents"])
            organized_data_init[map_name][key] = result

    organized_data_no_following[map_name] = {}
    for result in no_following_result:
        if map_name in result["map_name"]:
            key = (result["scen"], result["num_agents"])
            organized_data_no_following[map_name][key] = result

# for field in ["makespan"]:
for field in ["soc", "sum_of_loss", "makespan", "comp_time", "search_iteration"]:
    # print(f"Generating plots for {field}")
    (fig_dir / field).mkdir(parents=False, exist_ok=True)
    for map_name, data in organized_data_init.items():
        fig = plt.figure()
        init = {k: v for k, v in sorted(data.items(), key=lambda item: item[0][1])}
        bar_x = np.arange(len(init))
        vals_init = [v[field] for v in init.values()]
        for i in range(len(vals_init)):
            k = list(init.keys())[i]
            if not init[k]["solved"]:
                vals_init[i] = 0
        no_follow = {
            k: v
            for k, v in sorted(
                organized_data_no_following[map_name].items(),
                key=lambda item: item[0][1],
            )
        }
        unique_n = sorted(list(set([v["num_agents"] for v in no_follow.values()])))
        vals_no_following = [v[field] for v in no_follow.values()]
        for i in range(len(vals_no_following)):
            k = list(no_follow.keys())[i]
            if not no_follow[k]["solved"]:
                vals_no_following[i] = 0
        plt.bar(
            bar_x,
            vals_no_following,
            width=1.0,
            edgecolor="none",
        )
        plt.bar(
            bar_x,
            vals_init,
            width=1.0,
            alpha=0.7,
            edgecolor="none",
        )
        plt.legend(["No following (pLaCAM)", "Baseline (LaCAM)"], loc="center left")

        # Add background rectangles for each label range
        plt.xticks([])
        spacing = len(bar_x) / len(unique_n)
        label_positions = [(i * spacing) + (spacing / 2) for i in range(len(unique_n))]
        for i, pos in enumerate(label_positions):
            plt.gca().add_patch(
                Rectangle(
                    (pos - (spacing / 2) - 0.5, 0),
                    spacing,
                    max(vals_init + vals_no_following) + 1,
                    color="lightgray",
                    alpha=0.3,
                    zorder=0,
                )
            )
            plt.text(
                pos,
                max(vals_init + vals_no_following) + 0.5,
                f"n={unique_n[i]}",
                ha="center",
                va="bottom",
            )

        plt.gca().get_yaxis().set_major_formatter(
            plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x)))
        )

        plt.title(f"Map: {map_name}")
        plt.xlabel(f"{len(bar_x)//len(unique_n)} instances per n")
        plt.ylabel(field)

        plt.gcf().set_size_inches(10, 5)

        fname = fig_dir / field / f"{map_name}.png"
        print(f"Saving {fname}")
        plt.savefig(fname, dpi=300)
        plt.close(fig)
# plt.show()
