# Tools for BLAM

This module contains scripts to help debug BLAM.

Always source your `internal/devel/setup.bash` (or `internal/devel_isolated/setup.bash`) before running the Python nodes. The `blam_slam` ROS node must be running while executing any of the following services.

## `manual_graph_edge.py`
Allows the user to add a `BetweenEdge` to the graph that connects two keys in the isam2 pose graph.

```sh
manual_graph_edge.py key1 key2
```

`key1` and `key2` are unsigned integers that represent the keys of the two pose graph nodes to be connected.

While the service is executed, the current state of the factor graph will be saved to a log file to `~/Desktop/factor_graph.txt`.

## `save_graph.py`
Allows the user to save the entire pose graph, including all point clouds attached to it, to a zip file.

```sh
save_graph.py filename.zip
```

`filename.zip` is the path of the zip file which should be generated. Watch the output of the `blam_slam` ROS node for any error messages, or to learn the absolute path of the file that was generated.
