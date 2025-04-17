# LiDAR Corners

This project is under active development. The current goal is to
extract and visualize corner-like features from LiDAR point clouds.
Below is a short demo of the current processing pipeline:

![Demo](assets/demo.gif)

## Build

```sh
make build
```

or you can open the `Makefile` to create your own command.

## Usage

```sh
make launch
```

or you can open the `Makefile` to create your own command.

The parameters can be found in
[`config/lidar_detect_board_node.yaml`](config/lidar_detect_board_node.yaml).

## TODOs

- Write an advanced algorithm to remove the ground point rather than
  filtering with z-value.
- Filter out noisy planes.
