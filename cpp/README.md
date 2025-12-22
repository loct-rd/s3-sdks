### How to Use

```shell
mkdir build
cd build
cmake ..
make
./example
```

By default, the executable connects to **192.168.11.100** and does **not** request map data (occupancy grid map) or map points.

If you want to specify a different IP address and retrieve map data and map points, run the executable with the following options:

```shell
./example 192.168.11.200 --map --points
```

This example program simply receives sensor and map data from the S3 device and prints the results to the console. Please use it as a reference and implement your own applications according to your requirements.