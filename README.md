# Haply's HardwareAPI plugin

[//]: # (plugin_doc_begin)

Plugin to interface with Haply devices, such as the Inverse3 and the VerseGrip.

Note: for every open device (e.g. Inverse3, VerseGrip, ...) the plugin spawns a separate thread for running the control loop at 1kHz.

## Special operating modes:

### Halfplane constraint

Keeps the end-effector to one side of a plane specified by passing point (x, y, z) and plane normal (nx, ny, nz).

Enter this mode by calling `simHaply.setInverse3Constraint`.

### Attractor

Attracts the end-effector towards a specific point (x, y, z) with a force proportional to distance, whose gain and saturation can be set with `simHaply.setInverse3ForceParams`.

Enter this mode by calling `simHaply.setInverse3Attractor`.

[//]: # (plugin_doc_end)

### Compiling

1. Install required packages for simStubsGen: see simStubsGen's [README](https://github.com/CoppeliaRobotics/include/blob/master/simStubsGen/README.md)
2. Checkout, compile and install into CoppeliaSim:
```sh
$ git clone https://github.com/CoppeliaRobotics/simHaply.git
$ cd simHaply
$ git checkout coppeliasim-v4.5.0-rev0
$ mkdir -p build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ cmake --build .
$ cmake --install .
```

NOTE: replace `coppeliasim-v4.5.0-rev0` with the actual CoppeliaSim version you have.
