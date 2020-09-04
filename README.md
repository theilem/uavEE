# uavEE
Modular Emulation Environment for rapid Development and Testing of Unmanned Aerial Vehicles. Old version can be found on branch ros_based.

## Dependencies
 - cpsCore (https://github.com/theilem/cpsCore.git)
 - uavAP (https://github.com/theilem/uavAP.git)
 - uavGS [For monitoring only] (https://github.com/theilem/uavGS.git)

## Cloning and Compiling

```shell script
git clone https://github.com/theilem/uavEE.git

cd uavEE
mkdir -p bld/release && cd bld/release

cmake -DCMAKE_BUILD_TYPE=Release ../../
make
```

## XPlane Plugin
uavEE contains a plugin for XPlane11. It is configured through config files. Two config files are provided:
sitl.json:
    For software in the loop. SensorData is published directly on shared memory and is listening to actuation published by FlightControl in uavAP.
chitl.json:

### Installing plugin
```shell script
# Install the plugin
mkdir -p [xplane_dir]/Resources/plugins/uavee/64/
ln -s [uavee_dir]/bld/release/src/XPlaneInterface/lin.xpl [xplane_dir]/Resources/plugins/uavee/64/lin.xpl

# Install the config
mkdir [xplane_dir]/uavEEConfig
cp [uavee_dir]/config/*.json [xplane_dir]/uavEEConfig/
```

### Running plugin
1. Starting a new flight in XPlane.
2. Click Plugins/uavEE/Select Config/[config_file]
3. Click Plugins/uavEE/Start Node
4. Use Plugins/uavEE/Enable-Disable Autopilot to enable-disable autopilot
