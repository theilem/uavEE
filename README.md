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
uavEE contains plugins for XPlane11. It is configured through config files. Two config files are provided:

sitl.json:

    For software in the loop. SensorData is published directly on shared memory and is listening to actuation published by FlightControl in uavAP.
    
chitl.json:

    For computational hardware in the loop. This uses redis to communicate with uavAP running on a different board. The config file should be adapted according to the redis-server.

### Installing main simulation interface plugin
```shell script
# Install the plugin
mkdir -p [xplane_dir]/Resources/plugins/uavee/64/
ln -s [uavee_dir]/bld/release/src/XPlaneInterface/XPlanePlugin.xpl [xplane_dir]/Resources/plugins/uavee/64/lin.xpl

# Install the config
mkdir [xplane_dir]/uavEEConfig
cp [uavee_dir]/config/*.json [xplane_dir]/uavEEConfig/
```

### Installing the ServoListener Helper plugin
```shell script
mkdir -p [xplane_dir]/Resources/plugins/uavEE-ServoListener/64/
ln -s [uavee_dir]/bld/release/src/XPlaneInterface/ServoListener.xpl [xplane_dir]/Resources/plugins/uavEE-ServoListener/64/lin.xpl
```

### Running plugin
1. Starting a new flight in XPlane.
2. Click Plugins/uavEE/Select Config/[config_file]
3. Click Plugins/uavEE/Start Node
4. Use Plugins/uavEE/Enable-Disable Autopilot to enable-disable autopilot
