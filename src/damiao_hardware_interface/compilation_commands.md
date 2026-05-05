From workspace root:

```bash
cd src/damiao_hardware_interface
mkdir -p build_standalone
cd build_standalone
cmake -S ../standalone -B . -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

Then export the folder containing the compiled library in the Python path:

```bash
export PYTHONPATH=../ulixarm_ros2/src/damiao_hardware_interface/build_standalone:$PYTHONPATH
```

