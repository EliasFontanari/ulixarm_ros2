cd /home/elias/Documents/robot_in_prova/ulixarm_ros2/src/damiao_hardware_interface
rm -rf build_standalone
mkdir build_standalone && cd build_standalone
cmake -S ../standalone -B . -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)


export PYTHONPATH=/home/elias/Documents/robot_in_prova/ulixarm_ros2/src/damiao_hardware_interface/build_standalone:$PYTHONPATH
python3 -c "import damiao_motor_control as dmc; print(dmc)"