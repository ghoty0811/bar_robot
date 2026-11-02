# bar_robot

ROS 2 Python package containing the `bar_robot` task node for a Doosan M0609 robot.

## Dependencies

- ROS 2 (the workspace currently uses the Doosan ROS 2 driver packages `dsr_controller2` and `dsr_common2`)
- Python modules supplied by the Doosan driver: `DR_init`, `DSR_ROBOT2`, and `DR_common2`

Build this package from a ROS 2 workspace after placing it under `src/` and installing the Doosan driver dependencies:

```bash
colcon build --packages-select bar_robot
source install/setup.bash
ros2 run bar_robot bar_robot
```

The node currently selects robot ID `dsr01` and model `m0609`. It contains fixed joint and Cartesian waypoints and, when run, calls `washUsedCup()` by default. Review those settings and the workcell waypoints for your robot before running it.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
