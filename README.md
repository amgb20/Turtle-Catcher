# Turtle Catcher

The Master turtle catches the nearest sister turtle.

# Initialise

- Source your code in your bashrc
```bash
source .bashrc
```

- Build the project

```bash
colcon build --packages-select my_robot_bringup --symlink-install 
```

- Run the launching file
```bash
ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.py 
```


