Header header
geometry_msgs/Pose pose

uint32 id
string name

float32 left_size   # m
float32 right_size  # m
float32 front_size  # m
float32 back_size   # m
float32 height      # m

uint32 mode         # 0-none, 1-joystick, 2-remote, 3-auto
float32 battery     # 0%~100%
float32 linear_velocity   # m/s
float32 angular_velocity  # rad/s

uint32 magic        # 0x58535853
