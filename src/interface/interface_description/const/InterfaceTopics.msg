## Topics used for interfacing ##

# Topics for data received from STM
string STM_CODER = "/STM/Coder"
string STM_POSITION = "/STM/Position"
string STM_PWM = "/STM/PWM"
string STM_SPEED = "/STM/Speed"
string STM_ROBOT_BLOCKED = "/STM/RobotBlocked"

# Topics for data sent to STM
string STM_SET_MODE = "/STM/SetMode"
string STM_SET_SPEED = "/STM/SetSpeed"
string STM_GO_TO_ANGLE = "/STM/GoToAngle"
string STM_GO_TO = "/STM/GoTo"
string STM_ROTATE = "/STM/Rotation"
string STM_SET_LEFT_PID = "/STM/LeftPid"
string STM_SET_RIGHT_PID = "/STM/RightPid"
string STM_SET_ALL_PID = "/STM/AllPid"
string STM_SET_PWM = "/STM/PWM"
string STM_SET_POSE = "/STM/SetPose"
string STM_SET_PARAMS = "/STM/SetParam"

# Topic received from arduino
string ARDUINO_SONAR_DISTANCE = "/ARDUINO/SonarDistance"

# Topics for data received from all elements
string ALL_ORDER_DONE = "/ALL/Done"
string ALL_PING = "/ALL/Ping"