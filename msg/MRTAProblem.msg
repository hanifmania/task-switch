Header header

int64 problem_id                        # Problem ID in Monte Carlo Simulation
int64 robot_number                      # Total Number of Robot
int64[] robot_position_x                # Initial Position of Robot in x Dimension
int64[] robot_position_y                # Initial Position of Robot in y Dimension
float64[] robot_velocity                  # Max Velocity of the Robot
float64[] robot_battery_level             # Robot Battery Level 
float64[] robot_battery_discharge_rate  # Robot Battery Discharge Rate
float64[] robot_lambda                    # Robot Usage Penalty (Higher Value means higher penalty)

int64 task_number                       # Total number of task 
int64[] task_position_x                 # Initial Position of Task in x Dimension
int64[] task_position_y                 # Initial Position of Task in y Dimension
float64[] task_velocity_x                 # Velocity of Task in x dimension
float64[] task_velocity_y                 # Velocity of Task in y dimension
float64[] task_priority                   # Task priority level
float64[] task_operation_time             # Time needed for the robot to finish the task

int64 time_horizon