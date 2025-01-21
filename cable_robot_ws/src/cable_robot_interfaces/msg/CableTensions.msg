# cable_robot_ws/src/cable_robot_interfaces/msg/CableTension.msg
# Description: Message definition for cable tension values
# Represents cable tensions and their directions
string[] cable_names          # names of each cable
float64[] cable_tensions      # tension values for each cable
geometry_msgs/Vector3[] cable_directions    # direction vectors for each cable