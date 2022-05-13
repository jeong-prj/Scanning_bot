# The actual position by the encoder value (rad)
float64[2] position

# The actual velocity measured by the encoder as the actual RPM value (rad/s)
float64[2] velocity

# The current flowing through the motors (A)
float64[2] current             

# Internal temperature of the motor controller (C)
float64 temperature 

# The state of the controller fault conditions
string[] fault_flags 