uint64 timestamp                         # time since system start (microseconds)
uint32 sequence                          # sequence assigned by the external planner

uint8 STATE_CLEAR = 0                    # planner alive, mission target is unobstructed
uint8 STATE_ACTIVE = 1                   # use the temporary Local NED target
uint8 STATE_NO_PATH = 2                  # planner cannot provide a safe path
uint8 STATE_FAULT = 3                    # planner reported an internal fault
uint8 state

uint8 FLAG_USE_Z = 1                     # use z instead of preserving mission altitude
uint8 flags

uint8 xy_reset_counter                   # vehicle_local_position reset counter used for x/y
uint8 z_reset_counter                    # vehicle_local_position reset counter used for z

uint8 source_system                      # MAVLink sender system ID
uint8 source_component                   # MAVLink sender component ID

float32 x                                # absolute Local NED North position (m)
float32 y                                # absolute Local NED East position (m)
float32 z                                # absolute Local NED Down position (m)
float32 max_speed                        # optional upper speed bound (m/s); <= 0 disables it
