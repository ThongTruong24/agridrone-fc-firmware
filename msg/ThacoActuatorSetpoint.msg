uint64 timestamp                        # time since system start (microseconds)

uint8 NUM_ACTUATORS = 8

uint8[8] value                          # final output [0, 255], including clamped safe/default value
bool[8] valid                           # true: accepted command active; false: manager supplied safe/default value
