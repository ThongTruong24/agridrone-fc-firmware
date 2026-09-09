uint64 timestamp                        # time since system start (microseconds)

# Zero-based actuator slot (A1=0 ... A8=7).
uint8 ACTUATOR_INDEX_MIN = 0
uint8 ACTUATOR_INDEX_MAX = 7

uint8 SOURCE_RC       = 0
uint8 SOURCE_MAVLINK  = 1
uint8 SOURCE_DDS      = 2
uint8 SOURCE_INTERNAL = 3

uint8 actuator_index
uint8 value                            # actuator command in range [0, 255]
uint8 source                           # one of SOURCE_*

# Allow a source adapter to enqueue one command for every actuator.
uint8 ORB_QUEUE_LENGTH = 8
