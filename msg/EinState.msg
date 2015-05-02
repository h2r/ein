# the entire relevant state of the ein system

# is baxter in zero g mode? (0=no, 1=yes)
int32 zero_g

# IDLING = 0, SCANNING = 1, PICKING = 2, PLACING = 3, HANDING = 4
int32 patrol_state

# HAND = 0, WAREHOUSE = 1
int32 place_mode

# ONCE = 0, LOOP = 1
int32 scan_mode

# john says this is something?
int32 idle_mode

object_recongition_msgs/RecognizedObject[] objects
