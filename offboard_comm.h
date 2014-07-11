#include <tf/tfMessage.h>

#define BUFFER_SIZE 300

#define AXIS_SCALE 1000.0f

#define MAVLINK_OFFBOARD_CONTROL_MODE_NONE 0
#define MAVLINK_OFFBOARD_CONTROL_MODE_RATES 1
#define MAVLINK_OFFBOARD_CONTROL_MODE_ATTITUDE 2
#define MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY 3
#define MAVLINK_OFFBOARD_CONTROL_MODE_POSITION 4

void tf_proc_callback(const tf::tfMessage &m);
void sp_proc_callback(const tf::tfMessage &m);

int main(int argc, char **argv);
