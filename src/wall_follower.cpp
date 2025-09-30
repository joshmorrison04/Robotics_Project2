#include <iostream>
#include <cmath>
#include <signal.h>

#include <mbot_bridge/robot.h>
#include <wall_follower/common/utils.h>  // our helper functions

bool ctrl_c_pressed;
void ctrlc(int) { ctrl_c_pressed = true; }

int main(int argc, const char *argv[]) {
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize robot
    mbot_bridge::MBot robot;
    std::vector<float> ranges; // holds distances from Lidar
    std::vector<float> thetas; // holds angles for each distance

    // Control parameters
    float setpoint = 0.5;    // how far we want to stay from wall (m)
    float tolerance = 0.05;  // wiggle room (m)
    float v = 0.2;           // correction speed (m/s)

    while (true) {
        // 1. Get Lidar scan
        robot.readLidarScan(ranges, thetas);

        // 2. Find the nearest wall (smallest distance > 0)
        int min_idx = findMinDist(ranges);
        if (min_idx < 0) continue; // skip if nothing valid this cycle

        // 3. Distance and angle to the nearest wall
        float dist_to_wall = ranges[min_idx];
        float angle_to_wall = thetas[min_idx];

        // 4. Make a vector pointing directly AT the wall
        //    (cos, sin) turns angle into x/y arrow
        std::vector<float> wall_vec = {cos(angle_to_wall),
                                       sin(angle_to_wall),
                                       0.0};

        // 5. Cross product with z-axis rotates that vector 90°
        //    Now the arrow points PARALLEL to the wall
        std::vector<float> z_axis = {0.0, 0.0, 1.0};
        std::vector<float> drive_vec = crossProduct(wall_vec, z_axis);

        // 6. Correction step:
        //    If too far → add a push toward wall
        //    If too close → add a push away from wall
        if (dist_to_wall > setpoint + tolerance) {
            drive_vec[0] += v * cos(angle_to_wall);
            drive_vec[1] += v * sin(angle_to_wall);
        }
        else if (dist_to_wall < setpoint - tolerance) {
            drive_vec[0] -= v * cos(angle_to_wall);
            drive_vec[1] -= v * sin(angle_to_wall);
        }
        // else: we’re in the good zone → just keep parallel motion

        // 7. Send the drive command to robot
        robot.drive(drive_vec[0], drive_vec[1], 0.0);

        // Stop cleanly if Ctrl-C pressed
        if (ctrl_c_pressed) break;
    }

    robot.stop();
    return 0;
}
