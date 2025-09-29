#include <iostream>
#include <mbot_bridge/robot.h>
#include <wall_follower/common/utils.h>

int main(int argc, char *argv[]) {
    // Initialize the robot
    mbot_bridge::Mbot robot;

    int velx = 0.2; 
    int ang_speed = 0.5; 


    // Function to drive in a square
    auto driveSquare = [&](mbot_bridge::Mbot& robot) {

        for (int i = 0; i < 4; i++) {
            // Drive straight for 1m -> Move forward 0.2 m/s. Sleep for 5s (bcuz 0.2 m/s * 5 = 1 meter) 
            robot.drive(rob_velx, 0, 0); // robot.drive(forward/backwards, sideways, angle) -> The speed for all this movements. 
            robot.sleepFor(5.0); // Sleep makes robot drive for that many seconds

            // Turn 90 degrees
            robot.drive(0, 0, ang_speed); // angular speed = 0.5 radians per second. 
            robot.sleepFor(3.14);  
            // Calculation for turning 90degrees logic: 
            // 90 degrees = 1.57 radians
            // angular speed = 0.5 radians 
            // Time = angle/angular speed therefore: 1.57/0.5 = 3.14
        }

    };

    // Call function 3 times
    for (int i = 0; i < 3; i++) {
        driveSquare(robot);
    }

    // Stop the robot
    std::cout << "Stopping the robot!!" << std::endl;
    robot.stop();

    return 0;
}
