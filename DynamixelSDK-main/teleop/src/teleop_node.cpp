/**
 * teleop_node.cpp
 * A ROS node to control two DYNAMIXEL motors in Velocity Control Mode using keyboard input.
 * Publishes to /set_velocity topic to set motor velocities.
 *
 * Keyboard controls:
 *   - w: Both motors forward (positive velocity)
 *   - s: Both motors backward (negative velocity)
 *   - d: Motor ID 1 forward, Motor ID 2 backward
 *   - a: Motor ID 1 backward, Motor ID 2 forward
 *   - q: Stop both motors and quit
 * Auto-stop: Motors stop if no key is pressed for 1 second.
 */

#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <dynamixel_sdk_examples/SetVelocity.h>

class TeleopNode {
public:
  TeleopNode() : nh_(), velocity_step_(5), max_velocity_(200), timeout_(1.0) {
    pub_ = nh_.advertise<dynamixel_sdk_examples::SetVelocity>("/set_velocity", 10);
    velocities_[0] = 0; // Motor ID 1
    velocities_[1] = 0; // Motor ID 2
    last_key_time_ = ros::Time::now();
    ROS_INFO("Teleop node started. Use w/s for both motors, d/a for opposite directions, q to quit.");
    ROS_INFO("Motors will auto-stop if no key is pressed for %.1f seconds.", timeout_);
  }

  void run() {
    setNonBlockingInput();
    while (ros::ok()) {
      int c = getch();
      if (c != -1) { // Key pressed
        processKey(c);
        last_key_time_ = ros::Time::now(); // Update last key press time
      }

      // Check for auto-stop
      if ((ros::Time::now() - last_key_time_).toSec() > timeout_) {
        stopMotors();
      }

      ros::spinOnce();
      usleep(10000); // 10ms delay
    }
    stopMotors();
    restoreInput();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  int velocity_step_;
  int max_velocity_;
  double timeout_; // Timeout in seconds for auto-stop
  int velocities_[2]; // Velocities for motor ID 1 and ID 2
  ros::Time last_key_time_; // Time of last key press

  void setNonBlockingInput() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  }

  void restoreInput() {
    struct termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);
    oldt.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  int getch() {
    return getchar();
  }

  void processKey(int key) {
    dynamixel_sdk_examples::SetVelocity msg;
    bool publish = false;

    switch (key) {
      case 'w':
        velocities_[0] = std::min(velocities_[0] + velocity_step_, max_velocity_);
        velocities_[1] = std::min(velocities_[1] + velocity_step_, max_velocity_);
        publish = true;
        break;
      case 's':
        velocities_[0] = std::max(velocities_[0] - velocity_step_, -max_velocity_);
        velocities_[1] = std::max(velocities_[1] - velocity_step_, -max_velocity_);
        publish = true;
        break;
      case 'a':
        velocities_[0] = std::min(velocities_[0] + velocity_step_, max_velocity_);
        velocities_[1] = std::max(velocities_[1] - velocity_step_, -max_velocity_);
        publish = true;
        break;
      case 'd':
        velocities_[0] = std::max(velocities_[0] - velocity_step_, -max_velocity_);
        velocities_[1] = std::min(velocities_[1] + velocity_step_, max_velocity_);
        publish = true;
        break;
      case 'q':
        ROS_INFO("Quitting teleop node...");
        stopMotors();
        ros::shutdown();
        return;
    }

    if (publish) {
      // Send reversed velocity for motor ID 1
      msg.id = 1;
      msg.velocity = -velocities_[0];  // Invert direction
      pub_.publish(msg);
      ROS_INFO("Motor ID: %d, Velocity: %d", msg.id, msg.velocity);

      msg.id = 2;
      msg.velocity = velocities_[1];
      pub_.publish(msg);
      ROS_INFO("Motor ID: %d, Velocity: %d", msg.id, msg.velocity);
    }
  }

  void stopMotors() {
    if (velocities_[0] != 0 || velocities_[1] != 0) {
      dynamixel_sdk_examples::SetVelocity msg;
      velocities_[0] = 0;
      velocities_[1] = 0;
      msg.id = 1;
      msg.velocity = 0;
      pub_.publish(msg);
      msg.id = 2;
      msg.velocity = 0;
      pub_.publish(msg);
      ROS_INFO("Stopped both motors (auto-stop).");
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_node");
  TeleopNode teleop;
  teleop.run();
  return 0;
}

