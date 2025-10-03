#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>  // min, max
#include <nlohmann/json.hpp>    // json parsing
#include "uron/base.hpp"
#include "uron/robot.hpp"

using json = nlohmann::json;
// nlohmann::json msgString = { {"data", "hello ROS"} };
// nlohmann::json msgFloat32 = { {"data", 0.123} };

/*
cb(const std_msgs::String::ConstPtr& msg)
    string str(msg->data);
    json j = json::parse(str);
    string value = j["key"];
    vector<string> values = j["keys"];
    vector<boost::any> anys;
    for(unsigned int i = 0; i < values.size(); i++)
        anys.push_back(buffer[values[i]]);
===
send(vector<string> values, vector<boost::any> params)
    json j;
	j["keys"] = values;
	vector<string> output;
	for(unsigned int i = 0; i < params.size(); i++)
	    output.push_back(params[i]);
	j["keys"] = output;
	string str = j.dump();
	std_msgs::String msg;
	msg.data = str;
	pub.publish(msg);
=== examples
auto jsonObject = json::parse(msg->data.c_str());
    for (int i=0;i<num_tags;i++)
	    value = jsonObject[i].get<std::vector<float>>();
auto jsonObject = json::parse(msg->data.c_str());
data_vx_NED_m = jsonObject["vx_NED_m"].get<long double>();
*/
class MyRobot : public uron::UnicycleRobotFSM, public rclcpp::Node {
    uron::Polar robotVel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_command_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::high_resolution_clock::time_point t_start_for_prev_time_;
public:
    MyRobot() : rclcpp::Node("uron_node") {
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10,
                                                                                  std::bind(&MyRobot::odometryCallback,
                                                                                            this,
                                                                                            std::placeholders::_1));
        robot_command_subscriber_ = this->create_subscription<std_msgs::msg::String>("/robot_command", 10,
                                                                                     std::bind(
                                                                                             &MyRobot::robotCommandCallback,
                                                                                             this,
                                                                                             std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/uron/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&MyRobot::worker, this));
        t_start_for_prev_time_ = std::chrono::high_resolution_clock::now();
    }

    void worker() {
        auto t_now = std::chrono::high_resolution_clock::now();
        auto diff_time = std::chrono::duration<double, std::milli>(t_now - t_start_for_prev_time_).count();
        t_start_for_prev_time_ = t_now;
        /*RCLCPP_INFO(rclcpp::get_logger("worker"), "diff_time: %f", diff_time/1000);*/
        this->PeriodicTask(diff_time / 1000);  // milli to second
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robotVel_.linear = msg->twist.twist.linear.x;
        robotVel_.angular = msg->twist.twist.angular.z;
        /*RCLCPP_INFO(rclcpp::get_logger("odometryCallback"), "linear: %f, angular: %f", robotVel_.linear,
                    robotVel_.angular);*/
    }

    // ex):
    // rostopic pub /robot_command std_msgs/String '{data: "{ \"MoveForward\": 1}"}'
    // ros2 topic pub --once /robot_command std_msgs/String '{data: "{ \"TurnDelta\": -0.261799}"}'
    std::string getTypeOfValue(json value) {
        if (value.is_array()) return "array";
        if (value.is_boolean()) return "boolean";
        if (value.is_null()) return "null";
        if (value.is_number_integer()) return "integer";
        if (value.is_number_float()) return "double";
        if (value.is_string()) return "string";
        if (value.is_object()) return "object";
        return "Unknown";
    }

    void robotCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("robotCommandCallback"), "%s", msg->data.c_str());
        try {
            auto obj = json::parse(msg->data.c_str());
            for (json::iterator it = obj.begin(); it != obj.end(); ++it) {
                std::cout << "Key : \"" << it.key() << "\"" << std::endl;
                std::cout << "Type : " << getTypeOfValue(it.value()) << std::endl;
                std::cout << "Value : " << it.value() << std::endl;
                std::cout << std::endl;
                std::string key = it.key();
                if (key == "MoveForward") {
                    this->MoveForward();
                } else if (key == "MoveBackward") {
                    this->MoveBackward();
                } else if (key == "TurnLeft") {
                    this->TurnLeft();
                } else if (key == "TurnRight") {
                    this->TurnRight();
                } else if (key == "MoveDelta") {
                    this->MoveDelta(it.value());
                } else if (key == "TurnDelta") {
                    this->TurnDelta(it.value());
                } else if (key == "Stop") {
                    this->Stop();
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("robotCommandCallback"), "Unknown!! command");
                }
            }
            // float value = obj["MoveForward"].get<float>();
            // ROS_INFO("MoveForward key is %f", value);
        }
        catch (std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("robotCommandCallback"), "received a non-JSON msg. in the SYNC callback.");
            return;
        }
    }

    // An overriding function to set target velocity on the motor controller
    virtual bool ControlVelocity(uron::Polar velocity) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = velocity.linear;
        twist_msg.angular.z = velocity.angular;

        // Publish the Twist message
        cmd_vel_publisher_->publish(twist_msg);
        // RCLCPP_INFO(rclcpp::get_logger("ControlVelocity"), "%.3f, %.3f", twist_msg.linear.x, twist_msg.angular.z);
        return true;
    }

    virtual void PeriodicTask(double elapse) {
        ApplyWheelOdometry(elapse);
        /*RCLCPP_INFO(rclcpp::get_logger("PeriodicTask"),
                    "Pose: %.3f [m] %.3f [m] %.3f [deg] | Velocity: %.3f [m/s] %.3f [deg/s]",
                    poseData.x, poseData.y, uRON_RAD2DEG(poseData.theta),
                    robotVelocity.linear, uRON_RAD2DEG(robotVelocity.angular));*/
        UnicycleRobotFSM::PeriodicTask(elapse);
    }

    // Get wheel odometry and velocity, and update pose and velocity of the robot
    void ApplyWheelOdometry(const double &elapse) {
        uron::Polar robotDist;
        float delta_translation = elapse * robotVel_.linear;
        // RCLCPP_INFO(rclcpp::get_logger("ApplyWheelOdometry"), "delta_translation: %f", delta_translation);
        robotDist.linear = delta_translation;

        float delta_angle = elapse * robotVel_.angular;
        robotDist.angular = delta_angle;
        AddPose(robotDist);
        SetVelocity(robotVel_);
    }

    // An overriding callback function to notify arrival
    virtual void CallbackGoalComplete(void) {
        RCLCPP_INFO(rclcpp::get_logger("CallbackGoalComplete"), "The robot arrived at the goal!");
    }

    // An overriding callback function to notify error
    virtual void CallbackError(int errCode) {
        RCLCPP_ERROR(rclcpp::get_logger("CallbackError"), "An error is happened (ErrCode: %d)!", errCode);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyRobot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("uron_node"), "closed!");
    return 0;
}