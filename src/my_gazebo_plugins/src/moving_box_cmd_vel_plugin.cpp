#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <thread>
#include <memory>

namespace gazebo
{
  class MovingBoxCmdVelPlugin : public ModelPlugin
  {
  public:
    MovingBoxCmdVelPlugin() = default;

    void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = model;

      // IMPORTANT: Do NOT call rclcpp::init() here!

      // Create ROS2 node
      ros_node = std::make_shared<rclcpp::Node>("moving_box_cmd_vel");

      // Create subscriptions
      cmd_vel_sub = ros_node->create_subscription<geometry_msgs::msg::Twist>(
        "/moving_box/cmd_vel", 10,
        std::bind(&MovingBoxCmdVelPlugin::OnCmdVel, this, std::placeholders::_1));

      odom_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
        "/moving_box/odom", 10,
        std::bind(&MovingBoxCmdVelPlugin::OnOdom, this, std::placeholders::_1));

      // Setup executor and spin thread
      ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      ros_executor->add_node(ros_node);
      ros_spin_thread = std::thread([this]() { ros_executor->spin(); });

      last_print_time = ros_node->now();

      // Connect to the world update event
      updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MovingBoxCmdVelPlugin::OnUpdate, this));
    }

    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      last_cmd_vel = *msg;
    }

    void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      last_odom = *msg;
    }

    void OnUpdate()
    {
      // Apply velocities to the model
      model->SetLinearVel(ignition::math::Vector3d(
        last_cmd_vel.linear.x,
        last_cmd_vel.linear.y,
        last_cmd_vel.linear.z));

      model->SetAngularVel(ignition::math::Vector3d(
        last_cmd_vel.angular.x,
        last_cmd_vel.angular.y,
        last_cmd_vel.angular.z));

      // Log position every 1 second if odom received
      auto now = ros_node->now();
      if ((now - last_print_time).seconds() >= 1.0 && last_odom.header.stamp.sec != 0)
      {
        auto pos = last_odom.pose.pose.position;
        RCLCPP_INFO(
          ros_node->get_logger(),
          "Current position: x=%.3f y=%.3f z=%.3f",
          pos.x, pos.y, pos.z);
        last_print_time = now;
      }
    }

    ~MovingBoxCmdVelPlugin()
    {
      if (ros_executor)
        ros_executor->cancel();

      if (ros_spin_thread.joinable())
        ros_spin_thread.join();
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor;
    std::thread ros_spin_thread;

    geometry_msgs::msg::Twist last_cmd_vel{};
    nav_msgs::msg::Odometry last_odom{};
    rclcpp::Time last_print_time;
  };

  GZ_REGISTER_MODEL_PLUGIN(MovingBoxCmdVelPlugin)
}
