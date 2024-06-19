#include <gazebo/gazebo_client.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/twist.hpp>

class WheelOdometryPlugin : public gazebo::ModelPlugin
{
public:
  WheelOdometryPlugin() : gazebo::ModelPlugin(), node_(nullptr)
  {
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr /*sdf*/) override
  {
    model_ = model;
    node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node_->Init();

    // Create a publisher for the odometry topic
    odometry_pub_ = node_->Advertise<geometry_msgs::msg::Twist>("~/odometry", 10);

    // Connect to the world update event
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&WheelOdometryPlugin::OnUpdate, this, std::placeholders::_1));
  }

  void OnUpdate(const gazebo::common::UpdateInfo & /*info*/)
  {
    // Get wheel joint positions and velocities
    gazebo::physics::JointPtr left_front_wheel_joint = model_->GetJoint("left_front_wheel_joint");
    gazebo::physics::JointPtr right_front_wheel_joint = model_->GetJoint("right_front_wheel_joint");
    gazebo::physics::JointPtr left_rear_wheel_joint = model_->GetJoint("left_rear_wheel_joint");
    gazebo::physics::JointPtr right_rear_wheel_joint = model_->GetJoint("right_rear_wheel_joint");

    double left_front_wheel_velocity = left_wheel_joint->GetVelocity(0).Double();
    double right_front_wheel_velocity = right_wheel_joint->GetVelocity(0).Double();
    double left_rear_wheel_velocity = left_wheel_joint->GetVelocity(0).Double();
    double right_rear_wheel_velocity = right_wheel_joint->GetVelocity(0).Double();
  


    // Calculate linear and angular velocities
    double linear_velocity = (left_front_wheel_velocity + left_rear_wheel_velocity + right_front_wheel_velocity + right_rear_wheel_velocity) / 2.0;
    double angular_velocity = (-left_front_wheel_velocity - left_rear_wheel_velocity + right_front_wheel_velocity + right_rear_wheel_velocity) / wheel_separation_;

    // Create and publish odometry message
    geometry_msgs::msg::Twist odometry_msg;
    odometry_msg.linear.x = linear_velocity;
    odometry_msg.angular.z = angular_velocity;
    odometry_pub_->Publish(odometry_msg);
  }

private:
  gazebo::physics::ModelPtr model_;
  gazebo::transport::NodePtr node_;
  gazebo::transport::PublisherPtr odometry_pub_;
  gazebo::event::ConnectionPtr update_connection_;

  const double wheel_separation_ = 0.76; // Adjust based on your robot's wheel separation
};

GZ_REGISTER_MODEL_PLUGIN(WheelOdometryPlugin)
