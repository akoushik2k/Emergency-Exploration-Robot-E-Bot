#include <gazebo/gazebo_client.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @class WheelOdometryPlugin
 * @brief A Gazebo plugin for publishing wheel odometry data for a robot.
 *
 * This plugin computes linear and angular velocities based on wheel joint velocities and publishes the data
 * as an odometry message.
 */
class WheelOdometryPlugin : public gazebo::ModelPlugin
{
public:
  /**
   * @brief Constructor for the WheelOdometryPlugin class.
   *
   * Initializes the plugin and sets the ROS2 node to nullptr.
   */
  WheelOdometryPlugin() : gazebo::ModelPlugin(), node_(nullptr)
  {
  }

  /**
   * @brief Loads the plugin with the given model and initializes the transport node.
   *
   * This method sets up the Gazebo node, advertises the odometry topic, and connects to the world update event.
   *
   * @param model Pointer to the Gazebo model to which this plugin is attached.
   * @param sdf Pointer to the SDF element of the plugin (unused).
   */
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

  /**
   * @brief Called at each simulation update event.
   *
   * This method retrieves wheel velocities, calculates linear and angular velocities,
   * and publishes an odometry message based on these values.
   *
   * @param info Update information from Gazebo (currently unused).
   */
  void OnUpdate(const gazebo::common::UpdateInfo & /*info*/)
  {
    // Get wheel joint positions and velocities
    gazebo::physics::JointPtr left_front_wheel_joint = model_->GetJoint("left_front_wheel_joint");
    gazebo::physics::JointPtr right_front_wheel_joint = model_->GetJoint("right_front_wheel_joint");
    gazebo::physics::JointPtr left_rear_wheel_joint = model_->GetJoint("left_rear_wheel_joint");
    gazebo::physics::JointPtr right_rear_wheel_joint = model_->GetJoint("right_rear_wheel_joint");

    // Retrieve individual wheel velocities
    double left_front_wheel_velocity = left_front_wheel_joint->GetVelocity(0).Double();
    double right_front_wheel_velocity = right_front_wheel_joint->GetVelocity(0).Double();
    double left_rear_wheel_velocity = left_rear_wheel_joint->GetVelocity(0).Double();
    double right_rear_wheel_velocity = right_rear_wheel_joint->GetVelocity(0).Double();

    // Calculate linear velocity as the average of wheel velocities
    double linear_velocity = (left_front_wheel_velocity + left_rear_wheel_velocity +
                              right_front_wheel_velocity + right_rear_wheel_velocity) / 2.0;
                              
    // Calculate angular velocity based on the wheel separation
    double angular_velocity = (-left_front_wheel_velocity - left_rear_wheel_velocity +
                               right_front_wheel_velocity + right_rear_wheel_velocity) / wheel_separation_;

    // Create and populate odometry message
    geometry_msgs::msg::Twist odometry_msg;
    odometry_msg.linear.x = linear_velocity;
    odometry_msg.angular.z = angular_velocity;

    // Publish odometry message
    odometry_pub_->Publish(odometry_msg);
  }

private:
  gazebo::physics::ModelPtr model_; ///< Pointer to the robot model in Gazebo
  gazebo::transport::NodePtr node_; ///< Gazebo node for handling transport
  gazebo::transport::PublisherPtr odometry_pub_; ///< Publisher for the odometry topic
  gazebo::event::ConnectionPtr update_connection_; ///< Connection to Gazebo's update event

  const double wheel_separation_ = 0.76; ///< Separation distance between the wheels (adjustable for the robot)
};

// Register this plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(WheelOdometryPlugin)
