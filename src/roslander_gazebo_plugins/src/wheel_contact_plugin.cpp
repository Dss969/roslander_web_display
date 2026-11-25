#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace roslander_gazebo_plugins
{
class WheelContactPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->turning_lateral_slip_ = sdf->Get<double>("lateral_slip", 0.1).first;
    this->longitudinal_slip_ =
      sdf->Get<double>("longitudinal_slip", 0.0).first;
    this->cmd_vel_timeout_ = sdf->Get<double>("cmd_vel_timeout", 0.5).first;
    this->angular_threshold_ = sdf->Get<double>("angular_threshold", 0.01).first;

    std::vector<std::string> wheel_links;
    if (sdf->HasElement("wheel_link")) {
      auto wheel = sdf->GetElement("wheel_link");
      while (wheel) {
        wheel_links.push_back(wheel->Get<std::string>());
        wheel = wheel->GetNextElement("wheel_link");
      }
    }

    if (wheel_links.empty()) {
      gzerr << "[roslander_wheel_contact] No <wheel_link> elements were supplied.\n";
      return;
    }

    for (const auto & link_name : wheel_links) {
      const auto link = model->GetLink(link_name);
      if (!link) {
        gzerr << "[roslander_wheel_contact] Link not found: " << link_name << "\n";
        continue;
      }

      const auto collisions = link->GetCollisions();
      if (collisions.empty()) {
        gzerr << "[roslander_wheel_contact] No collision on link: " << link_name << "\n";
        continue;
      }

      for (const auto & collision : collisions) {
        auto ode_surface = boost::dynamic_pointer_cast<gazebo::physics::ODESurfaceParams>(
          collision->GetSurface());
        if (!ode_surface) {
          gzerr << "[roslander_wheel_contact] ODE surface unavailable for: "
                << collision->GetScopedName() << "\n";
          continue;
        }
        this->surfaces_.push_back(ode_surface);
      }
    }

    this->ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!this->ros_node_) {
      return;
    }
    this->cmd_vel_subscription_ =
      this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", rclcpp::QoS(1).best_effort(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->turning_requested_.store(
          std::abs(msg->angular.z) > this->angular_threshold_);
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        this->last_command_ns_.store(
          std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
      });

    // Start with rigid lateral contact. It is relaxed only while the drive
    // controller requests different left and right wheel velocities.
    this->SetSlip(0.0);
    this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&WheelContactPlugin::OnUpdate, this));

    gzmsg << "[roslander_wheel_contact] Configured adaptive lateral slip on "
          << wheel_links.size() << " wheel links.\n";
  }

private:
  void OnUpdate()
  {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    const auto age_ns = now_ns - this->last_command_ns_.load();
    const bool command_is_fresh =
      age_ns >= 0 && age_ns < static_cast<int64_t>(this->cmd_vel_timeout_ * 1e9);
    const bool turning = command_is_fresh && this->turning_requested_.load();
    this->SetSlip(turning ? this->turning_lateral_slip_ : 0.0);
  }

  void SetSlip(const double lateral_slip)
  {
    if (std::abs(lateral_slip - this->current_lateral_slip_) < 1e-12) {
      return;
    }
    for (const auto & surface : this->surfaces_) {
      surface->slip1 = lateral_slip;
      surface->slip2 = this->longitudinal_slip_;
    }
    this->current_lateral_slip_ = lateral_slip;
  }

  double turning_lateral_slip_{0.1};
  double longitudinal_slip_{0.0};
  double cmd_vel_timeout_{0.5};
  double angular_threshold_{0.01};
  double current_lateral_slip_{-1.0};
  std::atomic<bool> turning_requested_{false};
  std::atomic<int64_t> last_command_ns_{0};
  std::vector<boost::shared_ptr<gazebo::physics::ODESurfaceParams>> surfaces_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(WheelContactPlugin)
}  // namespace roslander_gazebo_plugins
