#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals ;
class GroundPlaneNode : public rclcpp::Node
{
public:
  GroundPlaneNode()
  : Node("ground_plane_node")
  {
    planning_scene_pub_ =
      this->create_publisher<moveit_msgs::msg::PlanningScene>("monitored_planning_scene", 10);

    rclcpp::sleep_for(1s);

    add_ground_plane();
  }

private:
  void add_ground_plane()
  {

    moveit_msgs::msg::CollisionObject collision;
    collision.id = "ground_plane";
    collision.header.frame_id = "base_link";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {
      5.0,   // X length (meters)
      5.0,   // Y breadth (meters)
      0.1     // thickness
    };

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 1.05;   // top surface at Z = 0

    collision.primitives.push_back(primitive);
    collision.primitive_poses.push_back(pose);
    collision.operation = collision.ADD;

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects.push_back(collision);

    planning_scene_pub_->publish(scene);

    RCLCPP_INFO(get_logger(), "Ground plane published.");
  }

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundPlaneNode>());
  rclcpp::shutdown();
  return 0;
}
