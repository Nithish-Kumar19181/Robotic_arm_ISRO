    #include <rclcpp/rclcpp.hpp>
    #include <std_msgs/msg/string.hpp>
    #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
    #include <trajectory_msgs/msg/joint_trajectory.hpp>
    #include <geometry_msgs/msg/wrench_stamped.hpp>
    #include <urdf/model.h>
    #include <kdl_parser/kdl_parser.hpp>
    #include <kdl/tree.hpp>
    #include <kdl/chain.hpp>
    #include <kdl/chainiksolverpos_lma.hpp>
    #include <kdl/jntarray.hpp>
    #include <kdl/frames.hpp>

    #include <visualization_msgs/msg/marker.hpp>
    #include <visualization_msgs/msg/marker_array.hpp>

    #include <cmath>
    #include <memory>
    #include <string>
    #include <deque>

    using std::placeholders::_1;

    class CartesianForceController : public rclcpp::Node {
    public:
        CartesianForceController()
        : Node("cartesian_force_controller"),
        z_height_(0.5),
        approach_speed_(0.001),
        wall_follow_speed_(0.001),  // Increased for better visibility
        desired_force_(-10.0),  // Negative for pushing against wall
        max_search_distance_(1.0),
        contact_detection_force_(-5.0),  // Negative for contact detection
        wall_follow_distance_(0.3),
        admittance_mass_(0.2),
        admittance_damping_(15.0),
        admittance_stiffness_(0.009),
        force_filter_alpha_(0.4),
        filtered_force_x_(0.0),
        safety_force_limit_(-50.0)  // Negative for safety limit
        {
            RCLCPP_INFO(this->get_logger(), "Initializing Simultaneous Force Control and Wall Following Controller...");

            last_update_time_ = this->now();

            urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
                "robot_description",
                rclcpp::QoS(rclcpp::KeepLast(1))
                    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
                std::bind(&CartesianForceController::on_urdf_received, this, _1));

            force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/wrench", 10,
                std::bind(&CartesianForceController::force_callback, this, _1));

            pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
                "/target_joint_positions", 10);

            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/ik_trajectory_markers", 10);

            status_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_status", 10);
            force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/desired_force", 10);

            RCLCPP_INFO(this->get_logger(), "Simultaneous Force Control and Wall Following Controller node created, waiting for URDF...");
        }

    private:
        enum MotionState { 
            IDLE=0, 
            FIND_WALL,          // Move toward wall until contact detection
            WALL_FOLLOWING      // Move along wall while maintaining force (combined state)
        };

        // ROS entities
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // KDL
        KDL::Chain kdl_chain_;
        std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
        KDL::JntArray q_init_;

        // Parameters / internal state
        double z_height_;
        double approach_speed_;
        double wall_follow_speed_;
        double desired_force_;
        double max_search_distance_;
        double contact_detection_force_;
        double wall_follow_distance_;
        double safety_force_limit_;

        // Admittance control parameters
        double admittance_mass_;
        double admittance_damping_;
        double admittance_stiffness_;
        
        // Force filtering
        double force_filter_alpha_;
        double filtered_force_x_;
        std::deque<double> force_history_;
        size_t force_history_size_ = 8;

        MotionState state_ = IDLE;
        double current_x_position_ = 0.0;
        double current_y_position_ = 0.0;
        double wall_follow_progress_ = 0.0;
        
        KDL::Rotation last_orientation_ = KDL::Rotation::Identity();
        double wall_contact_position_x_ = 0.0;
        
        // Admittance control state variables
        double admittance_velocity_ = 0.0;
        double admittance_position_ = 0.0;
        rclcpp::Time last_update_time_;

        // Force sign tracking
        bool force_sign_positive_ = false;
        double previous_force_ = 0.0;
        double force_change_threshold_ = 0.5;

        // Trajectory visualization
        std::vector<geometry_msgs::msg::Point> trajectory_points_;
        std::vector<geometry_msgs::msg::Point> planned_trajectory_;

        void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            double current_force = msg->wrench.force.x;
            
            // Detect force sign change
            bool current_sign_positive = (current_force >= 0);
            if (current_sign_positive != force_sign_positive_) {
                RCLCPP_WARN(this->get_logger(), "Force sign changed from %s to %s (force: %.2f N)", 
                        force_sign_positive_ ? "positive" : "negative",
                        current_sign_positive ? "positive" : "negative",
                        current_force);
                force_sign_positive_ = current_sign_positive;
            }
            
            // Detect significant force change
            double force_change = std::abs(current_force - previous_force_);
            if (force_change > force_change_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Significant force change: %.2f N (previous: %.2f N, current: %.2f N)", 
                        force_change, previous_force_, current_force);
            }
            previous_force_ = current_force;
            
            filtered_force_x_ = force_filter_alpha_ * current_force + 
                            (1.0 - force_filter_alpha_) * filtered_force_x_;
            
            force_history_.push_back(current_force);
            if (force_history_.size() > force_history_size_) {
                force_history_.pop_front();
            }
            
            double avg_force = 0.0;
            for (double f : force_history_) {
                avg_force += f;
            }
            if (!force_history_.empty()) {
                avg_force /= force_history_.size();
            }
            
            // CORRECTED SAFETY CHECK: Only trigger when force is TOO NEGATIVE (pushing too hard)
            // For negative forces, we want to check if they are BELOW (more negative than) safety limit
            if (avg_force <= safety_force_limit_) {
                RCLCPP_ERROR(this->get_logger(), 
                        "SAFETY LIMIT EXCEEDED! Avg Force: %.2f N, Limit: %.2f N.",
                        avg_force, safety_force_limit_);
                return;
            }
            
            // Wall contact detection - modified to handle force sign properly
            // For negative desired force (pushing), we detect when force becomes more negative than threshold
            // For positive desired force (pulling), we'd detect when force becomes more positive than threshold
            bool contact_detected = false;
            if (desired_force_ < 0) {
                // Pushing scenario: contact when force is negative enough
                contact_detected = (avg_force <= contact_detection_force_);
            } else {
                // Pulling scenario: contact when force is positive enough  
                contact_detected = (avg_force >= contact_detection_force_);
            }
            
            if (contact_detected && state_ == FIND_WALL) {
                wall_contact_position_x_ = current_x_position_;
                RCLCPP_WARN(this->get_logger(), 
                        "Wall contact detected! Avg force: %.2f N, Wall position: %.3f m, Force sign: %s", 
                        avg_force, wall_contact_position_x_,
                        force_sign_positive_ ? "positive" : "negative");
                
                // IMMEDIATELY switch to wall following
                state_ = WALL_FOLLOWING;
                
                // Initialize admittance controller at current position
                admittance_position_ = wall_contact_position_x_;
                admittance_velocity_ = 0.0;
                last_update_time_ = this->now();
                
                // Reset wall following progress to start immediately
                wall_follow_progress_ = 0.0;
                current_y_position_ = 0.0;
                
                force_history_.clear();
                
                // Generate planned trajectory for visualization
                generate_planned_trajectory();
                
                auto status_msg = std_msgs::msg::String();
                status_msg.data = "WALL_FOUND_STARTING_WALL_FOLLOWING";
                status_pub_->publish(status_msg);
                
                RCLCPP_INFO(this->get_logger(), "Starting simultaneous force control and wall following at X=%.3f m", wall_contact_position_x_);
            }
        }

        void generate_planned_trajectory() {
            planned_trajectory_.clear();
            
            // Start from current position
            geometry_msgs::msg::Point start_point;
            start_point.x = current_x_position_;
            start_point.y = current_y_position_;
            start_point.z = z_height_;
            planned_trajectory_.push_back(start_point);
            
            // Generate points along the wall following path
            int num_points = 50;
            for (int i = 1; i <= num_points; ++i) {
                geometry_msgs::msg::Point point;
                double progress = (double)i / num_points * wall_follow_distance_;
                point.x = wall_contact_position_x_; // Maintain same X position (force control will adjust)
                point.y = current_y_position_ + progress;
                point.z = z_height_;
                planned_trajectory_.push_back(point);
            }
        }

        double compute_admittance_control(double force_error, double dt) {
            if (dt <= 0.0) return admittance_position_;
            
            // Enhanced force error handling based on sign
            double max_force_error = 15.0;
            
            // Adjust behavior based on force sign and desired force sign
            if ((desired_force_ < 0 && force_sign_positive_) || 
                (desired_force_ > 0 && !force_sign_positive_)) {
                // Force sign mismatch - be more conservative
                max_force_error = 8.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Force sign mismatch with desired force. Using conservative control.");
            }
            
            if (force_error > max_force_error) force_error = max_force_error;
            if (force_error < -max_force_error) force_error = -max_force_error;
            
            // Enhanced admittance control with sign-aware damping
            double damping = admittance_damping_;
            
            // Adjust damping based on force change direction
            double force_change = filtered_force_x_ - previous_force_;
            if (std::abs(force_change) > 2.0) {
                // Rapid force change - increase damping for stability
                damping *= 1.5;
            }
            
            // FIXED ADMITTANCE CONTROL LOGIC:
            // For PUSHING scenario (desired_force_ < 0):
            // - If current force is MORE negative than desired (pushing too hard): we want to move AWAY from wall (decrease X)
            // - If current force is LESS negative than desired (pushing too soft): we want to move TOWARD wall (increase X)
            // 
            // Since force_error = desired_force_ - avg_force:
            // - When pushing too hard: avg_force is MORE negative -> force_error is POSITIVE -> we want NEGATIVE velocity (away from wall)
            // - When pushing too soft: avg_force is LESS negative -> force_error is NEGATIVE -> we want POSITIVE velocity (toward wall)
            //
            // So we need to NEGATE the force_error in the acceleration calculation
            double acceleration = -force_error / admittance_mass_;
            
            acceleration -= damping * admittance_velocity_;
            admittance_velocity_ += acceleration * dt;
            
            // Adaptive velocity limits based on force conditions
            double max_velocity = 0.003;
            if (std::abs(force_error) > 10.0) {
                // Large force error - allow faster response
                max_velocity = 0.005;
            }
            
            if (admittance_velocity_ > max_velocity) admittance_velocity_ = max_velocity;
            if (admittance_velocity_ < -max_velocity) admittance_velocity_ = -max_velocity;
            
            admittance_position_ += admittance_velocity_ * dt;
            
            return admittance_position_;
        }

        bool ik_ready() const { return (bool)ik_solver_; }

        void on_urdf_received(const std_msgs::msg::String::SharedPtr msg) {
            if (ik_solver_) return;

            RCLCPP_INFO(this->get_logger(), "URDF received, initializing...");

            urdf::Model model;
            if (!model.initString(msg->data)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
                return;
            }

            KDL::Tree kdl_tree;
            if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL tree.");
                return;
            }

            if (!kdl_tree.getChain("base_link", "tool0", kdl_chain_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base_link to tool0");
                return;
            }

            ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
            q_init_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
            
            double initial_joints[] = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};
            for (unsigned int i = 0; i < q_init_.rows(); ++i) {
                if (i < 6) {
                    q_init_(i) = initial_joints[i];
                } else {
                    q_init_(i) = 0.0;
                }
            }

            // Initialize
            state_ = FIND_WALL;
            current_x_position_ = 0.0;
            current_y_position_ = 0.0;
            wall_follow_progress_ = 0.0;
            filtered_force_x_ = 0.0;
            previous_force_ = 0.0;
            force_sign_positive_ = false;
            trajectory_points_.clear();
            planned_trajectory_.clear();
            force_history_.clear();
            last_orientation_ = KDL::Rotation::Identity();
            
            admittance_velocity_ = 0.0;
            admittance_position_ = 0.0;
            last_update_time_ = this->now();

            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(0.033),
                std::bind(&CartesianForceController::publishNextPoint, this));

            RCLCPP_INFO(this->get_logger(), "Simultaneous Force Control and Wall Following Controller initialized.");
            RCLCPP_INFO(this->get_logger(), "Will start wall following IMMEDIATELY after wall detection.");
            RCLCPP_INFO(this->get_logger(), "YES - Admittance control (X) and wall following (Y) happen SIMULTANEOUSLY.");
        }

        KDL::Rotation createToolOrientationFacingWall() {
            // For UR robots facing the wall
            return KDL::Rotation::RotY(M_PI/2);
        }

        void publishNextPoint() {
            if (!ik_ready()) return;

            auto current_time = this->now();
            double dt = 0.033;
            
            if (last_update_time_.seconds() > 0) {
                try {
                    dt = (current_time - last_update_time_).seconds();
                } catch (const std::runtime_error& e) {
                    dt = 0.033;
                }
            }
            last_update_time_ = current_time;

            KDL::Frame pose = KDL::Frame::Identity();
            bool should_publish = true;

            if (state_ == FIND_WALL) {
                current_x_position_ += approach_speed_;
                
                if (current_x_position_ > max_search_distance_) {
                    current_x_position_ = max_search_distance_;
                    RCLCPP_WARN(this->get_logger(), 
                            "Reached maximum search distance (%.2f m) without finding wall", 
                            max_search_distance_);
                    timer_->cancel();
                    return;
                }
                
                double x = current_x_position_;
                double y = current_y_position_;
                double z = z_height_;

                KDL::Rotation tool_orientation = createToolOrientationFacingWall();
                pose = KDL::Frame(tool_orientation, KDL::Vector(x, y, z));
                last_orientation_ = pose.M;

                // Store trajectory point
                geometry_msgs::msg::Point traj_point;
                traj_point.x = x;
                traj_point.y = y;
                traj_point.z = z;
                trajectory_points_.push_back(traj_point);

                static double last_print_progress = 0.0;
                if (current_x_position_ - last_print_progress >= 0.05) {
                    RCLCPP_INFO(this->get_logger(), "Searching for wall: X=%.3f m, Force: %.2f N, Force sign: %s", 
                            current_x_position_, filtered_force_x_,
                            force_sign_positive_ ? "positive" : "negative");
                    last_print_progress = current_x_position_;
                }
            }
            else if (state_ == WALL_FOLLOWING) {
                // Calculate average force
                double avg_force = 0.0;
                for (double f : force_history_) {
                    avg_force += f;
                }
                if (!force_history_.empty()) {
                    avg_force /= force_history_.size();
                } else {
                    avg_force = filtered_force_x_;
                }
                
                // CORRECTED FORCE ERROR CALCULATION:
                // For negative desired forces (pushing scenario):
                // - If current force is MORE negative than desired (pushing too hard): force_error is POSITIVE
                // - If current force is LESS negative than desired (pushing too soft): force_error is NEGATIVE
                double force_error = desired_force_ - avg_force;
                
                // Log force conditions for debugging
                static auto last_force_log = this->now();
                if ((this->now() - last_force_log).seconds() >= 1.0) {
                    RCLCPP_DEBUG(this->get_logger(), 
                            "Force control: Current=%.2f, Desired=%.2f, Error=%.2f, Sign=%s",
                            avg_force, desired_force_, force_error,
                            force_sign_positive_ ? "positive" : "negative");
                    last_force_log = this->now();
                }
                
                double new_x = compute_admittance_control(force_error, dt);
                
                // Enhanced safety limits for X with sign awareness
                // When pushing (desired_force_ < 0), we can move both into and away from the wall
                double max_x = wall_contact_position_x_ + 0.05;  // Can push further into wall
                double min_x = wall_contact_position_x_ - 0.05;  // Can move away from wall
                
                // Adjust limits based on force sign conditions
                if (force_sign_positive_ && desired_force_ < 0) {
                    // Unexpected positive force when we expect negative (pushing)
                    // This might indicate loss of contact or tool orientation issue
                    max_x = wall_contact_position_x_; // Don't push further
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                    "Unexpected positive force during pushing. Limiting forward movement.");
                }
                
                if (new_x > max_x) {
                    new_x = max_x;
                    admittance_velocity_ = 0.0;
                }
                if (new_x < min_x) {
                    new_x = min_x;
                    admittance_velocity_ = 0.0;
                }
                
                // FIXED: Wall following in Y direction - START IMMEDIATELY and update properly
                wall_follow_progress_ += wall_follow_speed_;
                if (wall_follow_progress_ > wall_follow_distance_) {
                    wall_follow_progress_ = wall_follow_distance_;
                }
                
                current_x_position_ = new_x;
                double y = wall_follow_progress_;  // Use progress directly for Y position
                double z = z_height_;

                KDL::Rotation tool_orientation = createToolOrientationFacingWall();
                pose = KDL::Frame(tool_orientation, KDL::Vector(new_x, y, z));
                last_orientation_ = pose.M;

                // Store trajectory point
                geometry_msgs::msg::Point traj_point;
                traj_point.x = new_x;
                traj_point.y = y;
                traj_point.z = z;
                trajectory_points_.push_back(traj_point);

                // Publish desired force for monitoring
                auto desired_wrench = geometry_msgs::msg::WrenchStamped();
                desired_wrench.header.stamp = this->now();
                desired_wrench.header.frame_id = "tool0";
                desired_wrench.wrench.force.x = desired_force_;
                force_pub_->publish(desired_wrench);

                // Print status with clear direction information
                static auto last_status_print = this->now();
                if ((this->now() - last_status_print).seconds() >= 2.0) {
                    std::string x_direction = "STABLE";
                    if (admittance_velocity_ > 0.001) {
                        x_direction = "TOWARD WALL";
                    } else if (admittance_velocity_ < -0.001) {
                        x_direction = "AWAY FROM WALL";
                    }
                    
                    RCLCPP_INFO(this->get_logger(), 
                            "Wall Following - Y: %.3f/%.3f m, Force: %.2f N, X: %.4f m, X-Direction: %s", 
                            y, wall_follow_distance_, avg_force, new_x, x_direction.c_str());
                    last_status_print = this->now();
                }

                // Check completion
                if (wall_follow_progress_ >= wall_follow_distance_) {
                    RCLCPP_INFO(this->get_logger(), "Wall following complete. Traveled %.2f m along wall.", wall_follow_distance_);
                    timer_->cancel();
                    
                    auto status_msg = std_msgs::msg::String();
                    status_msg.data = "WALL_FOLLOWING_COMPLETE";
                    status_pub_->publish(status_msg);
                }
            }
            else {
                should_publish = false;
            }

            if (should_publish) {
                KDL::JntArray q_sol(kdl_chain_.getNrOfJoints());
                int ret = ik_solver_->CartToJnt(q_init_, pose, q_sol);
                if (ret >= 0) {
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    pt.positions.resize(q_sol.rows());
                    for (unsigned int i = 0; i < q_sol.rows(); ++i)
                        pt.positions[i] = q_sol(i);
                    pt.time_from_start = rclcpp::Duration::from_seconds(0.15);
                    pub_->publish(pt);
                    q_init_ = q_sol;

                    publish_markers(pose);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "IK failed (ret=%d) at state=%d", ret, (int)state_);
                }
            }
        }

        void publish_markers(const KDL::Frame& pose) {
            visualization_msgs::msg::MarkerArray marker_array;
            
            // Current position marker - Use ARROW to show orientation
            visualization_msgs::msg::Marker tool_marker;
            tool_marker.header.frame_id = "base_link";
            tool_marker.header.stamp = this->now();
            tool_marker.ns = "current_position";
            tool_marker.id = 0;
            tool_marker.type = visualization_msgs::msg::Marker::ARROW;
            tool_marker.action = visualization_msgs::msg::Marker::ADD;
            tool_marker.pose.position.x = pose.p.x();
            tool_marker.pose.position.y = pose.p.y();
            tool_marker.pose.position.z = pose.p.z();
            
            // Convert KDL rotation to quaternion for marker orientation
            double x, y, z, w;
            pose.M.GetQuaternion(x, y, z, w);
            tool_marker.pose.orientation.x = x;
            tool_marker.pose.orientation.y = y;
            tool_marker.pose.orientation.z = z;
            tool_marker.pose.orientation.w = w;
            
            tool_marker.scale.x = 0.15;  // Length of arrow
            tool_marker.scale.y = 0.03;  // Width of arrow
            tool_marker.scale.z = 0.03;  // Height of arrow
            tool_marker.color.a = 1.0;
            tool_marker.color.r = 1.0;
            tool_marker.color.g = 0.0;
            tool_marker.color.b = 0.0;

            marker_array.markers.push_back(tool_marker);

            // Tool center point marker (small sphere at tool center)
            visualization_msgs::msg::Marker tool_center_marker;
            tool_center_marker.header.frame_id = "base_link";
            tool_center_marker.header.stamp = this->now();
            tool_center_marker.ns = "tool_center";
            tool_center_marker.id = 6;
            tool_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
            tool_center_marker.action = visualization_msgs::msg::Marker::ADD;
            tool_center_marker.pose.position.x = pose.p.x();
            tool_center_marker.pose.position.y = pose.p.y();
            tool_center_marker.pose.position.z = pose.p.z();
            tool_center_marker.pose.orientation.w = 1.0;
            
            tool_center_marker.scale.x = 0.03;
            tool_center_marker.scale.y = 0.03;
            tool_center_marker.scale.z = 0.03;
            tool_center_marker.color.a = 1.0;
            tool_center_marker.color.r = 0.0;
            tool_center_marker.color.g = 1.0;
            tool_center_marker.color.b = 0.0;

            marker_array.markers.push_back(tool_center_marker);

            // Actual trajectory
            if (trajectory_points_.size() > 1) {
                visualization_msgs::msg::Marker trajectory_marker;
                trajectory_marker.header.frame_id = "base_link";
                trajectory_marker.header.stamp = this->now();
                trajectory_marker.ns = "actual_trajectory";
                trajectory_marker.id = 1;
                trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
                trajectory_marker.points = trajectory_points_;
                
                trajectory_marker.scale.x = 0.01;
                trajectory_marker.color.a = 1.0;
                trajectory_marker.color.r = 0.0;
                trajectory_marker.color.g = 1.0;
                trajectory_marker.color.b = 0.0;
                
                marker_array.markers.push_back(trajectory_marker);
            }

            // Planned trajectory
            if (!planned_trajectory_.empty()) {
                visualization_msgs::msg::Marker planned_marker;
                planned_marker.header.frame_id = "base_link";
                planned_marker.header.stamp = this->now();
                planned_marker.ns = "planned_trajectory";
                planned_marker.id = 2;
                planned_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                planned_marker.action = visualization_msgs::msg::Marker::ADD;
                planned_marker.points = planned_trajectory_;
                
                planned_marker.scale.x = 0.008;
                planned_marker.color.a = 0.6;
                planned_marker.color.r = 0.0;
                planned_marker.color.g = 0.0;
                planned_marker.color.b = 1.0;
                
                marker_array.markers.push_back(planned_marker);
            }

            // Wall marker
            if (state_ == WALL_FOLLOWING) {
                visualization_msgs::msg::Marker wall_marker;
                wall_marker.header.frame_id = "base_link";
                wall_marker.header.stamp = this->now();
                wall_marker.ns = "wall";
                wall_marker.id = 3;
                wall_marker.type = visualization_msgs::msg::Marker::CUBE;
                wall_marker.action = visualization_msgs::msg::Marker::ADD;
                wall_marker.pose.position.x = wall_contact_position_x_ + 0.01;
                wall_marker.pose.position.y = wall_follow_distance_ / 2.0;
                wall_marker.pose.position.z = z_height_;
                wall_marker.pose.orientation.w = 1.0;
                
                wall_marker.scale.x = 0.02;
                wall_marker.scale.y = wall_follow_distance_;
                wall_marker.scale.z = 0.3;
                wall_marker.color.a = 0.5;
                wall_marker.color.r = 0.5;
                wall_marker.color.g = 0.5;
                wall_marker.color.b = 0.5;
                
                marker_array.markers.push_back(wall_marker);
            }

            // Enhanced force display with sign information
            visualization_msgs::msg::Marker force_marker;
            force_marker.header.frame_id = "base_link";
            force_marker.header.stamp = this->now();
            force_marker.ns = "force_display";
            force_marker.id = 4;
            force_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            force_marker.action = visualization_msgs::msg::Marker::ADD;
            force_marker.pose.position.x = pose.p.x();
            force_marker.pose.position.y = pose.p.y();
            force_marker.pose.position.z = pose.p.z() + 0.15;
            force_marker.pose.orientation.w = 1.0;
            
            force_marker.scale.z = 0.06;
            force_marker.color.a = 1.0;
            
            // Color code based on force sign and safety
            if (filtered_force_x_ <= safety_force_limit_) {
                force_marker.color.r = 1.0;  // Red for safety limit
                force_marker.color.g = 0.0;
                force_marker.color.b = 0.0;
            } else if (force_sign_positive_) {
                force_marker.color.r = 1.0;  // Yellow for positive force
                force_marker.color.g = 1.0;
                force_marker.color.b = 0.0;
            } else {
                force_marker.color.r = 0.0;  // Green for negative force (expected)
                force_marker.color.g = 1.0;
                force_marker.color.b = 0.0;
            }
            
            // Add direction information to force display
            std::string x_direction = "STABLE";
            if (admittance_velocity_ > 0.001) {
                x_direction = "TOWARD WALL";
            } else if (admittance_velocity_ < -0.001) {
                x_direction = "AWAY FROM WALL";
            }
            
            force_marker.text = "Force: " + std::to_string(static_cast<int>(filtered_force_x_)) + " N\n" +
                            "Target: " + std::to_string(static_cast<int>(desired_force_)) + " N\n" +
                            "X-Direction: " + x_direction;
            marker_array.markers.push_back(force_marker);

            // Status display
            visualization_msgs::msg::Marker status_marker;
            status_marker.header.frame_id = "base_link";
            status_marker.header.stamp = this->now();
            status_marker.ns = "status";
            status_marker.id = 5;
            status_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            status_marker.action = visualization_msgs::msg::Marker::ADD;
            status_marker.pose.position.x = 0.5;
            status_marker.pose.position.y = 0.0;
            status_marker.pose.position.z = 1.0;
            status_marker.pose.orientation.w = 1.0;
            
            status_marker.scale.z = 0.08;
            status_marker.color.a = 1.0;
            status_marker.color.r = 1.0;
            status_marker.color.g = 1.0;
            status_marker.color.b = 1.0;
            
            if (state_ == FIND_WALL) {
                status_marker.text = "STATE: FINDING WALL";
            } else {
                status_marker.text = "STATE: WALL FOLLOWING\n" +
                                    std::string("Y Progress: ") + 
                                    std::to_string(static_cast<int>(wall_follow_progress_ * 1000)) + " mm\n" +
                                    "SIMULTANEOUS X & Y CONTROL";
            }
            marker_array.markers.push_back(status_marker);

            marker_pub_->publish(marker_array);
        }
    };

    int main(int argc, char** argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<CartesianForceController>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }