#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/parameter.hpp>

#include <bits/stdc++.h>
#include <chrono>
#include <functional>
#include <string>
#include <regex>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace uosm
{
	namespace px4
	{
		constexpr float PUBLISH_RATE(20.0f);
		constexpr float HOVERING_TOLERANCE(0.1f); // based on short term vio drift
		constexpr float FLYING_TOLERANCE(0.1f);	  // based on short term vio drift
		constexpr float HEADING_TOLERANCE(0.1f);  //  0.1 rad ~= 5.73 deg
		constexpr float RAD2DEG(180 / M_PI);
		constexpr float DEG2RAD(M_PI / 180);
		constexpr double EARTH_RADIUS(6378137.0); // Earth radius in meters at equator

		static inline double computeEuclideanDistance(const px4_msgs::msg::TrajectorySetpoint &traj, const px4_msgs::msg::VehicleLocalPosition &vehicle_pos, bool with_z = false)
		{
			double dx = traj.position[0] - vehicle_pos.x;
			double dy = traj.position[1] - vehicle_pos.y;
			if (with_z)
			{
				double dz = traj.position[2] - vehicle_pos.z;
				return std::sqrt(dx * dx + dy * dy + dz * dz);
			}
			else
			{
				return std::sqrt(dx * dx + dy * dy);
			}
		}

		class PX4AgentControl : public rclcpp::Node
		{
		public:
			PX4AgentControl(std::string px4_namespace) : Node("px4_agent_control_node")
			{
				// Params
				// TODO: Param for frame convention ENU->NED
				declare_parameter("height", rclcpp::ParameterValue(1.00f));
				declare_parameter("mission_objective", rclcpp::ParameterValue(""));
				declare_parameter("introspector_object", rclcpp::ParameterValue("Aruco Marker"));
				declare_parameter("resend_commnad", rclcpp::ParameterValue(true));
				declare_parameter("resend_size", rclcpp::ParameterValue(10));

				height_ = get_parameter("height").as_double();
				mission_objective_ = get_parameter("mission_objective").as_string();
				introspector_object_ = get_parameter("introspector_object").as_string();
				resend_command_ = get_parameter("resend_commnad").as_bool();
				resend_size_ = get_parameter("resend_size").as_int();

				// Publishers & Subscribers setup
				const auto qos_profile = rclcpp::QoS(10)
											 .reliability(rclcpp::ReliabilityPolicy::BestEffort)
											 .durability(rclcpp::DurabilityPolicy::TransientLocal)
											 .history(rclcpp::HistoryPolicy::KeepLast);

				offboard_control_mode_pub_ = create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", qos_profile);
				trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", qos_profile);
				vehicle_command_client_ = create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command");

				vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(px4_namespace + "out/vehicle_status", qos_profile,
																						[this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
																						{
																							vehicle_status_ = *msg;
																						});

				vehicle_gp_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(px4_namespace + "out/vehicle_global_position", qos_profile,
																							[this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
																							{
																								vehicle_gp_ = *msg;
																							});
				vehicle_lp_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(px4_namespace + "out/vehicle_local_position", qos_profile,
																						   [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
																						   {
																							   vehicle_lp_ = *msg;
																						   });

				vln_query_pub_ = create_publisher<std_msgs::msg::String>("text0", 10);
				vln_response_sub_ = create_subscription<std_msgs::msg::String>("text1", 10, std::bind(&PX4AgentControl::vln_response_cb, this, std::placeholders::_1));
				introspector_query_pub_ = create_publisher<std_msgs::msg::String>("introspection", 10);
				introspector_response_sub_ = create_subscription<std_msgs::msg::String>("introspection_answer", 10, std::bind(&PX4AgentControl::introspector_response_cb, this, std::placeholders::_1));

				traj_.position = {0.0f, 0.0f, -std::abs((static_cast<float>(height_)))}; // convert to NED
				traj_.yaw = M_PI_2;

				goal_x_ = 0.0f;
				goal_y_ = 0.0f;
				is_init_ = true;
			}

			void arm();
			void disarm();

			void switch_to_offboard_mode();
			void switch_to_manual_mode();
			void publish_offboard_control_mode();

			void publish_trajectory_setpoint();
			void request_landing(float lat = 0.0f, float lon = 0.0f, float alt = 0.0f);
			void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f);

			void publish_introspector_query();
			void publish_vln_query();

			bool is_init_ = false;
			bool is_mission_ingested_ = false;
			bool is_vln_updated_ = false;
			bool is_introspection_updated_ = false;
			bool is_mission_done_ = false;

			px4_msgs::msg::VehicleStatus vehicle_status_;
			px4_msgs::msg::VehicleGlobalPosition vehicle_gp_;
			px4_msgs::msg::VehicleLocalPosition vehicle_lp_;
			px4_msgs::msg::TrajectorySetpoint traj_;

			std_msgs::msg::String vln_response_;
			std_msgs::msg::String introspector_response_;

		private:
			uint8_t service_result_;
			double height_;
			std::string mission_objective_;
			std::string introspector_object_;
			bool resend_command_;
			int resend_size_;
			std::vector<std::string> vln_cmd_history_;

			double goal_x_, goal_y_;
			std::vector<std::pair<float,float>> obstacles_;
			std::string obs_str_;

			// VLM
			rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vln_query_pub_;
			rclcpp::Publisher<std_msgs::msg::String>::SharedPtr introspector_query_pub_;
			rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vln_response_sub_;
			rclcpp::Subscription<std_msgs::msg::String>::SharedPtr introspector_response_sub_;

			// PX4
			rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
			rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
			rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
			rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
			rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gp_sub_;
			rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_lp_sub_;

			void px4_response_cb(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);

			void vln_response_cb(const std_msgs::msg::String::SharedPtr msg)
			{
				vln_response_ = *msg;
				RCLCPP_INFO(get_logger(), "VLN agent response: %s", vln_response_.data.c_str());
				std::string trimmed_response = vln_response_.data;
				trimmed_response.erase(0, trimmed_response.find_first_not_of(" \t\r\n"));
				trimmed_response.erase(trimmed_response.find_last_not_of(" \t\r\n") + 1);
				if (!trimmed_response.empty())
				{
					vln_cmd_history_.push_back(trimmed_response);
				}
				is_vln_updated_ = true;

				// Get current position and heading
				float current_x = vehicle_lp_.x;
				float current_y = vehicle_lp_.y;
				float current_heading = vehicle_lp_.heading;

				// Parse the response string line by line
				std::istringstream response_stream(vln_response_.data);
				std::string line;

				while (std::getline(response_stream, line))
				{
					// Trim whitespace from the line
					line.erase(0, line.find_first_not_of(" \t\r\n"));
					line.erase(line.find_last_not_of(" \t\r\n") + 1);

					if (line.empty())
						continue;

					// Parse Turn command
					if (line.find("Turn(") == 0)
					{
						// Extract angle in degrees
						float angle_deg = std::stof(line.substr(5, line.find(')') - 5));
						// Convert to radians and update heading
						float angle_rad = angle_deg * DEG2RAD;
						current_heading += angle_rad;

						// Normalize heading to [-π, π]
						current_heading = std::atan2(std::sin(current_heading), std::cos(current_heading));

						RCLCPP_INFO(get_logger(), "Turn command: %.2f degrees, new heading: %.2f rad", angle_deg, current_heading);
					}
					// Parse Move command
					else if (line.find("Move(") == 0)
					{
						// Extract distance in meters
						float distance = std::stof(line.substr(5, line.find(')') - 5));

						// Update position based on current heading and distance
						current_x += distance * cos(current_heading);
						current_y += distance * sin(current_heading);

						RCLCPP_INFO(get_logger(), "Move command: %.2f meters, new position: (%.2f, %.2f)",
									distance, current_x, current_y);
					}
				}

				// Update trajectory with new position and heading
				traj_.position = {current_x, current_y, -std::abs((static_cast<float>(height_)))};
				traj_.yaw = current_heading;

				RCLCPP_INFO(get_logger(), "Updated trajectory to position: (%.2f, %.2f, %.2f), yaw: %.2f",
							current_x, current_y, height_, current_heading);
			}

			void introspector_response_cb(const std_msgs::msg::String::SharedPtr msg)
			{
				introspector_response_ = *msg;
				RCLCPP_INFO(get_logger(), "Introspector response: %s", introspector_response_.data.c_str());
				is_introspection_updated_ = true;
				if (introspector_response_.data == "Yes")
				{
					is_mission_done_ = true;
				}
			}
		};

		/**
		 * @brief Send a command to switch to offboard mode
		 * https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/Commander.cpp#L367
		 * PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
		 */
		void PX4AgentControl::switch_to_offboard_mode()
		{
			RCLCPP_DEBUG(get_logger(), "requesting switch to Offboard mode");
			request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		}

		void PX4AgentControl::switch_to_manual_mode()
		{
			RCLCPP_DEBUG(get_logger(), "requesting switch to Manual mode");
			request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
		}

		/**
		 * @brief Send a command to Arm the vehicle
		 */
		void PX4AgentControl::arm()
		{
			RCLCPP_DEBUG(get_logger(), "requesting arm");
			request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
		}

		/**
		 * @brief Send a command to Disarm the vehicle
		 */
		void PX4AgentControl::disarm()
		{
			RCLCPP_DEBUG(get_logger(), "requesting disarm");
			request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
		}

		/**
		 * @brief Publish the offboard control mode.
		 *        Only position and attitude controls are active.
		 */
		void PX4AgentControl::publish_offboard_control_mode()
		{
			OffboardControlMode msg{};
			msg.position = true;
			msg.velocity = false;
			msg.acceleration = false;
			msg.attitude = true;
			msg.body_rate = false;
			msg.timestamp = get_clock()->now().nanoseconds() / 1000;
			offboard_control_mode_pub_->publish(msg);
		}

		/**
		 * @brief Publish a trajectory setpoint
		 */
		void PX4AgentControl::publish_trajectory_setpoint()
		{
			traj_.timestamp = get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_pub_->publish(traj_);
			// RCLCPP_DEBUG(get_logger(), "Setting Trajectory (x = %.2f m, y = %.2f m, z = %.2f m, yaw = %.2f rad)", traj_.position[0], traj_.position[1], traj_.position[2], traj_.yaw);
		}

		void PX4AgentControl::request_landing(float lat, float lon, float alt)
		{
			// https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
			auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

			VehicleCommand msg{};
			msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
			msg.param1 = 0.0f; // Abort Alt
			msg.param2 = 0.0f; // Land Mode
			// https://docs.px4.io/main/en/advanced_config/parameter_reference.html#MPC_LAND_SPEED
			msg.param3 = NAN;		// empty
			msg.param4 = traj_.yaw; // Yaw (rad)
			msg.param5 = lat;		// lat
			msg.param6 = lon;		// lon
			msg.param7 = alt;		// alt (m)
			msg.target_system = 1;
			msg.target_component = 1;
			msg.source_system = 1;
			msg.source_component = 1;
			msg.from_external = true;
			msg.timestamp = get_clock()->now().nanoseconds() / 1000;
			request->request = msg;

			auto result = vehicle_command_client_->async_send_request(request, std::bind(&PX4AgentControl::px4_response_cb, this,
																						 std::placeholders::_1));
			RCLCPP_DEBUG(get_logger(), "Land command send");
		}

		/**
		 * @brief Publish vehicle commands
		 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
		 * @param param1    Command parameter 1
		 * @param param2    Command parameter 2
		 */
		void PX4AgentControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3)
		{
			auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

			VehicleCommand msg{};
			msg.param1 = param1;
			msg.param2 = param2;
			msg.param3 = param3;
			msg.command = command;
			msg.target_system = 1;
			msg.target_component = 1;
			msg.source_system = 1;
			msg.source_component = 1;
			msg.from_external = true;
			msg.timestamp = get_clock()->now().nanoseconds() / 1000;
			request->request = msg;

			auto result = vehicle_command_client_->async_send_request(request, std::bind(&PX4AgentControl::px4_response_cb, this,
																						 std::placeholders::_1));
			RCLCPP_DEBUG(get_logger(), "Command send");
		}

		void PX4AgentControl::px4_response_cb(
			rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
		{
			auto status = future.wait_for(1s);
			if (status == std::future_status::ready)
			{
				auto reply = future.get()->reply;
				service_result_ = reply.result;
				switch (service_result_)
				{
				case reply.VEHICLE_CMD_RESULT_ACCEPTED:
					RCLCPP_DEBUG(get_logger(), "command accepted");
					break;
				case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
					RCLCPP_DEBUG(get_logger(), "command temporarily rejected");
					break;
				case reply.VEHICLE_CMD_RESULT_DENIED:
					RCLCPP_DEBUG(get_logger(), "command denied");
					break;
				case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
					RCLCPP_DEBUG(get_logger(), "command unsupported");
					break;
				case reply.VEHICLE_CMD_RESULT_FAILED:
					RCLCPP_DEBUG(get_logger(), "command failed");
					break;
				case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
					RCLCPP_DEBUG(get_logger(), "command in progress");
					break;
				case reply.VEHICLE_CMD_RESULT_CANCELLED:
					RCLCPP_DEBUG(get_logger(), "command cancelled");
					break;
				default:
					RCLCPP_DEBUG(get_logger(), "command reply unknown");
					break;
				}
			}
			else
			{
				RCLCPP_DEBUG(get_logger(), "Service In-Progress...");
			}
		}

		void PX4AgentControl::publish_introspector_query()
		{
			std_msgs::msg::String query;
			query.data = "Do you see any " + introspector_object_ + " ?";
			RCLCPP_INFO(get_logger(), "%s", query.data.c_str());
			introspector_query_pub_->publish(query);
		}

		void PX4AgentControl::publish_vln_query()
		{
			std_msgs::msg::String query;
			std::string query_content;

			if (is_mission_ingested_)
			{
				// Create an odometry string
				std::string pos = "x = " + std::to_string(vehicle_lp_.x) +
								  ", y = " + std::to_string(vehicle_lp_.y);
			
				// Compute distance to goal
				float dx = goal_x_ - vehicle_lp_.x;
				float dy = goal_y_ - vehicle_lp_.y;
				float distance_to_goal = std::sqrt(dx * dx + dy * dy);
			
				// Compose query
				query_content = "Give Move or Turn commands towards the goal based on the following information."
								" \nDistance to goal: {" + std::to_string(distance_to_goal) + "}."
								" \nCurrent position: {" + pos + "}."
								" \nCurrent heading: {" + std::to_string(vehicle_lp_.heading * RAD2DEG) + "}."
								" \nGoal position: {" + std::to_string(goal_x_) + ", " + std::to_string(goal_y_) + "}."
								" \nObstacles: " + obs_str_;
			}
			else
			{
				// Initial mission briefing: extract goal and populate obstacles once
				query_content = "You should navigate to the goal while avoiding obstacles.";
			
				// Extract goal coordinates
				std::regex goal_regex("Goal\\s*\\(\\s*(-?[0-9.]+)\\s*,\\s*(-?[0-9.]+)\\s*\\)", std::regex::icase);
				std::smatch matches;
				if (std::regex_search(mission_objective_, matches, goal_regex) && matches.size() == 3)
				{
					goal_x_ = std::stof(matches[1].str());
					goal_y_ = std::stof(matches[2].str());
				}
				else
				{
					RCLCPP_ERROR(get_logger(), "Failed to extract goal coordinates from mission objective!");
					goal_x_ = 0.0f;
					goal_y_ = 0.0f;
				}
			
				// Populate and format obs_str_
				obstacles_.clear();
				std::regex obs_regex("\\(\\s*(-?[0-9.]+)\\s*,\\s*(-?[0-9.]+)\\s*\\)");
				for (auto it = std::sregex_iterator(mission_objective_.begin(), mission_objective_.end(), obs_regex);
					 it != std::sregex_iterator(); ++it)
				{
					float ox = std::stof((*it)[1].str());
					float oy = std::stof((*it)[2].str());
					if (ox == goal_x_ && oy == goal_y_) continue;
					obstacles_.emplace_back(ox, oy);
				}
				obs_str_.clear();
				obs_str_ += "[";
				for (size_t i = 0; i < obstacles_.size(); ++i)
				{
					obs_str_ += "(" + std::to_string(obstacles_[i].first) + "," + std::to_string(obstacles_[i].second) + ")";
					if (i + 1 < obstacles_.size()) obs_str_ += ",";
				}
				obs_str_ += "]";
			
				query_content += " \nObstacles: " + obs_str_;
			
				is_mission_ingested_ = true;
			}
			
			// Append previous actions if needed
			if (resend_command_ && !vln_cmd_history_.empty())
			{
				size_t start_idx = 0;
				if (resend_size_ != -1 && vln_cmd_history_.size() > static_cast<size_t>(resend_size_))
				{
					start_idx = vln_cmd_history_.size() - static_cast<size_t>(resend_size_);
				}
				query_content += "\nPrevious actions:";
				for (size_t i = start_idx; i < vln_cmd_history_.size(); ++i)
				{
					std::string msg = vln_cmd_history_[i];
					msg.erase(0, msg.find_first_not_of(" \t\r\n"));
					msg.erase(msg.find_last_not_of(" \t\r\n") + 1);
					if (!msg.empty()) query_content += "\n" + msg;
				}
				if (resend_size_ != -1 && vln_cmd_history_.size() > static_cast<size_t>(resend_size_))
				{
					vln_cmd_history_.erase(vln_cmd_history_.begin(),
										   vln_cmd_history_.begin() + (vln_cmd_history_.size() - static_cast<size_t>(resend_size_)));
				}
			}
			
			// Publish
			query.data = query_content;
			RCLCPP_INFO(get_logger(), "Publishing VLN query: %s", query.data.c_str());
			vln_query_pub_->publish(query);
		}

	} // namespace uosm
} // namespace px4

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	int preflight_check_timeout_count_ = 0;

	enum STATE
	{
		DISARMED = 0,
		OFFBOARD_ARMED,
		HOVERING,
		FLYING,
		INTROSPECTION,
		LANDING
	} state_;

	state_ = STATE::DISARMED;
	bool is_done_ = false;

	auto node = std::make_shared<uosm::px4::PX4AgentControl>("/fmu/");
	if (node->is_init_)
	{
		rclcpp::Rate rate(uosm::px4::PUBLISH_RATE);
		RCLCPP_INFO(node->get_logger(), "Waiting Pre-flight Check");

		// Check vehicle pre-flight status
		while (rclcpp::ok() && !node->vehicle_status_.pre_flight_checks_pass)
		{
			rclcpp::spin_some(node);
			rate.sleep();
			preflight_check_timeout_count_++;
			if (preflight_check_timeout_count_ > 100)
			{
				RCLCPP_ERROR(node->get_logger(), "Pre-flight Check Failed!");
				return 1;
			}
		}

		const float home_lat = node->vehicle_gp_.lat;
		const float home_lon = node->vehicle_gp_.lon;
		const float home_alt = node->vehicle_gp_.alt;

		RCLCPP_WARN(node->get_logger(), "Pre-flight Check OK, home position: (lat = %.6f, lon = %.6f, alt = %.2f m)", home_lat, home_lon, home_alt);

		// auto last_request = node->now();

		while (rclcpp::ok())
		{
			auto nav_state = node->vehicle_status_.nav_state;
			auto arming_state = node->vehicle_status_.arming_state;
			node->publish_offboard_control_mode();
			node->publish_trajectory_setpoint();
			rclcpp::spin_some(node);
			rate.sleep();

			switch (state_)
			{
			case STATE::DISARMED:
			{
				// RCLCPP_WARN(node->get_logger(), "STATE::DISARMED");
				if (is_done_)
				{
					RCLCPP_INFO(node->get_logger(), "Flight mission completed, Exiting!");
					rclcpp::shutdown();
					return 0;
				}
				state_ = STATE::OFFBOARD_ARMED;
				break;
			}
			case STATE::OFFBOARD_ARMED:
			{
				// RCLCPP_WARN(node->get_logger(), "STATE::OFFBOARD_ARMED");

				node->switch_to_offboard_mode();
				node->arm();

				if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
					arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
				{
					// arm and takeoff
					state_ = STATE::HOVERING;
				}
				break;
			}
			case STATE::HOVERING:
			{
				// RCLCPP_WARN(node->get_logger(), "STATE::HOVERING");
				const double dist = uosm::px4::computeEuclideanDistance(node->traj_, node->vehicle_lp_, true);
				if (dist < uosm::px4::HOVERING_TOLERANCE)
				{
					// once takeoff to sufficient height, start mission
					node->is_vln_updated_ = false;
					node->publish_vln_query();
					state_ = STATE::FLYING;
				}
				break;
			}
			case STATE::FLYING:
			{
				// RCLCPP_WARN(node->get_logger(), "STATE::FLYING");
				if (node->is_vln_updated_)
				{
					const double dist = uosm::px4::computeEuclideanDistance(node->traj_, node->vehicle_lp_);
					const double heading_diff = node->traj_.yaw - node->vehicle_lp_.heading;

					if (dist < uosm::px4::FLYING_TOLERANCE && heading_diff < uosm::px4::HEADING_TOLERANCE)
					{
						// once vehicle reached traj location, start introspection
						node->is_vln_updated_ = false;
						node->is_introspection_updated_ = false;
						node->publish_introspector_query();
						state_ = STATE::INTROSPECTION;
					}
					// TODO: add VLN timeout
				}
				break;
			}
			case STATE::INTROSPECTION:
			{
				// RCLCPP_WARN(node->get_logger(), "STATE::INTROSPECTION");
				if (node->is_introspection_updated_)
				{

					if (node->is_mission_done_)
					{
						// mission completed, switch to auto land
						is_done_ = true;
						state_ = STATE::LANDING;
					}
					else
					{
						// mission not completed, keep flying with vln cmds
						node->is_introspection_updated_ = false;
						node->is_vln_updated_ = false;
						node->publish_vln_query();
						state_ = STATE::FLYING;
					}
					// TODO: add INTROSPECTION timeout
				}
				break;
			}
			case STATE::LANDING:
			{
				// RCLCPP_WARN(node->get_logger(), "STATE::LANDING");
				if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
				{
					// TODO: need a distance sensor for accurate landing for z if uneven terrain
					const auto current_x = node->vehicle_lp_.x;
					const auto current_y = node->vehicle_lp_.y;
					// Convert NED local position offset to lat/lon changes
					double lat_offset = current_x / uosm::px4::EARTH_RADIUS;
					double lon_offset = current_y / (uosm::px4::EARTH_RADIUS * cos(home_lat * uosm::px4::DEG2RAD));

					// Calculate the new lat/lon for landing
					double landing_lat = home_lat + lat_offset * uosm::px4::RAD2DEG;
					double landing_lon = home_lon + lon_offset * uosm::px4::RAD2DEG;
					node->request_landing(landing_lat, landing_lon, home_alt);
				}
				if (nav_state == px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_LAND)
				{
					node->switch_to_manual_mode();
					state_ = STATE::DISARMED;
				}
				break;
			}
			default:
				RCLCPP_WARN(node->get_logger(), "STATE::UNKNOWN");
				break;
			}
		}
	}

	rclcpp::shutdown();
	return 0;
}