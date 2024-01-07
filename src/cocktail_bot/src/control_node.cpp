#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <cocktail_bot/GetSceneObjectList.h>
#include <cocktail_bot/MoveToObject.h>
#include <cocktail_bot/GetState.h>
#include <cocktail_bot/ArrivedToObject.h>

#define START_COCKTAIL "START_COCKTAIL"

enum class State {
    IDLE,
    EXPLORING,
    AVAILABLE_TO_REQUEST,
    MOVING_TO_OBJECT
};

struct IngredientPoses {
    std::vector<geometry_msgs::Pose> instance_poses;
    std::vector<geometry_msgs::Pose> alternative_poses;
};

class Controller
{
private:

    std::string subs_topic_name_;      // Gazebo model_states topic name
    ros::Subscriber sub_gazebo_data_;  // Subscriber gazebo model_states

    ros::Publisher pub_controls_;      // Publisher gazebo key_vel
    ros::Timer controls_timer_;        // Timer to publish control actions
    
    std::string srv_move_to_object_name_;   // Name of the service to make the robot move to an object
    ros::ServiceServer move_to_object_srv_; // Service to make the robot move to an object
    
    std::string srv_get_state_name_;   // Name of the service to inform the state of the robot
    ros::ServiceServer get_state_srv_; // Service to inform the state of the robot
    
    std::string srv_get_scene_name_;          // Name of the service provided by the map generator node
    ros::ServiceClient client_map_generator_; // Client to request information about objects in the scene

    std::string srv_arrived_to_object_name_;      // Name of the service provided by the reasoning node
    ros::ServiceClient client_arrived_to_object_; // Client to inform that object robot arrive to object
    
    State state_ = State::EXPLORING;   // Current state of the robot

    double ANGULAR_VEL = 1.1;          // Angular velocity of the robot
    double LINAR_VEL   = 0.1;          // Linear velocity of the robot

    geometry_msgs::Pose tiago_pose;    // Pose of the robot
    geometry_msgs::Pose target_pose;   // Pose of the target
    std::string target_name;           // Name of the target

    std::vector<geometry_msgs::Pose> poi_poses; // Poses of the points of interest
    std::size_t poi_index;                      // Index of the current point of interest

public:

    Controller(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created Controller Node");

        // Create points of interest
        create_poi();

        // Create service to move the robot to an object
        srv_move_to_object_name_ = "move_to_object";
        move_to_object_srv_ = nh.advertiseService(srv_move_to_object_name_, &Controller::move_to_object_callback, this);

        // Create service to inform robot state
        srv_get_state_name_ = "get_state";
        get_state_srv_ = nh.advertiseService(srv_get_state_name_, &Controller::get_state_callback, this);

        // Create subscriber to receive gazebo model_states
        subs_topic_name_="/gazebo/model_states";
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &Controller::sub_gazebo_callback, this);
        
        // Create publisher to send controls to gazebo
        pub_controls_ = nh.advertise<geometry_msgs::Twist>("/key_vel", 100);
        controls_timer_ = nh.createTimer(ros::Duration(0.5), &Controller::controls_timer_callback, this);

        // Create client and wait until service is advertised
        srv_get_scene_name_ = "get_scene_object_list";
        client_map_generator_ = nh.serviceClient<cocktail_bot::GetSceneObjectList>(srv_get_scene_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_get_scene_name_.c_str());
        bool service_found = ros::service::waitForService(srv_get_scene_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_get_scene_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_get_scene_name_);

        // Create client and wait until service is advertised
        srv_arrived_to_object_name_ = "arrived_to_object";
        client_arrived_to_object_ = nh.serviceClient<cocktail_bot::ArrivedToObject>(srv_arrived_to_object_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_arrived_to_object_name_.c_str());
        service_found = ros::service::waitForService(srv_arrived_to_object_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_arrived_to_object_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_arrived_to_object_name_);
    };

    ~Controller()
    {
    };
    
private:

    /**
     * @brief Function to create the points of interest
     */
    void create_poi()
    {
        poi_index = 0;

        geometry_msgs::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = -3.0;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = -6.27;
        pose.orientation.z = -0.86;
        pose.orientation.w = 0.5;
        poi_poses.push_back(pose);

        pose.position.x = 3.24;
        pose.position.y = 0.0;
        pose.orientation.x = 2.38;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.488;
        pose.orientation.w = 0.87;
        poi_poses.push_back(pose);

        pose.position.x = 0.35;
        pose.position.y = 2.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 1.0;
        pose.orientation.w = 0.0;
        poi_poses.push_back(pose);

        pose.position.x = -1.0;
        pose.position.y = -1.0;
        pose.orientation.x = 1.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        poi_poses.push_back(pose);
    }

    /**
     * @brief Callback function for the service that returns the state of the robot
     *
     * @param Request empty request
     * @param Respose response from the service with the state of the robot
     */
    bool get_state_callback(cocktail_bot::GetState::Request  &req,
                            cocktail_bot::GetState::Response &res)
    {
        res.state = (state_ == State::EXPLORING ? "EXPLORING" : "OTHER");
        return true;
    }

    /**
     * @brief Callback function for the service that makes the robot look for ingredients
     *
     * @param Request requested ingredients information
     * @param Respose response from the service if the robot can look for ingredients (true/false)
     */
    bool move_to_object_callback(cocktail_bot::MoveToObject::Request  &req,
                                 cocktail_bot::MoveToObject::Response &res)
    {
        // Check if the robot is available to receive requests
        if (state_ != State::AVAILABLE_TO_REQUEST)
        {
            ROS_WARN_STREAM("Robot is not available to receive requests");
            return false;
        }

        // Change the state of the robot
        state_ = State::IDLE;

        // Check for cocktail start
        if (req.object_name == START_COCKTAIL)
        {
            target_pose = tiago_pose;
            target_name = "";
            state_ = State::MOVING_TO_OBJECT;
            return true;
        }

        
        cocktail_bot::GetSceneObjectList srv;
        srv.request.object_name = req.object_name;

        if (!client_map_generator_.call(srv))
        {
            ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
            return false;
        }

        if (!srv.response.obj_found)
        {
            ROS_ERROR_STREAM("Object not found!");
            return false;
        }

        target_name = srv.request.object_name;
        target_pose = srv.response.objects.pose[0];
        state_ = State::MOVING_TO_OBJECT;
        return true;
    }

    /**
     * @brief Function to convert a quaternion to a 2D rotation matrix
     *
     * @param quaternion quaternion to be converted
     * @return Eigen::Matrix2d 2D rotation matrix
     */
    Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &quaternion)
    {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        Eigen::Matrix2d rotationMatrix = eigenQuaternion.toRotationMatrix().block(0,0,2,2);
        return rotationMatrix;
    }

    /**
     * @brief Function to calculate the controls to move the robot to the target
     *
     * @param target_pose pose of the target
     * @return geometry_msgs::Twist controls to move the robot to the target
     */
    geometry_msgs::Twist calculate_controls_to_target(geometry_msgs::Pose target_pose)
    {
        // Create controls message
        geometry_msgs::Twist controls_cmd;

        // Get position vectors
        Eigen::Vector2d tiago_w = {tiago_pose.position.x, tiago_pose.position.y};
        Eigen::Vector2d target_w = {target_pose.position.x, target_pose.position.y};

        Eigen::Vector2d Dpose_w = target_w - tiago_w;

        // Get 2D Rotation matrix from quaternion
        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose.orientation);
        Eigen::Matrix2d Rw_tiago = Rtiago_w.inverse();
        
        // Calculate vector from the origin to the target
        Eigen::Vector2d Dpose_tiago = Rw_tiago * Dpose_w;
        double d = (Dpose_tiago.norm() <= 1.3) ? 0.0: Dpose_tiago.norm();

        // Calculate the angle to the target
        double theta = std::atan2(Dpose_tiago(1),Dpose_tiago(0));

        controls_cmd.linear.x  = LINAR_VEL * d;
        controls_cmd.angular.z = ANGULAR_VEL * theta;

        return controls_cmd;
    }

    /**
     * @brief Callback function to publish the controls to gazebo
     *
     * @param e timer event
     */
    void controls_timer_callback(const ros::TimerEvent& e)
    {
        // Check if the robot is trying to reach a target
        if (state_ == State::IDLE ||
            state_ == State::AVAILABLE_TO_REQUEST) return;

        geometry_msgs::Twist controls_cmd;

        if (state_ == State::EXPLORING) {
            if (poi_index == poi_poses.size()) {
                ROS_INFO_STREAM("Finished exploring");
                state_ = State::AVAILABLE_TO_REQUEST;
                return;
            }

            controls_cmd = calculate_controls_to_target(poi_poses[poi_index]);
        }

        if (state_ == State::MOVING_TO_OBJECT) {
            controls_cmd = calculate_controls_to_target(target_pose);
        }

        // Publish controls
        pub_controls_.publish(controls_cmd);
        ROS_DEBUG_STREAM("Published msg: " << controls_cmd);

        // Check if the robot has reached the target
        if (controls_cmd.linear.x == 0. && abs(controls_cmd.angular.z) < 0.002) {
            if (state_ == State::EXPLORING)
            {
                ROS_INFO_STREAM("Reached point of interest");
                poi_index++;
            }
            else
            {
                state_ = State::AVAILABLE_TO_REQUEST;
                cocktail_bot::ArrivedToObject srv;
                srv.request.object_name  = target_name;
                srv.request.current_pose = tiago_pose;
                client_arrived_to_object_.call(srv);
            }
        }
    }

    /**
     * @brief Callback function to receive the gazebo model_states topic
     *
     * @param msg message with the current gazebo model_states
     */
    void sub_gazebo_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        // Search for tiago pose
        auto it = std::find( msg->name.begin(),  msg->name.end(), "tiago");
        if (it != msg->name.end()) 
        {
            // Calculate the index
            int index = std::distance(msg->name.begin(), it);
            tiago_pose=msg->pose.at(index);
        }
        else
        {
            ROS_ERROR_STREAM("Tiago not found in the scene");
        }
    }
}; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    Controller myController(nh);

    ros::spin();

    return 0;
}