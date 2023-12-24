#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <cocktail_bot/GetSceneObjectList.h>
#include <cocktail_bot/FindIngredients.h>
#include <cocktail_bot/ArriveToObject.h>

enum class State {
    IDLE,
    EXPLORING,
    AVAILABLE_TO_REQUEST,
    MOVING_TO_TARGET,
    FIND_INGREDIENT
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
    
    std::string srv_find_ingredients_name_;   // Name of the service to make the robot look for ingredients
    ros::ServiceServer find_ingredients_srv_; // Service to make the robot look for ingredients
    
    std::string srv_get_scene_name_;          // Name of the service provided by the map generator node
    ros::ServiceClient client_map_generator_; // Client to request information about objects in the scene

    // TODO send arrive to object message
    std::string srv_arrive_to_object_name_;      // Name of the service provided by the reasoning node
    ros::ServiceClient client_arrive_to_object_; // Client to inform that object robot arrive to object
    
    //State state_ = State::EXPLORING;   // Current state of the robot
    State state_ = State::AVAILABLE_TO_REQUEST;   // TODO: testing purposes, remove later

    double ANGULAR_VEL = 1.1;          // Angular velocity of the robot
    double LINAR_VEL   = 0.1;          // Linear velocity of the robot

    geometry_msgs::Pose tiago_pose;                          // Pose of the robot
    std::map<std::string, IngredientPoses> ingredient_map;   // Poses of the objects in the scene

    std::vector<geometry_msgs::Pose> poi_poses; // Poses of the points of interest
    std::size_t poi_index;                      // Index of the current point of interest

public:

    Controller(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created Controller Node");

        // Create service to move the robot to an object
        srv_find_ingredients_name_ = "find_ingredients";
        find_ingredients_srv_ = nh.advertiseService(srv_find_ingredients_name_, &Controller::find_ingredients_callback, this);

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
        srv_arrive_to_object_name_ = "arrive_to_object";
        client_arrive_to_object_ = nh.serviceClient<cocktail_bot::ArriveToObject>(srv_arrive_to_object_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_arrive_to_object_name_.c_str());
        service_found = ros::service::waitForService(srv_arrive_to_object_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_arrive_to_object_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_arrive_to_object_name_);
    };

    ~Controller()
    {
    };
    
private:

    /**
     * @brief Auxiliary function to convert a string to a vector of strings
     *
     * @param input string to be converted
     * @return std::vector<std::string> vector of strings
     */
    std::vector<std::string> stringToVector(const std::string& input)
    {
        std::vector<std::string> result;
        std::istringstream iss(input.substr(1, input.size() - 2));
        std::string token;

        while (std::getline(iss, token, ',')) {
            // Add each token (substring) to the vector
            result.push_back(token);
        }

        return result;
    }

    /**
     * @brief Callback function for the service that makes the robot look for ingredients
     *
     * @param Request requested ingredients information
     * @param Respose response from the service if the robot can look for ingredients (true/false)
     */
    bool find_ingredients_callback(cocktail_bot::FindIngredients::Request  &req,
                                   cocktail_bot::FindIngredients::Response &res)
    {
        // Check if the robot is available to receive requests
        if (state_ != State::AVAILABLE_TO_REQUEST) {
            ROS_WARN_STREAM("Robot is not available to receive requests");
            res.confirmation = false;
            return true;
        }

        // Change the state of the robot
        state_ = State::IDLE;

        // Save the poses of the ingredients
        IngredientPoses ingredient_poses;
        for (int i = 0; i < req.ingredients.size(); i++)
        {
            // Save the poses of the ingredient instances
            for (std::string instance : stringToVector(req.ingredient_instances[i]))
            {
                cocktail_bot::GetSceneObjectList srv;
                srv.request.object_name = instance;

                if (!client_map_generator_.call(srv))
                {
                    ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
                    res.confirmation = false;
                    return true;
                }

                if (!srv.response.obj_found)
                {
                    ROS_ERROR_STREAM("Object not found!");
                    res.confirmation = false;
                    return true;
                }

                ingredient_poses.instance_poses.push_back(srv.response.objects.pose[0]);
            }

            // Save the poses of the alternative instances
            for (std::string instance : stringToVector(req.alternative_instances[i]))
            {
                cocktail_bot::GetSceneObjectList srv;
                srv.request.object_name = instance;

                if (!client_map_generator_.call(srv))
                {
                    ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
                    res.confirmation = false;
                    return true;
                }

                if (!srv.response.obj_found)
                {
                    ROS_ERROR_STREAM("Object not found!");
                    res.confirmation = false;
                    return true;
                }

                ingredient_poses.alternative_poses.push_back(srv.response.objects.pose[0]);
            }

            ingredient_map[req.ingredients[i]] = ingredient_poses;
        }

        state_ = State::FIND_INGREDIENT;
        res.confirmation = true;
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

        if (state_ == State::MOVING_TO_TARGET) {
            //controls_cmd = calculate_controls_to_target(target_pose);
        }

        // Publish controls
        pub_controls_.publish(controls_cmd);
        ROS_INFO_STREAM("Published msg: " << controls_cmd);

        // Check if the robot has reached the target
        if (controls_cmd.linear.x == 0. && abs(controls_cmd.angular.z) < 0.002) {
            if (state_ == State::EXPLORING)
            {
                ROS_INFO_STREAM("Reached point of interest");
                poi_index++;
            }
            else
            {
                ROS_INFO_STREAM("Reached object");
                state_ = State::IDLE;
                cocktail_bot::ArriveToObject srv;

                srv.request.object_name = "";
                srv.request.current_pose = tiago_pose;

                client_arrive_to_object_.call(srv);
                //Arrive to object

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