#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <cocktail_bot/GetSceneObjectList.h>
#include <cocktail_bot/GoToObject.h>

class Controller
{
private:

    std::string subs_topic_name_;        ///< gazebo model state topic name
    std::string srv_get_scene_name_;     ///< name of the service provided by the map generator node
    std::string srv_go_to_obj_name_;     ///< name of the service provided by the map generator node
    
    std::vector<std::string> v_seen_obj_;     ///< List of objects seen by the robot and sent to the map generator node
    ros::ServiceClient client_map_generator_; ///< Client to request the object list update in the map generator node
    ros::ServiceServer go_to_obj_srv_;        ///< Advertise service to calculate the path to an object

    ros::Subscriber sub_gazebo_data_;    ///< Subscriber gazebo model_states
    ros::Publisher  pub_gazebo_data_;    ///< Publisher gazebo model_states
    ros::Timer tf_timer_;                ///< Timer to run a parallel process

    geometry_msgs::Pose tiago_pose;       ///< Pose of the robot
    geometry_msgs::Pose obj_pose;         ///< Pose of the target
    bool requested_obj;                   ///< Requested obj flag

public:

    Controller(ros::NodeHandle& nh)
    {
        subs_topic_name_="/gazebo/model_states";

        srv_get_scene_name_ = "get_scene_object_list";
        srv_go_to_obj_name_ = "go_to_object";

        // This objects will not be sent to the Map generator node
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        // Create client and wait until service is advertised
        client_map_generator_ = nh.serviceClient<cocktail_bot::GetSceneObjectList>(srv_get_scene_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_get_scene_name_.c_str());
        bool service_found = ros::service::waitForService(srv_get_scene_name_, ros::Duration(30.0)); // You can adjust the timeout as needed

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_get_scene_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_get_scene_name_);

        // Advertising the new service
        go_to_obj_srv_ = nh.advertiseService(srv_go_to_obj_name_, &Controller::srv_go_to_obj_callback, this);

        // Create subscriber to gazebo
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &Controller::topic_callback, this);
        pub_gazebo_data_ = nh.advertise<geometry_msgs::Twist>("/key_vel", 100);

        tf_timer_ = nh.createTimer(ros::Duration(0.5), &Controller::publishControls, this);
    };

    ~Controller()
    {
    };
    
private:

    /**
     * @brief Callback function for the service that calculates the path to an object
     *
     * @param Request requested object to move to
     * @param Respose response from the service if the robot has reached the obj (true/false)
     */
    bool srv_go_to_obj_callback(cocktail_bot::GoToObject::Request  &req,
                                cocktail_bot::GoToObject::Response &res)
    {
        ROS_INFO_STREAM("Request move to object: " << req.object_name);

        cocktail_bot::GetSceneObjectList srv;

        srv.request.object_name = req.object_name;

        if (!client_map_generator_.call(srv))
        {
            ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
            res.confirmation = false;
            return res.confirmation;
        }

        if (!srv.response.obj_found)
        {
            ROS_ERROR_STREAM("Object not found");
            res.confirmation = false;
            return res.confirmation;
        }

        // Get the pose of the object
        obj_pose = srv.response.objects.pose[0];
        requested_obj = true;

        res.confirmation = true;
        return res.confirmation;
    }

    Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &quaternion)
    {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        Eigen::Matrix2d rotationMatrix = eigenQuaternion.toRotationMatrix().block(0,0,2,2);
        return rotationMatrix;
    }

    /**
     * @brief Callback function to publish the controls to gazebo
     *
     * @param e Timer event
     */
    void publishControls(const ros::TimerEvent& e)
    {
        if (!requested_obj) return;

        // Get position vectors
        geometry_msgs::Twist tiago_twist_cmd;
        Eigen::Vector2d tiago_w = {tiago_pose.position.x, tiago_pose.position.y};
        Eigen::Vector2d target_w = {obj_pose.position.x, obj_pose.position.y};

        Eigen::Vector2d Dpose_w = target_w - tiago_w;

        //Get 2D Rotation matrix from quaternion
        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose.orientation);
        Eigen::Matrix2d Rw_tiago = Rtiago_w.inverse();
        
        // Calculate vector from the origin to the target
        Eigen::Vector2d Dpose_tiago = Rw_tiago * Dpose_w;
        double d = (Dpose_tiago.norm() <= 1.3) ? 0.0: Dpose_tiago.norm();

        // Calculate the angle to the target
        double theta = std::atan2(Dpose_tiago(1),Dpose_tiago(0));
        double Kwz=1.1, Kvx=0.1;
        tiago_twist_cmd.linear.x = Kvx*d;
        tiago_twist_cmd.angular.z = Kwz*theta;

        ROS_INFO_STREAM("Published msg: "<<tiago_twist_cmd);
        pub_gazebo_data_.publish(tiago_twist_cmd);

        if (tiago_twist_cmd.linear.x == 0. && abs(tiago_twist_cmd.angular.z) < 0.002) {
            ROS_INFO_STREAM("Reached object");
            requested_obj = false;
        }
    }

    /**
     * @brief Callback function to receive the Gazebo Model State topic
     *
     * @param msg message with the current Gazebo model state
     */
    void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        // Search for tiago pose
        auto it = std::find( msg->name.begin(),  msg->name.end(), "tiago");
        if (it != msg->name.end()) 
        {
            // Calculate the index
            int index = std::distance(msg->name.begin(), it);
            tiago_pose=msg->pose.at(index);
        }
    } // callback
}; // Class 

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "tiago_control_node");
    ros::NodeHandle nh;

    Controller controllerNode(nh);

    // Run the node
    ros::spin();

    return 0;
}