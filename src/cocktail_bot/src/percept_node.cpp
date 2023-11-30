#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <cocktail_bot/UpdateObjectList.h>


class Percept
{
private:

    std::string subs_topic_name_;             // Gazebo model_states topic name
    ros::Subscriber sub_gazebo_data_;         // Subscriber gazebo model_states

    std::string srv_update_obj_name_;         // Name of the service provided by the map generator node
    ros::ServiceClient client_map_generator_; // Client to request updates to the seen objects in the map generator node
    
    std::vector<std::string> v_seen_obj_;     // List of objects seen by the robot and sent to the map generator node

public:

    Percept(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created Percept Node");

        // This objects will not be sent to the Map generator node
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        subs_topic_name_="/gazebo/model_states";

        // Create client and wait until service is advertised
        srv_update_obj_name_="update_object_list";
        client_map_generator_ = nh.serviceClient<cocktail_bot::UpdateObjectList>(srv_update_obj_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_update_obj_name_.c_str());
        bool service_found = ros::service::waitForService(srv_update_obj_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_update_obj_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_update_obj_name_);

        // Create subscriber to receive gazebo model_states
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &Percept::sub_gazebo_callback, this);
    };

    ~Percept()
    {
    };

private:

    /**
     * @brief Callback function to receive the gazebo model_states topic
     *
     * @param msg message with the current gazebo model_states
     */
    void sub_gazebo_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        geometry_msgs::Pose tiago_pose;

        // Search for tiago pose
        auto it = std::find( msg->name.begin(),  msg->name.end(), "tiago");

        // If found, get the pose
        if (it != msg->name.end())
        {
            int index = std::distance(msg->name.begin(), it);
            tiago_pose=msg->pose.at(index);
        }
        else
        {
            ROS_ERROR_STREAM("Tiago not found in the scene");
            return;
        }

        // Search new objects in the scene 
        for (int i = 0; i < msg->name.size(); i++)
        {
            // Get obj name
            std::string obj_name = msg->name[i];

            // Search for the obj name in the seen list
            auto it = std::find(v_seen_obj_.begin(), v_seen_obj_.end(), obj_name);

            // If obj already in the seen list, then skip current obj
            if (it != v_seen_obj_.end()) continue;

            // Get obj pose
            geometry_msgs::Pose obj_pose= msg->pose[i];

            // Get distance from tiago to obj
            double dx    = tiago_pose.position.x - obj_pose.position.x;
            double dy    = tiago_pose.position.y - obj_pose.position.y;
            double dist  = sqrt(pow(dx, 2)+ pow(dy, 2));

            // If the robot is close enought to the obj, then request the service
            if (dist < 20)
            {
                cocktail_bot::UpdateObjectList srv;
                srv.request.object_name = obj_name;
                srv.request.object_pose = obj_pose;

                if (client_map_generator_.call(srv))
                {
                    ROS_INFO_STREAM("Called service [" << srv_update_obj_name_ << "]\
                                        with object [" << obj_name << "]");

                    if(srv.response.confirmation)
                    {
                        v_seen_obj_.push_back(obj_name);

                        ROS_INFO_STREAM("Object [" << obj_name << "] added to the seen list");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Failed to call service " << srv_update_obj_name_);
                }
            }
        }

    // Debug print for the seen object list
    // for (size_t i = 2; i < v_seen_obj_.size(); i++)
    // {
    //     ROS_INFO_STREAM("[" << i << "]: " << v_seen_obj_.at(i));
    // } 
    }
}; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "percept_node");
    ros::NodeHandle nh;

    Percept myPercept(nh);

    ros::spin();

    return 0;
}