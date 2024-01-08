#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>

#include <cocktail_bot/UpdateObjectList.h>
#include <cocktail_bot/GetSceneObjectList.h>
#include <cocktail_bot/IsCloseToObject.h>


class MapGenerator
{
private:

    std::string srv_update_obj_name_;        // Name of the service to update the object list
    ros::ServiceServer update_obj_list_srv_; // Service to update object list

    std::string srv_get_scene_name_;         // Name of the service to provide information about requested seen objects
    ros::ServiceServer get_scene_obj_srv_;   // Service to send the pose of a target object(s) in the scene

    std::string srv_is_close_to_object_name_;     // Name of the is close to object service
    ros::ServiceServer is_close_to_object_srv_;   // Service to check if a object is close to tiago

    ros::Timer tf_timer_;                    // Timer to publish the TFs
    tf::TransformBroadcaster broadcaster_;   // TF broadcaster

    std::map<std::string, geometry_msgs::Pose> map_objs_; // Map with seen object information

public:

    MapGenerator(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created Map Generator Node");

        // Create service to update the obj list
        srv_update_obj_name_="update_object_list";
        update_obj_list_srv_ = nh.advertiseService(srv_update_obj_name_, &MapGenerator::srv_update_callback, this);
        
        // Create service to get the seen obj info
        srv_get_scene_name_ = "get_scene_object_list";
        get_scene_obj_srv_ = nh.advertiseService(srv_get_scene_name_, &MapGenerator::srv_get_scene_obj_callback, this);

        // Create service to check if object is close
        // TODO: remove this service and respective srv file
        // srv_is_close_to_object_name_ = "is_close_to_object";
        // is_close_to_object_srv_ = nh.advertiseService(srv_is_close_to_object_name_, &MapGenerator::srv_is_close_to_object_callback, this);

        // Create timer to publish TFs
        tf_timer_ = nh.createTimer(ros::Duration(1), &MapGenerator::tf_timer_callback, this);
    };

    ~MapGenerator()
    {
    };

private:

    /**
     * @brief Callback function for the service that adds objects to the map_objs
     *
     * @param Request requested object to be added to the list
     * @param Respose response from the service whether the object has been added (true/false)
     */
    bool srv_update_callback(cocktail_bot::UpdateObjectList::Request  &req,
                             cocktail_bot::UpdateObjectList::Response &res)
    {
        ROS_DEBUG_STREAM("Got new object: " << req.object_name);
        ROS_DEBUG_STREAM("Object Pose: " << req.object_pose);

        //Push the information that is obtained from the client via the request variables
        map_objs_[req.object_name] = req.object_pose;

        // Debug print for the seen object list
        // for (const auto &obj : map_objs_)
        // {
        //     ROS_DEBUG_STREAM(obj.first << ": " << obj.second);
        // }

        return true;
    }

    /**
     * @brief Callback function for the service that sends the info of a target object in the scene
     *
     * @param Request name of the requested object. Use "all" to get the list of all available objects
     * @param Respose response is a list of objects with name and pose
     */
    bool srv_get_scene_obj_callback(cocktail_bot::GetSceneObjectList::Request  &req,
                                    cocktail_bot::GetSceneObjectList::Response &res)
    {
        ROS_DEBUG_STREAM("Requested pose of object: " << req.object_name);

        // If the requested obj name is "all", we should send all the available objects
        if (req.object_name == "all")
        {
            for (const auto &obj : map_objs_)
            {
                res.objects.name.push_back(obj.first);
                res.objects.pose.push_back(obj.second);
            }
            res.obj_found = true;
            return res.obj_found;
        }

        // If the requested obj as been seen, then send the pose
        if (map_objs_.find(req.object_name) != map_objs_.end())
        {
            ROS_DEBUG_STREAM("The requested object " << req.object_name << " is in the list" << std::endl);
            res.objects.name.push_back(req.object_name);
            res.objects.pose.push_back(map_objs_[req.object_name]);
            res.obj_found = true;
        }
        else
        {
            std::stringstream s;
            s << "The requested object " << req.object_name << " is not in the list." << std::endl;

            s << "Available objects:" << std::endl;
            for (const auto &obj : map_objs_)
            {
                s << '\t' << obj.first << std::endl;
            }

            res.message = s.str();
            res.obj_found = false;
            ROS_INFO_STREAM("The requested object [" << req.object_name << "] is not in the list");
        }

        return true;
    }

    /**
     * @brief Callback function for the timer that publishes the TFs
     * 
     * @param e timer event
     */
    void tf_timer_callback(const ros::TimerEvent& e)
    {
        std::vector<geometry_msgs::TransformStamped> v_ts;

        // Get the current time
        ros::Time aux_time = ros::Time::now();
        
        for (const auto &obj : map_objs_)
        {
            geometry_msgs::TransformStamped ts;
            
            std::string object_name = obj.first;
            geometry_msgs::Pose obj_pose = obj.second;
            geometry_msgs::Point obj_position = obj_pose.position;
            geometry_msgs::Quaternion obj_orientation = obj_pose.orientation;

            // TF obj to populate our TF message
            tf2::Transform tf;

            tf.setOrigin(tf2::Vector3(obj_position.x, obj_position.y, obj_position.z)); 
            tf.setRotation(tf2::Quaternion(obj_orientation.x, obj_orientation.y, obj_orientation.z, obj_orientation.w));

            // Transform the TF obj to TF message
            ts.transform = tf2::toMsg(tf);

            // Set the reference frame for the TF (parent link)
            ts.header.frame_id = "world";
            // Set the time stamp for the message
            ts.header.stamp = aux_time;
            // Set the name for the TF
            ts.child_frame_id = object_name;

            // Add the TF to the vector of TFs
            v_ts.push_back(ts);
        }

        // Broadcast TFs to Rviz
        broadcaster_.sendTransform(v_ts);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_generator_node");
    ros::NodeHandle nh;

    MapGenerator myMapGenerator(nh);

    ros::spin();

    return 0;
}