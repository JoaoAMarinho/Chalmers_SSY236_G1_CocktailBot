#include <ros/ros.h>
#include <fstream>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <cocktail_bot/UpdateKnowledge.h>
#include <cocktail_bot/UpdateObjectList.h>
#include <cocktail_bot/ClassifyObject.h>
#include "std_msgs/String.h"

class Percept
{
private:

    std::string subs_topic_name_;      // Gazebo model_states topic name
    ros::Subscriber sub_gazebo_data_;  // Subscriber gazebo model_states

    std::string state_topic_name_;  // State topic name
    ros::Subscriber sub_state_;     // Subscriber robot state

    std::string srv_update_obj_name_;         // Name of the service provided by the map generator node
    ros::ServiceClient client_map_generator_; // Client to request updates to the seen objects in the map generator node

    std::string srv_get_state_name_;          // Name of the service provided by the controller node
    ros::ServiceClient client_get_state_;     // Client to request the current state of the robot

    std::string srv_update_knowledge_name_;   // Name of the service provided by the reasoning node
    ros::ServiceClient client_reasoning_;     // Client to request updates to the seen objects in the reasoning node

    std::string srv_classify_obj_name_;       // Name of the service provided by the classifier node
    ros::ServiceClient client_classifier_;    // Client to request classification to classifier node
    
    std::vector<std::string> v_seen_obj_;     // List of objects seen by the robot and sent to the map generator node

    std::map<std::string, cocktail_bot::ClassifyObject> map_objs_info_; // Map with seen object characteristics
    std::string state_ = "EXPLORING"; // Current state of the robot

public:

    Percept(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created Percept Node");

        // This objects will not be sent to the Map generator node
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        // Load object characteristics from the dataset
        load_obj_characteristics();

        state_topic_name_ = "/get_state";
        subs_topic_name_  = "/gazebo/model_states";

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

        // Create client and wait until service is advertised
        srv_update_knowledge_name_="update_knowledge";
        client_reasoning_ = nh.serviceClient<cocktail_bot::UpdateKnowledge>(srv_update_knowledge_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_update_knowledge_name_.c_str());
        service_found = ros::service::waitForService(srv_update_knowledge_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_update_knowledge_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_update_knowledge_name_);


        // Create client and wait until service is advertised
        srv_classify_obj_name_="classify_object";
        client_classifier_ = nh.serviceClient<cocktail_bot::ClassifyObject>(srv_classify_obj_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_classify_obj_name_.c_str());
        service_found = ros::service::waitForService(srv_classify_obj_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_classify_obj_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_classify_obj_name_);

        // Create subscriber to receive gazebo model_states
        sub_state_ = nh.subscribe(state_topic_name_, 100, &Percept::sub_state_callback, this);

        // Create subscriber to receive gazebo model_states
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &Percept::sub_gazebo_callback, this);

        bool DEBUG = false;
        if (DEBUG) {
            // Create test instances

            geometry_msgs::Pose pose;
            pose.position.x = 10.;
            pose.position.y = 10.;
            pose.position.z = 2.;
            pose.orientation.x = 0.;
            pose.orientation.y = 0.;
            pose.orientation.z = 0.;

            update_node_knowledge("Apple", pose);
            //update_node_knowledge("glass", pose);
            //update_node_knowledge("glass", pose);
        }
    };

    ~Percept()
    {
    };

private:

    /**
     * @brief Load object characteristics from the dataset
     * 
     */
    void load_obj_characteristics(){
        std::string FILE_PATH = "/home/user/exchange/src/cocktail_bot/model/datasets/env.csv";
        
        // Open the CSV file
        std::ifstream file(FILE_PATH);

        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open object characteristics file!");
            return;
        }

        std::string line;
        std::vector<std::string> values;
        std::string value;

        // Skip first line
        std::getline(file,line);
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            cocktail_bot::ClassifyObject srv_classifier;

            // Split the line by ','
            while (std::getline(iss, value, ',')) {
                values.push_back(value);
            }

            srv_classifier.request.mass = std::stof(values[1]);
            srv_classifier.request.width = std::stof(values[2]);
            srv_classifier.request.height = std::stof(values[3]);
            srv_classifier.request.shape = values[4];
            srv_classifier.request.red = std::stoi(values[5]);
            srv_classifier.request.green = std::stoi(values[6]);
            srv_classifier.request.blue = std::stoi(values[7]);
            srv_classifier.request.alcohol = std::stoi(values[8]);
            map_objs_info_[values[0]] = srv_classifier;
            ROS_INFO_STREAM("Object [" << values[0] << "] added to the map");
            values.clear();
        }

        // Close the file
        file.close();
    };

    void sub_state_callback(const std_msgs::String::ConstPtr& msg)
    {
        //ROS_INFO_STREAM("State: " << msg->data);
        state_ = msg->data;
    }

    /**
     * @brief Callback function to receive the gazebo model_states topic
     *
     * @param msg message with the current gazebo model_states
     */
    void sub_gazebo_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        geometry_msgs::Pose tiago_pose;

        // Search for tiago pose
        auto it = std::find(msg->name.begin(),  msg->name.end(), "tiago");

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

            if (dist > 3) continue;

            if (state_ == "EXPLORING") {
                if (classifiable_object(obj_name))
                    continue;
                update_node_knowledge(obj_name, obj_pose);
            }
            else if (dist < 1)
            {
                update_node_knowledge(obj_name, obj_pose);
            }
        }

    // Debug print for the seen object list
    // for (size_t i = 2; i < v_seen_obj_.size(); i++)
    // {
    //     ROS_INFO_STREAM("[" << i << "]: " << v_seen_obj_.at(i));
    // } 
    }

    /**
     * @brief Update the knowledge base of the reasoning node and the seen object list of the map generator node
     * 
     * @param obj_name name of the object to be updated
     * @param obj_pose pose of the object to be updated
     * @return true if the update was successful
     * @return false if the update failed
     */
    void update_node_knowledge(std::string obj_name, geometry_msgs::Pose obj_pose)
    {
        cocktail_bot::UpdateKnowledge srv_reasoning;
        srv_reasoning.request.class_name = call_classify_object(obj_name);
        srv_reasoning.request.instance_name = obj_name;

        if (!client_reasoning_.call(srv_reasoning))
        {
            ROS_ERROR_STREAM("Failed to call service " << srv_update_obj_name_);
            return;
        }

        cocktail_bot::UpdateObjectList srv_map_generator;
        srv_map_generator.request.object_name = obj_name;
        srv_map_generator.request.object_pose = obj_pose;

        if (client_map_generator_.call(srv_map_generator))
        {            
            v_seen_obj_.push_back(obj_name);
            ROS_INFO_STREAM("Object [" << obj_name << "] added to the seen list");
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call service " << srv_update_obj_name_);
        }
    }

    bool classifiable_object(std::string object_name)
    {
        auto it = map_objs_info_.find(get_object_class_name(object_name));
        return (it != map_objs_info_.end());
    }

    std::string call_classify_object(std::string object_name)
    {
        auto it = map_objs_info_.find(object_name);
        std::string classification_name = get_object_class_name(object_name);

        if (it != map_objs_info_.end()) {
            cocktail_bot::ClassifyObject object_info = it->second;
            if(client_classifier_.call(object_info)){
                ROS_INFO_STREAM("Called service [" << srv_classify_obj_name_ << "]\
                                with object [" << object_name << "]");
            }
            else {
                ROS_ERROR_STREAM("Failed to call service " << srv_classify_obj_name_);
            }

            if(object_info.response.confirmation){
               classification_name = object_info.response.class_name;
               ROS_INFO_STREAM("Object classified has [" << classification_name << "]");
            }
        }

        return classification_name;
    }

    std::string get_object_class_name(std::string name){
        size_t lastPos = name.rfind('_');
        std::string result = name; 

        if (lastPos != std::string::npos) {
            result = name.substr(0,lastPos);
        }

        return result;
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