#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <cocktail_bot/GoToObject.h>
#include <cocktail_bot/MakeCocktail.h>
#include <cocktail_bot/UpdateKnowledge.h>


class Reasoner
{
private:
    PrologClient pl_; // Prolog client to connect to the Prolog server

    std::string srv_go_to_obj_name_;          // Name of the service provided by the control node
    ros::ServiceClient client_go_to_obj_;     // Client to request moving the robot to an object

    std::string srv_make_cocktail_name_;      // Name of the service to receive cocktail requests
    ros::ServiceServer make_cocktail_srv_;    // Service to receive cocktail requests

    std::string srv_update_knowledge_name_;   // Name of the service to update the knowledge base
    ros::ServiceServer update_knowledge_srv_; // Service to update the knowledge base

    int ID_ = 0; // ID for the created instances

public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_WARN_STREAM("Created Reasoning Node");
        
        // Wait for the Prolog service to be advertised
        ROS_INFO_STREAM("Wait for the Prolog service...");
        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        // Create client and wait until service is advertised
        srv_go_to_obj_name_="go_to_object";
        client_go_to_obj_ = nh.serviceClient<cocktail_bot::GoToObject>(srv_go_to_obj_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_go_to_obj_name_.c_str());
        bool service_found = ros::service::waitForService(srv_go_to_obj_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_go_to_obj_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_go_to_obj_name_);

        // Create service to receive cocktail requests
        srv_make_cocktail_name_ = "make_cocktail";
        make_cocktail_srv_ = nh.advertiseService(srv_make_cocktail_name_, &Reasoner::srv_make_cocktail_callback, this);

        // Create service to update the knowledge base
        srv_update_knowledge_name_ = "update_knowledge";
        update_knowledge_srv_ = nh.advertiseService(srv_update_knowledge_name_, &Reasoner::srv_update_knowledge_callback, this);
    };

    ~Reasoner()
    {
    };

private:

    /**
     * @brief Callback function for the service that receives cocktail requests
     *
     * @param Request requested cocktail recipe
     * @param Respose response from the service
     */
    bool srv_make_cocktail_callback(cocktail_bot::MakeCocktail::Request  &req,
                                    cocktail_bot::MakeCocktail::Response &res)
    {
        ROS_INFO_STREAM("Requested cocktail: " << req.cocktail_name);

        std::stringstream ss;
        ss << "create_instance_from_class('" << req.cocktail_name << "', " << ID_ << ", Instance)";
        

        res.confirmation = true;
        return res.confirmation;
    }

    /**
     * @brief Callback function for the service that updates the knowledge base
     *
     * @param Request requested update class
     * @param Respose response from the service
     */
    bool srv_update_knowledge_callback(cocktail_bot::UpdateKnowledge::Request  &req,
                                       cocktail_bot::UpdateKnowledge::Response &res)
    {
        ROS_INFO_STREAM("Got new object of class: " << req.class_name);

        std::string instance_name = req.class_name + "_" + std::to_string(ID_);

        std::stringstream ss;
        ss << "create_instance_from_class('" << req.class_name << "', " << ID_ << ", Instance)";

        std::string query = ss.str();
        ROS_INFO_STREAM("Query: " << query);

        PrologQuery bdgs = pl_.query(query);

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            PrologBindings val = *it;

            std::stringstream instanceVal;
            instanceVal << val["Instance"];            
            instance_name = instanceVal.str();

            ROS_WARN_STREAM("new instance in knowledge base: " << instance_name);
        }

        bdgs.finish();
        ID_++;

        res.confirmation = true;
        res.instance_name = instance_name;
        return res.confirmation;
    }

    // Example of how to assert queries to prolog
    // TODO: remove both functions
    // bool assertKnowledge(std::string className)
    // {
    //     std::string instanceName;

    //     std::stringstream ss;
    //     ss << "create_instance_from_class('" << className << "', " << ID_ << ", Instance)";
        
    //     std:string query= ss.str();

    //     ROS_INFO_STREAM("query: "<<query);

    //     PrologQuery bdgs = pl_.query(query);

    //     for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    //     {
    //         PrologBindings val = *it;
    //         std::stringstream instanceVal;
    //         instanceVal << val["Instance"];
            
    //         instanceName = instanceVal.str();
    //         ROS_WARN_STREAM("new instance in knowledge base: " << instanceName);
    //     }

    //     bdgs.finish();
    //     ID_++;
        
    //     return true;
    // }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "reasoning_node");
    ros::NodeHandle nh;

    Reasoner myReasoner(nh);

    ros::spin();

    return 0;
}