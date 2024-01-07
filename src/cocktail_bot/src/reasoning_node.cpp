#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <queue>
#include <unistd.h> // For Unix-based systems

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <cocktail_bot/MoveToObject.h>
#include <cocktail_bot/MakeCocktail.h>
#include <cocktail_bot/UpdateKnowledge.h>
#include <cocktail_bot/GetSceneObjectList.h>
#include <cocktail_bot/ArrivedToObject.h>

#define START_COCKTAIL "START_COCKTAIL"

enum class State {
    AVAILABLE_TO_REQUEST,
    PREPARING_COCKTAIL,
    LOOKING_FOR_INSTANCE,
    LOOKING_FOR_ALTERNATIVE,
};

struct IngredientInstances {
    std::vector<std::string> instance_names;
    std::vector<std::string> alternative_names;
};

class Reasoner
{
private:
    PrologClient pl_; // Prolog client to connect to the Prolog server

    std::string srv_move_to_object_name_;      // Name of the service provided by the control node
    ros::ServiceClient client_move_to_object_; // Client to request the robot to find ingredients

    std::string srv_get_scene_name_;             // Name of the service provided by map generator node
    ros::ServiceClient client_get_scene_object_; // Client to ask for the pose of a target object(s) in the scene

    std::string srv_make_cocktail_name_;         // Name of the service to receive cocktail requests
    ros::ServiceServer make_cocktail_srv_;       // Service to receive cocktail requests

    std::string srv_update_knowledge_name_;      // Name of the service to update the knowledge base
    ros::ServiceServer update_knowledge_srv_;    // Service to update the knowledge base

    std::string srv_arrived_to_object_name_;   // Name of the service to receive arrive to object status
    ros::ServiceServer arrived_to_object_srv_; // Service to receive arrive to object request

    State state_ = State::AVAILABLE_TO_REQUEST;  // Current state of the robot

    struct CustomComparator {
        bool operator()(const std::pair<std::string, IngredientInstances>& ing1,
                        const std::pair<std::string, IngredientInstances>& ing2)
        {
            return ing1.second.instance_names.size() < ing2.second.instance_names.size();
        }
    };

    std::priority_queue<std::pair<std::string, IngredientInstances>,
                        std::vector<std::pair<std::string, IngredientInstances>>,
                        CustomComparator> ingredients_info; // Queue to store the ingredients for the requested cocktail
    int ID_ = 0; // ID for the created instances

public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_WARN_STREAM("Created Reasoning Node");
        
        // Wait for the Prolog service to be advertised
        ROS_INFO_STREAM("Wait for the Prolog service...");
        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        // Create service to receive cocktail requests
        srv_make_cocktail_name_ = "make_cocktail";
        make_cocktail_srv_ = nh.advertiseService(srv_make_cocktail_name_, &Reasoner::srv_make_cocktail_callback, this);

        // Create service to update the knowledge base
        srv_update_knowledge_name_ = "update_knowledge";
        update_knowledge_srv_ = nh.advertiseService(srv_update_knowledge_name_, &Reasoner::srv_update_knowledge_callback, this);

        // Create service to receive arrive to object
        srv_arrived_to_object_name_ = "arrived_to_object";
        arrived_to_object_srv_ = nh.advertiseService(srv_arrived_to_object_name_, &Reasoner::srv_arrived_to_object_callback, this);

        // Create client and wait until service is advertised
        srv_move_to_object_name_ = "move_to_object";
        client_move_to_object_ = nh.serviceClient<cocktail_bot::MoveToObject>(srv_move_to_object_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_move_to_object_name_.c_str());
        bool service_found = ros::service::waitForService(srv_move_to_object_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_move_to_object_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_move_to_object_name_);

        // Create client and wait until service is advertised
        srv_get_scene_name_ = "get_scene_object_list";
        client_get_scene_object_ = nh.serviceClient<cocktail_bot::GetSceneObjectList>(srv_get_scene_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_get_scene_name_.c_str());
        service_found = ros::service::waitForService(srv_get_scene_name_, ros::Duration(30.0));

        if (!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_get_scene_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_get_scene_name_);
    };

    ~Reasoner()
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
     * @brief Callback function for the service that receives cocktail requests
     *
     * @param Request requested cocktail recipe
     * @param Respose response from the service
     */
    bool srv_make_cocktail_callback(cocktail_bot::MakeCocktail::Request  &req,
                                    cocktail_bot::MakeCocktail::Response &res)
    {
        ROS_WARN_STREAM("Requested cocktail: " << req.cocktail_name);

        // Check if the robot is available to receive requests
        if (state_ != State::AVAILABLE_TO_REQUEST) {
            ROS_WARN_STREAM("Robot is not available to receive requests!");
            res.confirmation = false;
            return true;
        }

        state_ = State::PREPARING_COCKTAIL;

        // Build query to get the instances for the requested cocktail
        std::stringstream ss;
        ss << "get_instances_for_cocktail('" << req.cocktail_name << "', Ingredients, Ingred_inst, Alt_inst)";
        std::string query = ss.str();
        ROS_INFO_STREAM("Query: " << query);

        PrologQuery bdgs = pl_.query(query);
        
        // Iterate over one solution
        for(PrologQuery::iterator prolog_it=bdgs.begin(); prolog_it != bdgs.end(); prolog_it++)
        {
            PrologBindings val = *prolog_it;

            // Convert each returned value to a vector of strings
            std::map<std::string, std::vector<std::string>> query_result;
            for (std::map<std::string, PrologValue>::iterator map_it=val.begin(); map_it != val.end(); map_it++)
            {
                PrologValue value = map_it->second;

                if (value.isList())
                {
                    std::vector<PrologValue> values = value.as<std::vector<PrologValue>>();
                    std::vector<std::string> string_vect(values.size());
                    std::transform(values.begin(), values.end(), string_vect.begin(), [](const PrologValue& val) {
                        return val.toString();
                    });
                    query_result[map_it->first] = string_vect;
                }
            }

            // Build the queue with the ingredients and their instances
            for (int i = 0; i < query_result["Ingredients"].size(); i++)
            {
                std::string ingredient = query_result["Ingredients"][i];
                std::vector<std::string> ingredient_names  = stringToVector(query_result["Ingred_inst"][i]);
                std::vector<std::string> alternative_names = stringToVector(query_result["Alt_inst"][i]);

                // Check if there are any instances for the ingredient
                if (ingredient_names.empty() && alternative_names.empty())
                {
                    res.confirmation = false;
                    ROS_ERROR_STREAM("No possible instances to search for ingredient: " << ingredient);
                    state_ = State::AVAILABLE_TO_REQUEST;
                    return false;
                }
                // Add the ingredient to the map
                IngredientInstances ingredient_instances;
                ingredient_instances.instance_names    = ingredient_names;
                ingredient_instances.alternative_names = alternative_names;
                ingredients_info.push({ ingredient, ingredient_instances});
            }
            break;
        }
        
        bdgs.finish();

        // Check if the query returned any ingredients
        if (ingredients_info.empty())
        {
            res.confirmation = false;
            ROS_ERROR_STREAM("No ingredients found for cocktail: " << req.cocktail_name);
            state_ = State::AVAILABLE_TO_REQUEST;
            return true;
        }

        // Call service to start making cocktail
        ROS_INFO_STREAM("Started making cocktail: " << req.cocktail_name);

        cocktail_bot::MoveToObject srv;
        srv.request.object_name = START_COCKTAIL;

        if (!client_move_to_object_.call(srv))
        {
            res.confirmation = false;
            ROS_ERROR_STREAM("Failed to call service " << srv_move_to_object_name_);
            state_ = State::AVAILABLE_TO_REQUEST;
            return false;
        }

        res.confirmation = true;
        return true;
    }

    bool srv_arrived_to_object_callback(cocktail_bot::ArrivedToObject::Request  &req,
                                        cocktail_bot::ArrivedToObject::Response &res)
    {
        // Retrieve the first value from the queue
        auto iter = ingredients_info.top();
        std::string ingredient = iter.first;
        IngredientInstances ingredient_instances = iter.second;

        if (state_ == State::LOOKING_FOR_INSTANCE)
        {
            ROS_INFO_STREAM("Picked up instance [" << ingredient_instances.instance_names[0]
                            << "] of type [" << ingredient << "]");
            ingredients_info.pop();
        }
        else if (state_ == State::LOOKING_FOR_ALTERNATIVE)
        {
            ROS_INFO_STREAM("Arrived at container: " << ingredient_instances.alternative_names[0]);
            sleep(2); // Pauses execution for 2 seconds

            // Build query to get the instances for the requested cocktail
            std::stringstream ss;
            ss << "get_instances_for_class('" << ingredient << "', Ingred_inst, _)";
            std::string query = ss.str();
            ROS_INFO_STREAM("Query: " << query);

            PrologQuery bdgs = pl_.query(query);
            std::vector<std::string> new_ingredient_instances;
            
            // Iterate over one solution
            for(PrologQuery::iterator prolog_it = bdgs.begin(); prolog_it != bdgs.end(); prolog_it++)
            {
                PrologBindings val = *prolog_it;

                // Find new ingredients
                PrologValue value = val["Ingred_inst"];
                new_ingredient_instances = stringToVector(value.toString());
                break;
            }
            bdgs.finish();

            if (new_ingredient_instances.empty())
            {
                ROS_ERROR_STREAM("Could not find new instances of type [" << ingredient << "]");
                while (!ingredients_info.empty())
                    ingredients_info.pop();
                state_ = State::AVAILABLE_TO_REQUEST;
                return false;
            }

            // Get the pose of the new ingredient instance
            cocktail_bot::GetSceneObjectList srv;
            srv.request.object_name = new_ingredient_instances[0];
            if (!client_get_scene_object_.call(srv))
            {
                ROS_ERROR_STREAM("Failed to call service " << srv_get_scene_name_);
                while (!ingredients_info.empty())
                    ingredients_info.pop();
                state_ = State::AVAILABLE_TO_REQUEST;
                return false;
            }
            else if (!srv.response.obj_found)
            {
                ROS_ERROR_STREAM(srv.response.message);
                while (!ingredients_info.empty())
                    ingredients_info.pop();
                state_ = State::AVAILABLE_TO_REQUEST;
                return false;
            }

            // See if robot is close enough to the new ingredient instance
            geometry_msgs::Pose current_pose = req.current_pose;
            geometry_msgs::Pose obj_pose = srv.response.objects.pose[0];

            double distance = sqrt(pow(current_pose.position.x - obj_pose.position.x, 2) +
                                   pow(current_pose.position.y - obj_pose.position.y, 2) );
            if (distance > 0.5)
            {
                ROS_ERROR_STREAM("Robot is not close enough to the new instance of type [" << ingredient << "]");
                while (!ingredients_info.empty())
                    ingredients_info.pop();
                state_ = State::AVAILABLE_TO_REQUEST;
                return false;
            }
            else
            {
                ROS_INFO_STREAM("Picked up instance [" << new_ingredient_instances[0]
                                << "] of type [" << ingredient << "]");
                ingredients_info.pop();
            }

        }

        if (ingredients_info.empty())
        {
            ROS_WARN_STREAM("No more ingredients to find. Finished making cocktail!");
            state_ = State::AVAILABLE_TO_REQUEST;
            return true;
        }

        iter = ingredients_info.top();
        ingredient = iter.first;
        ingredient_instances = iter.second;

        ROS_WARN_STREAM("Looking for ingredient: " << ingredient);
        if (!ingredient_instances.instance_names.empty())
        {
            // TODO: Possibly send all instances and decide on the closest one 
            std::string instance = ingredient_instances.instance_names[0];

            ROS_INFO_STREAM("Moving to instance: " << instance);
            state_ = State::LOOKING_FOR_INSTANCE;

            cocktail_bot::MoveToObject srv;
            srv.request.object_name = instance;

            if (!client_move_to_object_.call(srv))
            {
                ROS_ERROR_STREAM("Failed to call service " << srv_move_to_object_name_);
                while (!ingredients_info.empty())
                    ingredients_info.pop();
                state_ = State::AVAILABLE_TO_REQUEST;
                return false;
            }
        }
        else
        {
            ROS_WARN_STREAM("No known instances for ingredient: " << ingredient 
                            << ". Looking for alternatives.");

            // TODO: Possibly send all instances and decide on the closest one
            std::string alternative = ingredient_instances.alternative_names[0];

            ROS_INFO_STREAM("Moving to container: " << alternative);
            state_ = State::LOOKING_FOR_ALTERNATIVE;

            cocktail_bot::MoveToObject srv;
            srv.request.object_name = alternative;

            if (!client_move_to_object_.call(srv))
            {
                ROS_ERROR_STREAM("Failed to call service " << srv_move_to_object_name_);
                while (!ingredients_info.empty())
                    ingredients_info.pop();
                state_ = State::AVAILABLE_TO_REQUEST;
                return false;
            }
        }

        return true;
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
        // Class name's first letter must be in uppercase
        ROS_INFO_STREAM("Got new object of class: " << req.class_name);

        std::stringstream ss;
        ss << "create_instance_from_class('" << req.class_name << "', " << ID_ << ", Instance)";

        std::string query = ss.str();
        ROS_INFO_STREAM("Query: " << query);

        PrologQuery bdgs = pl_.query(query);
        std::string instance_name;

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