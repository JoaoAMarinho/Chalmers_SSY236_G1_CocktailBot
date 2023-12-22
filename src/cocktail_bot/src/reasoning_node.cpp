#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <cocktail_bot/FindIngredients.h>
#include <cocktail_bot/MakeCocktail.h>
#include <cocktail_bot/UpdateKnowledge.h>

enum class State {
    AVAILABLE_TO_REQUEST,
    MAKING_COCKTAIL
};

struct IngredientInstances {
    std::vector<std::string> instance_names;
    std::vector<std::string> alternative_names;
};

class Reasoner
{
private:
    PrologClient pl_; // Prolog client to connect to the Prolog server

    std::string srv_find_ingredients_name_;      // Name of the service provided by the control node
    ros::ServiceClient client_find_ingredients_; // Client to request the robot to find ingredients

    std::string srv_make_cocktail_name_;         // Name of the service to receive cocktail requests
    ros::ServiceServer make_cocktail_srv_;       // Service to receive cocktail requests

    std::string srv_update_knowledge_name_;      // Name of the service to update the knowledge base
    ros::ServiceServer update_knowledge_srv_;    // Service to update the knowledge base

    State state_ = State::AVAILABLE_TO_REQUEST;  // Current state of the robot
    std::map<std::string, IngredientInstances> ingredients_info; // Map to store the ingredients for the requested cocktail
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
        srv_find_ingredients_name_ = "find_ingredients";
        client_find_ingredients_ = nh.serviceClient<cocktail_bot::FindIngredients>(srv_find_ingredients_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_find_ingredients_name_.c_str());
        bool service_found = ros::service::waitForService(srv_find_ingredients_name_, ros::Duration(30.0));

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_find_ingredients_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: " << srv_find_ingredients_name_);

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
        ROS_INFO_STREAM("Requested cocktail: " << req.cocktail_name);

        // Check if the robot is available to receive requests
        if (state_ != State::AVAILABLE_TO_REQUEST) {
            ROS_WARN_STREAM("Robot is not available to receive requests!");
            res.confirmation = false;
            return true;
        }

        state_ = State::MAKING_COCKTAIL;

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

            for (int i = 0; i < query_result["Ingredients"].size(); i++)
            {
                std::string ingredient = query_result["Ingredients"][i];
                std::vector<std::string> ingredient_names  = stringToVector(query_result["Ingred_inst"])[i];
                std::vector<std::string> alternative_names = stringToVector(query_result["Alt_inst"])[i];

                // Add the ingredient to the map
                IngredientInstances ingredient_instances;
                ingredient_instances.instance_names    = ingredient_names;
                ingredient_instances.alternative_names = alternative_names;
                ingredients_info[ingredient] = ingredient_instances;
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
        //TODO: call service to start making cocktail
        // cocktail_bot::FindIngredients srv;
        // srv.request.ingredients = query_result["Ingredients"];
        // srv.request.ingredient_instances  = query_result["Ingred_inst"];
        // srv.request.alternative_instances = query_result["Alt_inst"];

        // if (!client_find_ingredients_.call(srv))
        // {
        //     res.confirmation = false;
        //     ROS_ERROR_STREAM("Failed to call service " << srv_find_ingredients_name_);
        //     return false;
        // }

        // if (!srv.response.confirmation)
        // {
        //     res.confirmation = false;
        //     ROS_ERROR_STREAM("Unsuccessfull call to: " << srv_find_ingredients_name_);
        // }

        res.confirmation = true;
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