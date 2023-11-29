#include <ros/ros.h>
#include <fstream>
#include <string>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <cocktail_bot/LoadKnowledge.h>

class Knowledge
{
private: 
    PrologClient pl_;
    std::ifstream input_file;

    std::string srv_load_knowledge_name_;
    ros::ServiceServer load_knowledge_srv_;

public:
    Knowledge(ros::NodeHandle& nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        srv_load_knowledge_name_ = "load_knowledge";
        load_knowledge_srv_ = nh.advertiseService(srv_load_knowledge_name_, &Knowledge::srv_load_knowledge_callback, this);
        
    };

    ~Knowledge()
    {
        input_file.close();
    };

    void setQueryFile(std::string fileName_Q)
    {
        input_file.open(fileName_Q);
        
        if (input_file.is_open()) {
            ROS_INFO_STREAM("File is open " << fileName_Q);
        } else {
            ROS_WARN_STREAM("Failed to open file " << fileName_Q);
        }
    }

private:

    /**
    * @brief Callback function for the service to load previous knowledge
    *
    * @param Request requested object to be added to the list
    * @param Respose response from the service when the object has been added (true/false)
    */
    bool srv_load_knowledge_callback(cocktail_bot::LoadKnowledge::Request &req,
                                     cocktail_bot::LoadKnowledge::Response &res)
    {
        ROS_INFO_STREAM("Got new load knowledge request");
        
        loadQueries();
        res.confirmation = true;
        return res.confirmation;
    }

    void loadQueries()
    {
        std::string query;

        if (input_file.is_open())
        {
            while( getline(input_file,query) )
            {
                ROS_INFO_STREAM(query);
                PrologQuery bdgs = pl_.query(query);
            }
        }
        else ROS_WARN_STREAM("File not found!");
    }
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "knowledge_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create an instance of the Knowledge class
    Knowledge knowledgeNode(nh);

    std::string saveFilePath = argv[1]; // Information about the path for the file that will load the queries

    std::string saveQueryFile; // Variable about where to load the queries
    bool saveQueries_flag;     // Variable about whether or not to load the queries

    if (!nh.getParam("/read_prolog_queries/saved_query_file", saveQueryFile)) {
        ROS_WARN_STREAM("No query file path specified!");
    }

    if (!nh.getParam("/read_prolog_queries/save_flag", saveQueries_flag)) {
        ROS_WARN_STREAM("No save flag specified!");
    }  

    if(saveQueries_flag && !saveFilePath.empty())
    {   //If the flag is true, then I will configure the file to load the asserted queries
        saveQueryFile = saveFilePath + saveQueryFile;
        ROS_INFO_STREAM("Query file: "<< saveQueryFile);

        //Now we call a new function which will create and open the new file
        knowledgeNode.setQueryFile(saveQueryFile);    
    }

    // Run the node
    ros::spin();

    return 0;
}
