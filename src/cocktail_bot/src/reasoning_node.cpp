#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <cocktail_bot/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private:
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;
    std::ofstream output_file; //The file may have to stay open.
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);

        this->m_query_flag_save=false;
    };

    ~Reasoner(){
        output_file.close();
    };

    void setOutQueriesFile(string QueryfileName)
    {
        output_file.open(QueryfileName, std::ios::app);

        if (output_file.is_open()) {
            ROS_INFO_STREAM("File is open " << QueryfileName);
            this->m_query_flag_save=true; //This means that I want to save the queries in a file
        } else {
            ROS_WARN_STREAM("Failed to open file " << QueryfileName);
            this->m_query_flag_save=false;
        }
    }

private:    

    void save_query(string query)
    {
        if (!m_query_flag_save)
            return;

        if (output_file.is_open()) {
            output_file << query << endl;
        } else {
            ROS_WARN_STREAM("File not open");
        }
    }

     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(cocktail_bot::UpdateObjectList::Request &req,
                             cocktail_bot::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;
        
        object=req.object_name;

        getClass(object);

        res.confirmation = assertKnowledge(object);
        return res.confirmation;
    }


    void getClass(std::string className)
    {
        
        std::stringstream ss;
        ss << "get_class('" << className << "')";

        std:string query= ss.str();

        ROS_INFO_STREAM("query: "<<query);
        save_query(query);

        PrologQuery bdgs = pl_.query(query);

        bool res = false;
        for (auto &it : bdgs) 
        {
            res = true;
            ROS_INFO_STREAM("A new class was created in the ontology");
            break;
        }

       
    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;

        std::stringstream ss;
        ss << "create_instance_from_class('" << className << "', " << ID_ << ", Instance)";
        
        std:string query= ss.str();

        ROS_INFO_STREAM("query: "<<query);
        save_query(query);

        PrologQuery bdgs = pl_.query(query);

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            PrologBindings val = *it;
            std::stringstream instanceVal;
            instanceVal << val["Instance"];
            
            instanceName = instanceVal.str();
            ROS_WARN_STREAM("new instance in knowledge base: " << instanceName);
        }

        bdgs.finish();
        ID_++;
        
        return true;
    }

}; //class Reasoner

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "reasoning_node");

    ros::NodeHandle nh;

    Reasoner myReasoner(nh);

    std::string saveFilePath = argv[1]; // Information about the path for the file that will save the queries
    
    std::string saveQueryFile; // Variable about where to save the queries
    bool saveQueries_flag;     // Variable about whether or not to save the queries

    if (!nh.getParam("/read_prolog_queries/saved_query_file", saveQueryFile)) {
        ROS_WARN_STREAM("No query file path specified!");
    }

    if (!nh.getParam("/read_prolog_queries/save_flag", saveQueries_flag)) {
        ROS_WARN_STREAM("No save flag specified!");
    }  

    if(saveQueries_flag && !saveQueryFile.empty())
    {   //If the flag is true, then I will configure the file to save the asserted queries
        saveQueryFile = saveFilePath + saveQueryFile;
        ROS_INFO_STREAM("Query file: "<< saveQueryFile);

        //Now we call a new function which will create and open the new file
        myReasoner.setOutQueriesFile(saveQueryFile);    
    }

    ros::spin();

    return 0;
}