#include "support/configCosmos.h"
#include "agent/agentclass.h"
#include "support/convertdef.h" // for cartpos
#include "support/convertlib.h" // for ric2lvlh

Agent *agent;
string nodename;
string realmname = "sim";
vector<string> satname = {"mother", "child_01", "child_02", "child_03"};
//! Stores the positioning request as an LVLH offset
Convert::cartpos lvlhoffset;

//! For receiving agent request responses
beatstruc mbeat;

// Function forward declarations
/**
 * @brief Parses terminal arguments
 * @param args Terminal arguments
 * @return number of arguments parsed
 */
int32_t parse_control(string args);

/**
 * @brief Get the swarm controller agent object
 * @return true 
 * @return false 
 */
bool getSwarmControllerAgent();

/**
 * @brief Get any new positioning requests from the swarm controller agent
 */
void getNewPositioning();

/**
 * @brief Sends the current positioning to the Python controller
 */
void sendPositioningToPython();


// ==========================================================================
// Entrypoint
int main(int argc, char *argv[])
{
    int32_t iretn;

    if (argc > 1)
    {
        parse_control(argv[1]);
    }
    agent = new Agent(realmname, nodename, "simulate", 0.);
    agent->set_debug_level(2);
    if ((iretn = agent->wait(Agent::State::RUN, 2.)) < 0)
    {
        agent->debug_log.Printf("Error starting agent\n");
        exit(iretn);
    }

    agent->cinfo->agent0.aprd = .5;
    agent->start_active_loop();
    while (agent->running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Find the swarm controller agent
        if (!getSwarmControllerAgent())
        {
            continue;
        }
        // Get any new positioning requests
        getNewPositioning();

        agent->finish_active_loop();
    }
}

// ==========================================================================

int32_t parse_control(string args)
{
    uint16_t argcount = 0;
    string estring;
    json11::Json jargs = json11::Json::parse(args, estring);
    if (!jargs["nodename"].is_null())
    {
        ++argcount;
        nodename = jargs["nodename"].string_value();
    }
    if (!jargs["realmname"].is_null())
    {
        ++argcount;
        realmname = jargs["realmname"].string_value();
    }

    return argcount;
}

bool getSwarmControllerAgent()
{
    if (!mbeat.exists)
    {
        string response;
        mbeat = agent->find_agent("any", "propagate");
    }
    return mbeat.exists;
}

void getNewPositioning()
{
    string response;
    agent->send_request(mbeat, "get_offset_node " + nodename, response);
    if (response.find("\n") != string::npos)
    {
        vector<string> args = string_split(response.substr(response.find("\n")+1));
        if (args.size() > 2)
        {
            lvlhoffset.s.col[0] = atof(args[0].c_str());
            lvlhoffset.s.col[1] = atof(args[1].c_str());
            lvlhoffset.s.col[2] = atof(args[2].c_str());
            if (args.size() == 6)
            {
                lvlhoffset.v.col[0] = atof(args[3].c_str());
                lvlhoffset.v.col[1] = atof(args[4].c_str());
                lvlhoffset.v.col[2] = atof(args[5].c_str());
            }
            if (args[0] == "ric")
            {
                ric2lvlh(length_rv(agent->cinfo->node.loc.pos.eci.s), lvlhoffset, lvlhoffset);
            }
        }
    }
}
