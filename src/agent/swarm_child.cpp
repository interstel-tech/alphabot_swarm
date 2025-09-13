/**
 * @brief Swarm-child Agent
 * An agent program that can accept commands from the swarm controller agent
 * and periodically broadcasts its position (obtained from the python script).
 */

#include "swarm_params.h"
#include "support/configCosmos.h"
#include "agent/agentclass.h"
#include "support/convertdef.h" // for cartpos
#include "support/convertlib.h" // for ric2lvlh
#include "helpers/common.h"

// ==========================================================================
// Static variables
namespace
{
    Agent *agent;
    string nodename = "";
    vector<string> satname = {"child_01", "child_02", "child_03", "child_04"};
    //! Stores the positioning request as an LVLH offset
    Convert::cartpos lvlhoffset;
    //! For communicating with the python script
    socket_channel sock_in;
    socket_channel sock_out;
    const string ip = "127.0.0.1";
    const uint16_t port_in = 50002;
    const uint16_t port_out = 50001;
}

// ==========================================================================
// Function forward declarations
/**
 * @brief Parses terminal arguments
 * @param args Terminal arguments
 * @return number of arguments parsed
 */
int32_t parseControl(string args);

/**
 * @brief Initialize the UDP socket to communicate with the python script
 */
void initializeUdpChannel();

/**
 * @brief Handle messages from the python script
 */
void handleMessage(const std::string& msgstr);

// ==========================================================================
// Agent requests
/**
 * @brief Contains the desired position of the swarm.
 * Searched by swarm child agents to set their desired LVLH positions.
 */
int32_t request_desired_position_swarm(string &request, string &response, Agent *agent);

// ==========================================================================
// Entrypoint
int main(int argc, char *argv[])
{
    int32_t iretn;

    if (argc > 1)
    {
        parseControl(argv[1]);
    }
    else
    {
        cerr << "Usage: " << argv[0] << " '{\"nodename\":\"<nodename>\"}'" << endl;
        exit(1);
    }
    agent = new Agent(SWARM_REALM, nodename, SWARM_NODE_AGENTNAME, 0.);
    agent->set_debug_level(0);
    if ((iretn = agent->wait(Agent::State::RUN, 2.)) < 0)
    {
        agent->debug_log.Printf("Error starting agent\n");
        exit(iretn);
    }
    agent->add_request("desired_position_swarm", request_desired_position_swarm, "'[{\"nodename\":\"name\",\"lvlh\":{lvlh_position}},...]'", "Send the desired LVLH positions of the swarm");

    initializeUdpChannel();

    agent->cinfo->agent0.aprd = .5;
    agent->start_active_loop();
    std::vector<uint8_t> message(10000);
    while (agent->running())
    {
        if (socket_recvfrom(sock_in, message, 10000) <= 0)
        {
            // No incoming packet, wait for a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        string msgstr(message.begin(), message.end());
        handleMessage(msgstr);

    }
}

// ==========================================================================
// Function definitions
int32_t parseControl(string args)
{
    uint16_t argcount = 0;
    string estring;
    json11::Json jargs = json11::Json::parse(args, estring);
    if (!jargs["nodename"].is_null())
    {
        ++argcount;
        nodename = jargs["nodename"].string_value();
    }
    if (nodename.empty())
    {
        cerr << "Error: No nodename provided in arguments." << endl;
        exit(0);
    }

    return argcount;
}

void initializeUdpChannel()
{
    int32_t iretn = socket_open(sock_in, NetworkType::UDP,  "", port_in, SOCKET_LISTEN, SOCKET_BLOCKING, 50000);
    if (iretn < 0)
    {
        cout << "Error in socket_open: (" << std::to_string(iretn) << ") " << cosmos_error_string(iretn) << endl;
        exit(0);
    }
    iretn = socket_open(sock_out, NetworkType::UDP, ip.c_str(), port_out, SOCKET_TALK, SOCKET_BLOCKING);
    if (iretn < 0)
    {
        cout << "Error in socket_open: (" << std::to_string(iretn) << ") " << cosmos_error_string(iretn) << endl;
        exit(0);
    }
}

void handleMessage(const std::string& msgstr)
{
    string estring;
    json11::Json jargs = json11::Json::parse(msgstr, estring);
    if (!estring.empty())
    {
        cerr << "Error: Invalid JSON format in arguments: " << estring << endl;
        return;
    }
    if (!jargs.is_object())
    {
        cerr << "Error: Arguments should be a JSON object" << endl;
        return;
    }
    NodeState node_state;
    node_state.nodename = nodename;
    node_state.lvlh.from_json(msgstr);
    std::string state_json = node_state.to_json().dump();
    agent->post(Agent::AgentMessage::REQUEST, "node_state " + node_state.to_json().dump());

}

// ==========================================================================
// Agent requests
int32_t request_desired_position_swarm(string &request, string &response, Agent*)
{
    response.clear();
    std::size_t find_arg = request.find_first_of(" ");
    if (find_arg == std::string::npos)
    {
        response = "Error: No arguments found in request";
        cerr << response << endl;
        return 0;
    }
    string arg = request.substr(find_arg + 1);

    NodeState node_state;
    node_state.from_json(arg);
    string estring;
    json11::Json jargs = json11::Json::parse(arg, estring);
    if (!estring.empty())
    {
        response = "Error: Invalid JSON format in arguments: " + estring;
        cerr << response << endl;
        return 0;
    }
    if (!jargs.is_array())
    {
        response = "Error: Arguments should be a JSON array";
        cerr << response << endl;
        return 0;
    }
    string lvlh_json;
    for (const auto& el : jargs.array_items())
    {
        if (!el.is_object() || !el["nodename"].is_string() || !el["lvlh"].is_object())
        {
            response = "Error: Invalid object in arguments";
            return 0;
        }
        // Find the desired position for this node
        if (nodename != el["nodename"].string_value())
        {
            continue;
        }
        lvlh_json = el["lvlh"].dump();
        cout << "Setting position for " << nodename << ": " << lvlh_json << endl;
        break;
    }

    int32_t iretn = socket_sendto(sock_out, node_state.lvlh.to_json().dump());
    if (iretn < 0)
    {
        response = "Error in socket_sendto: (" + std::to_string(iretn) + ") " + cosmos_error_string(iretn);
        cerr << response << endl;
        return iretn;
    }

    return 0;
}
