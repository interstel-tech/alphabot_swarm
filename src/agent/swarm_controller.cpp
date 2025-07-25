/**
 * @brief Swarm-controller Agent
 * An agent program that can send commands to the swarm controller agents
 * and listens on the broadcast channel for the positions of the swarm children.
 */

#include "swarm_params.h"
#include "support/configCosmos.h"
#include "agent/agentclass.h"
#include "support/convertdef.h" // for cartpos
#include "support/convertlib.h" // for ric2lvlh

// ==========================================================================
// Static variables
namespace
{
    struct NodeState
    {
        string nodename;
        Convert::cartpos lvlh;
        json11::Json to_json() const {
            return json11::Json::object {
                { "nodename" , nodename },
                { "lvlh", lvlh },
            };
        }
    };
    Agent *agent;
    vector<string> swarm_nodenames = {"child_01", "child_02", "child_03", "child_04"};
    //! Stores the positioning request as in LVLH
    vector<NodeState> swarm_positions;
    //! Stores the desired positions for the swarm children
    vector<NodeState> desired_swarm_positions;
    //! Swarm state as represented by the controller
    SwarmState desired_swarm_state;
}

// ==========================================================================
// Function forward declarations
/**
 * @brief Initialize swarm structures
 */
void initSwarmObjects();
/**
 * @brief Takes the desired formation shape and angle and converts it into positions.
 * Call this after changing the swarm state.
 */
void convertFormationShapeToPositions();
/**
 * @brief Broadcast new swarm positions
 */
void setSwarmPositions();

// ==========================================================================
// Agent requests
/**
 * @brief Set the desired swarm formation
 */
int32_t request_set_swarm_formation(string &request, string &response, Agent *agent);

// ==========================================================================
// Entrypoint
int main()
{
    initSwarmObjects();
    agent = new Agent(SWARM_REALM, SWARM_CONTROLLER_NODENAME, SWARM_CONTROLLER_AGENTNAME, 0.);
    agent->set_debug_level(0);
    int32_t iretn;
    if ((iretn = agent->wait(Agent::State::RUN, 2.)) < 0)
    {
        agent->debug_log.Printf("Error starting agent\n");
        exit(iretn);
    }
    agent->add_request("set_swarm_formation", request_set_swarm_formation, "'{\"shape\":\"line\"|\"ngon\",\"sep\":<separation>", "Set the desired swarm formation");

    agent->cinfo->agent0.aprd = .5;
    agent->start_active_loop();
    while (agent->running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        setSwarmPositions();
    }
}

// ==========================================================================
// Function definitions
void initSwarmObjects()
{
    swarm_positions.resize(swarm_nodenames.size());
    desired_swarm_positions.resize(swarm_nodenames.size());
    for (size_t i = 0; i < swarm_nodenames.size(); ++i)
    {
        swarm_positions[i].nodename = swarm_nodenames[i];
        desired_swarm_positions[i].nodename = swarm_nodenames[i];
    }
}

void convertFormationShapeToPositions()
{
    // Convert the desired formation shape to positions
    switch(desired_swarm_state.formation_type)
    {
    case SwarmFormationType::LINE:
    {
        // For a line formation, positions are set in a straight line
        // where node 0 is at the reference location and each subsequent
        // node trails behind separated by the separation distance.
        for (size_t i = 0; i < swarm_positions.size(); ++i)
        {
            double x = desired_swarm_state.ref_location.s.col[0] - i * desired_swarm_state.separation * cos(desired_swarm_state.angle * M_PI / 180.0);
            double y = desired_swarm_state.ref_location.s.col[1] - i * desired_swarm_state.separation * sin(desired_swarm_state.angle * M_PI / 180.0);
            desired_swarm_positions[i].lvlh.s.col[0] = x;
            desired_swarm_positions[i].lvlh.s.col[1] = y;
            desired_swarm_positions[i].lvlh.s.col[2] = 0;
        }
        break;
    }
    case SwarmFormationType::NGON:
        // For an N-gon formation, positions are set in a circular pattern
        // where each node is separated by the separation distance from the
        // reference location, which is in the center of the circular pattern.
        for (size_t i = 0; i < swarm_positions.size(); ++i)
        {
            double angle = desired_swarm_state.angle + (i * 360 / swarm_positions.size()); // in degrees
            double x = desired_swarm_state.ref_location.s.col[0] + desired_swarm_state.separation * cos(angle * M_PI / 180.0);
            double y = desired_swarm_state.ref_location.s.col[1] + desired_swarm_state.separation * sin(angle * M_PI / 180.0);
            desired_swarm_positions[i].lvlh.s.col[0] = x;
            desired_swarm_positions[i].lvlh.s.col[1] = y;
            desired_swarm_positions[i].lvlh.s.col[2] = 0;
        }
        break;
    default:
        cout << "Unknown formation type!" << endl;
        return;
    }
}

void setSwarmPositions()
{
    cout << "Setting new positions for the swarm..." << endl;
    convertFormationShapeToPositions();
    auto new_swarm_state = json11::Json(desired_swarm_positions);
    agent->post(Agent::AgentMessage::REQUEST, "desired_position_swarm " + new_swarm_state.dump());
}

// ==========================================================================
// Agent requests
int32_t request_set_swarm_formation(string &request, string &response, Agent*)
{
    vector<string> args = string_split(request);
    response.clear();

    if (args.size() < 2)
    {
        response = "Error: Not enough arguments";
        return 0;
    }
    string estring;
    json11::Json jargs = json11::Json::parse(args[1], estring);
    if (!estring.empty())
    {
        response = "Error: Invalid JSON format in arguments";
        return 0;
    }
    if (!jargs.is_object())
    {
        response = "Error: Arguments should be a JSON object";
        return 0;
    }
    if (!jargs["shape"].is_string() || !jargs["sep"].is_number())
    {
        response = "Error: Invalid object in arguments";
        return 0;
    }
    string shape = jargs["shape"].string_value();
    double sep = jargs["sep"].number_value();
    if (swarm_formation_type_map.find(shape) == swarm_formation_type_map.end())
    {
        response = "Error: Invalid formation shape: " + shape;
        return 0;
    }
    if (sep <= 0)
    {
        response = "Error: Separation must be a positive number! Got: " + std::to_string(sep);
        return 0;
    }
    desired_swarm_state.formation_type = swarm_formation_type_map[shape];
    desired_swarm_state.separation = sep;
    setSwarmPositions();

    return 0;
}
