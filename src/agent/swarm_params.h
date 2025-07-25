#include "support/convertdef.h" // for cartpos
#include <map>
#include <string>

std::string SWARM_REALM = "swarm";
std::string SWARM_CONTROLLER_NODENAME = "main";
std::string SWARM_CONTROLLER_AGENTNAME = "controller";
std::string SWARM_NODE_AGENTNAME = "agent";

enum class SwarmFormationType
{
    LINE,
    NGON,
};

struct SwarmState
{
    //! Current formation type
    SwarmFormationType formation_type = SwarmFormationType::LINE;
    //! Separation of the swarm nodes (in meters)
    double separation = 0.5;
    //! Reference location of the swarm formation
    Convert::cartpos ref_location;
    //! Angle of the swarm formation (in degrees),
    //! where an angle of 0 faces toward the +X direction.
    double angle = 0.;
};

std::map<std::string, SwarmFormationType> swarm_formation_type_map = {
    {"line", SwarmFormationType::LINE},
    {"ngon", SwarmFormationType::NGON}
};

