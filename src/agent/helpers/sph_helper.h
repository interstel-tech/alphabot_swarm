#include "machcl/sttr.h"
#include "support/convertlib.h"
#include <string>
#include <vector>

//! Used to hold desired goal location info for swarm members
struct Goal_Location
{
    // Name of the node this is the goal location of
    std::string goal_of;
    // Formation offset from reference point (e.g., mothership) in RIC
    Cosmos::Convert::cartpos ric_offset;
    // Internal flag for determine if the offset value has changed.
    bool has_updated = false;
};

class SphModule
{
public:
    void initialize(const std::vector<std::string>& swarm_nodenames);

    //! Parameters to control the behaviour of SPH inside MAC
    sim_param sph_params;

    // Represents all the nodes in the physical swarm separately from the external truth model (i.e., the propagator)
    std::vector<sim_state_local> formation_state;
    // The mothership keeps track of goal locations for every swarm member that has a goal_location defined for it.
    std::vector<Goal_Location> goal_locations;
};
