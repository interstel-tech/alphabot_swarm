#include "sph_helper.h"

void SphModule::initialize(const std::vector<std::string>& swarm_nodenames)
{
    // Set SPH parameters
    sph_params.h = 500.;
    sph_params.h_attractor = 10000.;
    sph_params.Re = 1000.;
    sph_params.M = 1.;
    sph_params.gamma = 1.;
    sph_params.inter_agent_w = 1000000.;
    sph_params.attractor_w = 0.01;

    formation_state.resize(swarm_nodenames.size());
    for (size_t i=0; i<formation_state.size(); ++i)
    {
        formation_state[i].node_name = swarm_nodenames[i];   
    }
}
