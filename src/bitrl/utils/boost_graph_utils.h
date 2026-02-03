#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"

namespace bitrl
{
namespace utils {

template<typename GraphType, typename Predicate>
uint_t
find_vertex(const GraphType& graph, const Predicate& pred){

    for(uint_t v=0; v<graph.n_vertices(); ++v){
        const auto& vertex = graph.get_vertex(v);
        if(pred(vertex)){
            return vertex.id;
        }
    }

    return bitrl::consts::INVALID_ID;
}

}
}
#endif // BOOST_GRAPH_UTILS_H
