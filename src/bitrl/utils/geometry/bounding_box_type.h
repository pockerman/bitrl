#ifndef BOUNDING_BOX_TYPE_H
#define BOUNDING_BOX_TYPE_H

namespace rlenvscpp{
namespace utils{
namespace geom{

///
/// \brief The BBType enum. Helper enumeration
/// for distinguishing boundin box types e.g. for
/// rigid bodies
///
enum class BBType{
    RECTANGLE=0,
    CIRCLE,
    SPHERE
};
}
}
}

#endif // BOUNDING_BOX_TYPE_H
