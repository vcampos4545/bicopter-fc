#include "coordinate_convert.h"

namespace bicopter::visualization {

RenderVec3 nedToRender(const bicopter::Vec3& v)
{
    return RenderVec3{v.x, -v.z, v.y};
}

RenderQuat nedToRenderQuat(const bicopter::Quaternion& q)
{
    return RenderQuat{q.w, q.x, -q.z, q.y};
}

} // namespace bicopter::visualization
