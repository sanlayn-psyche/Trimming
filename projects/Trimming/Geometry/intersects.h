#include <algorithm>
namespace its
{


#ifdef LF_PARABOX
#include "LFParallelBox.h"
template<typename T>
static bool if_interSects(const ParallelBox<T> &pb1 ,const ParallelBox<T> &pb2)
{
    Range<T> r1( 0, pb1.m_size[0] );
    Range<T> r2( pb1.m_offset[0], pb1.m_offset[1]);

    auto p1 = pb2.m_endPoint[0] - pb1.m_endPoint[0];
    auto p2 = pb2.m_endPoint[1] - pb1.m_endPoint[0];
   
    double d1 = p1 * pb1.m_orth;
    double d2 = p2 * pb1.m_orth;
    double offset1 = pb2.m_offset[0] * (pb2.m_orth * pb1.m_orth);
    double offset2 = pb2.m_offset[1] * (pb2.m_orth * pb1.m_orth);

    if (r2.if_intersect(std::min({ d1 + offset1, d1 + offset2, d2 + offset1, d2 + offset2 }), std::max({ d1 + offset1, d1 + offset2, d2 + offset1, d2 + offset2 })))
    {
        d1 = p1 * pb1.m_dir;
        d2 = p2 * pb1.m_dir;
        offset1 = pb2.m_offset[0] * (pb2.m_orth * pb1.m_dir);
        offset2 = pb2.m_offset[1] * (pb2.m_orth * pb1.m_dir);
        return (r1.if_intersect(std::min({ d1 + offset1, d1 + offset2, d2 + offset1, d2 + offset2 }), std::max({ d1 + offset1, d1 + offset2, d2 + offset1, d2 + offset2 })));
    }
    return false;
}
#endif



#ifdef LF_POINT
#include "LFPoint.h"
static std::tuple<bool, Point> if_interSects_lineWithLine(Point p1, Point orth1, Point p2, Point orth2)
{
    double det = (orth1.get_cord(0) * orth2.get_cord(1) - orth1.get_cord(1) * orth2.get_cord(0));
    
    if (det != 0.0f)
    {
        double idet = 1.0 / det;
        double c1 = p1 * orth1;
        double c2 = p2 * orth2;
        Point res = idet * Point{
            orth2.get_cord(1) * c1 - orth1.get_cord(1) * c2 ,
            -orth2.get_cord(0) * c1 + orth1.get_cord(0) * c2
        };
        return std::make_tuple(true, res);
    }
    else
    {
        return std::make_tuple(false, Point{});
    }
}

static std::tuple<bool, Point> if_interSects_segWithLine(Point s1, Point s2, Point p1, Point orth1)
{
    double d1 = (s1 - p1) * orth1;
    double d2 = (s2 - p1) * orth1;
    if (d1 * d2 < 0.0f)
    {
        auto orth = s2 - s1;
        orth.act_orthrize();
        return if_interSects_lineWithLine(s1, orth, p1, orth1);
    }
    else
    {
        return std::make_tuple(false, Point{});
    }
}
static std::tuple<bool, Point> if_interSects_segWithSeg()
{
    return std::tuple<bool, Point>();
}


#endif

};


