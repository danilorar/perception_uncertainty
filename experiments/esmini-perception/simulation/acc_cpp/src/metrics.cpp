// Author: Zhuo Ma
// Ground-truth geometry only. Flat upright boxes, appropriate for scene 1549178.
#include "accsim/metrics.hpp"
#include <algorithm>
#include <array>
#include <cmath>
namespace accsim {
namespace {
struct Point { double x, y; };
using Corners = std::array<Point,4>;
Point center(const ObjectState& o) {
    const double c=std::cos(o.pose.h), s=std::sin(o.pose.h);
    return {o.pose.x+c*o.box.cx-s*o.box.cy, o.pose.y+s*o.box.cx+c*o.box.cy};
}
Corners corners(const ObjectState& o) {
    const auto middle=center(o);
    const double c=std::cos(o.pose.h), s=std::sin(o.pose.h);
    Corners points;
    const double xs[4]={-.5,.5,.5,-.5}, ys[4]={-.5,-.5,.5,.5};
    for (int i=0;i<4;++i) {
        const double x=xs[i]*o.box.length, y=ys[i]*o.box.width;
        points[i]={middle.x+c*x-s*y,middle.y+s*x+c*y};
    }
    return points;
}
bool intersects(const Corners& a, const Corners& b) {
    for (const auto* poly : {&a,&b}) for (int i=0;i<2;++i) {
        const Point edge{(*poly)[i+1].x-(*poly)[i].x,(*poly)[i+1].y-(*poly)[i].y};
        const Point axis{-edge.y,edge.x};
        double amin=infinity,amax=-infinity,bmin=infinity,bmax=-infinity;
        for (auto p:a) { const auto d=p.x*axis.x+p.y*axis.y; amin=std::min(amin,d); amax=std::max(amax,d); }
        for (auto p:b) { const auto d=p.x*axis.x+p.y*axis.y; bmin=std::min(bmin,d); bmax=std::max(bmax,d); }
        if (amax<bmin || bmax<amin) return false;
    }
    return true;
}
double point_segment(Point p,Point a,Point b) {
    const double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
    const double w=den>0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/den,0.0,1.0):0;
    return std::hypot(p.x-a.x-w*dx,p.y-a.y-w*dy);
}
}
Geometry geometry(const ObjectState& ego,const ObjectState& target) {
    Geometry g;
    const auto ec=center(ego),tc=center(target);
    const double dx=tc.x-ec.x,dy=tc.y-ec.y,c=std::cos(ego.pose.h),s=std::sin(ego.pose.h);
    const double dh=target.pose.h-ego.pose.h,ac=std::abs(std::cos(dh)),as=std::abs(std::sin(dh));
    g.longitudinal_m=dx*c+dy*s; g.lateral_m=-dx*s+dy*c;
    g.gap_m=g.longitudinal_m-ego.box.length/2-(target.box.length*ac+target.box.width*as)/2;
    g.closing_speed_mps=ego.speed-target.speed*std::cos(dh);
    g.in_path=g.longitudinal_m>0 && std::abs(g.lateral_m)<=(ego.box.width+target.box.width*ac+target.box.length*as)/2;
    const auto e=corners(ego),t=corners(target);
    const bool xy_overlap=intersects(e,t);
    double xy_distance=0;
    if (!xy_overlap) {
        xy_distance=infinity;
        for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
            xy_distance=std::min(xy_distance,point_segment(e[i],t[j],t[(j+1)%4]));
            xy_distance=std::min(xy_distance,point_segment(t[i],e[j],e[(j+1)%4]));
        }
    }
    const double z_gap=std::max(0.0,std::abs(ego.pose.z+ego.box.cz-target.pose.z-target.box.cz)-(ego.box.height+target.box.height)/2);
    g.collision=xy_overlap && z_gap==0;
    g.distance_m=std::hypot(xy_distance,z_gap);
    return g;
}
StepMetrics evaluate_truth(const WorldTruthFrame& truth) {
    StepMetrics metrics;
    for(const auto& target:truth.targets) {
        const auto g=geometry(truth.ego,target);
        metrics.collision=metrics.collision||g.collision;
        metrics.minimum_distance_m=std::min(metrics.minimum_distance_m,g.distance_m);
        if(g.in_path && g.gap_m<metrics.gap_m) { metrics.gap_m=g.gap_m; metrics.target_id=target.id; }
        if(g.in_path && g.closing_speed_mps>1e-6) metrics.ttc_s=std::min(metrics.ttc_s,std::max(0.0,g.gap_m)/g.closing_speed_mps);
    }
    return metrics;
}
} // namespace accsim
