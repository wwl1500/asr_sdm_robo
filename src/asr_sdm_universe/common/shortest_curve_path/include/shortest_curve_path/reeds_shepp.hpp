#ifndef REEDS_SHEPP_HPP_
#define REEDS_SHEPP_HPP_

#include <cassert>
#include <limits>

typedef int (*ReedsSheppPathSamplingCallback)(double q[3], void* user_data);
typedef int (*ReedsSheppPathTypeCallback)(int t, void* user_data);

class ReedsSheppStateSpace
{
public:
    double q0[3], q1[3];

    /** \brief The Reeds-Shepp path segment types */
    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };

    /** \brief Reeds-Shepp path types */
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

    /** \brief Complete description of a ReedsShepp path */
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
            double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
            double w=0., double x=0.);

        double length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType* type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
    };

    ReedsSheppPath pathInfo;

    ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

    double distance(ReedsSheppStateSpace &traj);

    void type(double q0[3], double q1[3], ReedsSheppPathTypeCallback cb, void* user_data);

    void sample(ReedsSheppStateSpace &traj, double dist, double qnew[3]);//, ReedsSheppPathSamplingCallback cb, void* user_data);

    /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(double q0[3], double q1[3], ReedsSheppStateSpace &traj);

protected:
    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]);

    /** \brief Turning radius */
    double rho_;
};

#endif  // REEDS_SHEPP_HPP_
