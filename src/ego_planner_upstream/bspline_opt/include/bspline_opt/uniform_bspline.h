#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  struct BsplineParameterizationDiagnostics
  {
    double matrix_condition_number{0.0};
    double control_point_max_jump{0.0};
    int first_bad_control_point{-1};
  };

  struct ReferenceParameterizationDiagnostics
  {
    int attempts{0};
    int sample_count{0};
    double sample_path_length{0.0};
    bool sample_progress_monotonic{false};
    bool sample_self_intersection{false};
    double maximum_deviation{0.0};
    int backtrack_segments{0};
    int self_intersections{0};
    double maximum_heading_jump{0.0};
    bool feasible{false};
    BsplineParameterizationDiagnostics linear_solve;
  };

  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts,
                                      BsplineParameterizationDiagnostics * diagnostics = nullptr);

    // Preserve a geometric reference while retaining exact P/V/A start
    // boundaries. When more duration is required, add reference samples rather
    // than increasing one fixed control polygon's knot interval.
    static bool parameterizeReferenceToBspline(
      double & ts, vector<Eigen::Vector3d> & point_set,
      const vector<Eigen::Vector3d> & start_end_derivative,
      const vector<Eigen::Vector3d> & reference_path,
      const double maximum_deviation, const double maximum_velocity,
      const double maximum_acceleration, const double feasibility_tolerance,
      Eigen::MatrixXd & ctrl_pts,
      ReferenceParameterizationDiagnostics & diagnostics);

    // Compare a trajectory with its geometric reference without changing
    // either object.  This is shared by parameterization tests and the final
    // pre-commit geometry gate.
    static void evaluateReferenceGeometry(
      UniformBspline trajectory,
      const vector<Eigen::Vector3d> & reference_path,
      ReferenceParameterizationDiagnostics & diagnostics);

    /* check feasibility, adjust time */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif
