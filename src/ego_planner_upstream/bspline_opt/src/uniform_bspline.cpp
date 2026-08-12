#include "bspline_opt/uniform_bspline.h"

#include <cmath>
#include <limits>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      // Bridge validates the full command vector, so feasibility must use the
      // same norm instead of accepting three individually bounded components.
      if (vel.norm() > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        max_vel = max(max_vel, vel.norm());
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (acc.norm() > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        max_acc = max(max_acc, acc.norm());
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}

  // 将一组点转换为控制点
  void UniformBspline::parameterizeToBspline(
    const double &ts, const vector<Eigen::Vector3d> &point_set,
    const vector<Eigen::Vector3d> &start_end_derivative,
    Eigen::MatrixXd &ctrl_pts,
    BsplineParameterizationDiagnostics * diagnostics)
  {
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
      return;
    }

    int K = point_set.size();

    // For the cubic uniform knot convention used by evaluateDeBoorT(), the
    // first three control points determine the exact start boundary:
    //   P(0)   = (Q0 + 4 Q1 + Q2) / 6
    //   P'(0)  = (Q2 - Q0) / (2 ts)
    //   P''(0) = (Q0 - 2 Q1 + Q2) / ts^2
    // Solve these three equations analytically.  The former implementation put
    // all samples and all four endpoint derivatives into one overdetermined
    // least-squares system, so P/V/A at t=0 were only approximate.
    const Eigen::Vector3d & p0 = point_set.front();
    const Eigen::Vector3d & v0 = start_end_derivative[0];
    const Eigen::Vector3d & a0 = start_end_derivative[2];
    const Eigen::Vector3d dt_v = ts * v0;
    const Eigen::Vector3d dt2_a = ts * ts * a0;

    ctrl_pts.resize(3, K + 2);
    ctrl_pts.col(0) = p0 - dt_v + dt2_a / 3.0;
    ctrl_pts.col(1) = p0 - dt2_a / 6.0;
    ctrl_pts.col(2) = p0 + dt_v + dt2_a / 3.0;

    // Reference seeds always contain at least seven samples.  For that normal
    // case, make the end P/V/A exact as well.  A soft end derivative fit can
    // overshoot the last reference point and manufacture negative progress.
    // Retain the legacy fit for unusually short generic point sets.
    const bool hard_end_boundary = K >= 7;
    if (hard_end_boundary)
    {
      const Eigen::Vector3d & p1 = point_set.back();
      const Eigen::Vector3d & v1 = start_end_derivative[1];
      const Eigen::Vector3d & a1 = start_end_derivative[3];
      ctrl_pts.col(K - 1) = p1 - ts * v1 + ts * ts * a1 / 3.0;
      ctrl_pts.col(K) = p1 - ts * ts * a1 / 6.0;
      ctrl_pts.col(K + 1) = p1 + ts * v1 + ts * ts * a1 / 3.0;
    }

    // Fit only the free middle controls to the geometric samples, treating
    // both hard boundary blocks as constants when available.
    const int unknown_count = hard_end_boundary ? K - 4 : K - 1;
    const int equation_count = hard_end_boundary ? K - 2 : (K - 1) + 2;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(equation_count, unknown_count);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(equation_count, 3);

    auto add_coefficient = [&](const int row, const int control_index, const double coefficient)
      {
        if (control_index < 3 ||
          (hard_end_boundary && control_index >= K - 1))
        {
          b.row(row) -= coefficient * ctrl_pts.col(control_index).transpose();
        }
        else
        {
          A(row, control_index - 3) += coefficient;
        }
      };

    int row = 0;
    const int last_fitted_sample = hard_end_boundary ? K - 2 : K - 1;
    for (int sample = 1; sample <= last_fitted_sample; ++sample, ++row)
    {
      b.row(row) += point_set[sample].transpose();
      add_coefficient(row, sample, 1.0 / 6.0);
      add_coefficient(row, sample + 1, 4.0 / 6.0);
      add_coefficient(row, sample + 2, 1.0 / 6.0);
    }

    if (!hard_end_boundary)
    {
      b.row(row) += start_end_derivative[1].transpose();
      add_coefficient(row, K - 1, -1.0 / (2.0 * ts));
      add_coefficient(row, K + 1, 1.0 / (2.0 * ts));
      ++row;

      b.row(row) += start_end_derivative[3].transpose();
      add_coefficient(row, K - 1, 1.0 / (ts * ts));
      add_coefficient(row, K, -2.0 / (ts * ts));
      add_coefficient(row, K + 1, 1.0 / (ts * ts));
    }

    const Eigen::MatrixXd free_controls = A.colPivHouseholderQr().solve(b);
    ctrl_pts.block(0, 3, 3, unknown_count) = free_controls.transpose();

    if (diagnostics != nullptr)
    {
      const Eigen::VectorXd singular_values =
        A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues();
      diagnostics->matrix_condition_number =
        singular_values.size() > 0 &&
        singular_values(singular_values.size() - 1) > 1.0e-12
          ? singular_values(0) / singular_values(singular_values.size() - 1)
          : std::numeric_limits<double>::infinity();
      diagnostics->control_point_max_jump = 0.0;
      diagnostics->first_bad_control_point = -1;
      for (int index = 1; index < ctrl_pts.cols(); ++index)
      {
        const double jump = (ctrl_pts.col(index) - ctrl_pts.col(index - 1)).norm();
        if (jump > diagnostics->control_point_max_jump)
        {
          diagnostics->control_point_max_jump = jump;
          diagnostics->first_bad_control_point = index;
        }
      }
    }

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  namespace
  {
    struct PolylineProjection
    {
      double distance{std::numeric_limits<double>::infinity()};
      double arc{0.0};
    };

    PolylineProjection projectToPolyline(
      const std::vector<Eigen::Vector3d> & polyline,
      const std::vector<double> & arc,
      const Eigen::Vector3d & point)
    {
      PolylineProjection best;
      for (std::size_t index = 0; index + 1 < polyline.size(); ++index)
      {
        const Eigen::Vector3d segment = polyline[index + 1] - polyline[index];
        const double squared_length = segment.squaredNorm();
        const double ratio = squared_length > 1.0e-12
          ? std::clamp((point - polyline[index]).dot(segment) / squared_length, 0.0, 1.0)
          : 0.0;
        const Eigen::Vector3d projection = polyline[index] + ratio * segment;
        const double distance = (point - projection).norm();
        if (distance < best.distance)
        {
          best.distance = distance;
          best.arc = arc[index] + ratio * std::sqrt(squared_length);
        }
      }
      return best;
    }

    double cross2d(
      const Eigen::Vector3d & a, const Eigen::Vector3d & b,
      const Eigen::Vector3d & c)
    {
      return (b.x() - a.x()) * (c.y() - a.y()) -
        (b.y() - a.y()) * (c.x() - a.x());
    }

    bool segmentsIntersect2d(
      const Eigen::Vector3d & a, const Eigen::Vector3d & b,
      const Eigen::Vector3d & c, const Eigen::Vector3d & d)
    {
      constexpr double epsilon = 1.0e-9;
      const double ab_c = cross2d(a, b, c);
      const double ab_d = cross2d(a, b, d);
      const double cd_a = cross2d(c, d, a);
      const double cd_b = cross2d(c, d, b);
      return ((ab_c > epsilon && ab_d < -epsilon) ||
              (ab_c < -epsilon && ab_d > epsilon)) &&
             ((cd_a > epsilon && cd_b < -epsilon) ||
              (cd_a < -epsilon && cd_b > epsilon));
    }

    std::vector<Eigen::Vector3d> densifyPolyline(
      const std::vector<Eigen::Vector3d> & points)
    {
      std::vector<Eigen::Vector3d> dense;
      dense.reserve(points.size() * 2 - 1);
      for (std::size_t index = 0; index + 1 < points.size(); ++index)
      {
        dense.push_back(points[index]);
        dense.push_back(0.5 * (points[index] + points[index + 1]));
      }
      dense.push_back(points.back());
      return dense;
    }
  }  // namespace

  void UniformBspline::evaluateReferenceGeometry(
    UniformBspline trajectory,
    const vector<Eigen::Vector3d> & reference_path,
    ReferenceParameterizationDiagnostics & diagnostics)
  {
    diagnostics.maximum_deviation = 0.0;
    diagnostics.backtrack_segments = 0;
    diagnostics.self_intersections = 0;
    diagnostics.maximum_heading_jump = 0.0;
    if (reference_path.size() < 2)
      return;

    std::vector<double> reference_arc(reference_path.size(), 0.0);
    for (std::size_t index = 1; index < reference_path.size(); ++index)
      reference_arc[index] =
        reference_arc[index - 1] + (reference_path[index] - reference_path[index - 1]).norm();

    const double duration = trajectory.getTimeSum();
    const double base_sample_step =
      std::max(1.0e-4, std::min(0.01, trajectory.getInterval() / 10.0));
    // Geometry diagnostics are a guard, not an unbounded O(N^2) workload.
    // Two thousand temporal samples retain millimetre-level resolution for
    // the short local trajectories while bounding intersection checks.
    const double sample_step =
      std::max(base_sample_step, duration / 2000.0);
    std::vector<Eigen::Vector3d> samples;
    std::vector<double> progress;
    for (double time = 0.0; time < duration; time += sample_step)
    {
      samples.push_back(trajectory.evaluateDeBoorT(time));
      const PolylineProjection projection =
        projectToPolyline(reference_path, reference_arc, samples.back());
      diagnostics.maximum_deviation =
        std::max(diagnostics.maximum_deviation, projection.distance);
      progress.push_back(projection.arc);
    }
    samples.push_back(trajectory.evaluateDeBoorT(duration));
    const PolylineProjection end_projection =
      projectToPolyline(reference_path, reference_arc, samples.back());
    diagnostics.maximum_deviation =
      std::max(diagnostics.maximum_deviation, end_projection.distance);
    progress.push_back(end_projection.arc);

    Eigen::Vector3d previous_heading = Eigen::Vector3d::Zero();
    bool have_previous_heading = false;
    for (std::size_t index = 1; index < samples.size(); ++index)
    {
      // Ignore small nearest-segment switching at curved polyline vertices.
      // The guard targets a visible fold/backtrack (the historical defect was
      // decimetres), not centimetre-scale projection ambiguity.
      if (progress[index] + 0.02 < progress[index - 1])
        ++diagnostics.backtrack_segments;
      const Eigen::Vector3d delta = samples[index] - samples[index - 1];
      if (delta.head<2>().norm() > 1.0e-6)
      {
        const Eigen::Vector3d heading(delta.x(), delta.y(), 0.0);
        if (have_previous_heading)
        {
          const double cosine = std::clamp(
            previous_heading.dot(heading) /
              (previous_heading.norm() * heading.norm()), -1.0, 1.0);
          diagnostics.maximum_heading_jump =
            std::max(diagnostics.maximum_heading_jump, std::acos(cosine));
        }
        previous_heading = heading;
        have_previous_heading = true;
      }
    }

    for (std::size_t first = 0; first + 1 < samples.size(); ++first)
    {
      for (std::size_t second = first + 2; second + 1 < samples.size(); ++second)
      {
        if (segmentsIntersect2d(
            samples[first], samples[first + 1],
            samples[second], samples[second + 1]))
          ++diagnostics.self_intersections;
      }
    }
  }

  bool UniformBspline::parameterizeReferenceToBspline(
    double & ts, vector<Eigen::Vector3d> & point_set,
    const vector<Eigen::Vector3d> & start_end_derivative,
    const vector<Eigen::Vector3d> & reference_path,
    const double maximum_deviation, const double maximum_velocity,
    const double maximum_acceleration, const double feasibility_tolerance,
    Eigen::MatrixXd & ctrl_pts,
    ReferenceParameterizationDiagnostics & diagnostics)
  {
    if (point_set.size() < 4 || reference_path.size() < 2 || ts <= 0.0)
      return false;

    diagnostics = ReferenceParameterizationDiagnostics();
    diagnostics.sample_count = static_cast<int>(point_set.size());
    diagnostics.sample_progress_monotonic = true;
    diagnostics.sample_self_intersection = false;
    for (std::size_t index = 1; index < point_set.size(); ++index)
      diagnostics.sample_path_length += (point_set[index] - point_set[index - 1]).norm();
    std::vector<double> reference_arc(reference_path.size(), 0.0);
    for (std::size_t index = 1; index < reference_path.size(); ++index)
      reference_arc[index] =
        reference_arc[index - 1] +
        (reference_path[index] - reference_path[index - 1]).norm();
    double previous_progress =
      projectToPolyline(reference_path, reference_arc, point_set.front()).arc;
    for (std::size_t index = 1; index < point_set.size(); ++index)
    {
      const double progress =
        projectToPolyline(reference_path, reference_arc, point_set[index]).arc;
      if (progress + 0.02 < previous_progress)
        diagnostics.sample_progress_monotonic = false;
      previous_progress = progress;
    }
    for (std::size_t first = 0; first + 1 < point_set.size(); ++first)
    {
      for (std::size_t second = first + 2; second + 1 < point_set.size(); ++second)
      {
        if (segmentsIntersect2d(
            point_set[first], point_set[first + 1],
            point_set[second], point_set[second + 1]))
          diagnostics.sample_self_intersection = true;
      }
    }
    if (!diagnostics.sample_progress_monotonic ||
      diagnostics.sample_self_intersection)
      return false;

    constexpr int maximum_attempts = 12;
    constexpr std::size_t maximum_sample_count = 1025;
    for (int attempt = 0; attempt < maximum_attempts; ++attempt)
    {
      diagnostics.attempts = attempt + 1;
      parameterizeToBspline(
        ts, point_set, start_end_derivative, ctrl_pts, &diagnostics.linear_solve);
      if (ctrl_pts.cols() < 4 || !ctrl_pts.allFinite())
        return false;

      UniformBspline trajectory(ctrl_pts, 3, ts);
      trajectory.setPhysicalLimits(
        maximum_velocity, maximum_acceleration, feasibility_tolerance);
      double ratio = 1.0;
      diagnostics.feasible = trajectory.checkFeasibility(ratio, false);
      evaluateReferenceGeometry(trajectory, reference_path, diagnostics);
      const bool geometry_safe =
        diagnostics.maximum_deviation <= maximum_deviation &&
        diagnostics.backtrack_segments == 0 &&
        diagnostics.self_intersections == 0;
      if (diagnostics.feasible && geometry_safe)
        return true;

      if (!geometry_safe)
      {
        if (point_set.size() * 2 - 1 > maximum_sample_count)
          break;
        point_set = densifyPolyline(point_set);
        ts *= 0.5;
      }
      else if (!diagnostics.feasible)
      {
        // Search the interval gradually. A raw feasibility ratio can jump
        // over the narrow interval where both dynamics and reference geometry
        // are valid, especially with exact moving P/V/A boundaries.
        const double requested_growth = std::max(1.01, ratio * 1.01);
        ts *= std::min(1.25, requested_growth);
      }
    }
    return false;
  }

  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
