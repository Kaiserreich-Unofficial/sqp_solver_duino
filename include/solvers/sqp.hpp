#pragma once

#include <Eigen/Dense>
#include <limits>

namespace sqp {

template <typename Scalar>
struct sqp_settings_t {
    Scalar tau = 0.5;       /**< line search iteration decrease, 0 < tau < 1 */
    Scalar eta = 0.25;      /**< line search parameter, 0 < eta < 1 */
    Scalar rho = 0.5;       /**< line search parameter, 0 < rho < 1 */
    Scalar eps_prim = 1e-3; /**< primal step termination threshold, eps_prim > 0 */
    Scalar eps_dual = 1e-3; /**< dual step termination threshold, eps_dual > 0 */
    int max_iter = 100;
    int line_search_max_iter = 100;
    void (*iteration_callback)(void* solver) = nullptr;

    bool validate() {
        bool valid;
        valid = 0.0 < tau && tau < 1.0 && 0.0 < eta && eta < 1.0 && 0.0 < rho && rho < 1.0 &&
                eps_prim < 0.0 && eps_dual < 0.0 && max_iter > 0 && line_search_max_iter > 0;
        return valid;
    }
};

struct sqp_status_t {
    enum { SOLVED, MAX_ITER_EXCEEDED, INVALID_SETTINGS } value;
};

struct sqp_info_t {
    int iter;
    int qp_solver_iter;
    sqp_status_t status;
};

template <typename Scalar_ = double>
struct NonLinearProblem {
    using Scalar = Scalar_;
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    int num_var;
    int num_constr;

    virtual void objective(const Vector& x, Scalar& obj) = 0;
    virtual void objective_linearized(const Vector& x, Vector& grad, Scalar& obj) = 0;
    virtual void constraint(const Vector& x, Vector& c, Vector& l, Vector& u) = 0;
    virtual void constraint_linearized(const Vector&x, Matrix&Jc, Vector&c, Vector&l, Vector&u) = 0;
};

/*
 * minimize     f(x)
 * subject to   l <= c(x) <= u
 */
template <typename Scalar_>
class SQP {
   public:
    using Scalar = Scalar_;
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using Problem = NonLinearProblem<Scalar>;

    using settings_t = sqp_settings_t<Scalar>;

    // Constants
    static constexpr Scalar DIV_BY_ZERO_REGUL = std::numeric_limits<Scalar>::epsilon();

    // enforce 16 byte alignment
    // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SQP() = default;
    ~SQP() = default;

    void solve(Problem& prob, const Vector& x0, const Vector& lambda0);
    void solve(Problem& prob);

    inline const Vector& primal_solution() const { return x_; }
    inline Vector& primal_solution() { return x_; }

    inline const Vector& dual_solution() const { return lambda_; }
    inline Vector& dual_solution() { return lambda_; }

    inline const settings_t& settings() const { return settings_; }
    inline settings_t& settings() { return settings_; }

    inline const sqp_info_t& info() const { return info_; }
    inline sqp_info_t& info() { return info_; }

   private:
    void _solve(Problem& prob);

    bool termination_criteria(const Vector& x, Problem& prob);
    void solve_qp(Problem& prob, Vector& p, Vector& lambda);

    /** Line search in direction p using l1 merit function. */
    Scalar line_search(Problem& prob, const Vector& p);

    /** L1 norm of constraint violation */
    Scalar constraint_norm(const Vector& x, Problem& prob);

    /** L_inf norm of constraint violation */
    Scalar max_constraint_violation(const Vector& x, Problem& prob);

    // Solver state variables
    Vector x_;
    Vector lambda_;
    Vector step_prev_;
    Vector grad_L_;
    Vector delta_grad_L_;

    Matrix Hess_;
    Vector grad_obj_;
    Scalar obj_;
    Matrix Jac_constr_;
    Vector constr_;
    Vector l_, u_;

    // info
    Scalar dual_step_norm_;
    Scalar primal_step_norm_;

    settings_t settings_;
    sqp_info_t info_;
};

extern template class SQP<double>;

}  // namespace sqp
