#include <sqp.hpp>
#include <utils.hpp>
#include <ArduinoEigenDense.h>

using namespace sqp;

struct SimpleNLP : public NonLinearProblem<float>
{
    using Vector = NonLinearProblem<float>::Vector;
    using Matrix = NonLinearProblem<float>::Matrix;

    const Scalar infinity = std::numeric_limits<Scalar>::infinity();
    Eigen::Vector2f SOLUTION = {1, 1};

    SimpleNLP()
    {
        num_var = 2;
        num_constr = 3;
    }

    void objective(const Vector &x, Scalar &obj) { obj = -x.sum(); }

    void objective_linearized(const Vector &x, Vector &grad, Scalar &obj)
    {
        grad.resize(num_var);

        objective(x, obj);
        grad << -1, -1;
    }

    void constraint(const Vector &x, Vector &c, Vector &l, Vector &u)
    {
        // 1 <= x0^2 + x1^2 <= 2
        // 0 <= x0
        // 0 <= x1
        c << x.squaredNorm(), x;
        l << 1, 0, 0;
        u << 2, infinity, infinity;
    }

    void constraint_linearized(const Vector &x, Matrix &Jc, Vector &c, Vector &l, Vector &u)
    {
        Jc.resize(3, 2);

        constraint(x, c, l, u);
        Jc << 2 * x.transpose(), Matrix::Identity(2, 2);
    }
};

SimpleNLP problem;
SQP<float> solver;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println(F("[INFO] SQP infeasible problem test"));
    // infeasible initial point
    Eigen::Vector2f x0 = {2, -1};
    Eigen::Vector3f y0 = {1, 1, 1};

    solver.settings().max_iter = 100;
    solver.settings().second_order_correction = true;

    solver.solve(problem, x0, y0);
    Serial.print(F("[INFO] Primal Solution: "));
    printMat(solver.primal_solution().transpose());
    Serial.print(F("[INFO] Dual Solution: "));
    printMat(solver.dual_solution().transpose());
    Serial.print(F("[INFO] Solution Bias: "));
    printMat((solver.primal_solution() - problem.SOLUTION).transpose());
    Serial.print(F("[INFO] Iters: "));
    Serial.print(solver.info().iter);
    Serial.print(F("  Max iter: "));
    Serial.println(solver.settings().max_iter);
}

void loop()
{
}
