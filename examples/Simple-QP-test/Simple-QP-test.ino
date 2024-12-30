#include <sqp.hpp>
#include <utils.hpp>
#include <ArduinoEigenDense.h>

using namespace sqp;

struct SimpleQP : public NonLinearProblem<float>
{
    using Vector = NonLinearProblem<float>::Vector;
    using Matrix = NonLinearProblem<float>::Matrix;

    Eigen::Matrix2f P;
    Eigen::Vector2f q = {1, 1};
    Eigen::Vector2f SOLUTION = {0.3, 0.7};

    SimpleQP()
    {
        P << 4, 1, 1, 2;
        num_var = 2;
        num_constr = 3;
    }

    void objective(const Vector &x, Scalar &obj) final { obj = 0.5 * x.dot(P * x) + q.dot(x); }

    void objective_linearized(const Vector &x, Vector &grad, Scalar &obj) final
    {
        objective(x, obj);
        grad = P * x + q;
    }

    void constraint(const Vector &x, Vector &c, Vector &l, Vector &u) final
    {
        // x1 + x2 = 1
        c << x.sum(), x;
        l << 1, 0, 0;
        u << 1, 0.7, 0.7;
    }

    void constraint_linearized(const Vector &x, Matrix &Jc, Vector &c, Vector &l, Vector &u) final
    {
        constraint(x, c, l, u);
        Jc << 1, 1, Matrix::Identity(2, 2);
    }
};

SimpleQP problem;
SQP<float> solver;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println(F("[INFO] Simple QP problem test"));
    Eigen::VectorXf x0 = Eigen::Vector2f(0, 0);
    Eigen::VectorXf y0 = Eigen::Vector3f(0, 0, 0);

    solver.settings().second_order_correction = true;
    solver.solve(problem, x0, y0);

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
