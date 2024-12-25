#ifndef UTILS_HPP
#define UTILS_HPP

#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Eigenvalues>

void printMat(const Eigen::MatrixXf &m)
{
    for (int i = 0; i < m.rows(); i++) {
        Serial.print("[");
        for (int j = 0; j < m.cols() - 1; j++) {
            Serial.print(m(i, j));
            Serial.print(F(", "));
        }
        Serial.print(m(i, m.cols() - 1));
        Serial.println("]");
    }
}

template <typename qp_t>
void print_qp(qp_t qp)
{
    Serial.print(F("P = "));
    printMat(qp.P.transpose());
    Serial.print(F("q = "));
    printMat(qp.q.transpose());
    Serial.print(F("A = "));
    printMat(qp.A.transpose());
    Serial.print(F("l = "));
    printMat(qp.l.transpose());
    Serial.print(F("u = "));
    printMat(qp.u.transpose());
}

template <typename Mat>
bool is_psd(Mat &h)
{
    Eigen::EigenSolver<Mat> eigensolver(h);
    for (int i = 0; i < eigensolver.eigenvalues().RowsAtCompileTime; i++) {
        float v = eigensolver.eigenvalues()(i).real();
        if (v <= 0) {
            return false;
        }
    }
    return true;
}

#endif /* UTILS_HPP */
