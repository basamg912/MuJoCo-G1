#ifndef LINEAR_H
#define LINEAR_H

#include <eigen3/Eigen/Dense>
#include <string>

class Linear {
public:
    Linear(int input_dim, int output_dim);
    void load_weights(const std::string &prefix);
    Eigen::MatrixXd forward(const Eigen::MatrixXd &input);

    Eigen::MatrixXd W;
    Eigen::VectorXd b;

private:
    int input_dim;
    int output_dim;

};

#endif // LINEAR_H
