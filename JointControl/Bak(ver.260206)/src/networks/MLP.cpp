#include "networks/MLP.h"
#include "networks/Linear.h"
#include "networks/LSTM.h"
#include <iostream>

MLP::MLP(int input_dim, int hidden_dim, int output_dim) {
    layer1 = new Linear(input_dim, hidden_dim);
    layer2 = new Linear(hidden_dim, hidden_dim);
    layer3 = new Linear(hidden_dim, output_dim);
}

void MLP::load_weights(const std::string &directory) {
    layer1->load_weights(directory + "net_model_0");
    layer2->load_weights(directory + "net_model_2");
    layer3->load_weights(directory + "net_model_4");
}

Eigen::MatrixXd MLP::forward(const Eigen::MatrixXd &input) {
    Eigen::MatrixXd x = relu(layer1->forward(input));
    x = relu(layer2->forward(x));
    return layer3->forward(x);
}

MLPs::MLPs(int input_dim, int output_dim) {
    // layer1 = new Linear(input_dim, 256);
    // layer2 = new Linear(256, 1024);
    // layer3 = new Linear(1024, 1024);
    // layer4 = new Linear(1024, 256);
    // layer5 = new Linear(256, output_dim);
    layer1 = new Linear(input_dim, 32);
    layer2 = new Linear(32, 16);
    layer3 = new Linear(16, output_dim);
}

void MLPs::load_weights(const std::string &directory) {
    layer1->load_weights(directory + "fc1");
    layer2->load_weights(directory + "fc2");
    layer3->load_weights(directory + "fc3");
    // layer4->load_weights(directory + "fc_");
    // layer5->load_weights(directory + "fc4");
}

Eigen::MatrixXd MLPs::forward(const Eigen::MatrixXd &input) {
    Eigen::MatrixXd x = relu(layer1->forward(input));
    x = relu(layer2->forward(x));
    // x = relu(layer3->forward(x));
    // x = relu(layer4->forward(x));
    
    // return layer5->forward(x);
    return layer3->forward(x);
}

Eigen::MatrixXd MLPs::dydx(const Eigen::MatrixXd &input){

    Eigen::MatrixXd dzdx0 = layer1->W;
    Eigen::MatrixXd dzdx1 = layer2->W;
    Eigen::MatrixXd dzdx2 = layer3->W;

    Eigen::MatrixXd x1 = relu(layer1->forward(input));


	Eigen::MatrixXd dx1dx0 = drelu(layer1->forward(input))*dzdx0;
	Eigen::MatrixXd dx2dx1 = drelu(layer2->forward(x1))*dzdx1;
    Eigen::MatrixXd dx3dx2 = dzdx2;

	return dx3dx2*dx2dx1*dx1dx0;

}