#ifndef TOOLS_H_
#define TOOLS_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:

  class RMSE {
  public:
    RMSE();

    VectorXd Update(const VectorXd& estimation, const VectorXd& ground_truth);

  private:
    int count_ = 0;
    VectorXd square_error_ = VectorXd(4);
  };

};

#endif /* TOOLS_H_ */
