#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <tuple>
#include <list>


using namespace Eigen;
using namespace std;

#define inf numeric_limits<double>::infinity()

class FLCP
{
public:
    FLCP();
    ~FLCP();

    void setup(int _dim, MatrixXd _A, VectorXd _b,
                double _mu, VectorXi _f_idx,
                double _tol, bool _debug);

    void put_friction_end();

    void solve_vanilla();

    void delta_mu();



// private:
    int dim;
    list<int> C;
    list<int> Cf;
    list<int> N;
    list<int> Nh;
    list<int> Nl;

    MatrixXd A;
    VectorXd b;
    VectorXi f_idx;
    VectorXi p;

    VectorXd x;
    VectorXd w;
    VectorXd dx;
    VectorXd dw;

    double target_mu;
    double mu;
    double step;
    double tol;
    VectorXi mover;

    bool debug;
};