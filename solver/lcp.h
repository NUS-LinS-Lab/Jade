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
typedef tuple<double, int, int, int> TP;

struct Solution
{
  VectorXd x;
  list<int> C;
  list<int> Cf;
  list<int> N;
  list<int> Nh;
  list<int> Nl;
};

class LCP
{
public:
  LCP();
  ~LCP();

  void setup(int _dim, MatrixXd _A, VectorXd _b,
             VectorXd _hi, VectorXd _lo, VectorXi _f_idx,
             double _eps, double _tol, bool _debug);
  
  Solution save();

  void load(Solution s);

  void put_friction_at_end();

  void solve_delta(VectorXd db);

  void compare(double _s, int i);

  bool max_step(int i, bool f);
  // void max_step(int i, bool f, bool reverse);

  void move_step(int i,  bool f);

  bool drive_to_boundary(int i, bool f);

  VectorXd solve();

  VectorXd violet_solve();

  bool valid(double _tol);

//  VectorXd alterRatio(MatrixXd origin_dx, MatrixXd origin_dw, double std);
//
//  MatrixXd alterGradient(MatrixXd dB);
//
//  MatrixXd modifyGradient(MatrixXd dB, double std);


private:
  int dim;
  list<int> C;
  list<int> Cf;
  list<int> N;
  list<int> Nh;
  list<int> Nl;
  list<int> U;

  MatrixXd A;
  VectorXd b;
  VectorXd hi;
  VectorXd lo;
  VectorXi f_idx;

  VectorXd x;
  VectorXd w;
  VectorXd dx;
  VectorXd dw;

  VectorXi p;
  VectorXi pf_idx;

  double step;
  double tol;
  double eps;
  VectorXi mover;

  bool debug;
};
