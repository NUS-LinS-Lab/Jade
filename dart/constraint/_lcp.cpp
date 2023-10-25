#include "_lcp.h"

LCP::LCP() {}
LCP::~LCP() {}

Eigen::IOFormat FMT(3, 0, ", ", ";\n", "[", "]", "[", "]");

void LCP::setup(int _dim, MatrixXd _A, VectorXd _b,
                VectorXd _hi, VectorXd _lo, VectorXi _f_idx,
                double _eps, double _tol)
{
  dim = _dim;
  C.clear();
  Cf.clear();
  N.clear();
  Nh.clear();
  Nl.clear();
  U.clear();

  for (int i = 0; i < _dim; ++i)
    U.push_back(i);

  A = _A;
  b = _b;
  hi = _hi;
  lo = _lo;
  f_idx = _f_idx;

  x = VectorXd::Zero(_dim);
  w = -_b;
  dx = VectorXd::Zero(_dim);
  dw = VectorXd::Zero(_dim);

  p = VectorXi::LinSpaced(_dim, 0, _dim - 1);

  step = 0;
  eps = _eps;
  tol = _tol;
  mover = VectorXi::Zero(_dim);

}

void LCP::put_friction_at_end()
{
  int position = dim - 1;

  int temp;
  for (int i = dim - 1; i >= 0; --i) {
    if (f_idx[i] >= 0) {
      temp = p(i);
      for (int j = i; j < position; ++j)
        p(j) = p(j+1);
      p(position) = temp;
      position--;
    }
  }

  PermutationMatrix<Dynamic, Dynamic> P(p);
  A = P.inverse() * A * P;
  b = P.inverse() * b;
  hi = P.inverse() * hi;
  lo = P.inverse() * lo;
  f_idx = P.inverse() * f_idx;
  x = P.inverse() * x;
  w = P.inverse() * w;

  pf_idx = f_idx;
  for (int j = 0; j < dim; ++j) {
    for (int k = 0; k < dim; ++k) {
      if (pf_idx(j) == p(k)) {
        pf_idx(j) = k;
        break;
      }
    }
  }
}

void LCP::solve_delta(VectorXd db)
{
  dx = VectorXd::Zero(dim);
  dw = VectorXd::Zero(dim);

  int idx = C.size() + Cf.size() + N.size() + Nh.size() + Nl.size();

  if (C.size() + Cf.size() > 0)
  {
    MatrixXd _A = A;

    vector<int> clamp;
    for (int c : C)
      clamp.push_back(c);
    for (int cf : Cf)
      clamp.push_back(cf);

    for (int h : Nh)
      _A.col(pf_idx(h)) += hi(h) * A.col(h);
    for (int l : Nl)
      _A.col(pf_idx(l)) += lo(l) * A.col(l);

    MatrixXd A_c = _A(clamp, clamp);
    VectorXd db_c = db(clamp);
    VectorXd dx_c = A_c.completeOrthogonalDecomposition().solve(db_c);

    dx(clamp) = dx_c;
  }

  for (int h : Nh)
    dx(h) = hi(h) * dx(pf_idx(h));

  for (int l : Nl)
    dx(l) = lo(l) * dx(pf_idx(l));

  dx(idx) = (w(idx) < 0) ? 1 : -1;

  dw = A * dx;

  for (int c : C)
    dw(c) = 0;

  for (int cf : Cf)
    dw(cf) = 0;

//  cout<<"dx: "<<dx<<endl;
//  cout<<"dw: "<<dw<<endl;
}

void LCP::compare(double _s, int i)
{
  if (_s < step - tol && _s >= -tol)
  {
    mover = VectorXi::Zero(dim);
    step = min(_s, step);
    mover(i) = 1;
    return;
  }
  if (_s < step + tol && _s >= -tol)
  {
    mover(i) = 1;
    step = max(_s, step);
    return;
  }
}


void LCP::max_step(int i, bool f)
{
  mover = VectorXi::Zero(dim);
  mover(i) = 1;
  step = fabs(dw(i)) > eps ? -w(i) / dw(i) : 0;
  double _s;

  if (f) {
    double x_h = x(i) - hi(i) * x(pf_idx(i));
    double dx_h = dx(i) - hi(i) * dx(pf_idx(i));
    if (dx_h > eps) {
      _s = -x_h / dx_h;
      compare(_s, i);
    }
    double x_l = x(i) - lo(i) * x(pf_idx(i));
    double dx_l = dx(i) - lo(i) * dx(pf_idx(i));
    if (dx_l < -eps) {
      _s = -x_l / dx_l;
      compare(_s, i);
    }
  }

  for (int c : C) {
    if (dx(c) < -eps) {
      _s = -x(c) / dx(c);
      compare(_s, c);
    }
  }

  for (int n : N) {
    if (dw(n) < -eps) {
      _s = -w(n) / dw(n);
      compare(_s, n);
    }
  }

  for (int cf : Cf) {
    double x_h = x(cf) - hi(cf) * x(pf_idx(cf));
    double dx_h = dx(cf) - hi(cf) * dx(pf_idx(cf));
    if (dx_h > eps) {
      _s = -x_h / dx_h;
      compare(_s, cf);
    }
    double x_l = x(cf) - lo(cf) * x(pf_idx(cf));
    double dx_l = dx(cf) - lo(cf) * dx(pf_idx(cf));
    if (dx_l < -eps) {
      _s = -x_l / dx_l;
      compare(_s, cf);
    }
  }

  for (int nh : Nh) {
    if (dw(nh) > eps) {
      _s = -w(nh) / dw(nh);
      compare(_s, nh);
    }
  }

  for (int nl : Nl) {
    if (dw(nl) < -eps) {
      _s = -w(nl) / dw(nl);
      compare(_s, nl);
    }
  }
}

void LCP::move_step(int i,  bool f)
{
  // DEBUG
  // cout << "Begin to move_step" << endl;
  // cout << "x: " << x.transpose().format(FMT) << endl;
  // cout << "dx: " << dx.transpose().format(FMT) << endl;
  // cout << "w: " << w.transpose().format(FMT) << endl;
  // cout << "dw: " << dw.transpose().format(FMT) << endl;

  x += step * dx;
  w += step * dw;

  list<int> _C = C;
  list<int> _Cf = Cf;
  list<int> _N = N;
  list<int> _Nh = Nh;
  list<int> _Nl = Nl;

  if (mover(i) > 0)
  {
    if (!f) {
      U.remove(i);
      _C.push_back(i);
      // DEBUG
      // cout<<i<<" from U to C"<<endl;
    }
    else if (fabs(w(i)) <= tol) {
      U.remove(i);
      _Cf.push_back(i);
      // DEBUG
      // cout<<i<<" from U to Cf"<<endl;
    }
    else if (w(i) < -tol) {
      U.remove(i);
      _Nh.push_back(i);
      // DEBUG
      // cout<<i<<" from U to Nh"<<endl;
    }
    else if (w(i) > tol) {
      U.remove(i);
      _Nl.push_back(i);
      // DEBUG
      // cout<<i<<" from U to Nl"<<endl;
    }
  }

  for (int c : C) {
    if (mover(c) > 0) {
      _C.remove(c);
      _N.push_back(c);
      // DEBUG
      // cout<<c<<" from C to N"<<endl;
    }
  }

  for (int n : N) {
    if (mover(n) > 0) {
      _N.remove(n);
      _C.push_back(n);
      // DEBUG
      // cout<<n<<" from N to C"<<endl;
    }
  }

  for (int cf : Cf) {
    if (mover(cf) > 0) {
      _Cf.remove(cf);
      if(w(cf) < 0)
      {
        _Nh.push_back(cf);
        // DEBUG
        // cout<<cf<<" from Cf to Nh"<<endl;
      }
      else
      {
        _Nl.push_back(cf);
        // DEBUG
        // cout<<cf<<" from Cf to Nl"<<endl;
      }
    }
  }

  for (int nh : Nh) {
    if (mover(nh) > 0) {
      if (fabs(x(nh)) <= tol)
      {
        _Nh.remove(nh);
        _Nl.push_back(nh);
        // DEBUG
        // cout<<nh<<" from Nh to Nl"<<endl;
      }
      else
      {
        _Nh.remove(nh);
        _Cf.push_back(nh);
        // DEBUG
        // cout<<nh<<" from Nh to Cf"<<endl;
      }
    }
  }

  for (int nl : Nl) {
    if (mover(nl) > 0) {
      if (fabs(x(nl)) <= tol)
      {
        _Nl.remove(nl);
        _Nh.push_back(nl);
        // DEBUG
        // cout<<nl<<" from Nl to Nh"<<endl;
      }
      else
      {
        _Nl.remove(nl);
        _Cf.push_back(nl);
        // DEBUG
        // cout<<nl<<" from Nl to Cf"<<endl;
      };
    }
  }

  C = _C;
  Cf = _Cf;
  N = _N;
  Nh = _Nh;
  Nl = _Nl;

  /// Zero out
  for (int c : C)
    w(c) = 0;

  for (int n : N)
    x(n) = 0;

  for (int cf : Cf)
    w(cf) = 0;

  for (int nh : Nh)
    x(nh) = hi(nh) * x(pf_idx(nh));

  for (int nl : Nl)
    x(nl) = lo(nl) * x(pf_idx(nl));

  // DEBUG
  // cout<< "end of move_step()" <<endl;
  // cout << "step: " << step << endl;
  // cout << "x: " << x.transpose().format(FMT) << endl;
}

void LCP::drive_to_boundary(int i, bool f)
{
  int loop = 0;
  while (loop < 3 * dim) {
    double dx_i = (w(i) < 0) ? 1 : -1;

    solve_delta(- dx_i * A.col(i));

    max_step(i, f);

    move_step(i, f);

//    cout<<"s: "<<step<<endl;
//    cout<<"move: "<<mover.transpose()<<endl;
//    cout<<"x: "<<x.transpose()<<endl;

    if (mover(i) >0)
      return;

    loop++;
  }

  cout<<"\nOver loop!!!"<<endl;
}

//void print_vector(VectorXd z, int num_c)
//{
//  for (int k = 0; k < num_c; ++k)
//  {
//    cout<<z(3*k)<<" "<<z(3*k+1)<<" "<<z(3*k+2)<<endl;
//  }
//}

bool if_contain(list<int> l, int x)
{
  return find(l.begin(), l.end(), x) != l.end();
}

VectorXd LCP::solve()
{
  put_friction_at_end();

  for (int i = 0; i < dim; ++i) {
    // DEBUG
    // cout<<"\nStep "<<i<<endl;
    if (pf_idx(i) < 0) {
      if (w(i) <= - tol)
        drive_to_boundary(i, false);
      else {
        U.remove(i);
        N.push_back(i);
        // DEBUG
        // cout<<i<<" from U to N"<<endl;
      }
    }
    else {
      if (if_contain(N, pf_idx(i))) {
        if (w(i) < 0) {
          U.remove(i);
          Nh.push_back(i);
          // DEBUG
          // cout<<i<<" from U to Nh"<<endl;
        }
        else {
          U.remove(i);
          Nl.push_back(i);
          // DEBUG
          // cout<<i<<" from U to Nl"<<endl;
        }
      }
      else if (fabs(w(i)) <= tol) {
        U.remove(i);
        Cf.push_back(i);
        // DEBUG
        // cout<<i<<" from U to Cf"<<endl;
      }
      else
        drive_to_boundary(i, true);
    }
  }


  PermutationMatrix<Dynamic, Dynamic> P(p);
  b = P * b;
  x = P * x;
  w = P * w;

  // DEBUG
  // cout<< "end of LCP::solve()" <<endl;
  // cout<<"x: "<<x.transpose()<<endl;
  // cout<<"w: "<<w.transpose()<<endl;
  return x;
}

//VectorXd LCP::alterRatio(MatrixXd origin_dx, MatrixXd origin_dw, double std)
//{
//  VectorXd ratio = VectorXd::Zero(dim);
//  double norm;
//  double dist;
//
//  for (int c : C) {
//    norm = max(origin_dx.row(c).squaredNorm(), 1e-8);
//    dist = x(c) / norm;
//    //    cout<<"dist "<<dist<<endl;
//    //    cout<<"std "<<std<<endl;
//    if (fabs(dist) < std)
//      ratio(c) = 1 - fabs(dist) / std;
//  }
//
//  for (int cf : Cf) {
//    norm = max(origin_dx.row(cf).squaredNorm(), 1e-8);
//    /// wrong
//    dist = x(cf) / norm;
//    if (fabs(dist) < std)
//      ratio(cf) = 1 - fabs(dist) / std;
//  }
//
//  for (int n : N) {
//    norm = max(origin_dw.row(n).squaredNorm(), 1e-8);
//    dist = w(n) / norm;
//    if (fabs(dist) < std)
//      ratio(n) = 1 - fabs(dist) / std;
//  }
//
//  for (int nh : Nh) {
//    norm = max(origin_dw.row(nh).squaredNorm(), 1e-8);
//    dist = w(nh) / norm;
//    if (fabs(dist) < std)
//      ratio(nh) = 1 - fabs(dist) / std;
//  }
//
//  for (int nl : Nl) {
//    norm = max(origin_dw.row(nl).squaredNorm(), 1e-8);
//    dist = w(nl) / norm;
//    if (fabs(dist) < std)
//      ratio(nl) = 1 - fabs(dist) / std;
//  }
//
//  return ratio;
//}
//
//MatrixXd LCP::alterGradient(MatrixXd dB)
//{
//  MatrixXd alter;
//  alter.conservativeResize(dim, dB.cols());
//
//  list<int> _C = C;
//  list<int> _Cf = Cf;
//  list<int> _N = N;
//  list<int> _Nh = Nh;
//  list<int> _Nl = Nl;
//
//
//  for (int c : _C) {
//    C.remove(c);
//    N.push_back(c);
//
//    for (int j = 0; j < dB.cols(); ++j) {
//      solve_delta(dB.col(j));
//      alter(c, j) = dx(c);
//    }
//
//    N.remove(c);
//    C.push_back(c);
//  }
//
//  for (int cf : _Cf) {
//    Cf.remove(cf);
//    if (x(cf) > 0)
//      Nh.push_back(cf);
//    else
//      Nl.push_back(cf);
//
//    for (int j = 0; j < dB.cols(); ++j) {
//      solve_delta(dB.col(j));
//      alter(cf, j) = dx(cf);
//    }
//
//    if (x(cf) > 0)
//      Nh.remove(cf);
//    else
//      Nl.remove(cf);
//    Cf.push_back(cf);
//  }
//
//  for (int n : _N) {
//    N.remove(n);
//    C.push_back(n);
//
//    for (int j = 0; j < dB.cols(); ++j) {
//      solve_delta(dB.col(j));
//      alter(n, j) = dx(n);
//    }
//
//    C.remove(n);
//    N.push_back(n);
//  }
//
//  for (int nh : _Nh) {
//    Nh.remove(nh);
//    Cf.push_back(nh);
//
//    for (int j = 0; j < dB.cols(); ++j) {
//      solve_delta(dB.col(j));
//      alter(nh, j) = dx(nh);
//    }
//
//    Cf.remove(nh);
//    Nh.push_back(nh);
//  }
//
//  for (int nl : _Nl) {
//    Nl.remove(nl);
//    Cf.push_back(nl);
//
//    for (int j = 0; j < dB.cols(); ++j) {
//      solve_delta(dB.col(j));
//      alter(nl, j) = dx(nl);
//    }
//
//    Cf.remove(nl);
//    Nl.push_back(nl);
//  }
//
//  return alter;
//}
//
//MatrixXd LCP::modifyGradient(MatrixXd dB, double std = 0)
//{
//
//  MatrixXd modify_dx;
//  MatrixXd origin_dx;
//  MatrixXd origin_dw;
//  modify_dx.conservativeResize(dim, dB.cols());
//  origin_dx.conservativeResize(dim, dB.cols());
//  origin_dw.conservativeResize(dim, dB.cols());
//  for (int j = 0; j < dB.cols(); ++j) {
//    solve_delta(dB.col(j));
//    origin_dx.col(j) = dx;
//    origin_dw.col(j) = dw;
//  }
//
//  MatrixXd alter_dx = alterGradient(dB);
//
////  cout<<"origin_dx"<<endl;
////  cout<<origin_dx<<endl;
////  cout<<"alter_dx"<<endl;
////  cout<<alter_dx<<endl;
//
//  if (std < 0)
//  {
//    for (int i = 0; i < dim; ++i)
//    {
//      double origin_norm = origin_dx.row(i).squaredNorm();
//      double alter_norm = alter_dx.row(i).squaredNorm();
//
//      if (origin_norm < alter_norm)
//        modify_dx.row(i) = origin_dx.row(i);
//      else
//        modify_dx.row(i) = alter_dx.row(i);
//    }
//    //    cout<<"modify_dx"<<endl;
//    //    cout<<modify_dx<<endl;
//    return modify_dx;
//  }
//
//  VectorXd ratio = alterRatio(origin_dx, origin_dw, std);
////  cout<<"ratio"<<endl;
////  cout<<ratio<<endl;
//
//  for (int i = 0; i < dim; ++i)
//  {
//    modify_dx.row(i) = origin_dx.row(i) * (1 - ratio(i))
//                       + alter_dx.row(i) * ratio(i);
//  }
//  //  cout<<"modify_dx"<<endl;
//  //  cout<<modify_dx<<endl;
//  return modify_dx;
//}
