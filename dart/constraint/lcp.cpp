#include "dart/constraint/lcp.h"

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
  pf_idx = f_idx;

  x = VectorXd::Zero(_dim);
  w = -_b;
  dx = VectorXd::Zero(_dim);
  dw = VectorXd::Zero(_dim);

  p = VectorXi::LinSpaced(_dim, 0, _dim - 1);

  step = 0;
  eps = _eps;
  tol = _tol;
  mover = VectorXi::Zero(_dim);

  debug = false;

}

Solution LCP::save()
{
  Solution s;
  s.x = x;
  s.C = C;
  s.N = N;
  s.Cf = Cf;
  s.Nh = Nh;
  s.Nl = Nl;
  return s;
}

void LCP::load(Solution s)
{
  x = s.x;
  x = s.x;
  C = s.C;
  N = s.N;
  Cf = s.Cf;
  Nh = s.Nh;
  Nl = s.Nl;
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

  // dx(idx) = 1;

  dw = A * dx;

  for (int c : C)
    dw(c) = 0;

  for (int cf : Cf)
    dw(cf) = 0;

  // if (debug)
  // {
  //   std::cout<<"dx: "<<dx.transpose()<<endl;
  //   std::cout<<"dw: "<<dw.transpose()<<endl;
  //   for (int i : C)
  //     std::cout<<"C: "<<i<<endl;
  //   for (int i : N)
  //     std::cout<<"N: "<<i<<endl;
  //   std::cout<<endl;
  // }
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


bool LCP::max_step(int i, bool f)
{
  mover = VectorXi::Zero(dim);
  mover(i) = 1;
  step = inf;
  double _s = inf;
  double _sh = inf;
  double _sl = inf;
  double _sb = inf;

  _s = fabs(dw(i)) > eps ? -w(i) / dw(i) : inf;
  compare(_s, i);
    
  if (f) {
    double x_h = x(i) - hi(i) * x(pf_idx(i));
    double dx_h = dx(i) - hi(i) * dx(pf_idx(i));
    if (fabs(dx_h) > eps) {
      _sh = -x_h / dx_h;
    }
    double x_l = x(i) - lo(i) * x(pf_idx(i));
    double dx_l = dx(i) - lo(i) * dx(pf_idx(i));
    if (fabs(dx_l) > eps) {
      _sl = -x_l / dx_l;
    }
    if (w(i) > 0) 
    {
      _s = _sl;
      if (_sh > 0) _sb = _sh;
    }
    else
    {
      _s = _sh;
      if (_sl > 0) _sb = _sl;
    }
  }

  compare(_s, i);

  if (debug)
  {
    std::cout << "x: " << x.transpose().format(FMT) << endl;
    std::cout << "w: " << w.transpose().format(FMT) << endl;
    std::cout<<"step: "<<step<<endl; 
    std::cout<<"sb: "<<_sb<<endl; 
    std::cout<<"dx: "<<dx.transpose()<<endl;
    std::cout<<"dw: "<<dw.transpose()<<endl;
    std::cout<<endl;
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

  // if (clamp) {
  //   if (_sw * _sb > 0 && fabs(_sb) < step) {
  //     // cout<<"larger step"<<endl;
  //     return false;
  //   }
  // }
  // else {
  //   if (_sw * _sb > 0 && fabs(_sw) < step) {
  //     // cout<<"larger step"<<endl;
  //     return false;
  //   }
  // }

  if (step < tol) {
    if (debug) std::cout<<"zero step"<<endl;
    return false;
  }
  if (step > _sb) {
    if (debug) std::cout<<"touch boundary"<<endl;
    return false;
  }
  
  return true;
}


// void LCP::max_step(int i, bool f, bool reverse)
// {
//   mover = VectorXi::Zero(dim);
//   mover(i) = 1;
//   double _s;
//   double _sw;
//   double _sb = inf;
//   double _sn;

//   if (reverse) {
//     step = inf;
//     dx *= -1;
//     dw *= -1;
//   }

//   _sw = fabs(dw(i)) > eps ? -w(i) / dw(i) : inf;
//   // cout<<"_sw: "<<_sw<<endl;
    
//   if (f) {
//     double x_h = x(i) - hi(i) * x(pf_idx(i));
//     double dx_h = dx(i) - hi(i) * dx(pf_idx(i));
//     if (fabs(dx_h) > eps && w(i) < -tol) {
//       _sb = -x_h / dx_h;
//     }
//     double x_l = x(i) - lo(i) * x(pf_idx(i));
//     double dx_l = dx(i) - lo(i) * dx(pf_idx(i));
//     if (fabs(dx_l) > eps && w(i) > tol) {
//       _sb = -x_l / dx_l;
//     }
//     _sn = -x(pf_idx(i)) / dx(pf_idx(i));
//     if (_sb * _sn > 0 && fabs(_sn) < fabs(_sb))
//       _sb = inf;

//     // cout<<"_sb: "<<_sb<<endl;
//   }

//   if (reverse) {
//     compare(_sw, i);
//     compare(_sb, i);
//   }
//   else {
//     if (fabs(_sw) < fabs(_sb))
//       step = _sw;
//     else
//       step = _sb;

//     if (step < 0) {
//       step *= -1;
//       dx *= -1;
//       dw *= -1;
//     }
//   }


//   if (debug)
//   {
//     cout<<"step: "<<step<<endl; 
//     cout<<"dx: "<<dx.transpose()<<endl;
//     cout<<"dw: "<<dw.transpose()<<endl;
//     cout<<endl;
//   }


//   for (int c : C) {
//     if (dx(c) < -eps) {
//       _s = -x(c) / dx(c);
//       compare(_s, c);
//     }
//   }

//   for (int n : N) {
//     if (dw(n) < -eps) {
//       _s = -w(n) / dw(n);
//       compare(_s, n);
//     }
//   }

//   for (int cf : Cf) {
//     double x_h = x(cf) - hi(cf) * x(pf_idx(cf));
//     double dx_h = dx(cf) - hi(cf) * dx(pf_idx(cf));
//     if (dx_h > eps) {
//       _s = -x_h / dx_h;
//       compare(_s, cf);
//     }
//     double x_l = x(cf) - lo(cf) * x(pf_idx(cf));
//     double dx_l = dx(cf) - lo(cf) * dx(pf_idx(cf));
//     if (dx_l < -eps) {
//       _s = -x_l / dx_l;
//       compare(_s, cf);
//     }
//   }

//   for (int nh : Nh) {
//     if (dw(nh) > eps) {
//       _s = -w(nh) / dw(nh);
//       compare(_s, nh);
//     }
//   }

//   for (int nl : Nl) {
//     if (dw(nl) < -eps) {
//       _s = -w(nl) / dw(nl);
//       compare(_s, nl);
//     }
//   }

// }

void LCP::move_step(int i,  bool f)
{

  x += step * dx;
  w += step * dw;

  // // DEBUG
  // if (debug)
  // {
  //   // cout<< "end of move_step()" <<endl;
  //   std::cout << "step: " << step << endl;
  //   std::cout << "x: " << x.transpose().format(FMT) << endl;
  //   std::cout << "w: " << w.transpose().format(FMT) << endl;
  //   std::cout << "w: " << (A * x - b).transpose().format(FMT) << endl;
  //   std::cout<<endl;
  // }

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
      if (debug) std::cout<<i<<" from U to C"<<endl;
    }
    else if (fabs(w(i)) <= tol) {
      U.remove(i);
      _Cf.push_back(i);
      // DEBUG
      if (debug) std::cout<<i<<" from U to Cf"<<endl;
    }
    else if (w(i) < -tol) {
      U.remove(i);
      _Nh.push_back(i);
      // DEBUG
      if (debug) std::cout<<i<<" from U to Nh"<<endl;
    }
    else if (w(i) > tol) {
      U.remove(i);
      _Nl.push_back(i);
      // DEBUG
      if (debug) std::cout<<i<<" from U to Nl"<<endl;
    }
  }

  for (int c : C) {
    if (mover(c) > 0) {
      _C.remove(c);
      _N.push_back(c);
      // DEBUG
      if (debug) std::cout<<c<<" from C to N"<<endl;
    }
  }

  for (int n : N) {
    if (mover(n) > 0) {
      _N.remove(n);
      _C.push_back(n);
      // DEBUG
      if (debug) std::cout<<n<<" from N to C"<<endl;
    }
  }

  for (int cf : Cf) {
    if (mover(cf) > 0) {
      _Cf.remove(cf);
      if(x(cf) > tol)
      {
        _Nh.push_back(cf);
        // DEBUG
        if (debug) std::cout<<cf<<" from Cf to Nh"<<endl;
      }
      else
      {
        _Nl.push_back(cf);
        // DEBUG
        if (debug) std::cout<<cf<<" from Cf to Nl"<<endl;
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
        if (debug) std::cout<<nh<<" from Nh to Nl"<<endl;
      }
      else
      {
        _Nh.remove(nh);
        _Cf.push_back(nh);
        // DEBUG
        if (debug) std::cout<<nh<<" from Nh to Cf"<<endl;
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
        if (debug) std::cout<<nl<<" from Nl to Nh"<<endl;
      }
      else
      {
        _Nl.remove(nl);
        _Cf.push_back(nl);
        // DEBUG
        if (debug) std::cout<<nl<<" from Nl to Cf"<<endl;
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

  
}

bool LCP::drive_to_boundary(int i, bool f)
{
  int level = 0;
  std::vector<Solution> stack;
  std::vector<bool> dirty;
  stack.push_back(save());
  dirty.push_back(false);
  

  for (int iter = 0; iter < 3 * dim; iter++) 
  {
    dx(i) = (w(i) < 0) ? 1 : -1;
    solve_delta(-dx(i) * A.col(i));
    if (dirty[level])
    {
      dx *= -1;
      dw *= -1;
    }
    bool flag = max_step(i, f);
    if (!flag)
    {
      while (dirty[level])
      {
        level--;
        if (level < 0)
          return false;
      }
      load(stack[level]);
      dirty[level] = true;
      continue;
    }
    else
    {
      move_step(i, f);
      if (level < stack.size())
      {
        stack.push_back(save());
        dirty.push_back(false);
      }
      else
      {
        stack[level] = save();
        dirty[level] = false;
      }
      level++;
    }
    if (mover(i) > 0)
      return true;
  }


  if (debug) 
    std::cout<<"\nOver loop!!!"<<endl;
  return false;
}

// bool LCP::drive_to_boundary(int i, bool f)
// {
//   for (int iter = 0; iter < dim; iter++) {
//     solve_delta(- A.col(i));
//     max_step(i, f, true);
//     move_step(i, f);
//     if (mover(i) > 0)
//       return true;
//   }


//   if (debug) 
//     cout<<"\nOver loop!!!"<<endl;
//   return false;
// }


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
  C.clear();
  Cf.clear();
  N.clear();
  Nh.clear();
  Nl.clear();
  U.clear();

  x = VectorXd::Zero(dim);
  w = -b;
  dx = VectorXd::Zero(dim);
  dw = VectorXd::Zero(dim);


  put_friction_at_end();
  // DEBUG
  if (debug)
  {
    std::cout << "x: " << x.transpose().format(FMT) << endl;
    std::cout << "w: " << w.transpose().format(FMT) << endl;
    std::cout<<endl;
  }

  

  for (int i = 0; i < dim; ++i) {
    // DEBUG
    if (debug) std::cout<<"\nPoint "<<i<<endl;
    if (pf_idx(i) < 0) {
      if (w(i) <= - tol) {
        if (!drive_to_boundary(i, false))
          break;
      }
      else {
        U.remove(i);
        N.push_back(i);
        // DEBUG
        if (debug) std::cout<<i<<" from U to N"<<endl;
      }
    }
    else {
      if (if_contain(N, pf_idx(i))) {
        if (w(i) < 0) {
          U.remove(i);
          Nh.push_back(i);
          // DEBUG
          if (debug) std::cout<<i<<" from U to Nh"<<endl;
        }
        else {
          U.remove(i);
          Nl.push_back(i);
          // DEBUG
          if (debug) std::cout<<i<<" from U to Nl"<<endl;
        }
      }
      else if (fabs(w(i)) <= tol) {
        U.remove(i);
        Cf.push_back(i);
        // DEBUG
        if (debug) std::cout<<i<<" from U to Cf"<<endl;
      }
      else {
        if (!drive_to_boundary(i, true))
          break;
      }
    }
  }


  PermutationMatrix<Dynamic, Dynamic> P(p);
  VectorXd _x = P * x;

  return _x;
}


VectorXd LCP::violet_solve()
{
  int contacts = dim/3;
  for (int s = 0; s < pow(10, contacts); s++)
  {
    x = VectorXd::Zero(dim);
    C.clear();
    Cf.clear();
    N.clear();
    Nh.clear();
    Nl.clear();
    // std::cout<<s<<endl;

    for (int c = 0; c < contacts; c++)
    {
      int sit = (s % (int)pow(10, c + 1)) / (int)pow(10, c);
      // std::cout<<sit<<endl;
      switch (sit)
      {
        case 0:
          N.push_back(c*3);
          N.push_back(c*3+1);
          N.push_back(c*3+2);
          break;
        case 1:
          C.push_back(c*3);
          Cf.push_back(c*3+1);
          Cf.push_back(c*3+2);
          break;
        case 2:
          C.push_back(c*3);
          Nh.push_back(c*3+1);
          Cf.push_back(c*3+2);
          break;
        case 3:
          C.push_back(c*3);
          Nl.push_back(c*3+1);
          Cf.push_back(c*3+2);
          break;
        case 4:
          C.push_back(c*3);
          Cf.push_back(c*3+1);
          Nh.push_back(c*3+2);
          break;
        case 5:
          C.push_back(c*3);
          Nh.push_back(c*3+1);
          Nh.push_back(c*3+2);
          break;
        case 6:
          C.push_back(c*3);
          Nl.push_back(c*3+1);
          Nh.push_back(c*3+2);
          break;
        case 7:
          C.push_back(c*3);
          Cf.push_back(c*3+1);
          Nl.push_back(c*3+2);
          break;
        case 8:
          C.push_back(c*3);
          Nh.push_back(c*3+1);
          Nl.push_back(c*3+2);
          break;
        case 9:
          C.push_back(c*3);
          Nl.push_back(c*3+1);
          Nl.push_back(c*3+2);
          break;
      }
    }

    // std::cout<<C<<endl;
    // std::cout<<N<<endl;
    // std::cout<<Cf<<endl;
    // std::cout<<Nh<<endl;
    // std::cout<<Nl<<endl;

    if (C.size() > 0)
    {
      MatrixXd _A = A;

      vector<int> clamp;
      for (int c : C)
        clamp.push_back(c);
      for (int cf : Cf)
        clamp.push_back(cf);

      for (int h : Nh)
        _A.col(f_idx(h)) += hi(h) * A.col(h);
      for (int l : Nl)
        _A.col(f_idx(l)) += lo(l) * A.col(l);

      MatrixXd A_c = _A(clamp, clamp);
      VectorXd b_c = b(clamp);
      VectorXd x_c = A_c.completeOrthogonalDecomposition().solve(b_c);

      x(clamp) = x_c;

      for (int h : Nh)
        x(h) = hi(h) * x(f_idx(h));

      for (int l : Nl)
        x(l) = lo(l) * x(f_idx(l));
    }

    w = A * x - b;
    if (valid(1e-4))
      return x;
  }

  if (debug) std::cout<<"fail"<<endl;
  return x;
}


bool LCP::valid(double _tol)
{
  for (int i = 0; i < dim; i++)
  {
    double upperLimit = hi(i);
    double lowerLimit = lo(i);

    if (pf_idx(i) != -1)
    {
      upperLimit *= x(pf_idx(i));
      lowerLimit *= x(pf_idx(i));
    }

    /// Solves constriant impulses for a constrained group. The LCP formulation
    /// setting that this function solve is A*x = b + w where each x[i], w[i]
    /// satisfies one of
    ///   (1) x = lo, w >= 0
    ///   (2) x = hi, w <= 0
    ///   (3) lo < x < hi, w = 0

    // If force has a zero bound, and we're at a zero bound (this is common with
    // friction being upper-bounded by a near-zero normal force) then allow
    // velocity in either direction.
    if (abs(lowerLimit) < _tol && abs(upperLimit) < _tol && abs(x(i)) < _tol)
    {
      // This is always allowed
    }
    // If force is at the lower bound, velocity must be >= 0
    else if (abs(x(i) - lowerLimit) < _tol)
    {
      if (w(i) < -_tol)
      {
        // if (debug) 
        //   std::cout<<"w(i) < -tol"<<endl;
        return false;
      }
        
    }
    // If force is at the upper bound, velocity must be <= 0
    else if (abs(x(i) - upperLimit) < _tol)
    {
      if (w(i) > _tol)
      {
        // if (debug) 
        //   std::cout<<"w(i) > tol"<<endl;
        return false;
      }
    }
    // If force is within bounds, then velocity must be zero
    else if (x(i) > (lowerLimit + _tol) && x(i) < (upperLimit - _tol))
    {
      if (abs(w(i)) > _tol)
      {
        // if (debug)
        //   std::cout<<"abs(w(i)) > tol"<<endl;
        return false;
      }
    }
    // If force is out of bounds, we're always illegal
    else
    {
      // if (debug) 
      //   std::cout<<"out of bounds"<<endl;
      return false;
    }
  }
  // If we make it here, the solution is fine
  return true;
}



// Matrix3d hat(Vector3d v)
// {
//     Matrix3d h;
//     h << 0, -v(2), v(1),
//             v(2), 0, -v(0),
//             -v(1), v(0), 0;
//     return h;
// }

// MatrixXd grasp(Vector3d n, Vector3d c, Vector2d p)
// {
//     MatrixXd G (6, 3);

//     Vector3d up (0, 0, 1);
//     Vector3d z = n.normalized();
//     Vector3d x = (up.cross(n)).normalized();
//     Vector3d y = (n.cross(x)).normalized();

//     Matrix3d R;
//     R.col(0) = z;
//     R.col(1) = x;
//     R.col(2) = y;

//     c = c + p(0) * x + p(1) * y;

//     G.block<3, 3>(0, 0) = R;
//     G.block<3, 3>(3, 0) = hat(c) * R;

//     return G;
// }


// int main()
// {
//     srand((unsigned int) time(0));

//     int n_contact = 1;

//     int dim = n_contact * 3;

//     VectorXd hi(dim);
//     VectorXd lo(dim);
//     VectorXi f_idx(dim);

//     for (int i = 0; i < n_contact; i++)
//     {
//         hi(i*3) = inf;
//         lo(i*3) = 0;
//         f_idx(i*3) = -1;

//         hi(i*3+1) = 1;
//         lo(i*3+1) = -1;
//         f_idx(i*3+1) = i*3;

//         hi(i*3+2) = 1;
//         lo(i*3+2) = -1;
//         f_idx(i*3+2) = i*3;
//     }


//     vector<MatrixXd> A;
//     vector<VectorXd> b;


//     MatrixXd A1 {
//       {0.39927, 0.570545, 0.513623},
//       {0.570545, 0.861959, 0.759178},
//       {0.513623, 0.759178, 1.13652}
//     };
//     VectorXd b1 {{0.092445, 0.114803, 0.288382}};
//     A.push_back(A1);
//     b.push_back(b1);

//     MatrixXd A2 {
//       {1.89379, 0.476582, 1.83473},
//       {0.476582, 1.11793, 0.557618},
//       {1.83473, 0.557618, 1.78856}
//     };
//     VectorXd b2 {{0.760811, 1.24075, 0.839169}};
//     A.push_back(A2);
//     b.push_back(b2);

//     MatrixXd A3 {
//       {0.471101, -0.59182, 0.469038},
//       {-0.59182, 1.21474, -0.955505},
//       {0.469038, -0.955505, 1.14923}
//     };
//     VectorXd b3 {{0.00584033, 0.181876, -0.0709052}};
//     A.push_back(A3);
//     b.push_back(b3);

//     MatrixXd A4 {
//       {0.558583, -0.595426, -0.283869},
//       {-0.595426, 0.653463, 0.256217},
//       {-0.283869, 0.256217, 0.772368}
//     };
//     VectorXd b4 {{0.395052, -0.43882, 0.0194989}};
//     A.push_back(A4);
//     b.push_back(b4);

//     MatrixXd A5 {
//       {0.911583, -0.403224, -0.590273},
//       {-0.403224, 1.11809, -0.351638},
//       {-0.590273, -0.351638, 0.800146}
//     };
//     VectorXd b5 {{0.487633, 0.837292, -1.01817}};
//     A.push_back(A5);
//     b.push_back(b5);

//     MatrixXd A6 {
//       {0.627218, 0.697115, -0.758928},
//       {0.697115, 1.78364, -1.31403},
//       {-0.758928, -1.31403, 1.1591}
//     };
//     VectorXd b6 {{0.0289075, 0.0854709, -0.0528137}};
//     A.push_back(A6);
//     b.push_back(b6);

//     MatrixXd A7 {
//       {1.38017, -0.363013, 0.263135},
//       {-0.363013, 0.122347, -0.220239},
//       {0.263135, -0.220239, 0.899204}
//     };
//     VectorXd b7 {{0.900837, -0.282116, 0.42573}};
//     A.push_back(A7);
//     b.push_back(b7);



//     for (int i = 0; i < A.size(); i++)
//     {
//       LCP lcp;
//       lcp.setup(dim, A[i], b[i], hi, lo, f_idx, 1e-6, 1e-6, false);
//       VectorXd x = lcp.solve();
//       // VectorXd x = lcp.violet_solve();
//       std::cout<<"\nValid: "<<lcp.valid(1e-4)<<std::endl;
//       std::cout<<"A: "<<std::endl;
//       std::cout<<A[i]<<std::endl;
//       std::cout<<"b: "<<std::endl;
//       std::cout<<b[i].transpose()<<std::endl;
//       std::cout<<"x: "<<std::endl;
//       std::cout<<x.transpose()<<std::endl;
//       std::cout<<"w: "<<std::endl;
//       std::cout<<(A[i] * x - b[i]).transpose()<<std::endl;
//     }
// }