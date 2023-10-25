#include "mu.h"

FLCP::FLCP() {}
FLCP::~FLCP() {}

Eigen::IOFormat FMT(3, 0, ", ", ";\n", "[", "]", "[", "]");

void FLCP::setup(int _dim, MatrixXd _A, VectorXd _b,
                double _mu, VectorXi _f_idx,
                double _tol, bool _debug = true)
{
    dim = _dim;

    A = _A;
    b = _b;
    f_idx = _f_idx;

    p = VectorXi::LinSpaced(_dim, 0, _dim - 1);
    put_friction_end();

    solve_vanilla();

    target_mu = _mu;
    mu = 0;
    tol = _tol;
    mover = VectorXi::Zero(_dim);

    debug = _debug;
}


void FLCP::put_friction_end()
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
    f_idx = P.inverse() * f_idx;

    for (int j = 0; j < dim; ++j) {
        for (int k = 0; k < dim; ++k) {
            if (f_idx(j) == p(k)) {
            f_idx(j) = k;
            break;
            }
        }
    }
}


void FLCP::solve_vanilla()
{
    x = VectorXd::Zero(dim);
    w = -b;

    for (int i = 0; i < dim/3; ++i) {
        if (w(i) < -tol) {
            for (int iter = 0; iter < dim/3; iter++) {
                dx = VectorXd::Zero(dim);
                dw = VectorXd::Zero(dim);
                VectorXd db = -A.col(i);
                
                if (C.size() > 0) {
                    vector<int> clamp;
                    for (int c : C)
                        clamp.push_back(c);
                    MatrixXd A_c = A(clamp, clamp);
                    VectorXd db_c = db(clamp);
                    VectorXd dx_c = A_c.completeOrthogonalDecomposition().solve(db_c);
                    dx(clamp) = dx_c;
                }
                dx(i) = 1;
                dw = A * dx;

                double s = -w(i) / dw(i);
                double _s;

                for (int c : C) {
                    if (dx(c) < -tol) {
                        _s = -x(c) / dx(c);
                        if (_s > tol) s = min(s, _s);
                    }
                }

                for (int n : N) {
                    if (dw(n) < -tol) {
                        _s = -w(n) / dw(n);
                        if (_s > tol) s = min(s, _s);
                    }
                }

                x += s * dx;
                w += s * dw;

                list<int> _C = C;
                list<int> _N = N;

                for (int c : C) {
                    if (x(c) < tol) {
                        _C.remove(c);
                        _N.push_back(c);
                    }
                }

                for (int n : N) {
                    if (w(n) < tol) {
                        _N.remove(n);
                        _C.push_back(n);
                    }
                }

                C = _C;
                N = _N;

                if (w(i) > -tol) {
                    C.push_back(i);
                    break;
                }
            }
        }
        else N.push_back(i);
    }

    for (int i = dim/3; i < dim; ++i) {
        if (w(i) > tol) Nl.push_back(i);
        else if (w(i) < -tol) Nh.push_back(i);
        else Cf.push_back(i);
    }
}


void FLCP::delta_mu()
{
    if (C.size() + Cf.size() == 0) return;

    dx = VectorXd::Zero(dim);
    dw = VectorXd::Zero(dim);

    VectorXd db = VectorXd::Zero(dim);
    MatrixXd _A = A;

    for (int h : Nh) {
        db -= A.col(h) * x(f_idx(h));
        _A.col(f_idx(h)) += mu * A.col(h);
    }
    for (int l : Nl) {
        db += A.col(l) * x(f_idx(l));
        _A.col(f_idx(l)) -= mu * A.col(l);
    }
                
    vector<int> clamp;
    for (int c : C)
        clamp.push_back(c);
    for (int cf : Cf)
        clamp.push_back(cf);

    MatrixXd A_c = _A(clamp, clamp);
    VectorXd db_c = db(clamp);
    VectorXd dx_c = A_c.completeOrthogonalDecomposition().solve(db_c);
    dx(clamp) = dx_c;

    for (int h : Nh)
        dx(h) = mu * dx(f_idx(h)) + x(f_idx(h));

    for (int l : Nl)
        dx(l) = -mu * dx(f_idx(l)) - x(f_idx(l));
    
    dw = A * dx;
}









// int main()
// {
//     int contacts = 3;
//     VectorXi f_idx(contacts*3);
//     for (int i = 0; i < 2; i++)
//     {
//         f_idx(i*3) = -1;
//         f_idx(i*3+1) = i*3;
//         f_idx(i*3+2) = i*3;
//     }

//     for (int iter = 0; iter < 10; iter++)
//     {
//         cout<<iter<<endl;
//         MatrixXd J = MatrixXd::Random(contacts*3, contacts*3);
//         MatrixXd A = J.transpose() * J;
        
//         VectorXd c = VectorXd::Random(contacts*3);
//         VectorXd b = A * c;

//         FLCP flcp;
//         flcp.setup(contacts*3, A, b, 0, f_idx, 1e-6);
        
//         cout<<flcp.x.head(contacts).transpose()<<endl;
//         cout<<flcp.w.head(contacts).transpose()<<endl;
//     }
// }