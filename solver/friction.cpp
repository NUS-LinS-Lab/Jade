#include <Eigen/Dense>
#include <iostream>


using namespace Eigen;
using namespace std;

int main()
{
    MatrixXd A {
      {0.39927, 0.570545, 0.513623},
      {0.570545, 0.861959, 0.759178},
      {0.513623, 0.759178, 1.13652}
    };
    VectorXd b {{0.092445, 0.114803, 0.288382}};

    // MatrixXd A {
    //   {1.89379, 0.476582, 1.83473},
    //   {0.476582, 1.11793, 0.557618},
    //   {1.83473, 0.557618, 1.78856}
    // };
    // VectorXd b {{0.760811, 1.24075, 0.839169}};

    // MatrixXd A {
    //   {0.471101, -0.59182, 0.469038},
    //   {-0.59182, 1.21474, -0.955505},
    //   {0.469038, -0.955505, 1.14923}
    // };

    // VectorXd b {{0.00584033, 0.181876, -0.0709052}};

    // MatrixXd A {
    //   {0.558583, -0.595426, -0.283869},
    //   {-0.595426, 0.653463, 0.256217},
    //   {-0.283869, 0.256217, 0.772368}
    // };
    // VectorXd b {{0.395052, -0.43882, 0.0194989}};

    // MatrixXd A {
    //   {0.911583, -0.403224, -0.590273},
    //   {-0.403224, 1.11809, -0.351638},
    //   {-0.590273, -0.351638, 0.800146}
    // };
    // VectorXd b {{0.487633, 0.837292, -1.01817}};

    double mu = 1;
    double tol = 1e-5;

    Vector3d x;
    Vector3d w;
    bool valid;

    cout<<"Seperate: \n"<<endl;

    x << 0, 0, 0;
    w = - b;
    valid = true;
    if (w(0) < -tol) valid = false;
    
    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nBoth static: \n"<<endl;

    x = A.completeOrthogonalDecomposition().solve(b);
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(x(1)) > mu * x(0) + tol) valid = false;
    if (fabs(x(2)) > mu * x(0) + tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (fabs(w(1)) > tol) valid = false;
    if (fabs(w(2)) > tol) valid = false;
    
    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }
    
    cout<<"\nOne static: \n"<<endl;

    cout<<"\nX higher bound: \n"<<endl;
    MatrixXd A_xh {
      {A(0, 0) + mu * A(0, 1), A(0, 2)},
      {A(2, 0) + mu * A(2, 1), A(2, 2)}
    };
    VectorXd b_xh {{b(0), b(2)}};
    VectorXd x_xh = A_xh.completeOrthogonalDecomposition().solve(b_xh);
    x << x_xh(0), mu * x_xh(0), x_xh(1);
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(x(2)) > mu * x(0) + tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (w(1) > tol) valid = false;
    if (fabs(w(2)) > tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nX lower bound: \n"<<endl;
    MatrixXd A_xl {
      {A(0, 0) - mu * A(0, 1), A(0, 2)},
      {A(2, 0) - mu * A(2, 1), A(2, 2)}
    };
    VectorXd b_xl {{b(0), b(2)}};
    VectorXd x_xl = A_xl.completeOrthogonalDecomposition().solve(b_xl);
    x << x_xl(0), - mu * x_xl(0), x_xl(1);
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(x(2)) > mu * x(0) + tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (w(1) < -tol) valid = false;
    if (fabs(w(2)) > tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nY higher bound: \n"<<endl;
    MatrixXd A_yh {
      {A(0, 0) + mu * A(0, 2), A(0, 1)},
      {A(1, 0) + mu * A(1, 2), A(1, 1)}
    };
    VectorXd b_yh {{b(0), b(1)}};
    VectorXd x_yh = A_yh.completeOrthogonalDecomposition().solve(b_yh);
    x << x_yh(0), x_yh(1), mu * x_yh(0);
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(x(1)) > mu * x(0) + tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (fabs(w(1)) > tol) valid = false;
    if (w(2) > tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nY lower bound: \n"<<endl;
    MatrixXd A_yl {
      {A(0, 0) - mu * A(0, 2), A(0, 1)},
      {A(1, 0) - mu * A(1, 2), A(1, 1)}
    };
    VectorXd b_yl {{b(0), b(1)}};
    VectorXd x_yl = A_yh.completeOrthogonalDecomposition().solve(b_yl);
    x << x_yl(0), x_yl(1), - mu * x_yl(0);
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(x(1)) > mu * x(0) + tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (fabs(w(1)) > tol) valid = false;
    if (w(2) < -tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nNone static: \n"<<endl;

    cout<<"\nHigher higher bound: \n"<<endl;
    double A_hh = A(0, 0) + mu * (A(0, 1) + A(0, 2));
    double b_hh = b(0);
    double x_hh = b_hh / A_hh;
    x << x_hh, mu * x_hh, mu * x_hh;
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (w(1) > tol) valid = false;
    if (w(2) > tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nHigher lower bound: \n"<<endl;
    double A_hl = A(0, 0) + mu * (A(0, 1) - A(0, 2));
    double b_hl = b(0);
    double x_hl = b_hl / A_hl;
    x << x_hl, mu * x_hl, - mu * x_hl;
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (w(1) > tol) valid = false;
    if (w(2) < -tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nLower higher bound: \n"<<endl;
    double A_lh = A(0, 0) + mu * (- A(0, 1) + A(0, 2));
    double b_lh = b(0);
    double x_lh = b_lh / A_lh;
    x << x_lh, - mu * x_lh, mu * x_lh;
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (w(1) < -tol) valid = false;
    if (w(2) > tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }

    cout<<"\nLower lower bound: \n"<<endl;
    double A_ll = A(0, 0) + mu * (- A(0, 1) - A(0, 2));
    double b_ll = b(0);
    double x_ll = b_ll / A_ll;
    x << x_ll, - mu * x_ll, - mu * x_ll;
    w = A * x - b;
    valid = true;
    if (x(0) < -tol) valid = false;
    if (fabs(w(0)) > tol) valid = false;
    if (w(1) < -tol) valid = false;
    if (w(2) < -tol) valid = false;

    std::cout<<"valid: "<<valid<<std::endl;
    if (valid)
    {
      std::cout<<"x: "<<std::endl;
      std::cout<<x.transpose()<<std::endl;
      std::cout<<"w: "<<std::endl;
      std::cout<<w.transpose()<<std::endl;
    }
}