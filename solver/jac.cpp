#include "lcp.cpp"
#include <stdlib.h>


Matrix3d hat(Vector3d v)
{
    Matrix3d h;
    h << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return h;
}

MatrixXd grasp(Vector3d n, Vector3d c, Vector2d p)
{
    MatrixXd G (6, 3);

    Vector3d up (0, 0, 1);
    Vector3d z = n.normalized();
    Vector3d x = (up.cross(n)).normalized();
    Vector3d y = (n.cross(x)).normalized();

    Matrix3d R;
    R.col(0) = z;
    R.col(1) = x;
    R.col(2) = y;

    c = c + p(0) * x + p(1) * y;

    G.block<3, 3>(0, 0) = R;
    G.block<3, 3>(3, 0) = hat(c) * R;

    return G;
}

int main(int argc, char* argv[])
{
    srand((unsigned int) time(0));

    int n_contact = atoi(argv[1]);
    // n_contact = 2;

    int dim = n_contact * 3;

    VectorXd hi(dim);
    VectorXd lo(dim);
    VectorXi f_idx(dim);

    for (int i = 0; i < n_contact; i++)
    {
        hi(i*3) = inf;
        lo(i*3) = 0;
        f_idx(i*3) = -1;

        hi(i*3+1) = 1;
        lo(i*3+1) = -1;
        f_idx(i*3+1) = i*3;

        hi(i*3+2) = 1;
        lo(i*3+2) = -1;
        f_idx(i*3+2) = i*3;
    }


    int success = 0;
    int n_iter = atoi(argv[2]);

    for (int iter = 0; iter < n_iter; iter++)
    {
        MatrixXd J (6, dim);
        VectorXd b (dim);
        // VectorXd b = VectorXd::Random(dim);

        // Vector3d n = Vector3d::Random();
        // Vector3d c = Vector3d::Random();

        Vector3d n (1, 0, 0);
        Vector3d c = -n;

        Vector3d v = Vector3d::Random();
        Vector3d o = Vector3d::Random();
        // v(0) = 1;

        MatrixXd p = MatrixXd::Random(2, n_contact);
        // Matrix2d p {
        //     {1, 0},
        //     {1, 0},
        // };




        for (int i = 0; i < n_contact; i++)
        {   
            J.block<6, 3>(0, i*3) = grasp(n, c, p.col(i));

            b(i*3) = v(0) + o(1) * p(1, i) - o(2) * p(0, i);
            b(i*3+1) = v(1) - o(0) * p(0, i);
            b(i*3+2) = v(2) + o(0) * p(1, i);
            // b(i*3) = 1;
        }

        // MatrixXd M = MatrixXd::Random(dim, dim);
        // MatrixXd A = M * M.transpose();
        MatrixXd A = J.transpose() * J;
        

        LCP lcp;
        lcp.setup(dim, A, b, hi, lo, f_idx, 1e-6, 1e-6);
        VectorXd x = lcp.solve();
        if (lcp.valid(1e-4)) success += 1;
        if (!(lcp.valid(1e-4)))
        {
            std::cout<<"A: "<<std::endl;
            std::cout<<A<<std::endl;
            std::cout<<"b: "<<std::endl;
            std::cout<<b.transpose()<<std::endl;
            std::cout<<"p: "<<std::endl;
            std::cout<<p.transpose()<<std::endl;
            std::cout<<"v: "<<std::endl;
            std::cout<<v.transpose()<<std::endl;
            std::cout<<"o: "<<std::endl;
            std::cout<<o.transpose()<<std::endl;
            std::cout<<"x: "<<std::endl;
            std::cout<<x.transpose()<<std::endl;
            std::cout<<"w: "<<std::endl;
            std::cout<<(A * x - b).transpose()<<std::endl;
            LCP lcp_debug;
            lcp_debug.setup(dim, A, b, hi, lo, f_idx, 1e-6, 1e-6, true);
            lcp_debug.solve();
            lcp_debug.valid(1e-4);
            break;
        }
    }

    double success_rate = success / double(n_iter);
    std::cout<<"Contacts: "<<n_contact
    <<" Success Rate: "<<success_rate
    <<std::endl;

}