#include "lcp.cpp"
#include <stdlib.h>

int main(int argc, char* argv[])
{
    srand((unsigned int) time(0));

    // bool friction = atoi(argv[1]);
    bool friction = true;

    for (int contact = 1; contact <= 5; contact++)
    {
        int dim;
        if (friction) dim = contact * 3;
        else dim = contact;

        VectorXd hi(dim);
        VectorXd lo(dim);
        VectorXi f_idx(dim);

        for (int i = 0; i < contact; i++)
        {
            if (friction) 
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
            else
            {
                hi(i) = inf;
                lo(i) = 0;
                f_idx(i) = -1;
            }
        }


        int success = 0;

        for (int iter = 0; iter < 10000; iter++)
        {
            MatrixXd J = MatrixXd::Random(dim, dim);
            MatrixXd A = J.transpose() * J;
            
            VectorXd c = VectorXd::Random(dim);
            VectorXd b = A * c;

            LCP lcp;
            lcp.setup(dim, A, b, hi, lo, f_idx, 1e-6, 1e-6);
            VectorXd x = lcp.solve();
            if (lcp.valid(1e-4)) success += 1;
            // if (!(lcp.valid(1e-4)))
            // {
            //     LCP lcp_debug;
            //     lcp_debug.setup(dim, A, b, hi, lo, f_idx, 1e-6, 1e-6, true);
            //     x = lcp_debug.violet_solve();
            //     std::cout<<"A: "<<std::endl;
            //     std::cout<<A<<std::endl;
            //     std::cout<<"b: "<<std::endl;
            //     std::cout<<b.transpose()<<std::endl;
            //     std::cout<<"x: "<<std::endl;
            //     std::cout<<x.transpose()<<std::endl;
            //     std::cout<<"w: "<<std::endl;
            //     std::cout<<(A * x - b).transpose()<<std::endl;
            //     lcp_debug.solve();
            //     lcp_debug.valid(1e-4);
            //     // break;
            // }
        }

        double success_rate = success / 10000.0;
        std::cout
        <<" Contact: "<<contact
        <<" Success Rate: "<<success_rate
        <<std::endl;
    }
}