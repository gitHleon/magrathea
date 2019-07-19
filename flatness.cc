#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <iomanip>
#include <alglibtools.h>
#include <statistics.h>
#include <optimization.h>



void compute_G_and_Fi(const alglib::real_2d_array &U,
                      const alglib::real_2d_array &T,
                      alglib::real_2d_array &G,
                      alglib::real_2d_array &Fi,
                      alglib::real_2d_array &S)
{

    alglib::real_2d_array cTU = cross(T, U);
    double dTU = vdot(T, U);
    double ncTU = vnorm(cTU);

    double vG[3][3] = {
                       { dTU, -ncTU, 0.0},
                       {ncTU,  dTU,  0.0},
                       {0.0, 0.0, 1.0}
    };
    G.setcontent(3, 3, (const double *)vG);
    //std::cout << "G: \n" << G << std::endl;

    alglib::real_2d_array tmp = U-dTU*T;
    auto col2 = tmp/vnorm(tmp);
    auto col3 = cross(U, T);

    Fi.setlength(3, 3);
    alglib::rmatrixcopy(T.rows(), T.cols(), T, 0, 0, Fi, 0, 0);
    alglib::rmatrixcopy(col2.rows(), col2.cols(), col2, 0, 0, Fi, 0, 1);
    alglib::rmatrixcopy(col3.rows(), col3.cols(), col3, 0, 0, Fi, 0, 2);
    //std::cout << "Fi: \n" << Fi << std::endl;

    S = Fi*G*inv(Fi);
    //std::cout << "S: \n" << S << std::endl;
}


double compute_distance(const alglib::real_2d_array &T,
                        const alglib::real_2d_array &X,
                        const alglib::real_2d_array &Y,
                        const alglib::real_2d_array &Z)
{
    auto dis = (T(0,0)*X + T(1,0)*Y + T(2,0)*Z)/vnorm(T);
    return max(dis) - min(dis);
}

double flatness_lin(const alglib::real_2d_array &M)
    throw (AlglibException)
{
    // The size of the data sample
    int N = M.rows();

    /*
     * Compute and diagonalize the covariance matrix to get a first estimate
     * of the data plane.
     */
    alglib::real_2d_array CoV;
    alglib::covm(M, CoV);
    //std::cout << "CoV\n" << CoV << std::endl;

    alglib::real_1d_array wr, wi;
    alglib::real_2d_array VR, VL;
    if (!alglib::rmatrixevd(CoV, CoV.cols(), 1, wr, wi, VL, VR))
    {
        std::cout << "Cannot diagonalize the CoVM" << std::endl;
        throw AlglibException("Cannot diagonalize the Covariance Matrix");
    }
    //std::cout << "Eigenvalues:  " << wr.tostring(5) << std::endl;
    //std::cout << "EigenVectors:\n " << VR << std::endl;

    alglib::real_2d_array T_0;
    int iemax = -1;
    double eigmin = std::numeric_limits<double>::max();
    for (int i=0; i<3; i++)
    {
        if (wr(i)<eigmin)
        {
            iemax = i;
            eigmin = wr(i);
        }
    }
    T_0 = get_column(iemax, VR, true);
    //std::cout << "T_0 \n" << T_0 << std::endl;

    /*
     * Create the data vectors
     */
    alglib::real_2d_array X = get_column(0, M);
    alglib::real_2d_array Y = get_column(1, M);
    alglib::real_2d_array Z = get_column(2, M);

    /*
     * Compute the initial values of the matrices
     */
    alglib::real_2d_array S, G, Fi, FiInv, cTU;
    alglib::real_2d_array U = ones(3, 1) * (1.0/sqrt(3.0));
    compute_G_and_Fi(U, T_0, G, Fi, S);

    /*
     * Set up stuff for the solver
     */
    double _vf[] = {-1., -1., -1., 0., 0.};
    alglib::real_2d_array vf;
    vf.setcontent(1, 5, _vf);
    alglib::real_2d_array vones = ones(N, 1);
    alglib::real_2d_array vzero = zeros(N, 1);

    alglib::real_1d_array AL = real_1d_array_with_value(2*N+1, alglib::fp_neginf);
    alglib::real_1d_array AU = real_1d_array_with_value(2*N+1, 0.0);
    AU[2*N] = -1;

    alglib::real_1d_array C = zeros(5);
    C[3]=-1.0;
    C[4]=1.0;
    //std::cout << "C: " << C.tostring(3) << std::endl;

    alglib::real_1d_array scale = real_1d_array_with_value(5, 1);
    alglib::real_1d_array bndl  = zeros(5);
    bndl[3] = alglib::fp_neginf;
    bndl[4] = alglib::fp_neginf;
    alglib::real_1d_array bndu  = real_1d_array_with_value(5, alglib::fp_posinf);

    /*
     * Prepare for the iteration
     */
    double error = 1.0e-12;
    double d_0 = compute_distance(T_0, X, Y, Z);
    alglib::real_2d_array T_p = zeros(3, 1);
    while (true)
    {
        alglib::real_2d_array A, SM, SMt, SMtn;
        SM.setlength(M.cols(), M.rows());
        SMt.setlength(M.rows(), M.cols());
        SMtn.setlength(M.rows(), M.cols());

        alglib::rmatrixgemm(S.rows(), M.rows(), M.cols(), 1.0, S, 0, 0, 0, M, 0, 0, 1, 0.0, SM, 0, 0);
        alglib::rmatrixtranspose(SM.rows(), SM.cols(), SM, 0, 0, SMt, 0, 0);
        alglib::rmatrixtranspose(SM.rows(), SM.cols(), -SM, 0, 0, SMtn, 0, 0);

        A.setlength(2*N+1, 5);
        alglib::rmatrixcopy(N, 3, SMtn , 0, 0, A, 0, 0);
        alglib::rmatrixcopy(N, 1, vones, 0, 0, A, 0, 3);
        alglib::rmatrixcopy(N, 1, vzero, 0, 0, A, 0, 4);
        alglib::rmatrixcopy(N, 3, SMt  , 0, 0, A, N, 0);
        alglib::rmatrixcopy(N, 1, vzero, 0, 0, A, N, 3);
        alglib::rmatrixcopy(N, 1,-vones, 0, 0, A, N, 4);
        alglib::rmatrixcopy(1, 5, vf   , 0, 0, A, 2*N, 0);
        //std::cout << "A\n" << A << std::endl;


        alglib::minlpstate state;
        alglib::minlpreport rep;
        alglib::real_1d_array x;

        alglib::minlpcreate(5, state);
        alglib::minlpsetcost(state, C);
        alglib::minlpsetbc(state, bndl, bndu);
        alglib::minlpsetlc2dense(state, A, AL, AU);
        alglib::minlpsetscale(state, scale);
        alglib::minlpoptimize(state);
        alglib::minlpresults(state, x, rep);
        //std::cout << "Result: " << x.tostring(3) << std::endl;

        for (int i=0; i<3; ++i)
            T_p[i][0] = x[i];

        auto T = (inv(S) * T_p)/vnorm(T_p);
        //std::cout << "T: [" << vnorm(T) << "]\n" << T << std::endl;

        double d = compute_distance(T, X, Y, Z);
        double e = fabs(d-d_0);
        //std::cout << "EEE: " << e << " Flatness " << d_0 << std::endl;
        if (e<error)
            break;

        d_0 = d;
        compute_G_and_Fi(U, T_0, G, Fi, S);
    }
    //std::cout << "Flatness: " << d_0 << std::endl;
    return d_0;
}
