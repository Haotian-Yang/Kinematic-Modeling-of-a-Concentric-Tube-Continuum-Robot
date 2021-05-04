#include <robot_independent.h>

Eigen::MatrixXd bishop_frame(double k, double phi, double s)
{
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> p;
    double ks = k*s;
    if(k == 0)
    {
        p << 0,0,s;
    }
    else{
        p << cos(phi)*(1-cos(ks))/k,sin(phi)*(1-cos(ks))/k,sin(ks)/k;
    }
    R << pow(cos(phi), 2)*(cos(ks)-1) + 1, sin(phi)*cos(phi)*(cos(ks) - 1), cos(phi)*sin(ks),
            sin(phi)*cos(phi)*(cos(ks)-1), pow(cos(phi), 2)*(1-cos(ks))+cos(ks), sin(phi)*sin(ks),
            -cos(phi)*sin(ks), -sin(phi)*sin(ks), cos(ks);

    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = p;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;

}

Eigen::MatrixXd frenet_frame(double k, double phi, double s)
{
    Eigen::Matrix4d T;
    double ks = k*s;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix3d R;
    if(k == 0)
    {
        p << 0,0,s;
    }
    else{
        p << cos(phi)*(1-cos(ks))/k, sin(phi)*(1-cos(ks))/k, sin(ks)/k;

    }
    R << cos(phi)*cos(ks), -sin(phi), cos(phi)*sin(ks),
            sin(phi)*cos(ks), cos(phi), sin(phi)*sin(ks),
            -sin(ks), 0, cos(ks);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = p;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;


}


// This function should implement the robot independent mapping (i.e. mapping arc parameters in configuration space to a series of discrete frames in task space)
// Inputs:
// init_frame           4x4 Matrix, specifying the initial frame of the curve
// kappa                m-dimensional vector, storing the curvature of each segment of the curve
// l                    m-dimensional vector, storing the length of each segment of the curve
// phi                  m-dimensional vector, storing the angle of the bending plane of each segment of the curve
// n                    number of requested frames to be returned for each segment (equally distributed along the circular arc)
// bishop               boolean value, specifying whether the material frame of the curve should be maintained or not (meaning a constant local rotation around the z-axis)
//
// Output (return):
//
// Eigen::MatrixXd      4x4(m*n+1) dimensional matrix storing all of the returned frames along the curve (stacked 4x4 blocks). First, leftmost 4x4 block should store the initial frame (init_frame).
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop)
{


    int m = l.size(); // number of segments
    Eigen::MatrixXd result(4, 4*(m*n+1));
    result.setZero();
    result.block(0, 0, 4, 4) = init_frame;
    Eigen::Matrix4d segment_init_frame = init_frame;
    //int count = 1;

    // loop through segments
    int num_T = 1;
    for(int i=1; i<=m; i++)
    {
        double kappa_i = kappa.at(i-1);
        double phi_i = phi.at(i-1);
        double l_i = l.at(i-1)/(float)n;


        Eigen::Matrix4d curr_frame;
        // loop through frames on each segment
        for(int j=1; j<=n; j++)
        {

            if(bishop)
            {
                curr_frame = bishop_frame(kappa_i, phi_i, l_i*j);
            }
            else
            {
                curr_frame = frenet_frame(kappa_i, phi_i, l_i*j);

            }
            curr_frame = segment_init_frame*curr_frame;
            //result.block(0, 4*(num_T + j - 1), 4, 4) << curr_frame;
            result.block(0, 4*num_T, 4, 4) << curr_frame;


            //if(j == n)
            //{
            //    //num_T += n; // count how many T(4x4) in the result
            //}
            ++num_T;
        }
        segment_init_frame = curr_frame; // set the end of the previous frame as the initial frame
    }


    return result;
}



