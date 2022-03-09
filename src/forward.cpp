

#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Eigen>

using namespace ros;
using namespace std;
using namespace Eigen;

// Link lengths assumed as 1, except link 0 which is 0.5
// Height of base is 0.3
static const double a0 = 1;
static const int a1 = 1;
static const int a2 = 1;
static const int a3 = 1;
static const double h0 = 0.3;

class robot_forward
{
private:
    NodeHandle n;
    Subscriber sub_frwd;
    Publisher pub;
    
    // DH Table is hard coded into the algorithm, except joint angles which will be taken as user inputs
    //================ DH Table ==================
    // |    a   |   d   |   theta   |   alpha   |
    // ==========================================
    // |    0   |   h0  |   0       |    0      |
    // |    a1  |   a0  |   theta1  |    0      |
    // |    a2  |   0   |   theta2  |    PI     |
    // |    0   |   d3  |   0       |    0      |
    

public:
    robot_forward(/* args */)
    {
        sub_frwd = n.subscribe("/scara_robot/joint_states", 1000, &robot_forward::compute_frwd, this);
        pub = n.advertise<geometry_msgs::Pose>("/scara_robot/output_pose", 1000);
    }

    // void compute_frwd(const geometry_msgs::Vector3::ConstPtr& msg)
    void compute_frwd(const sensor_msgs::JointState& msg)
    {
        double q1 = msg.position[0];
        double q2 = msg.position[1];
        double d3 = msg.position[2];
        // cout << q1 << ", " << q2 << ", " << d3 << endl;

        double a[4] = {0, a1, a2, 0};
        double d[4] = {h0, a0, 0, d3};
        double theta[4] = {0, q1, q2, 0};
        double alpha[4] = {0, 0, (M_PI), 0};
        
        // Calculating A1, A2, A3
        double c_theta;
        double s_theta;
        double c_alpha;
        double s_alpha;

        c_theta = cos(theta[0]);
        s_theta = sin(theta[0]);
        c_alpha = cos(alpha[0]);
        s_alpha = sin(alpha[0]);

        Eigen::Matrix4d A_i;
        A_i << c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[0]*c_theta,
                s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[0]*s_theta,
                0, s_alpha, c_alpha, d[0],
                0, 0, 0, 1;
        // cout << "Eigen matrix: " << A_i << endl;

        // cout << c_theta << ", " << s_theta << ", " << c_alpha << ", " << s_alpha << endl;
        double A1[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[0]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[0]*s_theta},
        {0, s_alpha, c_alpha, d[0]},
        {0, 0, 0, 1}};

        c_theta = cos(theta[1]);
        s_theta = sin(theta[1]);
        c_alpha = cos(alpha[1]);
        s_alpha = sin(alpha[1]);

        // cout << c_theta << ", " << s_theta << ", " << c_alpha << ", " << s_alpha << endl;
        double A2[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[1]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[1]*s_theta},
        {0, s_alpha, c_alpha, d[1]},
        {0, 0, 0, 1}};

        c_theta = cos(theta[2]);
        s_theta = sin(theta[2]);
        c_alpha = cos(alpha[2]);
        s_alpha = sin(alpha[2]);

        // cout << c_theta << ", " << s_theta << ", " << c_alpha << ", " << s_alpha << endl;
        double A3[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[2]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[2]*s_theta},
        {0, s_alpha, c_alpha, d[2]},
        {0, 0, 0, 1}};

        c_theta = cos(theta[3]);
        s_theta = sin(theta[3]);
        c_alpha = cos(alpha[3]);
        s_alpha = sin(alpha[3]);

        // cout << c_theta << ", " << s_theta << ", " << c_alpha << ", " << s_alpha << endl;
        double A4[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[3]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[3]*s_theta},
        {0, s_alpha, c_alpha, d[3]},
        {0, 0, 0, 1}};
        
        // Multiply T2_0 = A1 * A2
        double T2_0[4][4], T3_0[4][4], T4_0[4][4];
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {    
                T2_0[i][j]=0;    
                for(int k=0;k<4;k++)    
                {    
                    T2_0[i][j]+=A1[i][k]*A2[k][j];
                }    
            }    
        }
        
        // Multiply T3_0 = T2_0 * A3
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {    
                T3_0[i][j]=0;    
                for(int k=0;k<4;k++)    
                {    
                    T3_0[i][j]+=T2_0[i][k]*A3[k][j];
                }    
            }    
        }

        // Multiply T4_0 = T3_0 * A4
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {    
                T4_0[i][j]=0;    
                for(int k=0;k<4;k++)    
                {    
                    T4_0[i][j]+=T3_0[i][k]*A4[k][j];
                }    
            }    
        }

        // // Display computation steps for A1, A2, A3, A4 and T4_0

        // cout << "======A1=======" << endl;
        // display_matrix(A1);
        
        // cout << "======A2=======" << endl;
        // display_matrix(A2);
        
        // cout << "======A3=======" << endl;
        // display_matrix(A3);

        // cout << "======A4=======" << endl;
        // display_matrix(A4);

        // cout << "\n\nForward Kinematics: The transformation matrix is: " << endl;
        // display_matrix(T4_0);

        geometry_msgs::Pose pose;
        pose.position.x = T4_0[0][3];
        pose.position.y = T4_0[1][3];
        pose.position.z = T4_0[2][3];

        tf::Matrix3x3 mat(1,0,0,0,1,0,0,0,1);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        tf::Quaternion quat;
        quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

        pose.orientation.x = quat.getX();
        pose.orientation.y = quat.getY();
        pose.orientation.z = quat.getZ();
        pose.orientation.w = quat.getW();

        // // Display Final Pose Values
        // cout << "Position:" << endl;
        // cout << pose.position.x << endl;
        // cout << pose.position.y << endl;
        // cout << pose.position.z << endl;
        // cout << "Quat:" << endl;
        // cout << pose.orientation.x << endl;
        // cout << pose.orientation.y << endl;
        // cout << pose.orientation.z << endl;
        // cout << pose.orientation.w << endl;

        pub.publish(pose);
    }

    void display_matrix(double mat[4][4]){
        // cout << "====== Display Matrix =======" << endl;
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {     
                cout<<mat[i][j]<<" ";    
            }    
            cout<<"\n";    
        }
    }
};

int main(int argc, char **argv)
{
    init(argc, argv, "forward_hw3");
    // ROS_INFO("Enter Joint Angle values... Topic Name: \n 1. /input/joints - for forward kinematics \n 2. /input/pose_msg - for inverse kinematics\n");
    ROS_INFO("============================ FORWARD KINEMATICS ====================================");
    ROS_INFO("Subsribe to the topic: /scara_robot/output_pose");

    // Calculate robot forward kinematics
    robot_forward robot_obj;
    spin();
    return 0;
}