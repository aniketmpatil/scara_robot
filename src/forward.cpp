

#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "tf/tf.h"

using namespace ros;
using namespace std;

//Link lengths assumed as 1
static const int l1 = 1;
static const int l2 = 1;
static const int l3 = 1;

class robot_forward
{
private:
    NodeHandle n;
    Subscriber sub_frwd;
    Subscriber sub_inv;
    Publisher pub;

    // DH Table is hard coded into the algorithm, except joint angles which will be taken as user inputs
    //=========================== DH Table ==============================
    // |    a   |   d   |   theta   |   alpha   |
    // ==========================================
    // |    0   |   l1  |   theta1  |   PI/2    |
    // |    l2  |   0   |   theta2  |    0      |
    // |    l3  |   0   |   theta3  |    0      |
    
    const float a[3] = {0, l2, l3};
    const float d[3] = {l1, 0, 0};
    float theta[3];
    const float alpha[3] = {(M_PI/2), 0, 0};

public:
    robot_forward(/* args */)
    {
        sub_frwd = n.subscribe("/scara_robot/joint_states", 1000, &robot_forward::compute_frwd, this);
        pub = n.advertise<geometry_msgs::Pose>("input/pose_msg", 1000);
    }

    // void compute_frwd(const geometry_msgs::Vector3::ConstPtr& msg)
    void compute_frwd(const sensor_msgs::JointState& msg)
    {
        theta[0] = msg.position[0];
        theta[1] = msg.position[1];
        theta[2] = msg.position[2];
        
        // Calculating A1, A2, A3
        double c_theta;
        double s_theta;
        double c_alpha;
        double s_alpha;

        c_theta = cos(theta[0]);
        s_theta = sin(theta[0]);
        c_alpha = cos(alpha[0]);
        s_alpha = sin(alpha[0]);

        double A1[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[0]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[0]*s_theta},
        {0, s_alpha, c_alpha, d[0]},
        {0, 0, 0, 1}};

        c_theta = cos(theta[1]);
        s_theta = sin(theta[1]);
        c_alpha = cos(alpha[1]);
        s_alpha = sin(alpha[1]);

        double A2[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[1]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[1]*s_theta},
        {0, s_alpha, c_alpha, d[1]},
        {0, 0, 0, 1}};

        c_theta = cos(theta[2]);
        s_theta = sin(theta[2]);
        c_alpha = cos(alpha[2]);
        s_alpha = sin(alpha[2]);

        double A3[4][4] = {{c_theta, -1*s_theta*c_alpha, s_theta*s_alpha, a[2]*c_theta},
        {s_theta, c_theta*c_alpha, -1*c_theta*s_alpha, a[2]*s_theta},
        {0, s_alpha, c_alpha, d[2]},
        {0, 0, 0, 1}};
        
        // Multiply temp = A1 * A2
        double temp[4][4], T3_0[4][4];
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {    
                temp[i][j]=0;    
                for(int k=0;k<4;k++)    
                {    
                    temp[i][j]+=A1[i][k]*A2[k][j];
                }    
            }    
        }
        
        // Multiply T3_0 = temp * A3
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {    
                T3_0[i][j]=0;    
                for(int k=0;k<4;k++)    
                {    
                    T3_0[i][j]+=temp[i][k]*A3[k][j];
                }    
            }    
        }

        // cout << "======A1=======" << endl;
        // for(int i=0;i<4;i++)    
        // {    
        //     for(int j=0;j<4;j++)    
        //     {     
        //         cout<<A1[i][j]<<" ";    
        //     }    
        //     cout<<"\n";    
        // }
        // cout << "======A2=======" << endl;
        // for(int i=0;i<4;i++)    
        // {    
        //     for(int j=0;j<4;j++)    
        //     {     
        //         cout<<A2[i][j]<<" ";    
        //     }    
        //     cout<<"\n";    
        // }
        // cout << "======A3=======" << endl;
        // for(int i=0;i<4;i++)    
        // {    
        //     for(int j=0;j<4;j++)    
        //     {     
        //         cout<<A3[i][j]<<" ";    
        //     }    
        //     cout<<"\n";    
        // }
        

        //Display Final Transformation matrix T3_0

        cout << "\n\nForward Kinematics: The transformation matrix is: " << endl;
        for(int i=0;i<4;i++)    
        {    
            for(int j=0;j<4;j++)    
            {     
                cout<<T3_0[i][j]<<" ";
            }    
            cout<<"\n\n";    
        }

        geometry_msgs::Pose pose;
        pose.position.x = T3_0[0][3];
        pose.position.y = T3_0[1][3];
        pose.position.z = T3_0[2][3];

        tf::Matrix3x3 mat(1,0,0,0,1,0,0,0,1);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        tf::Quaternion quat;
        quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

        pose.orientation.x = quat.getX();
        pose.orientation.y = quat.getY();
        pose.orientation.z = quat.getZ();
        pose.orientation.w = quat.getW();
        cout << "Quat:" << endl;
        cout << pose.orientation.x << endl;
        cout << pose.orientation.y << endl;
        cout << pose.orientation.z << endl;
        cout << pose.orientation.w << endl;

        pub.publish(pose);
    }
};

int main(int argc, char **argv)
{
    init(argc, argv, "forward_hw3");
    ROS_INFO("Enter Joint Angle values... Topic Name: \n 1. /input/joints - for forward kinematics \n 2. /input/pose_msg - for inverse kinematics\n");
    robot_forward robot;
    spin();
    return 0;
}