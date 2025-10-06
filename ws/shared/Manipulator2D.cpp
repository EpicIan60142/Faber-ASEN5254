//
// Created by ianmf on 9/23/25.
//

#include "Manipulator2D.h"

#include "Eigen/Dense"

Manipulator2D::Manipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

Manipulator2D::Manipulator2D(const std::vector<double>& link_lengths)
    : LinkManipulator2D(link_lengths)
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d Manipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const
{
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
        // Vector to store joint positions
    std::vector<Eigen::Vector2d> joint_positions;

        // Extract base position
    Eigen::Vector2d baseLocation = getBaseLocation();
    std::vector<double> linkLengths = getLinkLengths();

        // Loop through number of joints in the manipulator to get transforms
    std::vector<Eigen::Matrix3d> transforms;
    for (int i = 0; i <= nLinks(); i++)
    {
            // Placeholder transform matrix
        Eigen::Matrix3d transform;

            // Base joint
        if (i == 0)
        {
            transform << cos(state[i]), -sin(state[i]), baseLocation[0], sin(state[i]), cos(state[i]), baseLocation[1], 0, 0, 1.0;
        }
            // End effector
        else if (i == nLinks())
        {
            transform << 1.0, 0, linkLengths[i-1], 0, 1.0, 0, 0, 0, 1.0;
        }
        else
        {
            transform << cos(state[i]), -sin(state[i]), linkLengths[i-1], sin(state[i]), cos(state[i]), 0, 0, 0, 1.0;
        }
            // Add transform to vector of transforms
        transforms.push_back(transform);
    }

        // Assign joint positions
    Eigen::Vector2d intJoint; // Intermediate joint position
    for (int i = 0; i <= nLinks(); i++)
    {
        Eigen::Vector3d temp = {0.0, 0.0, 1.0}; // Temporary transformation vector
            // Construct overall transform
        Eigen::Matrix3d transform;
        transform.setIdentity();
        for (int ii = 0; ii <= i; ii++)
        {
            transform *= transforms[ii];
        }
        temp = transform*temp;
        joint_positions.push_back({temp[0], temp[1]});
    }

    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState Manipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const
{
    // Implement inverse kinematics here
        // Extract end effector coordinates and link lengths
    double x = end_effector_location[0];
    double y = end_effector_location[1];
    std::vector<double> linkLengths = getLinkLengths();

        // Define empty joint angles state
    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();

    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2)
    {
            // Theta 2 from x^2 + y^2
        double cos2 = (pow(x,2) + pow(y,2) - pow(linkLengths[0],2) - pow(linkLengths[1],2))/(2*linkLengths[0]*linkLengths[1]);
        double sin2 = sqrt(1-pow(cos2,2));

            // Theta 1 from forward kinematics
        double cos1 = (1/(pow(x,2) + pow(y,2)))*(x*(linkLengths[1]*cos2 + linkLengths[0]) + y*(linkLengths[1]*sin2));
        double sin1 = (1/(pow(x,2) + pow(y,2)))*(y*(linkLengths[1]*cos2 + linkLengths[0]) - x*(linkLengths[1]*sin2));

            // Assign joint angles
        joint_angles[0] = atan2(sin1,cos1);
        joint_angles[1] = atan2(sin2,cos2);

            // Make sure angles are between 0 and 2pi
        if (joint_angles[0] < 0)
        {
            joint_angles[0] += 2*M_PI;
        }

        if (joint_angles[1] < 0)
        {
            joint_angles[1] += 2*M_PI;
        }

        return joint_angles;
    }
    else if (nLinks() == 3)
    {
        double bestError = std::numeric_limits<double>::max();

        // Sweep through possible values of theta1
        for (double cos1 = -1; cos1 <= 1; cos1 += 0.001)
        {
                // Calculate sin(theta1) from cos(theta1)
            double sin1_base = sqrt(1-pow(cos1,2));

                // Try both positive and negative sin(theta1)
            for (int sign1 = 1; sign1 >= -1; sign1 -= 2)
            {
                    // Apply sign, positive first
                double sin1 = sign1*sin1_base;

                    // Calculate xPrime and yPrime by translating x and y to link 2's frame
                double xTranslate = x - linkLengths[0] * cos1;
                double yTranslate = y - linkLengths[0] * sin1;

                        // Rotation matrix about theta1 transposed, we are going to local frame from global frame
                double xPrime = xTranslate * cos1 + yTranslate * sin1; // Reduces to xcos1 + ysin1 + a1
                double yPrime = -xTranslate * sin1 + yTranslate * cos1; // Reduces to -xsin1 + ycos1

                    // Calculate cos(theta3) from xPrime and yPrime
                double cos3 = (pow(xPrime,2) + pow(yPrime,2) - pow(linkLengths[1],2) - pow(linkLengths[2],2))/(2*linkLengths[1]*linkLengths[2]);

                    // cos(theta3) is valid if it's between -1 and 1
                if (abs(cos3) > 1.0)
                {
                    continue;
                }

                    // Calculate sin(theta3) from cos(theta3)
                double sin3_base = sqrt(1-pow(cos3,2));

                    // Try both positive and negative sin(theta3)
                for (int sign3 = 1; sign3 >= -1; sign3 -= 2)
                {
                        // Apply sign, positive first
                    double sin3 = sign3*sin3_base;

                        // Calculate cos(theta2) and sin(theta2) from cos(theta3) and sin(theta3)
                    double cos2 = (1/(pow(xPrime,2) + pow(yPrime,2)))*(xPrime*(linkLengths[2]*cos3 + linkLengths[1]) + yPrime*(linkLengths[2]*sin3));
                    double sin2 = (1/(pow(xPrime,2) + pow(yPrime,2)))*(yPrime*(linkLengths[2]*cos3 + linkLengths[1]) - xPrime*(linkLengths[2]*sin3));

                        // Check that cos(theta2) and sin(theta2) are valid
                    if (abs(cos2) > 1.0 || abs(sin2) > 1.0)
                    {
                        continue;
                    }

                        // Calculate candidate angles
                    double theta1 = atan2(sin1, cos1);
                    double theta2 = atan2(sin2, cos2);
                    double theta3 = atan2(sin3, cos3);

                        // Test candidate state via FK on the end effector
                    amp::ManipulatorState testState(nLinks());
                    testState << theta1, theta2, theta3;
                    Eigen::Vector2d checkPosition = getJointLocation(testState, nLinks());

                    Eigen::Vector2d diff = end_effector_location - checkPosition;
                    double error = diff.norm();

                    // Keep track of the best solution found
                    if (error < bestError)
                    {
                        bestError = error;
                        joint_angles = testState;

                        // If solution is good enough, terminate function
                        if (error < 1e-6)
                        {
                            // Make sure angles are between 0 and 2pi
                            if (joint_angles[0] < 0)
                            {
                                joint_angles[0] += 2*M_PI;
                            }

                            if (joint_angles[1] < 0)
                            {
                                joint_angles[1] += 2*M_PI;
                            }

                            if (joint_angles[2] < 0)
                            {
                                joint_angles[2] += 2*M_PI;
                            }

                            return joint_angles;
                        }
                    }
                }
            }
        }

        // Make sure angles are between 0 and 2pi
        if (joint_angles[0] < 0)
        {
            joint_angles[0] += 2*M_PI;
        }

        if (joint_angles[1] < 0)
        {
            joint_angles[1] += 2*M_PI;
        }

        if (joint_angles[2] < 0)
        {
            joint_angles[2] += 2*M_PI;
        }

        // Return the best solution found, even if not perfect
        return joint_angles;
    }
    else
    {

        return joint_angles;
    }

    return joint_angles;
}
