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
Eigen::Vector2d Manipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
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
amp::ManipulatorState Manipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
        // Extract end effector coordinates and link lengths
    double x = end_effector_location[0];
    double y = end_effector_location[1];
    std::vector<double> linkLengths = getLinkLengths();

        // Define empty joint angles state
    amp::ManipulatorState joint_angles(nLinks());
    joint_angles.setZero();

    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
            // Theta 2 from x^2 + y^2
        double cos2 = (pow(x,2) + pow(y,2) - pow(linkLengths[0],2) - pow(linkLengths[1],2))/(2*linkLengths[0]*linkLengths[1]);
        double sin2 = sqrt(1-pow(cos2,2));

            // Theta 1 from forward kinematics
        double cos1 = (1/(pow(linkLengths[0],2) + pow(linkLengths[1],2)))*(x*(linkLengths[1]*cos2 + linkLengths[0]) + y*(linkLengths[1]*sin2));
        double sin1 = (1/(pow(linkLengths[0],2) + pow(linkLengths[1],2)))*(y*(linkLengths[1]*cos2 + linkLengths[0]) - x*(linkLengths[1]*sin2));

            // Assign joint angles
        joint_angles[0] = atan2(sin1,cos1);
        joint_angles[1] = atan2(sin2,cos2);

        return joint_angles;
    } else if (nLinks() == 3) {
            // Do a cos(theta1) sweep from -1 to 1 until we get a valid angle
        for (double cos1 = -1; cos1 <= 1; cos1 += 0.001)
        {
                // Calculate sin(theta1) based on current cos(theta1) guess, assuming positive sin(theta1)
            double sin1 = sqrt(1-pow(cos1,2));

                // Calculate xPrime and yPrime from given x and y - x and y in link 2's frame
            double x_translate = x - linkLengths[0]*cos1;
            double y_translate = y - linkLengths[0]*sin1;

            double xPrime = x_translate*cos1 + y_translate*sin1;// - linkLengths[0];//+ linkLengths[0]*cos1;
            double yPrime = -x_translate*sin1 + y_translate*cos1;// - linkLengths[0]*sin1;

                // Calculate cos(theta3) based on cos(theta1) guess
            double cos3 = (pow(xPrime,2) + pow(yPrime,2) - pow(linkLengths[1],2) - pow(linkLengths[2],2))/(2*linkLengths[1]*linkLengths[2]);
            if (abs(cos3) > 1.0)
            {
                continue;
            }

            double sin3 = sqrt(1-pow(cos3,2));

                // Calculate cos(theta2) based on cos(theta1) guess
            double cos2 = (1/(pow(linkLengths[1],2) + pow(linkLengths[2],2)))*(xPrime*(linkLengths[2]*cos3 + linkLengths[1]) + yPrime*(linkLengths[2]*sin3));
            double sin2 = (1/(pow(linkLengths[1],2) + pow(linkLengths[2],2)))*(yPrime*(linkLengths[2]*cos3 + linkLengths[1]) - xPrime*(linkLengths[2]*sin3));

                // Calculate candidate angles in radians
            double theta1 = atan2(sin1,cos1);
            double theta2 = atan2(sin2,cos2);
            double theta3 = atan2(sin3,cos3);

                // Check if candidate state gets sufficiently close to the requested point via forward kinematics. If so, good solution and break out of sweep
            amp::ManipulatorState testState(nLinks());
            testState << theta1, theta2, theta3;
            Eigen::Vector2d checkPosition = getJointLocation(testState, nLinks());

            Eigen::Vector2d diff = end_effector_location - checkPosition;
            if (diff.norm() < 1e-10)
            {
                joint_angles[0] = theta1;
                joint_angles[1] = theta2;
                joint_angles[2] = theta3;
                break;
            }
        }

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}

