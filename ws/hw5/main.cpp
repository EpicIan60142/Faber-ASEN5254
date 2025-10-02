// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Vector field plotting resolution
    int n_grid = 100;

    // Test your gradient descent algorithm.
        // Algorithm termination radius
    double epsilon = 0.25;

        // Algorithm tuning knobs
    double d_star = 0.5; double zeta = 1; double eta = 1;
    std::vector<double> Q_star;

        // HW 5 Workspace 1
            // Define problem
    Problem2D probWS1 = HW5::getWorkspace1();
            // Assign Qstars
    for (int i = 0; i<probWS1.obstacles.size(); i++)
    {
        Q_star.push_back(0.5);
    }
            // Make algorithm
    MyGDAlgorithm algoWS1(d_star, zeta, Q_star, eta, epsilon);
            // Generate path
    std::vector<Eigen::Vector2d> collision_points;
    Path2D pathWS1 = algoWS1.plan(probWS1);
            // Check solution
    bool WS1success = HW5::check(pathWS1, probWS1, collision_points);
            // Make figures
    //Visualizer::makeFigure(probWS1, pathWS1, collision_points);
    //Visualizer::makeFigure(MyPotentialFunction{algoWS1, probWS1}, probWS1, n_grid);

        // HW 2 Workspace 1
    Problem2D probWS2 = HW2::getWorkspace1();
    Q_star.clear();
    for (int i = 0; i<probWS2.obstacles.size(); i++)
    {
        Q_star.push_back(0.25);
    }
    d_star = 0.5; zeta = 1; eta = 0.1;
    MyGDAlgorithm algoWS2(d_star, zeta, Q_star, eta, epsilon);
    collision_points.clear();
    Path2D pathWS2 = algoWS2.plan(probWS2);
    bool WS2success = HW5::check(pathWS2, probWS2, collision_points);
    //Visualizer::makeFigure(probWS2, pathWS2, collision_points);
    //Visualizer::makeFigure(MyPotentialFunction{algoWS2, probWS2}, probWS2, n_grid);

        // HW 2 Workspace 2
    Problem2D probWS3 = HW2::getWorkspace2();
    Q_star.clear();
    for (int i = 0; i<probWS3.obstacles.size(); i++)
    {
        Q_star.push_back(0.25);
    }
    MyGDAlgorithm algoWS3(d_star, zeta, Q_star, eta, epsilon);
    collision_points.clear();
    Path2D pathWS3 = algoWS3.plan(probWS3);
    bool WS3success = HW5::check(pathWS3, probWS3, collision_points);
    //Visualizer::makeFigure(probWS3, pathWS3, collision_points);
    //Visualizer::makeFigure(MyPotentialFunction{algoWS3, probWS3}, probWS3, n_grid);

        // Random problem
    MyGDAlgorithm algo(d_star, zeta, Q_star, eta, epsilon);
    collision_points.clear();
    Path2D path;
    Problem2D prob;
    bool success = HW5::generateAndCheck(algo, path, prob, collision_points);
    //Visualizer::makeFigure(prob, path);
    //Visualizer::makeFigure(MyPotentialFunction{algo, prob}, prob, n_grid);

        // Save figures
    //Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    Q_star.clear();
    HW5::grade<MyGDAlgorithm>("Ian.Faber@colorado.edu", argc, argv, d_star, zeta, Q_star, eta, epsilon);
    return 0;
}