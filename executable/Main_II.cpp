// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com

#include <iostream>
#include <memory>
#include "CTR.hpp"
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <unistd.h>
#include "Robot.hpp"

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &trajectoryName);

std::vector<double> q2qRobot(const blaze::StaticVector<double, 6UL> &input);

blaze::StaticVector<double, 6UL> qRobot2q(const std::vector<double> &input);

int main()
{
    // dump files for CTR calibration
    std::ofstream EM_Trajectory, Joint_Positions;
    // If the file already exists, its previous content is deleted and replaced by the new one.
    EM_Trajectory.open("../../Output_Files/EM_Trajectory.dat", std::ios::out | std::ios::trunc);
    Joint_Positions.open("../../Output_Files/Joint_Values.dat", std::ios::out | std::ios::trunc);

    if (!EM_Trajectory.is_open())
    {
        std::cerr << "Failed opening the trajectory dump file!\n";
        return 0;
    }

    constexpr double inf = std::numeric_limits<double>::infinity();

    /** initialize the robot **/
    std::vector<double> max_acc = {1000.0 * M_PI / 180, 50.00E-3, 1000.0 * M_PI / 180, 50.00E-3}; // [deg/s^2] and [mm/s^2]
    std::vector<double> max_vel = {200.0 * M_PI / 180, 10.00E-3, 200.0 * M_PI / 180, 10.00E-3};   // [deg/s] and [mm/s]
    int sample_time = 50;                                                                         //[ms]
    int operation_mode = 1;
    bool position_limit = true;
    std::unique_ptr<CTRobot> rbt;
    rbt = std::make_unique<CTRobot>(sample_time,
                                    operation_mode,
                                    max_acc, max_vel,
                                    position_limit);
    rbt->Start_Thread();
    rbt->Enable_Operation(true);
    std::cout << "Robot enabled" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Waiting to get home!" << std::endl;
    rbt->Wait_until_reach();
    std::cout << "Homed!" << std::endl;

    //  # # # # # # # # ---- Properties of Nitinol Tubes ---- # # # # # # # #
    double E1 = 45.9446E9; // Young's modulus GPa - Tube 1
    double E2 = 178.312E9; // Young's modulus GPa - Tube 2
    double E3 = 200.000E9; // Young's modulus GPa - Tube 3
    double G1 = 25.3602E9; // Shear modulus GPa - Tube 1
    double G2 = 94.7906E9; // Shear modulus GPa - Tube 2
    double G3 = 100.000E9; // Shear modulus GPa - Tube 3

    // Precurvature radii for the tubes
    double R1 = 41.00E-3; // (4.1cm curvature radius)
    double R2 = 95.00E-3; // (9.5 cm curvature radius)
    double R3 = inf;      // (infinite curvature radius)

    // -- ** -- Precurvature vectors (for curved portions of the tubes) -- ** -- [u_x* u_y* 0]
    blaze::StaticVector<double, 3UL> u1 = {1.00 / R1, 0.00, 0.00};
    blaze::StaticVector<double, 3UL> u2 = {1.00 / R2, 0.00, 0.00};
    blaze::StaticVector<double, 3UL> u3 = {1.00 / R3, 0.00, 0.00};

    // --** --Lengths of the tubes' straight sections (meters) -- ** --
    blaze::StaticVector<double, 3UL> ls = {195.00E-3, 105.00E-3, 101.00E-3};

    // --** --Lengths of the tubes' curved sections (meters) -- ** --
    blaze::StaticVector<double, 3UL> lc = {58.00E-3, 78.00E-3, 0.00};

    // --** --Outer and Inner diameters of the tubes (meters)--** --
    blaze::StaticVector<double, 3UL> ID = {0.737E-3, 0.965E-3, 1.1448E-3};
    blaze::StaticVector<double, 3UL> OD = {0.940E-3, 1.372E-3, 2.045E-3};

    // # # # # # ---- Instantiating the three Tube objects ---- # # # # #
    std::shared_ptr<Tube> T1 = std::make_shared<Tube>(OD[0UL], ID[0UL], E1, G1, ls[0UL], lc[0UL], u1); // innermost tube
    std::shared_ptr<Tube> T2 = std::make_shared<Tube>(OD[1UL], ID[1UL], E2, G2, ls[1UL], lc[1UL], u2); // intermediate tube
    std::shared_ptr<Tube> T3 = std::make_shared<Tube>(OD[2UL], ID[2UL], E3, G3, ls[2UL], lc[2UL], u3); // outermost tube

    // instantiating an array of smart pointers to CTR component tubes
    std::array<std::shared_ptr<Tube>, 3UL> Tb = {T1, T2, T3};

    // initial joint actuation values "home position" - q = [Beta Alpha]
    blaze::StaticVector<double, 3UL> Beta_0 = {-152.00E-3, -82.00E-3, 0.00}; // -115, -100, -80
    blaze::StaticVector<double, 3UL> Alpha_0 = {0.00, 0.00, 0.00};

    blaze::StaticVector<double, 6UL> q_0;
    blaze::subvector<0UL, 3UL>(q_0) = Beta_0;
    blaze::subvector<3UL, 3UL>(q_0) = Alpha_0;

    // Determining the accuracy of BVP solutions
    double Tol = 1.00E-6;
    // Determining the accuracy of the position control loop
    double pos_tol = 1.00E-3;

    // Method for solving the BVP Problem
    // 1: Newton-Raphson
    // 2: Levenberg-Marquardt
    // 3: Powell's Dog-Leg
    // 4: Modified Newton-Raphson (globally convergent)
    // 5: Broyden
    // # # # # # ---- Instantiating the CTR object ---- # # # # #
    CTR CTR_robot(Tb, q_0, Tol, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON);

    // initial guess for the BVP
    blaze::StaticVector<double, 5UL> initGuess;

    // actuate the CTR to its home position
    CTR_robot.actuate_CTR(initGuess, q_0);

    // Trajectory to be tracked by the control loop ==> Either a Helix or a Hypocycloid
    blaze::HybridMatrix<double, 2000UL, 3UL> Trajectory;
    // speficy which trajectory to consider
    std::string trajectory("Helix");
    readFromCSV(Trajectory, trajectory);

    // position target for the CTR
    blaze::StaticVector<double, 3UL> target, position_FG, position_CTR, tipPos_Cosserat;

    const size_t numRows = Trajectory.rows();
    blaze::StaticVector<double, 6UL> q;

    // save the actual robot joint positions in the file
    std::vector<double> temp(4, 0.00);
    rbt->Get_Position(&temp);
    q = qRobot2q(temp);

    CTR_robot.setConfiguration(q);

    // lambda function that actuates the Maxon motors
    auto actuateMaxonMotors = [](blaze::StaticVector<double, 6UL> &q, const blaze::StaticVector<double, 6UL> &q_IK, std::unique_ptr<CTRobot>& rbt) -> void
    {
        std::vector<double> aux(4, 0.00), q_inRobot = q2qRobot(q_IK);

        // actuating the revolute joints with the shortest possible rotation
        q_inRobot[0UL] = mathOp::shortestRotation(q[3UL], q_inRobot[0UL]);
        q_inRobot[2UL] = mathOp::shortestRotation(q[4UL], q_inRobot[2UL]);
        q_inRobot[4UL] = mathOp::shortestRotation(q[5UL], q_inRobot[4UL]);

        rbt->Set_Target_AbsPosition(q_inRobot);

        std::cout << "Target sent to the robot:  "
                  << "R1: " << q_inRobot[0UL] * 180.00 * M_1_PI << " [deg]"
                  << "   "
                  << "T1: " << q_inRobot[1UL] * 1.00E3 << " [mm]"
                  << "   "
                  << "R2: " << q_inRobot[2UL] * 180.00 * M_1_PI << " [deg]"
                  << "   "
                  << "T2: " << q_inRobot[3UL] * 1.00E3 << " [mm]"
                  << "   " << std::endl;

        rbt->Wait_until_reach();

        // reads the actual robot joint positions from the controllers
        rbt->Get_Position(&aux);
        q = qRobot2q(aux);
    }

    std::chrono::milliseconds minimumSampleTime(50);

    double error = 100.00;

    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    size_t row = 0UL;
    double start_time, end_time, elapsed_time, current_time;

    elapsed_time = 0.00;
    end_time = Trajectory(numRows - 1UL, 0UL);
    start_time = std::chrono::high_resolution_clock::now();

    while ((elapsed_time < end_time) && (row < numRows - 1UL))
    {
        // updates the target based on trajectory time stamp
        if (elapsed_time >= Trajectory(row, 0UL))
        {
            target[0UL] = Trajectory(row, 1UL);
            target[1UL] = Trajectory(row, 2UL);
            target[2UL] = Trajectory(row, 3UL);
            row++;
        }

        // engages the position control
        CTR_robot.posCTRL(initGuess, target, pos_tol);

        // retrieves the converged configuration from the inverse kinematics
        q_0 = CTR_robot.getConfiguration();

        // actuates the motors
        actuateMaxonMotors(q, q_0, rbt);

        // Saving commanded joint values to file
        Joint_Positions << q[0UL] << ","
                        << q[1UL] << ","
                        << q[2UL] << ","
                        << q[3UL] << ","
                        << q[4UL] << ","
                        << q[5UL] << std::endl;

        // actuates the CTR object to the same configuration as read from the Faulhaber motor controllers
        CTR_robot.actuate_CTR(initGuess, q);

        // reading EM tip position -- after motor actuation
        std::tie(position_FG, position_CTR) = CTR_robot.acquireEMData();

        // updates the error for display
        error = blaze::norm(target - position_CTR);

        // writes (EM) tip position data to dump file
        EM_Trajectory << position_CTR[0UL] << ","
                      << position_CTR[1UL] << ","
                      << position_CTR[2UL] << std::endl;

        tipPos_Cosserat = CTR_robot.getTipPos();

        

        std::cout << "CTR Configuration: " << blaze::trans(CTR_robot.getConfiguration())
                  << "Model predc: " << blaze::trans(tipPos_Cosserat)
                  << "EM readings: " << blaze::trans(position_CTR)
                  << "Error norm: " << blaze::norm(target - position_CTR) * 1.00E3 << " [mm]" << std::endl
                  << "Row = " << row << std::endl
                  << "Elapsed time = " << elapsed_time << "[ms]\n"
                  << "--------------------------------------\n"
                  << std::endl;

        current_time = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    }

    rbt->Enable_Operation(false);
    // close the dump file
    EM_Trajectory.close();
    Joint_Positions.close();

    return 0;
}

std::vector<double> q2qRobot(const blaze::StaticVector<double, 6UL> &input)
{
    std::vector<double> offset = {152.00E-3, 82.00E-3, 0.00, 0.00, 0.00, 0.00}; // tran inner, tran middle, tran outer, rot inner, rot middle, rot outer
    std::vector<double> out = {
        input[3UL] + offset[3UL],
        input[0UL] + offset[0UL],
        input[4UL] + offset[4UL],
        input[1UL] + offset[1UL]};
    return out;
}

blaze::StaticVector<double, 6UL> qRobot2q(const std::vector<double> &input)
{
    std::vector<double> offset = {152.00E-3, 82.00E-3, 0.00, 0.00, 0.00, 0.00}; // tran inner, tran middle, tran outer, rot inner, rot middle, rot outer

    const blaze::StaticVector<double, 6UL> out = {
        (input[1] - offset[0]),
        (input[3] - offset[1]),
        0.00,
        (input[0] - offset[3]),
        (input[2] - offset[4]),
        0.00};

    return out;
}

// function that reads relevant clinical data from CSV files for each case
template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::string &trajectoryName)
{
    std::string filePath, file;

    // relative path to the folder containing clinical data
    filePath = "../../Trajectories/";
    file = trajectoryName + ".csv";

    // file from which the information will be read from
    std::ifstream CSV_file;
    std::cout << filePath + file << std::endl;
    CSV_file.open((filePath + file).c_str(), std::ifstream::in);
    if (!CSV_file.is_open())
    {
        std::cerr << "Error opening the CSV file within: " << __PRETTY_FUNCTION__ << "\n";
        return Mat = -1.00;
    }

    typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

    std::string line;

    size_t row = 0UL, col = 0UL;
    double value;

    while (std::getline(CSV_file, line))
    {
        Tokenizer tokenizer(line);
        col = 0UL;

        for (Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
        {
            value = std::stod(*it);
            Mat(row, col) = value;
            ++col;
        }
        ++row;
    }

    CSV_file.close();
    Mat.resize(row, col, true);

    return Mat;
}