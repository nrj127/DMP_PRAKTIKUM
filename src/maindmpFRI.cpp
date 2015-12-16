#include <ros/ros.h>
#include <ros/package.h>

#include <FastResearchInterface.h>
#include <FastResearchInterfaceTest.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <fstream>
#include <vector>
#include <OSAbstraction.h>

// includes for dmp project
#include "onlinegmr.h"
#include <ctime>
#include "camerahandle.h"

using namespace std;

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000
#define SIZE_OF_TRANSFER_STRING					32

#define MIN_STIFFNESS 0.01


int startCartImpedanceCtrl(FastResearchInterface *fri, float *commCartPose){
    unsigned int controlScheme = FastResearchInterface::CART_IMPEDANCE_CONTROL;
    int resultValue;
    if(fri->GetCurrentControlScheme() != controlScheme || !fri->IsMachineOK()){
        // Stop
        fri->StopRobot();

        fri->GetMeasuredCartPose(commCartPose);
        fri->SetCommandedCartPose(commCartPose);

        // Restart
        resultValue	= fri->StartRobot(controlScheme);
        if (resultValue != EOK){
            std::cout << "An error occurred during starting up the robot..." << std::endl;
            return -1;
        }
    }
    return 0;
}


void loadVectorMatrixFromFile (std::string fileName, int cols, vector < vector <float> > &outMat)
{
    ifstream in(fileName.data());
    if (!in)
    {
        cout << "No file found: " << fileName << endl;
        return;
    }
    int counter = 0;
    while (!in.eof())
    {
        outMat.push_back( vector <float>() );
        for (int j = 0; j < cols; ++j)
        {
            double readf;
            in >> readf;
            outMat[counter].push_back(readf);
        }
        counter++;
    }
    outMat.pop_back();
    in.close();
    return;
}


void saveVectorMatrixToFile (string fileName, vector < vector <float> > outMat)
{
    ofstream out(fileName.data());
    if (!out)
    {
        cout << "No file found: " << fileName << endl;
        return;
    }
    int rows = (int)outMat.size();
    int cols = (int)outMat[0].size();
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            out << outMat[i][j] << "\t";
        }
        out << endl;
    }
    out.close();
    return;
}


float getSquaredDistance(float a[3], float b[3]){
    return (a[0]-b[0])*(a[0]-b[0]) +
           (a[1]-b[1])*(a[1]-b[1]) +
           (a[2]-b[2])*(a[2]-b[2]) ;
}

//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{
    bool					Run							=	true
                        ,	StartRobotCalled			=	false;

    char					c							=	0
                        ,	d							=	0;

    unsigned int			ControlScheme				=	FastResearchInterface::JOINT_POSITION_CONTROL
                        ,	i							=	0
                        ,	LoopValue					=	4000;

    int						ResultValue					=	0;

    float JointStiffnessValues[LBR_MNJ],
          JointDampingValues[LBR_MNJ],
          CartStiffnessValues[FRI_CART_VEC],
          CartDampingValues[FRI_CART_VEC];

    float currentCartPose[FRI_CART_FRM_DIM], commCartPose[FRI_CART_FRM_DIM];
    float prevPosition[3], currPosition[3];

    std::vector< std::vector <float> >  demo_, motion_;
    for(i = 0; i<LoopValue; ++i){
        demo_.push_back( std::vector <float>() );
        for(int j=0; j<FRI_CART_FRM_DIM; ++j){
            demo_[i].push_back(0.0);
        }
    }

    FastResearchInterface	*FRI;

    fprintf(stdout, "You may need superuser permission to run this program.\n");
    fflush(stdout);

    std::string packPath = ros::package::getPath("dmp_praktikum");

    FRI = new FastResearchInterface((packPath + "/data/Control-FRI-Driver_5ms.init").c_str());
    fprintf(stdout, "OK-OK\n");
    fflush(stdout);

    std::string initJointFile    = packPath + "/data/InitAnglePos.txt";
    std::string genCartTrajFile  = packPath + "/data/GeneratedTrajectory.txt";
    std::string demoCartTrajFile = packPath + "/data/DemonstratedTrajectory.txt";

    for (i = 0; i < LBR_MNJ; i++){
        JointStiffnessValues	[i] =	(float)1000.0;
        JointDampingValues		[i]	=	(float)0.7;
    }

    for (i = 0; i < FRI_CART_VEC; i++){
        if(i<3)
            CartStiffnessValues[i] = (float)2000.0;
        else
            CartStiffnessValues[i] = (float)200.0;

        CartDampingValues		[i]	=	(float)0.7;
    }

    FRI->SetCommandedCartDamping(CartDampingValues);
    FRI->SetCommandedCartStiffness(CartStiffnessValues);
    FRI->SetCommandedJointDamping(JointDampingValues);
    FRI->SetCommandedJointStiffness(JointStiffnessValues);

    while (Run)
    {
        printf("---------------------------------------------------------------------------------------\n");
        printf("Press     q  for exit this program\n");
        printf("          s  for starting the KUKA Fast Research Interface\n");
        printf("          x  for stopping the KUKA Fast Research Interface\n");
        printf("          h  for start the joint position controller and go to home position\n");
        printf("          e  for executing dmp motion\n");
        printf("          g  for the gravity compensation mode\n");
        printf("          r  for reproducing demonstrated motion\n");
        printf("---------------------------------------------------------------------------------------\n\n");
        printf("Please press any key...\n");

        c	=	WaitForKBCharacter(NULL);

        printf("\n\n\n");

        switch (c)
        {
        case 'q':
        case 'Q':
            Run	=	false;
            break;
        case 's':
        case 'S':
            printf("Starting the robot through the FRI...\n");
            printf("Please select one of the following control strategies:\n\n");
            printf(" 1: Joint position control\n");
            printf(" 2: Cartesian impedance control\n");
            printf(" 3: Joint impedance control\n");
            printf(" 9: Joint torque control\n");
            printf(" a: Abort\n\n");
            d	=	0;
            while ( (d != '1') && (d != '2') && (d != '3') && (d != '9') && (d != 'a') && (d != 'A')){
                d	=	WaitForKBCharacter(NULL);
                printf("%c\n", c);
            }
            if ( (d == 'a') || (d == 'A')){
                printf("Control strategy remains unchanged.\n");
                break;
            }

            switch (d)
            {
            case '1':
                ControlScheme	=	FastResearchInterface::JOINT_POSITION_CONTROL;
                printf("Control strategy set to joint position control.\n");
                break;
            case '2':
                ControlScheme	=	FastResearchInterface::CART_IMPEDANCE_CONTROL;
                printf("Control strategy set to Cartesian impedance control.\n");
                break;
            case '3':
                ControlScheme	=	FastResearchInterface::JOINT_IMPEDANCE_CONTROL;
                printf("Control strategy set to joint impedance control.\n");
                break;
            }

            ResultValue	=	FRI->StartRobot(ControlScheme);

            if (ResultValue != EOK){
                printf("An error occurred during starting up the robot...\n");
            }
            else{
                StartRobotCalled	=	true;
            }
            break;
        case 'x':
        case 'X':
            printf("Stopping the FRI...\n");
            ResultValue	=	FRI->StopRobot();
            StartRobotCalled	=	false;

            if (ResultValue != EOK){
                printf("An error occurred during stopping the robot...\n");
            }
            break;

        case 'h':
        case 'H':
            printf("Going to home position... (please wait)\n");
            RunJointTrajectory(FRI, initJointFile);
            printf("Motion completed.\n");
            break;

        case 'e':
        case 'E':
            printf("Executing DMP motion... (please wait)\n");
            if(startCartImpedanceCtrl(FRI, currentCartPose)==0){
                // Set stiffness
                for (i = 0; i < FRI_CART_VEC; i++){
                    if(i<3)
                        CartStiffnessValues[i] = (float)2000.0;
                    else
                        CartStiffnessValues[i] = (float)200.0;
                }
                FRI->SetCommandedCartStiffness(CartStiffnessValues);

                // Clear vector
                if(motion_.size()>0){
                    for(i = 0; i<(int)motion_.size(); ++i){
                        for(int j=0; j<FRI_CART_FRM_DIM; ++j){
                            motion_[i].pop_back();
                        }
                        motion_[i].clear();
                    }
                    motion_.clear();
                }

                loadVectorMatrixFromFile(genCartTrajFile, FRI_CART_FRM_DIM, motion_);
                // Execute motion
                int it_  = 0;
                int dim_ = (int)motion_.size();

                // TODO implementation -----------------------------------------

                ros::init(argc, argv, "dmp_listener");
                ros::NodeHandle n;

                // object to handle camera topic callback
                //CameraHandle Camera;

                //ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1000, &CameraHandle::callback, &Camera);

                const char* inputFile = "../data/ModelDMPGaussBetaManyData.mat";
                const char* outputFile = "../data/TestOnlineGmrOutput.mat";

                onlineGMR gmr = onlineGMR(inputFile, outputFile);
                gmr.readMatlabFile();

                // create vector for 3 DMPs
                vector<double> F(3);
                // create dummy vector for X_in
                vector<double> X_in(3);
                // create vector for object positions
                vector<double> obj_pos;

                while ((FRI->IsMachineOK()) && it_<dim_){
                    FRI->WaitForKRCTick();

                    // start time measurement
                    clock_t begin = clock();

                    // TODO implement dmp and GMR here -----------------------------

                    //obj_pos = Camera.getPos();

                    F = gmr.regression(X_in);

                    for(i=0; i<FRI_CART_FRM_DIM; ++i){
                        commCartPose[i] = motion_[it_][i];
                    }
                    FRI->SetCommandedCartPose(commCartPose);
                    it_++;

                    ros::spinOnce();

                    // finish time measurement
                    clock_t end = clock();
                    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                    // cout << "Elapsed time for regression function (in ms) : " << elapsed_secs * 1000.0 << endl;
                }
            }
            printf("Motion completed.\n");
            break;

        case 'g':
        case 'G':
            printf("Gravity compansation... (please wait)\n");
            //slepp command
            if(startCartImpedanceCtrl(FRI, currentCartPose)==0){
                // Set stiffness
                for(i = 0; i < FRI_CART_VEC; i++){
                    CartStiffnessValues[i] = (float)MIN_STIFFNESS; // max stiffness 0-2 -> 2000.0, max 3-5 200.0
                }
                CartStiffnessValues[2] = 2000.0;
                CartStiffnessValues[4] = 200.0;
                CartStiffnessValues[5] = 200.0;

                FRI->SetCommandedCartStiffness(CartStiffnessValues);
                LoopValue = 4000;
                int it_ = 0;
                bool exit_  = false;
                //int switch_ = 0;
                while ((FRI->IsMachineOK()) && LoopValue>1 && !exit_){
                    // Feedback current pose to avoid
                    FRI->WaitForKRCTick();
                    FRI->GetMeasuredCartPose(currentCartPose);
                    FRI->SetCommandedCartPose(currentCartPose);

                    // Store data
                    for(i=0; i<FRI_CART_FRM_DIM; ++i){
                        demo_[it_][i] = currentCartPose[i];

                        if(i%100==0){
                            rowvec a(12);
                            cout << 'demo_[it_].size()' <<  demo_[it_].size() << endl;
                            a = utility::cvec2armadilloRowVec(demo_[it_]);
                            cout << 'current end.frame: (12 values)' <<  a << endl;
                        }
                    }
                    // Check demonstration finished
                    /*if(LoopValue<2000){
                        currPosition[0] = currentCartPose[3];
                        currPosition[1] = currentCartPose[7];
                        currPosition[2] = currentCartPose[11];

                        if(getSquaredDistance(prevPosition, currPosition)>0.001*0.001 && switch_==0){
                            switch_ = 1;
                            // Store data
                            for(i=0; i<FRI_CART_FRM_DIM; ++i){
                                demo_[it_][i] = currentCartPose[i];

                                it_++;
                            }
                        }
                        //else if(getSquaredDistance(prevPosition, currPosition)<0.001*0.001 && switch_==1){
                        //    exit_ = true;
                        //}
                    }

                    prevPosition[0] = currentCartPose[3];
                    prevPosition[1] = currentCartPose[7];
                    prevPosition[2] = currentCartPose[11];*/

                    it_++;
                    LoopValue--;
                }
            }
            printf("Motion completed.\n");
            saveVectorMatrixToFile(demoCartTrajFile, demo_);
            break;

        case 'r':
        case 'R':
            printf("Repeat demonstrated motion... (please wait)\n");
            if(startCartImpedanceCtrl(FRI, currentCartPose)==0){
                // Set stiffness
                for (i = 0; i < FRI_CART_VEC; i++){
                    if(i<3)
                        CartStiffnessValues[i] = (float)2000.0;
                    else
                        CartStiffnessValues[i] = (float)200.0;
                }
                FRI->SetCommandedCartStiffness(CartStiffnessValues);
                // Execute motion
                int dim_ = (int)demo_.size();
                int it_ = 0;
                while ((FRI->IsMachineOK()) && it_<dim_){
                    FRI->WaitForKRCTick();
                    // Store data
                    for(i=0; i<FRI_CART_FRM_DIM; ++i){
                        currentCartPose[i] = demo_[it_][i];
                    }

                    FRI->SetCommandedCartPose(currentCartPose);

                    it_++;
                }
            }
            printf("Motion completed.\n");
            break;

        default:
            printf("This key is not supported yet...\n");
            break;
        }
    }

    delete FRI;

    sleep(5);

    printf("\nGood bye.\n\n");

    return(EXIT_SUCCESS);
}


