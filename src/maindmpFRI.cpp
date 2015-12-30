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
#include "dmp_integrator.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>

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
                        ,	LoopValue					=	2100; //4000;  //changed!!

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
                        CartStiffnessValues[i] = 300;//(float)2000.0;
                    else
                        CartStiffnessValues[i] = 300;//(float)200.0;
                }
                FRI->SetCommandedCartStiffness(CartStiffnessValues);


                printf("Please press any key to move to starting position\n");
                c	=	WaitForKBCharacter(NULL);

                printf("going to starting position now ...\n");
                float pose_array0[FRI_CART_FRM_DIM];
                double dstartingCartPose[FRI_CART_FRM_DIM];

                float goalCartPose[FRI_CART_FRM_DIM];
                double dgoalCartPose[FRI_CART_FRM_DIM];

                dmp_integrator integrator1;

                //getting inital frame
                FRI->GetMeasuredCartPose(pose_array0);

                mat rot_pose0 = zeros(3,3);
                vec trans0 = zeros(3);

                utility::array2mat_vec(rot_pose0, trans0, pose_array0, FRI_CART_FRM_DIM);

                cout << "rot_pose0" << endl;
                cout << rot_pose0 << endl;
                cout << "trans_pose0" << endl;
                cout << trans0 << endl;


                vec angles0=utility::rotationMatrix2eulerAngles(rot_pose0);
                cout << " rotation eulerangles0: " << endl;
                cout << angles0 << endl;

                double startz = 0.1;
                vec trans_start(3);
                trans_start(0) = integrator1.getx_0().at(0);
                trans_start(1) = integrator1.getx_0().at(1);
                trans_start(2) = startz;

                cout << "cout1" << endl;

                vec angles_start(3);
                angles_start(0) = -PI;
                angles_start(1) = 0;
                angles_start(2) = integrator1.getx_0().at(2);

                cout << "0-position:" << endl;
                for(int q=0; q<12 ; q++)
                    cout << pose_array0[q] << " " ;
                cout << endl;

                int n_inter = 300;

                for(int ni=0; ni< n_inter ; ni++)
                {                                       
                    vec des_x = trans0 + (trans_start-trans0)*((float) ni+1)/((float) n_inter);  //interpolation of translation
                    vec des_angles = angles0 + (angles_start-angles0)*((float) ni+1)/((float) n_inter);

                    mat des_rotmat = utility::eulerAngles2rotationMatrix(des_angles);
                    //cout << des_rotmat << endl;

                    float des_pose_array[FRI_CART_FRM_DIM];
                    utility::mat_vec2array(des_rotmat, des_x, des_pose_array);

                    //sleep(1);

                    cout << "interpolated - position:" << endl;
                    for(int q=0; q<12 ; q++)
                        cout << des_pose_array[q] << " " ;
                    cout << endl;

                    FRI->WaitForKRCTick();
                    if (FRI->IsMachineOK())
                        FRI->SetCommandedCartPose(des_pose_array);
                    else
                        cout << "error !" << endl;

                }

                sleep(2);

                printf("halt 1\n");
                c	=	WaitForKBCharacter(NULL);

                printf("starting frame reached\n");


                // Execute motion
                int it_  = 0;
                int dim_ = (int)motion_.size();

                // TODO implementation -----------------------------------------

                ros::init(argc, argv, "dmp_listener");
                ros::NodeHandle n;

                tf::StampedTransform transform;
                tf::TransformListener listener;

                bool exception_flag = false;

               do {
                    try{
                        // reading transform of marker for the first time
                        ros::Time now = ros::Time::now();
                        listener.waitForTransform("/ar_marker_1", "/robot", now, ros::Duration(2.0) );
                        ROS_INFO("Reading tf for the first time... ");
                        listener.lookupTransform("/ar_marker_1", "/robot", now, transform);
                        ROS_INFO("done");
                        exception_flag = false;
                    }
                    catch (tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(0.1).sleep();
                        exception_flag = true;
                    }
                } while (exception_flag == true);


               int display_counter = 0;
               int display_cycles = 100;
               clock_t elapsed_time = 0;




               tf::Vector3 pos = transform.getOrigin();
               vec taskParams(2);
               taskParams[0] = pos.getX();
               taskParams[1] = pos.getY();

               taskParams.print(cout,"task paramts");

               printf("Please press any key to start execution \n");
               c	=	WaitForKBCharacter(NULL);

               cout << "starting the dmp-loop" << endl;
               while ((FRI->IsMachineOK()) && it_< LoopValue  ){
                    FRI->WaitForKRCTick();

                    display_counter++;

                    // ---------------------------------------------------
                    // start time measurement
                    // clock_t begin = clock();
                    // ---------------------------------------------------

                    // get object pos from ROS

                    // transform from camera to robot coordinate system
                    ros::Time now = ros::Time::now();
                    //listener.waitForTransform("/ar_marker_1", "/robot", now, ros::Duration(2.0) );
                    if (display_counter%display_cycles == 0) ROS_INFO("Reading tf... ");
                    /*
                    try {
                        listener.lookupTransform("/ar_marker_1", "/robot", now - ros::Duration(0.18), transform);
                        if (display_counter%display_cycles == 0) ROS_INFO("done");
                    }
                    catch (tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                        ROS_ERROR("could not look up transform from marker 1");
                    }
                    */

                    tf::Vector3 pos = transform.getOrigin();
                    vec taskParams(2);
                    taskParams[0] = pos.getX();
                    taskParams[1] = pos.getY();
                    if (display_counter%display_cycles == 0) ROS_INFO("Pos: x: %f y: %f",  taskParams[0], taskParams[1]);

                    it_++;

                    vector <double> x_dmp = integrator1.integrate_onestep(taskParams);
                    float des_pose[12];

                    vec des_anlges(3);
                    des_anlges(0) = -PI;
                    des_anlges(1) = 0;
                    des_anlges(2) = x_dmp.at(2);

                    mat newrotmat = utility::eulerAngles2rotationMatrix(des_anlges);

                    vec des_trans(3);
                    des_trans(0) = x_dmp.at(0);
                    des_trans(1) = x_dmp.at(1);
                    des_trans(2) = startz;


                    utility::mat_vec2array(newrotmat, des_trans, des_pose);

//                    memcpy(des_pose, pose0, sizeof(float) * 12);
//                    des_pose[3] = x_dmp.at(0);
//                    des_pose[7] = x_dmp.at(1);
//                    des_pose[11] = startz;

                    FRI->SetCommandedCartPose(des_pose);

                    if((it_-1)%20==0){
                        cout << "calling integrator, step" << integrator1.iteration << endl;
                        cout <<"it_ "<< it_ << endl;
                        cout <<"dmp0  "<< x_dmp[0] << "   dmp1: "<< x_dmp[1] << "dmp2:  "<< x_dmp[2] << endl;

                        cout << "commanded pose:   " << endl;
                        for(int q=0; q<12 ; q++)
                            cout << des_pose[q] << " " ;
                        cout << endl;

                        cout << "desired orientation in euler angles:" << endl;
                        utility::array2mat_vec(rot_pose0, trans0, des_pose, FRI_CART_FRM_DIM);

                        cout << " rot_pose0: " << rot_pose0 << endl;
                        cout << " trans_pose0: " << trans0 << endl;

                        cout << " rotation angles: " << endl;
                        vec eulerangles=utility::rotationMatrix2eulerAngles(rot_pose0);
                        cout << eulerangles << endl;
                    }

                    ros::spinOnce();

                    // ---------------------------------------------------
                    // finish time measurement
                    /*
                    time_t end = clock();
                    elapsed_time += double(end - begin) / CLOCKS_PER_SEC;
                    if (display_counter%display_cycles == 0) {
                        cout << "FRI cycle time (in ms) : " << (elapsed_time * 1000.0) / ((float)display_cycles) << endl;
                    }
                    */
                    // ---------------------------------------------------
                }

                cout << "saving data..." << endl;

                mxArray* x_traj_mat;
                cout << "creating matlab matrix..." << endl;
                x_traj_mat = mxCreateDoubleMatrix(integrator1.x_traj.size(),integrator1.x_traj[0].size(),mxREAL);
                cout << "call stdVectorMatrix2MatlabMatrix..." << endl;
                utility::stdVectorMatrix2matlabMatrix(&integrator1.x_traj,x_traj_mat);
                cout << "write Matlab file..." << endl;
                utility::writeMatlabFile(x_traj_mat,"xtraj","../data/xtraj.mat");


                mxArray* v_traj_mat;
                v_traj_mat = mxCreateDoubleMatrix(integrator1.v_traj.size(),integrator1.v_traj[0].size(),mxREAL);
                utility::stdVectorMatrix2matlabMatrix(&integrator1.v_traj,v_traj_mat);
                utility::writeMatlabFile(v_traj_mat,"vtraj","../data/vtraj.mat");

                mxArray* s_traj_mat;
                s_traj_mat = mxCreateDoubleMatrix(integrator1.s_traj.size(),integrator1.s_traj[0].size(),mxREAL);
                utility::stdVectorMatrix2matlabMatrix(&integrator1.s_traj,s_traj_mat);
                utility::writeMatlabFile(s_traj_mat,"straj","../data/straj.mat");


                cout << "DONE" << endl;

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
                //CartStiffnessValues[2] = 2000.0;
                //CartStiffnessValues[4] = 200.0;
                //CartStiffnessValues[5] = 200.0;

                FRI->SetCommandedCartStiffness(CartStiffnessValues);
                //LoopValue = 2100;
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


                    }

                    if(it_%100==0){
                        colvec a(12);

                        vector<double> v_double(demo_[it_].begin(), demo_[it_].end());

                        for(int q=0; q<12 ; q++)
                            cout << v_double.at(q) << " " ;

                        a = utility::cvec2armadilloColVec(v_double);
                        cout << "pose" << a << endl;
                        mat b = reshape(a,4,3);
                        b = b.t();
                        cout << "pose mat" << b << endl;
                        mat rot_mat = b.submat(0,0,2,2);
                        vec x_trans = b.submat(0,3,2,3);
                        cout << "rot mat" << endl << rot_mat << endl;
                        cout <<"x_trans" << endl << x_trans << endl;
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


