#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>
#include <mutex>
#include <time.h>


#define GREEN 0
#define RED 100
#define OFF 0
#define NUMTHREADS 4
#define CLIFFMAX 100
#define FULLSPEED 220
#define TURNSPEED 105
#define RANGE 2
#define SAMPLES 3
#define OCTHRESH 1
#define REVERSESPEED -200
#define REVERSESLEEPTIME 350
#define REVERSEDISTANCE -40
#define ROTATEANGLE 60


using namespace iRobot;
using namespace LibSerial;
using namespace std;

//MUTEXES
mutex robot_mutex;
mutex safety_mutex;
mutex bump_mutex;
mutex speed_mutex;


//FLAGS
volatile bool safe_flag = true;
volatile bool speed_set = false;
volatile bool _bumped = false;


enum states {DRIVE, CALLIBRATE, IDLE};


//Thread resposnible for safety core
void* safetyThread(void* data){
	//Local variables
	short cL, cR, cFL, cFR, sum_oR, sum_oL;
	bool wL, wR, wC, oR, oL, set = false, bump_flag = false;
	bool or_arr[SAMPLES] = {0};
	bool ol_arr[SAMPLES] = {0};
	Create* robot_ptr = (Create*)data;
	short sampleWall = 1, wallSignal;


	while (1){
		//cout<< "In safety thread"<<endl;
		robot_mutex.lock();
		//looks for cliff signal, wheel drop & wheel overcurrent
		cL = robot_ptr->cliffLeftSignal();
		cR = robot_ptr->cliffRightSignal();
		cFL = robot_ptr->cliffFrontLeftSignal();
		cFR = robot_ptr->cliffFrontRightSignal();
		//Wheel Drop Sensors
		wL = robot_ptr->wheeldropLeft();
		wR = robot_ptr->wheeldropRight();
		wC = robot_ptr->wheeldropCaster();
		//wheel overcurrent
		oR = robot_ptr->leftWheelOvercurrent();
		oL = robot_ptr->rightWheelOvercurrent();
		//bump sensors
		bump_flag = robot_ptr->bumpLeft () || robot_ptr->bumpRight ();
		robot_mutex.unlock();

		while( (cL<CLIFFMAX)||(cR<CLIFFMAX)|| (cFL<CLIFFMAX)||(cFR<CLIFFMAX) || wL||wR||wC || (sum_oR>OCTHRESH) || (sum_oL>OCTHRESH) ){
			cout<<"NOT SAFE"<<endl;
			//STOP DRIVING & PLAY SONG & Set safety flag
			safety_mutex.lock();
			safe_flag =false;
			safety_mutex.unlock();

			//STOP THE ROBOT & PLAY SONG
			robot_mutex.lock();
			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			robot_ptr->sendPlaySongCommand((unsigned char)0);



			//Get new signal values
			cL = robot_ptr->cliffLeftSignal();
			cR = robot_ptr->cliffRightSignal();
			cFL = robot_ptr->cliffFrontLeftSignal();
			cFR = robot_ptr->cliffFrontRightSignal();
			//Wheel drop
			wL = robot_ptr->wheeldropLeft();
			wR = robot_ptr->wheeldropRight();
			wC = robot_ptr->wheeldropCaster();
			//wheel overcurrent
			oR = robot_ptr->leftWheelOvercurrent();
			oL = robot_ptr->rightWheelOvercurrent();
			robot_mutex.unlock();


			//TELL ROBOT THAT IT HAS TO SET SPEED AGAIN
			speed_mutex.lock();
			speed_set = false;
			speed_mutex.unlock();


			//store & average overcurrent signals
			//move all the signals down by one
			sum_oL =0;
			sum_oR =0;
			for (int i=1; i<SAMPLES; i++){
				//right side
				or_arr[i] = or_arr[i-1];
				sum_oR += or_arr[i];
				//left side
				ol_arr[i] = ol_arr[i-1];
				sum_oL += ol_arr[i];
			}
			//store new signals
			or_arr[0] = oR;
			ol_arr[0]= oL;
			//finish sum
			sum_oR +=or_arr[0];
			sum_oL +=ol_arr[0];

			//cout << "NEW WAVE.........." << endl;
			//cout<< "cliff signals: "<<cL<<" "<<cR<<" "<<cFL<<" "<<cFR<<endl;
			//cout<< "wheel drop signals: "<<wL<<" "<<wR<<" "<<wC<<endl;
			//cout<< "Overcurrent signals: "<<oL<<" "<<oR<<" "<<sum_oL<<" "<<sum_oR<<endl;
			//cout << "bump flags:"<<bump_flag<<endl;
			//cout << "..................." << endl;
		}

		//If it bumped into something, stop, reverse then stop again
		if (bump_flag){
			//its not safe
			safety_mutex.lock();
			safe_flag =false;
			safety_mutex.unlock();

			//this_thread::sleep_for(chrono::milliseconds(10));

			//let the stream clear
			//robot_mutex.lock();
			//robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			//robot_mutex.unlock();

			//reverse for a few secs
			cout<<"BUMP! REVERSING!"<<endl;

			//ROTATE UNTIL SENSOR READING IS ZERO
			//reverse
			robot_mutex.lock();
			robot_ptr->sendDriveCommand(REVERSESPEED, Create::DRIVE_STRAIGHT);
			this_thread::sleep_for(chrono::milliseconds(100));


			robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
			wallSignal = robot_ptr->wallSignal();
			while ((sampleWall>0) || (wallSignal>0)){
			//while(1){
				cout<<"STOPPING SIGNAL IS: "<<wallSignal<<endl;
				sampleWall = wallSignal;
				wallSignal = robot_ptr->wallSignal();
			}
			cout<<"STOPPING SIGNAL IS: "<<robot_ptr->wallSignal()<<endl;
			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			robot_ptr->sendDriveCommand(REVERSESPEED, Create::DRIVE_STRAIGHT);
			robot_ptr->sendWaitDistanceCommand(REVERSEDISTANCE);
			//robot_mutex.unlock();



			//wait until bump sensor is clear
			while (bump_flag){
				//robot_mutex.lock();
				bump_flag = robot_ptr->bumpLeft () || robot_ptr->bumpRight ();
				//robot_mutex.unlock();
			}

			//stop the robot
			//robot_mutex.lock();
			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			robot_mutex.unlock();


			robot_mutex.lock();
			bump_flag = robot_ptr->bumpLeft () || robot_ptr->bumpRight ();
			robot_mutex.unlock();

			cout<<"Bump sensor is now:"<<bump_flag<<endl;

			//tell wall follow thread to callibrate sensors
			bump_mutex.lock();
			_bumped = true;
			bump_mutex.unlock();

			//TELL ROBOT THAT IT HAS TO SET SPEED AGAIN
			speed_mutex.lock();
			speed_set = false;
			speed_mutex.unlock();
		}

		//set the flag if needed
		safety_mutex.lock();
		if (!safe_flag){
			safe_flag = true;
			cout<<"IT IS NOW SAFE"<<endl;
		}
		safety_mutex.unlock();
	}

	this_thread::sleep_for(chrono::milliseconds(10));

}





//Thread resposnible for safety core
void* wallFollowThread(void* data){
	//Local variables
	short wallSignal, prevWallSignal = 0, diff=0;
	short max_wall_signal=0, flag =0, angle =0;
	short low_bound=0, upper_bound =4096;
	bool bump_flag = false, in_range;
	bool is_safe =false;
	bool speed_is_set = false;
	Create* robot_ptr = (Create*)data;
	auto end_time = std::chrono::system_clock::now();
	auto max_time = std::chrono::system_clock::now();
	auto turn_time = std::chrono::system_clock::now();

	time_t t_max;
	time_t t_end;
	double diff_time;
	long sleep_time;

	//START IN THE DRIVE STATE
	enum states state = DRIVE;

	cout<< "STARTED WALL THREAD"<<endl;


	while (1){
		/////////////////////////
		//CHECK IF IT IS SAFE TO DRIVE
		safety_mutex.lock();
		is_safe = safe_flag;
		safety_mutex.unlock();

		//CHECK IF THERE WAS A BUMP : NEED RECALLIBRATION
		bump_mutex.lock();
		bump_flag = _bumped;
		bump_mutex.unlock();


		if (bump_flag){
			//cout<<"GOT A BUMP FLAG"<<endl;
			state = CALLIBRATE;
		}
		if (is_safe){
				////////////////////	STATES 	///////////////////////
				if (state == DRIVE){
						//check safety
						safety_mutex.lock();
						is_safe = safe_flag;
						safety_mutex.unlock();
						//Check if we have already set the speed before
						speed_mutex.lock();
						speed_is_set = speed_set;
						speed_mutex.unlock();

						if (is_safe && !speed_is_set){
							cout<< " SETTING SPEED IN DRIVE"<<endl;
							//Drive straight at fullspeed
							robot_mutex.lock();
							robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
							robot_mutex.unlock();
							speed_mutex.lock();
							speed_set=true;
							speed_mutex.unlock();
						}
				}
				else if (state == CALLIBRATE){
						cout<<"Callibrating"<<endl;
						//TURN UNTIL WE GET THE FIRST READING
						robot_mutex.lock();
						robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
						wallSignal = robot_ptr->wallSignal();
						//robot_mutex.unlock();
						cout<<"LOOKING FOR VALID WAL"<<endl;
						while (wallSignal<4){
							//robot_mutex.lock();
							wallSignal = robot_ptr->wallSignal();
							//robot_mutex.unlock();
						}
						//robot_mutex.lock();

						//ROTATE UNTIL WE SEE NO MORE WALL SIGNAL
						while (wallSignal >0){
							//IF THIS SIGNAL IS GREATER THAN THE MAX,
							if (wallSignal>max_wall_signal){
								max_wall_signal = wallSignal;
								//store the time we got this
								//max_time = std::chrono::system_clock::now();
								t_max = time(NULL);
								cout<<"sensor: "<<max_wall_signal<<endl; ;
							}
							//robot_mutex.lock();
							wallSignal = robot_ptr->wallSignal();
							//robot_mutex.unlock();
						}
						//robot_mutex.lock();
						robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
						t_end = time(NULL);
						robot_mutex.unlock();
						//end_time = std::chrono::system_clock::now();
						//std::chrono::duration<double> turn_time = end_time - max_time;
						//t_end = time(NULL);



						diff_time = difftime(t_end, t_max);
						sleep_time = (diff_time*1000) - 200; //+ 200;


						cout<<"MaxSignal is: "<<max_wall_signal<<"Turn time in ms: "<<sleep_time<<endl;
						//ROTATE TO THE MAX TIME
						robot_mutex.lock();
						robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
						//robot_mutex.unlock();
						if (sleep_time<=0)
							sleep_time = 50;

						this_thread::sleep_for(chrono::milliseconds(sleep_time));

						//robot_mutex.lock();
						robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
						robot_mutex.unlock();


						while(1){

							robot_mutex.lock();
							wallSignal = robot_ptr->wallSignal();
							robot_mutex.unlock();
							cout<<"Wall signal is: "<<wallSignal<<endl;

						}
						//STOP WHEN WE GET NO READING





						//Get wall Sensor & rotate
						robot_mutex.lock();
						robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
						robot_ptr->sendWaitAngleCommand(ROTATEANGLE);
						robot_mutex.unlock();
						//this_thread::sleep_for(chrono::milliseconds(10000));
						cout<< "TOLD IT TO TURN............................"<<endl;

						while (1){
							robot_mutex.lock();
							//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
							//robot_ptr->sendWaitAngleCommand(ROTATEANGLE);
							angle = robot_ptr->angle();
							//robot_ptr->sendDriveCommand(0, Create::DRIVE_INPLACE_CLOCKWISE);
							wallSignal = robot_ptr->wallSignal();
							robot_mutex.unlock();
							cout<<"wallSignal is:"<<wallSignal<<" angle is"<<angle<<endl;
							//this_thread::sleep_for(chrono::milliseconds(10000));
						}
				}
		}
	}
}





//Thread resposnible for safety core
void* mazePlotThread(void* data){
	//Local variables

	Create* robot_ptr = (Create*)data;

}

//Thread resposnible for safety core
void* objectIdentificationThread(void* data){
	//Local variables

	Create* robot_ptr = (Create*)data;

}


int main ()
{
  char serial_loc[] = "/dev/ttyUSB0";

   try {
    raspicam::RaspiCam_Cv Camera;
    cv::Mat rgb_image, bgr_image;
    if (!Camera.open()) {
      cerr << "Error opening the camera" << endl;
      return -1;
    }
    cout << "Opened Camera" << endl;
    SerialStream stream (serial_loc, LibSerial::SerialStreamBuf::BAUD_57600);
    cout << "Opened Serial Stream to" << serial_loc << endl;
    this_thread::sleep_for(chrono::milliseconds(1000));
    Create robot(stream);
    cout << "Created iRobot Object" << endl;
    robot.sendFullCommand();
    cout << "Setting iRobot to Full Mode" << endl;
    this_thread::sleep_for(chrono::milliseconds(1000));
    cout << "Robot is ready" << endl;


    // Let's stream some sensors.
    Create::sensorPackets_t sensors;
    sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
    sensors.push_back(Create::SENSOR_WALL_SIGNAL);
    sensors.push_back (Create::SENSOR_BUTTONS);
	sensors.push_back (Create::SENSOR_CLIFF_LEFT_SIGNAL);
	sensors.push_back (Create::SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
	sensors.push_back (Create::SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
	sensors.push_back (Create::SENSOR_CLIFF_RIGHT_SIGNAL);
	sensors.push_back (Create::SENSOR_OVERCURRENTS);
    robot.sendStreamCommand (sensors);
    cout << "Sent Stream Command" << endl;

	//Create songs
    Create:: note_t note(pair<unsigned char, unsigned char>(85, 8));
	Create:: song_t song;
	song.push_back(note);
	robot.sendSongCommand((unsigned char)0, song);

	//Create a few threads
	//struct thread_data robot_data;
	//robot_data.robot = robot;

	pthread_t thread_ids[NUMTHREADS];


	pthread_create(&thread_ids[0], NULL, safetyThread,(void*)&robot);
	pthread_create(&thread_ids[1], NULL, wallFollowThread,(void*)&robot);
	//pthread_create(&thread_ids[2], NULL, mazePlotThread,(void*)&robot);
	//pthread_create(&thread_ids[3], NULL, objectIdentificationThread,(void*)&robot);



	while(1){
			// cout << "Main thread:" << robot.wallSignal() <<" "<<robot.bumpRight()<< endl;
			// this_thread::sleep_for(chrono::milliseconds(100));
	}





















    //Start Driving
    int speed = 287;
    robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
    robot.sendLedCommand (Create::LED_PLAY, 0, 0);
    cout << "Sent Drive Command" << endl;

    short wallSignal, prevWallSignal = 0;
    //Start running program until play button is pressed
    while (!robot.playButton ())
    {
	//Check the bump sensors
	if (robot.bumpLeft () || robot.bumpRight ()) {
		cout << "Bump !" << endl;
		//Stop the robot
		robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		//Create thread for led stuff
		//Get mutex & set pic status


		//take a picture
		// Camera.grab();
		// Camera.retrieve (bgr_image);
		// cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		// cv::imwrite("irobot_image.jpg", rgb_image);
		// cout << "Taking photo" << endl;
          	//this_thread::sleep_for(chrono::milliseconds(5000));

		//rotate by a random angle
		int rand_sleep = 2800 + rand()%1900;
		robot_mutex.lock();
		robot.sendDriveCommand(107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
		robot_mutex.unlock();
		//robot.sendWaitAngleCommand (rand_angle);

		robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
	}


	//Play the right tone
	short wallSignal = robot.wallSignal();
 	//cout << "Wall signal " << robot.wallSignal() << endl;
	if (wallSignal > 0) {
		 cout << "Wall signal " << robot.wallSignal() << endl;
		//Play the right sound
		if (wallSignal<4){
		robot.sendPlaySongCommand((unsigned char)0);
		}
		else if(wallSignal < 8){

		robot.sendPlaySongCommand((unsigned char)1);
		}
		else{

		robot.sendPlaySongCommand((unsigned char)2);
		}
	}
	// You can add more commands here.
	//this_thread::sleep_for(chrono::milliseconds(100));
    } //End of while loop


    cout << "Play button pressed, stopping Robot" << endl;
    robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
  }
  catch (InvalidArgument& e)
  {
    cerr << e.what () << endl;
    return 3;
  }
  catch (CommandNotAvailable& e)
  {
    cerr << e.what () << endl;
    return 4;
  }
}
