#include <SerialStream.h>
#include "irobot-create.hh"
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <pthread.h>
#include <mutex>
#include <time.h>
#include <signal.h>

using namespace cv;
using namespace cv::xfeatures2d;

#define GREEN 0
#define RED 100
#define OFF 0
#define NUMTHREADS 4
#define CLIFFMAX 50
#define RANGE 2
#define SAMPLES 3
#define WALLSAMPLES 100
#define MIDINDEX 50
#define OCTHRESH 10

#define RIGHTWHEELSPEED 170
#define LEFTWHEELSPEED 30
#define ADJUST_SAMPLES 15

//PERIODS FOR EACH PRIORITY IN MS
#define PRIORITY_1_PERIOD 75
#define PRIORITY_2_PERIOD 105 //120
#define PRIORITY_3_PERIOD 3000


//ROBOT SPEEDS
#define FULLSPEED 135//138
#define TURNSPEED 100
#define REVERSESPEED -200


//WALL SIGNAL
#define WALL_SIGNAL_SAMPLES 3
#define SWEEP_SAMPLE_SIZE 150
#define SAFETY_MARGIN 60

//FOR CONTOUR PLOTTING
#define LEFT_TURN 1
#define RIGHT_TURN 0



using namespace iRobot;
using namespace LibSerial;
using namespace std;
using chrono::duration;
using chrono::steady_clock;

//MUTEXES
mutex robot_mutex;
mutex safety_mutex;
mutex bump_mutex;
mutex speed_mutex;
mutex restart_mutex;



//FLAGS
volatile bool safe_flag = true;
volatile bool speed_set = false;
volatile bool _bumped = false;
volatile bool done = false;
volatile bool waypoint_change = false;
volatile bool restart = true;
volatile bool finished = false;
volatile bool first = true;
volatile unsigned int photos_taken = 0;
volatile bool safe_abort = false;
auto robo_start = std::chrono::system_clock::now();


//GLOBAL VARS
volatile short rot_angle;
vector <cv::Mat> image_vect;
vector <unsigned int> turn_vect; 	//


enum states {DRIVE, CALLIBRATE, LEFT_ADJUST, RIGHT_ADJUST, HARD_RIGHT};



typedef struct objectInput{
	map<string, vector<KeyPoint>>*  keypoints_query_map_ptr;
	map<string, Mat>* descriptors_query_map_ptr;
	map<string, Mat>* queryImages_ptr;
	Create * robot_ptr;
	vector <cv::Mat>* image_vect_ptr;
}objectInput;


//FUNCTION PROTOTYPES
void wall_alignment (Create* robot_ptr);


bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene, Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners);
void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction);
void drawProjection(Mat& img_matches, Mat& img_query, vector<Point2f>& scene_corners);
string type2str(int type);
void usage();
map<string, Mat> detect_object(map<string, Mat>& img_querys, Mat &img_scene_full, objectInput data_str, bool* found_lamp) ;

struct sigaction act;

void sighandler(int signum, siginfo_t *info, void *ptr){
	safe_abort = true;
}










//Thread resposnible for safety core
void* safetyThread(void* data){
	//Local variables
	short cL, cR, cFL, cFR, sum_oR, sum_oL, sum_cL, sum_cR, sum_cFL, sum_CFR;
	bool wL, wR, wC, oR, oL, bump_flag = false;
	bool or_arr[SAMPLES] = {0};
	bool ol_arr[SAMPLES] = {0};
	vector<bool> ol_vector, or_vector, cliff_vector;
	Create* robot_ptr = (Create*)data;
	short wallSignal;
	size_t i;
	int lock_status;
	double sleep_time_ms, runtime_ms;

	//debugging
	size_t max_c =0;

//ALWAYS RUNNING TASK
	while (1){


		//cout<< "In safety thread"<<endl;
		//STORE THE START TIME OF THE THREAD
		//auto t_start = std::chrono::system_clock::now();
		//GE THE LOCK OR SLEEP
		while (!robot_mutex.try_lock()){
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_1_PERIOD));
		}

		//check timing
		auto current_time = std::chrono::system_clock::now();
		auto runtime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - robo_start);
		cout<< "TIME IS: "<<runtime_ms.count() /1000<<endl;
		if (  (runtime_ms.count() > 120000) || 	robot_ptr->playButton()	){
			cout<<"*********	TIME LIMIT EXCEEDED	**********"<<endl;
			finished = true;
			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			robot_mutex.unlock();
			pthread_exit(NULL);
		}


		if(safe_abort){
			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			robot_ptr->sendLedCommand(Create::LED_NONE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
			robot_mutex.unlock();
			finished = true;
			pthread_exit(NULL);
		}


		//cout<<"---------	IN SAFETY---------"<<endl;
		auto t_start = std::chrono::system_clock::now();
		//cout<<"GOT LOCK"<<endl;
		//robot_mutex.lock();
		//looks for cliff signal, wheel drop & wheel overcurrent
		cL = robot_ptr->cliffLeftSignal();
		cR = robot_ptr->cliffRightSignal();
		cFL = robot_ptr->cliffFrontLeftSignal();
		cFR = robot_ptr->cliffFrontRightSignal();


		//seen_cliff = false;
		//for (size_t i =0; i< )

		// sum_cL.push_back(cL);
		// sum_cR. push_back(cR);
		// sum_cFL.push_back(CFL);
		// sum_CFR.push_back(CFR);

		//Wheel Drop Sensors
		wL = robot_ptr->wheeldropLeft();
		wR = robot_ptr->wheeldropRight();
		wC = robot_ptr->wheeldropCaster();
		//wheel overcurrent: TAKE A FEW SAMPLES
		ol_vector.push_back(robot_ptr->leftWheelOvercurrent());
		or_vector.push_back(robot_ptr->rightWheelOvercurrent());
		//bump sensors
		bump_flag = robot_ptr->bumpLeft () || robot_ptr->bumpRight ();

		//CHECK IF PLAY BUTTON PRESSED
		// if(robot_ptr->playButton()){
		// 	cout<< "*****************	PLAY BUTTON PRESSED	***********"<<endl;
		// 	finished = true;
		// }
			//done = true;
		//robot_mutex.unlock();


		if (ol_vector.size() > SAMPLES)
			ol_vector.erase(ol_vector.begin());
		if (or_vector.size() > SAMPLES)
			or_vector.erase(or_vector.begin());
		// if (cliff_vector.size() > SAMPLES)
		// 	cliff_vector.erase(or_vector.begin());

		//GET AVG OVERCURRENT SIGNALS
		sum_oL =0;	sum_oR =0; ; //sum_cliff =0;
		for (i =0; i<SAMPLES; i++){
			sum_oR += or_vector[i];
			sum_oL += ol_vector[i];
			//sum_cliff += cliff_vector[i];
		}

		if( (cL<CLIFFMAX)||(cR<CLIFFMAX)|| (cFL<CLIFFMAX)||(cFR<CLIFFMAX) || wL||wR||wC || (sum_oR>=OCTHRESH) || (sum_oL>=OCTHRESH) || (bump_flag) ){
			cout<<"NOT SAFE"<<endl;
			//STOP DRIVING & PLAY SONG & Set safety flag
			safety_mutex.lock();
			safe_flag =false;
			safety_mutex.unlock();

			//STOP THE ROBOT & PLAY SONG
			//robot_mutex.lock();

			if (bump_flag){
				cout<<"BUMP DETECTED"<<endl;
				robot_ptr->sendDriveCommand(REVERSESPEED, Create::DRIVE_STRAIGHT);
				this_thread::sleep_for(chrono::milliseconds(30));
				robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
				_bumped = true;
				if (first){
					robo_start = std::chrono::system_clock::now();
					first = false;
				}


			}
			else{		//PLAY SONG IF IT WAS ANYTHING BUT A BUMP

				robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
				restart = true;
				speed_set = false;
				_bumped = false;
				robot_ptr->sendPlaySongCommand((unsigned char)0);

			}




			//robot_mutex.unlock();


			//TELL ROBOT THAT IT HAS TO SET SPEED AGAIN
			speed_mutex.lock();
			speed_set = false;
			speed_mutex.unlock();
			cout << "NEW WAVE.........." << endl;
			cout<< "cliff signals: "<<cL<<" "<<cR<<" "<<cFL<<" "<<cFR<<endl;
			cout<< "wheel drop signals: "<<wL<<" "<<wR<<" "<<wC<<endl;
			cout<< "Overcurrent signals: "<<oL<<" "<<oR<<" "<<sum_oL<<" "<<sum_oR<<endl;
			cout << "bump flags:"<<bump_flag<<endl;
			cout << "..................." << endl;
		}
		else{
			//TELL THE ROBOT THAT IT IS SAFE TO MOVE
			safety_mutex.lock();
			if (!safe_flag){
				safe_flag = true;
				cout<<"IT IS NOW SAFE"<<endl;
			}
			safety_mutex.unlock();
		}
		//release the lock
		robot_mutex.unlock();

		//Store the end time of the thread
		auto t_end = std::chrono::system_clock::now();


		//COMPUTE SLEEP TIME
		auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
		sleep_time_ms = PRIORITY_1_PERIOD - time_diff_ms.count();

		if (time_diff_ms.count() > max_c){
			max_c = time_diff_ms.count();
			cout<< "SAFETY DIFF TIME: "<<time_diff_ms.count()<<endl;
			cout<< "SAFETY SLEEP TIME: "<<sleep_time_ms<<endl;
		}

		//cout<<"---------	LEAVING SAFETY---------"<<endl;
		//Sleep to complete period
		if (time_diff_ms.count() > 0)
			this_thread::sleep_for(chrono::milliseconds((long)sleep_time_ms));
		else
			this_thread::sleep_for(chrono::milliseconds((long)sleep_time_ms));
	}
}


void * test (void * data){
	//DEBUGGING
	//WAIT FOR A FEW SECONDS THEN END
	int count =0;
	while (1){
		for (size_t i =0; i<30; i++){}
		this_thread::sleep_for(chrono::milliseconds(PRIORITY_2_PERIOD));
		count++;
		cout<< "COUNT IS: "<<count<<endl;
		if (count == 300)
			pthread_exit(NULL);
	}

}




//THREAD THAT IS RESPONSIBLE FOR NAVIGATING THE ROBOT AROUND THE MAZE
void* navigationThread(void* data){
	//LOCAL VARIABLES
	size_t i;
	int tilt_sum, signal_diff;
	//wall signal stuff
	short wallSignal,  lastWallSignal;
	vector <short> wallSignal_vector;
	wallSignal_vector.reserve(WALL_SIGNAL_SAMPLES);		//reserve enough space
	short vector_size, lost_wall_count =0;
	//flags
	bool is_safe, has_bumped, has_set_speed, lost_wall, aligned = false, to_restart;
	//Timing
	double sleep_time_ms;
	Create* robot_ptr = (Create*)data;




	//ALWAYS EXECUTING TASK
	while(1){
		if (finished){
			pthread_exit(NULL);
		}
		//Get start time
		//cout<<"---------	IN NAVIGATION	---------"<<endl;
		auto t_start = std::chrono::system_clock::now();

		//POLL THE WALL SIGNALS, SAFETY FLAG, BUMPED FLAG, SET SPEED FLAGS
		safety_mutex.lock();
		is_safe = safe_flag;
		safety_mutex.unlock();

		bump_mutex.lock();
		has_bumped = _bumped;
		bump_mutex.unlock();

		speed_mutex.lock();
		has_set_speed = speed_set;
		speed_mutex.unlock();



		if (!is_safe){
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_2_PERIOD));
			continue;
		}


		while (!robot_mutex.try_lock()){
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_1_PERIOD));
			continue;
		}


		//SET THE SPEED TO MOVE FORWARD IF IT HAS NOT ALREADY BEEN SET
		if (!has_set_speed && !has_bumped){
			robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
			cout<< "SETTING SPEED"<<endl;
			speed_mutex.lock();
			speed_set=true;
			aligned = false;
			speed_mutex.unlock();
			wallSignal_vector.clear();
		}
		//ADD NEW WALL SIGNAL TO THE BUFFER
		wallSignal_vector.push_back( robot_ptr->wallSignal());
		robot_mutex.unlock();



		//make sure vector remains the right size
		if (wallSignal_vector.size() > WALL_SIGNAL_SAMPLES ){
			//DELETE OLDEST ENTRY
			wallSignal_vector.erase(wallSignal_vector.begin());
		}
		//update vector size
		vector_size = wallSignal_vector.size();

		//cout<< "VECTOR SIZE: "<<vector_size<<" BUMPED: "<< has_bumped<< " ALIGNED: "<<aligned<<endl;

		robot_mutex.lock();
		//CHECK FOR ANY EVENTS
		if ( (vector_size == WALL_SIGNAL_SAMPLES) && (!has_bumped) && (aligned) ){		//taken enough samples & not a bump event
			//cout<< "Checking walls....."<<endl;
			//Check for lost wall signal
			lost_wall = true;
			cout<< "WALL SIGNAL WAS: ";
			for (i = 1; i<4; i++){
				lost_wall = lost_wall &&  (wallSignal_vector[vector_size - i] <=2) ;
				cout<< wallSignal_vector[vector_size -i]<< " ";
			}
			cout<<endl;
			if (lost_wall){

				lost_wall_count++;
				cout<<"LOST WALL; COUNT IS: "<<lost_wall_count<<endl;
			}
			else {
				lost_wall_count =0;
			}

			//size_t array_sum =0;
			//check for left/right sway
			// tilt_sum =0;
			// for (i=1; i<vector_size; i++){
			// 	signal_diff = wallSignal_vector[i] - wallSignal_vector[i-1];		//+ve is an increase in wall sig
			// 	//ADD IF INCREASE SUBTRACT IF DECREASE
			// 	if (signal_diff>0){
			// 		tilt_sum++;
			// 		if (signal_diff > 5)
			// 			tilt_sum +=5;
			// 	}
			// 	else if (signal_diff<0){
			// 		tilt_sum--;
			// 		if (signal_diff < -5)
			// 			tilt_sum -=5;
			// 	}
			// 	//array_sum= wallSignal_vector[i];
			// }
			//cout<< "TILT SUM: "<<tilt_sum<<endl;

			tilt_sum =0;
			i = vector_size-1;
			signal_diff = wallSignal_vector[i] - wallSignal_vector[1];		//+ve is an increase in wall sig
			//ADD IF INCREASE SUBTRACT IF DECREASE
			if (signal_diff>2){
				tilt_sum++;
			}else if (signal_diff<-2){
				tilt_sum--;
			}



			lastWallSignal =wallSignal_vector[WALL_SIGNAL_SAMPLES-1] ;
			//ACTIONS FOR ANY OF THE EVENTS
			if(lost_wall_count > 3){
				cout<< "LOST WALL...ADJUSTING:"<<endl;
				// if (turn_vect.size()>0){
				// 	if (turn_vect.back() == LEFT_TURN){
						turn_vect.push_back(RIGHT_TURN);
					//}
				//}

				photos_taken =0;


				//DRIVE FORWARD A LITTLE (250mm)
				//robot_ptr->sendDriveCommand(0, Create::DRIVE_INPLACE_CLOCKWISE);
				//robot_mutex.unlock();
				//this_thread::sleep_for(chrono::milliseconds(100));
				//robot_mutex.lock();
				//robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
				robot_mutex.unlock();
				//this_thread::sleep_for(chrono::milliseconds(770));
				this_thread::sleep_for(chrono::milliseconds(720));
				robot_mutex.lock();

				if ( robot_ptr->wallSignal()<2){
					//ROTATE 100 DEGREES CLOCKWISE
					//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
					//CURVE
					robot_ptr->sendDriveDirectCommand( 250, 5);
					robot_mutex.unlock();
					//this_thread::sleep_for(chrono::milliseconds(1610));
					this_thread::sleep_for(chrono::milliseconds(1610));
					robot_mutex.lock();
					//CONTINUE DRIVING STRAIGHT
					//robot_ptr->sendDriveCommand(0, Create::DRIVE_INPLACE_CLOCKWISE);
					robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
					// //CLEAR THE SAMPLES
					wallSignal_vector.clear();
					lost_wall_count =0;
				}
				else{
					cout<< "**************	FALSE ALARM	**************"<<endl;
					//CONTINUE DRIVING STRAIGHT
					robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
				}

			}
			else if (tilt_sum > 0){		//increasing wall signal values
				cout<< " ADJUSTING TO THE LEFT "<<endl;
				//ROTATE A FEW DEGREES COUNTERCLOCKWISE
				//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

				if (lastWallSignal >25){
					robot_ptr->sendDriveDirectCommand( 10, 150);
				}
				else {
					robot_ptr->sendDriveDirectCommand( 10, 100);
				}

				//robot_ptr->sendDriveDirectCommand( 10, 220);

				robot_mutex.unlock();
				//this_thread::sleep_for(chrono::milliseconds(185));
				//this_thread::sleep_for(chrono::milliseconds(114));		//BEST
				if (lastWallSignal > 25){
					this_thread::sleep_for(chrono::milliseconds(160));
				}
				else{
					this_thread::sleep_for(chrono::milliseconds(120));
				}


				robot_mutex.lock();
				//CONTINUE DRIVING STRAIGHT
				robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
				// //CLEAR THE SAMPLES
				//wallSignal_vector.clear();

			}
			else if (tilt_sum < 0){		//decreasing wall signal values
				cout<< " ADJUSTING TO THE RIGHT "<<endl;
				//ROTATE A FEW DEGREES CLOCKWISEA
				//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
				//robot_ptr->sendDriveDirectCommand( 250, 10);
				if (lastWallSignal < 5){
					robot_ptr->sendDriveDirectCommand( 150, 10);
				}
				else{
					robot_ptr->sendDriveDirectCommand( 100, 10);
				}

				robot_mutex.unlock();

				//this_thread::sleep_for(chrono::milliseconds(170));
				//this_thread::sleep_for(chrono::milliseconds(109));	//BEST
				if (lastWallSignal < 5){
					this_thread::sleep_for(chrono::milliseconds(150));
				}
				else {
					this_thread::sleep_for(chrono::milliseconds(110));
				}

				robot_mutex.lock();
				//CONTINUE DRIVING STRAIGHT
				robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
				// //CLEAR THE SAMPLES
				//wallSignal_vector.clear();
			}
		}
		robot_mutex.unlock();

		//IF A BUMP HAPPENED: REALIGN
		if (has_bumped){
			cout<< "BUMP IN NAV"<<endl;
			robot_mutex.lock();
			wall_alignment(robot_ptr);
			robot_mutex.unlock();
			bump_mutex.lock();
			_bumped = false;
			bump_mutex.unlock();
			aligned = true;
			speed_mutex.lock();
			speed_set=true;
			speed_mutex.unlock();
			photos_taken =0;	//reset photos taken

			// if (turn_vect.size()>0){
			// 	if (turn_vect.back() == RIGHT_TURN){
					turn_vect.push_back(LEFT_TURN);
			// 	}
			// }

		}


		//COMPUTE SLEEP TIME & SLEEP
		auto t_end = std::chrono::system_clock::now();
		auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
		sleep_time_ms = PRIORITY_2_PERIOD - time_diff_ms.count();
		//cout<< "NAVIGATION DIFF TIME: "<<time_diff_ms.count()<<endl;
		//cout<< " NAVIGATION SLEEP TIME: "<<sleep_time_ms<<endl;
		//Sleep to complete period
		//cout<<"---------	LEAVING NAVIGATION	---------"<<endl;
		if (sleep_time_ms > 0)
			this_thread::sleep_for(chrono::milliseconds((long)sleep_time_ms));
		else {
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_2_PERIOD));
			//cout<<"SPENT TOO LONG: "<<time_diff_ms.count()<<endl;
		}

	}

}



//FUNCTION THAT ALIGNS THE ROBOT WITH THE WALL
/*
*	ALGORITHM: SWEEP THROUGH CLOCKWISE UNTIL A DECREASE IS SEEN
*			   ROTATE BACK BY HOW LONG IT TOOK TO GET  HERE
*
*
*/
void wall_alignment (Create* robot_ptr){

	//LOCAL VARIABLES
	short wallSignal, size;
	vector<short> signal_vector;
	vector <std::chrono::milliseconds> time_vector;
	uint32_t index = SWEEP_SAMPLE_SIZE - SAFETY_MARGIN;


	cout<< "IN ALIGNMENT THREAD"<< endl;

	//MOVE CLOCKWISE A LITTLE
	robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
	//robot_mutex.unlock();
	this_thread::sleep_for(chrono::milliseconds(800));
	//REVERSE A LITTLE
	robot_ptr->sendDriveCommand(REVERSESPEED, Create::DRIVE_STRAIGHT);
	this_thread::sleep_for(chrono::milliseconds(8));

	robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
	//FIND THE LOCAL MAXIMA
	while (1){
		//check if finished
		if (finished){
			robot_mutex.unlock();
			return;
		}

		//robot_mutex.lock();
		//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

		//cout<< "TRYING TO ALIGN"<<endl;

		//SAMPLE WALL SIGNAL & store time of wall signal
		wallSignal = robot_ptr->wallSignal();
		signal_vector.push_back(wallSignal);
		size = signal_vector.size();
		//cout<< "NEW WALL SIGNAL: "<< wallSignal<<" SAMPLE SIZE: "<<endl;
		//time_vector.push_back(std::chrono::system_clock::now());

		//KEEP SIZE THE SAME
		if (size > SWEEP_SAMPLE_SIZE){
			//cout <<"CULLING SIZE BY ONE: "<<signal_vector.size()<<endl;
			signal_vector.erase(signal_vector.begin());
			//cout<< "NEW SIZE IS: "<<signal_vector.size()<<endl;
			size = signal_vector.size();
		}


		//IF ENOUGH SAMPLES
		if (size== SWEEP_SAMPLE_SIZE){
			//cout<< "vector size: "<< signal_vector.size()<<" INDEX IS:"<<index<<endl;
			//CHECK FOR A LOCAL MAXIMA
			auto it = max_element(signal_vector.begin(), signal_vector.end());
			//cout<< "MAX VALUE IS :" <<*it<<endl;
			if ( (signal_vector[index] == *it)  && (signal_vector[index] > 5) && (signal_vector[index] > signal_vector[SWEEP_SAMPLE_SIZE-1]) ){
			//if ( (signal_vector[index] > 40) ){
				cout<< "GOING THE OTHER WAY"<<endl;
				//robot_ptr->sendDriveCommand(0, Create::DRIVE_INPLACE_CLOCKWISE);
				cout<< "MAXIMA IS: "<< signal_vector[index];
				//this_thread::sleep_for(chrono::milliseconds(10000));
				//cout<< "MAXIMA IS: "<< signal_vector[index];
				//ROTATE BACK TO MAXIMA
				robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
				//robot_mutex.unlock();
				this_thread::sleep_for(chrono::milliseconds(531));
				//robot_mutex.lock();
				//CONTINUE DRIVING STRAIGHT
				robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);

				// cout<< "PRINTING ARRAY"<<endl;
				// for (size_t i =0; i<size; i++){
				// 	cout<< signal_vector[i]<<endl;
				// }

				robot_mutex.unlock();
				//EXIT THE FUNCTION
				break;
			}
		}
		robot_mutex.unlock();
		this_thread::sleep_for(chrono::milliseconds(10));
		robot_mutex.lock();
	}
}











//Thread resposnible for safety core
void* wallFollowThread(void* data){
	//Local variables
	short wallSignal, prevWallSignal = 0, diff=0;
	double clock_diff = 0;
	short max_wall_signal=0, flag =0, angle =0;
	short low_bound=0, upper_bound =4096;
	bool bump_flag = false, in_range;
	bool is_safe =false;
	bool speed_is_set = false;
	bool is_local_maxima = false;
	bool find_wall = false;
	bool callibrated = false;
	Create* robot_ptr = (Create*)data;
	auto end_time = std::chrono::system_clock::now();
	auto max_time = std::chrono::system_clock::now();
	auto turn_time = std::chrono::system_clock::now();

	//Arrays to store wall signals & corresponding time
	time_t time_array[WALLSAMPLES];
	clock_t clock_array[WALLSAMPLES];
	short signal_array[WALLSAMPLES] = {0};
	vector<short> adjust_vector;
	short adjust_array[ADJUST_SAMPLES] = {0};
	//short turn_signal_array[TURNSAMPLES] ={0}

	short lost_signal_count =0;
	short samples_taken =0;
	short adjust_samples_taken =0;

	double recent_avg, prev_avg;
	double diff_avg;
	short cnt;
	short pos_diff, neg_diff, array_sum, new_sum;
	short sum_cache[ADJUST_SAMPLES] = {0};
	short max_increase, max_decrease;
	short dif_buf =0;

	bool left_flag, right_flag;

	time_t t_max, t_sample;
	time_t t_end;
	clock_t c_max, c_sample, c_end;

	//



	double diff_time;
	long sleep_time;

	//START IN THE DRIVE STATE
	enum states state = DRIVE;

	cout<< "STARTED WALL THREAD"<<endl;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 	*
	*																*
	*																*
	*																*
	*																*
	*																*
	*	REMEMBER TO UPDATE DONE AND WAYPOINT CHANGE FLAGS!!!!		*
	*																*
	*																*
	*																*
	*																*
	*																*
	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *	*/

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
			cout<<"GOT A BUMP FLAG" << "SAFE? "<<is_safe<<endl;
			state = CALLIBRATE;
			waypoint_change = true;
		}
		if (is_safe){
			//cout<<"state is:"<<state<<endl;
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


						//POLL THE ROBOT
						robot_mutex.lock();
						if (is_safe && !speed_is_set){
							cout<< " SETTING SPEED IN DRIVE"<<endl;
							//Drive straight at fullspeed
							robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
							speed_mutex.lock();
							speed_set=true;
							speed_mutex.unlock();
							lost_signal_count=0;
							//clear sample
							memset(adjust_array, 0, ADJUST_SAMPLES);
							adjust_samples_taken =0;
						}
						//LOG IN THE NEW WALL SIGNAL
						wallSignal = robot_ptr->wallSignal();
						robot_mutex.unlock();



						//ADD SAMPLE TO THE ARRAYS
						for (size_t i =0; i<ADJUST_SAMPLES-1; i++){
							adjust_array[i] = adjust_array[i+1];
						}
						adjust_array [ADJUST_SAMPLES-1] = wallSignal;
						adjust_samples_taken++;


						//Get total difference btween the samples
						array_sum =0;
						pos_diff =0;
						neg_diff =0;
						max_increase =0;
						max_decrease =0;
						left_flag = false;
						right_flag = false;
						for (size_t i=0; i<ADJUST_SAMPLES-1; i++){
							if (adjust_array[i] < adjust_array[i+1]){
								pos_diff++;
								dif_buf = adjust_array[i+1] -adjust_array[i];
								if (dif_buf> 5){
									left_flag =true;
									right_flag = false;
								}
							}
							if (adjust_array[i] > adjust_array[i+1]){
								neg_diff++;
								//big difference
								dif_buf = adjust_array[i] -adjust_array[i+1];
								if (dif_buf > 5){
									left_flag =false;
									right_flag =true;
								}
							}
							array_sum+= adjust_array[i];
						}

						//ADD ARRAY SUM TO THE CACHE
						new_sum =0;
						//3 zeroes in a row
						// for (size_t i=ADJUST_SAMPLES-3; i<ADJUST_SAMPLES-2; i++){
						// 	sum_cache[i] = sum_cache[i+1];
						// 	new_sum +=sum_cache[i];
						// }
						sum_cache[0] = sum_cache[1];
						sum_cache[1] = sum_cache[2];
						sum_cache[2] = adjust_array[ADJUST_SAMPLES-1] + adjust_array[ADJUST_SAMPLES-2];
						new_sum = sum_cache[2] + sum_cache[0] + sum_cache[1];

						cout<< "SUM VALUES: "<<sum_cache[0]<< sum_cache[1]<< sum_cache[2];



						cout<< "POS DIFF: "<< pos_diff<< " NEG DIFF: "<<neg_diff<<" SUM: "<<new_sum<<endl;

						//IF THRE HAS BEEN A CONSISTENT INCREASE IN WALL SENSOR
						if (  ((adjust_samples_taken>ADJUST_SAMPLES)   &&   (callibrated)   &&    (pos_diff>2))  || left_flag ){
							cout<<"Goint into left adjust"<<endl;
							state = LEFT_ADJUST;
						}
						//IF THERE HAS BEEN A CONSISTENT DECREASE IN WALL SENSOR

						if (	((adjust_samples_taken>ADJUST_SAMPLES) && (callibrated) &&  (neg_diff>2))	|| right_flag  ){
							cout<<"going into right adjust"<<endl;
							state = RIGHT_ADJUST;
						}

						if ( (new_sum <2)  &&   (callibrated) && (adjust_samples_taken>=3) ){
							cout<<"Doing a hard right"<<endl;
							state = HARD_RIGHT;
						}


						//debugging
						if ((neg_diff>0) || (pos_diff>0)){
							//print them out
							cout<< "PRINTING ARRAY"<<endl;
							for (size_t i=0; i<ADJUST_SAMPLES-1; i++){
								cout<<"_ "<< adjust_array[i];
							}
						}



				}
				else if (state == CALLIBRATE){
						cout<<"Callibrating"<<endl;
						memset(signal_array, 0,WALLSAMPLES );

						robot_mutex.lock();
						robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
						//this_thread::sleep_for(chrono::milliseconds(400));


						robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);

						//this_thread::sleep_for(chrono::milliseconds(50));

						cout<< "Getting"<<is_local_maxima<<" "<<(signal_array[MIDINDEX]<3)<<" "<<( (!is_local_maxima) && (signal_array[MIDINDEX]<3) ) << endl;

						//WHILE NO LOCAL MAXIMA
						is_local_maxima = false;
						samples_taken =0;
						//|| (samples_taken < WALLSAMPLES)
						while ( (!is_local_maxima) || (signal_array[MIDINDEX]<8) || (samples_taken <WALLSAMPLES)  ){
							//cout<<"IN WHILE LOOP"<<endl;
							//GET NEW WALL SIGNAL SAMPLE
							wallSignal = robot_ptr->wallSignal();
							//t_sample = time(NULL);
							c_sample = clock();



							//ADD SAMPLE TO THE ARRAYS
							for (size_t i =0; i<WALLSAMPLES-1; i++){
								signal_array[i] = signal_array[i+1];
							}
							signal_array[WALLSAMPLES-1] = wallSignal;
							for (size_t i =0; i<WALLSAMPLES-1; i++){
								time_array[i] = time_array[i+1];
								clock_array[i] = clock_array[i+1];
							}
							time_array[WALLSAMPLES-1] = t_sample;
							clock_array[WALLSAMPLES-1] = c_sample;


							//CHECK IF IT IS THE LOCAL MAXIMA
							is_local_maxima = true;
							//GREATER THAN THOSE BEFORE
							for (size_t i =0; i<MIDINDEX; i++){
								is_local_maxima = is_local_maxima && (signal_array[MIDINDEX] >= signal_array[i]);
								//cout<<"i = "<<i<<" - "<<signal_array[i]<<endl;
							}
							//GREATER THAN THOSE AFTER
							for (size_t i =MIDINDEX+1; i<WALLSAMPLES; i++){
								is_local_maxima = is_local_maxima && (signal_array[MIDINDEX] > signal_array[i]);
								///cout<<"i = "<<i<<" - "<<signal_array[i]<<endl;
							}
							samples_taken++;
							this_thread::sleep_for(chrono::milliseconds(5));
							// cout<< "IS LOCAL MAXIMA: "<<is_local_maxima<<endl;
							// cout<< "MID INDEX: "<<(signal_array[MIDINDEX] <3)<<endl;
							// cout<< "NEW SAMPLE IS "<<wallSignal<<" VS "<< signal_array[WALLSAMPLES-1]<<endl;
							//
							// cout<<"CHOSEN SAMPLES:"<<endl;
							// for (size_t i =0; i<WALLSAMPLES; i++){
							// 	cout<<"i "<<i<<" VALUE:"<<signal_array[i]<< endl;
							// }
							// cout<< "........................................"<<endl;


						}

						/*cout<<"CHOSEN SAMPLES:"<<endl;
						for (size_t i =0; i<WALLSAMPLES; i++){
							cout<<"i "<<i<<" VALUE:"<<signal_array[i]<< endl;
						}*/

						robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
						t_end = time(NULL);
						c_end = clock();
						t_max = time_array[MIDINDEX];
						c_max = clock_array[MIDINDEX];
						robot_mutex.unlock();

						max_wall_signal = signal_array[MIDINDEX];

						//TURN TO THE TIME FOR THE LOCAL MAXIMA
						diff_time = difftime(t_end, t_max);
						clock_diff = ((float)c_end - (float)c_max) / CLOCKS_PER_SEC;
						//sleep_time = (diff_time*1000) - 50; //- 200; //+ 200;
						sleep_time = (long)(clock_diff*1000.00) - 150;

						//cout<<"MaxSignal is: "<<max_wall_signal<<"Turn time in ms: "<<sleep_time<<endl;
						//ROTATE TO THE MAX TIME
						robot_mutex.lock();
						robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
						//robot_mutex.unlock();
						if (sleep_time<=0){
							cout<< "SLEEP TIME IS LESS THAN 0"<<endl;
							sleep_time = 50;
						}

						cout<< "GETTING SLEEP TIME: "<<sleep_time<<endl;


						this_thread::sleep_for(chrono::milliseconds(sleep_time));

						//robot_mutex.lock();
						//robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);
						//wallSignal = robot_ptr->wallSignal();
						//cout<<"NEW WALL SIGNAL IS: "<<robot_ptr->wallSignal()<<endl;
						//ROTATE TO THE MAX TIME
						robot_mutex.unlock();

						cout<<"JUST CALLIBRATED"<<endl;
						callibrated = true;


						state = DRIVE;
						find_wall = false;
						speed_mutex.lock();
						speed_set = false;
						speed_mutex.unlock();
						bump_mutex.lock();
						_bumped = false;
						bump_mutex.unlock();
						lost_signal_count =0;


				}
				else if(state == LEFT_ADJUST){
					//ROTATE A SMALL ANGLE TO THE LEFT
					robot_mutex.lock();
					//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
					robot_ptr->sendDriveDirectCommand( 90, 170);
					this_thread::sleep_for(chrono::milliseconds(22));
					//GO BACK INTO DRIVE MODE
					robot_mutex.unlock();
					//GO BACK INTO DRIVE MODE
					adjust_samples_taken =0;


					speed_mutex.lock();
					speed_set =false;
					speed_mutex.unlock();
					state = DRIVE;

					cout<<"ADJUSTING TO THE LEFT"<< endl;


				}
				else if(state == RIGHT_ADJUST){
					//ROTATE A SMALL ANGLE TO THE RIGHT
					robot_mutex.lock();
					//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
					robot_ptr->sendDriveDirectCommand( 170, 90);
					this_thread::sleep_for(chrono::milliseconds(22));
					robot_mutex.unlock();
					//GO BACK INTO DRIVE MODE
					adjust_samples_taken =0;

					speed_mutex.lock();
					speed_set =false;
					speed_mutex.unlock();
					state = DRIVE;

					cout<<"ADJUSTING TO THE RIGHT"<< endl;
				}
				else if (state == HARD_RIGHT){
					//TAKE A HARD RIGHT UNTIL WE FIND A WALL SIGNAL

					robot_mutex.lock();
					//GO STRAIGHT A LITTLE
					this_thread::sleep_for(chrono::milliseconds(285));
					//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
					robot_ptr->sendDriveDirectCommand( 170, 10);
					this_thread::sleep_for(chrono::milliseconds(295));
					while (robot_ptr->wallSignal()< 3 ){}
					//CONTINUE DRIVING STRAIGHT WITH THE ADJUSTMENTS
					//GO BACK INTO DRIVE MODE
					adjust_samples_taken =0;
					robot_mutex.unlock();

					speed_mutex.lock();
					speed_set =false;
					speed_mutex.unlock();
					state = DRIVE;
				}
		}
		else{
			callibrated = false;
		}
	}
}




//Thread resposnible for safety core
void* mazePlotThread(void* data){
	//Local variables

	Create* robot_ptr = (Create*)data;

	//Creates list of waypoints
	std::vector<Point2f> waypoints;
	Point2f dir_vec, last_wp, curr_wp;
	time_t start_time, end_time;
	float ang_rad;

	//Waits for initial bump
	while(!_bumped);

	//Inserts initial (0,0) position and updates last and curr wp
	waypoints.push_back(Point2f(0,0));
	last_wp.x = 0;
	last_wp.y = 0;
	curr_wp = last_wp;

	//Inserts initial direction vector to (0,1)
	dir_vec.x = 0;
	dir_vec.y = 1;

	rot_angle = 90;

	//Start of robot movement
	start_time = time(NULL);

	//While the robot is not done with the maze
	while(safe_flag){

		//Waits for a waypoint change
		while(!waypoint_change);

		//End of robot movement
		end_time = time(NULL);

		//Calculates current WayPoint
		curr_wp = last_wp + FULLSPEED*(difftime(end_time,start_time))*dir_vec;

		//Inserts and updates waypoints
		waypoints.push_back(curr_wp);
		last_wp = curr_wp;

		//Convert angle to radian
		ang_rad = rot_angle*2*3.14/360.0;

		//Calculates new Direction Vector
		dir_vec.x = dir_vec.x*cos(ang_rad) - dir_vec.y*sin(ang_rad);
		dir_vec.y = dir_vec.x*sin(ang_rad) + dir_vec.y*cos(ang_rad);

		//Start of robot_movement
		start_time = time(NULL);

		waypoint_change = false;
	}


	//Plots the contour using the waypoint vector

	// Creates a drawing context, and use white background.
	Mat img_output(200, 300, CV_8UC3, Scalar(255, 255, 255));

	// Plots the waypoints using blue color.
	Scalar lineColor(255, 0, 0);
	int lineWidth = 1;
	int radius = 3;
	for (int i = 0; i < waypoints.size() - 1; i++) {
		line(img_output, waypoints[i], waypoints[i + 1],
		lineColor, lineWidth, CV_AA);
		circle(img_output, waypoints[i], radius,
		lineColor, CV_FILLED, CV_AA);
	}

	// Draw the bounding rectangle using orange color
	Rect bound = boundingRect(waypoints);
	rectangle(img_output, bound, Scalar(0, 165, 255));

	// Finally store it as a png file

	imwrite("irobot_plot.png", img_output);

}





//Thread resposnible for safety core
void* objectIdentificationThread(void* data){
	//Local variables
	double sleep_time_ms;
	bool found_lamp = false;

	objectInput data_str = *((objectInput*)(data));
	Create * robot_ptr = data_str.robot_ptr;
    //
	// //READ IN THE IMAGES
	// map<string, Mat> queryImages ;
	// //Mat img_query = imread(argv[1], IMREAD_GRAYSCALE);
	// queryImages["ancient-lamp-600"] = imread("object-identification/query-image/low-resolution/ancient-lamp-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["audio-cassette-600"] = imread("object-identification/query-image/low-resolution/audio-cassette-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["mammoth-600"] = imread("object-identification/query-image/low-resolution/mammoth-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["mayan-calandar-600"] = imread("object-identification/query-image/low-resolution/mayan-calendar-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["mjolnir-hammer-600"] = imread("object-identification/query-image/low-resolution/mjolnir-hammer-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["one-ring-600"] = imread("object-identification/query-image/low-resolution/one-ring-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["pueblo-pot-600"] = imread("object-identification/query-image/low-resolution/pueblo-pot-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["roman-glass-600"] = imread("object-identification/query-image/low-resolution/roman-glass-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["willow-plate-600"] = imread("object-identification/query-image/low-resolution/willow-plate-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["magic-lamp-600"] = imread("object-identification/query-image/low-resolution/magic-lamp-600.jpg", IMREAD_GRAYSCALE);
    //
 	// map<string, Mat>::iterator it = queryImages.begin();
	// cout<< "type is: "<<it->first<<endl;

	//FIND ALADIN
	raspicam::RaspiCam_Cv Camera;
	bool is_safe;

	if (!Camera.open()) {
		cerr << "Error opening the camera in thread" << endl;
	}

	//WAIT FOR FIRST BUMP
	while (first){
		this_thread::sleep_for(chrono::milliseconds(PRIORITY_3_PERIOD/5));
	}

	//WHILE THERE IS STILL SOMETHING TO FIND
	while(  data_str.queryImages_ptr->size() > 0  && (!found_lamp)   ){
		if (finished){
			pthread_exit(NULL);
		}


		safety_mutex.lock();
		is_safe = safe_flag;
		safety_mutex.unlock();
		if (!is_safe || (photos_taken >= 2)){
			cout<< "TOO MANY PHOTOS"<<endl;
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_3_PERIOD/5));
			continue;
		}
		//cout<<"---------	IN OBJ IDENTIFICATION	---------"<<endl;
		auto t_start = std::chrono::system_clock::now();
		cv::Mat rgb_image, bgr_image;
	    Camera.grab();
		//cv::Mat img_scene_full, rgb;
		Camera.retrieve(bgr_image);
		cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		cv::imwrite("results/imageTaken.jpg", rgb_image);
		photos_taken ++;
		cout<<"...............TAKEN A PIC................. NO:"<< photos_taken<< endl;
		// this_thread::sleep_for(chrono::milliseconds(2500));
		//queryImages = detect_object(queryImages, bgr_image, data_str);
		image_vect.push_back(bgr_image);
		*(data_str.queryImages_ptr) = detect_object(*(data_str.queryImages_ptr), bgr_image, data_str, &found_lamp);

		//cout<< "FOUND LAMP: "<<found_lamp<<endl;
		//FOUND THE LAMP:
		if (found_lamp){
			//TURN THE LIGHT ON & SHOW A RED LIGHT
			cout<<"...............TURNING ON LIGHT ................."<<endl;
			robot_mutex.lock();

			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			robot_ptr->sendLedCommand(Create::LED_NONE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
			//robot_mutex.unlock();
			//SLEEP FOR 2S
			cout<<"...............FOUND LAMP ................."<<endl;
			this_thread::sleep_for(chrono::milliseconds(2000));

			found_lamp = true; ;
			//robot_mutex.lock();
			robot_ptr->sendLedCommand(Create::LED_NONE, Create::LED_COLOR_RED, Create::LED_INTENSITY_OFF);
			robot_ptr->sendDriveCommand(FULLSPEED, Create::DRIVE_STRAIGHT);


			robot_mutex.unlock();
			if (finished){
				pthread_exit(NULL);
			}
		}


		auto t_end = std::chrono::system_clock::now();
		//COMPUTE SLEEP TIME
		auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
		sleep_time_ms = PRIORITY_3_PERIOD - time_diff_ms.count();

		//cout<<"---------	LEAVING OBJ IDENTIFICATION	---------"<<endl;
		//cout<<"............FINISHED DETECTING............."<<endl;
		//Sleep to complete period
		if (time_diff_ms.count() > 0)
			this_thread::sleep_for(chrono::milliseconds((long)sleep_time_ms));
		else
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_3_PERIOD));


	}



	cout<< "<<<<<<<<<<<<<<<<<<<<<<<<<<<OUT OF LOOP: LAMP FOUND IS: "<< found_lamp<<endl;
	//JUST KEEP TAKING PICS EVERY PERIOD
	while (1){
		auto t_start = std::chrono::system_clock::now();
		cv::Mat rgb_image, bgr_image;
		if (finished){
			pthread_exit(NULL);
		}

		if (!is_safe || (photos_taken >= 2)){
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_3_PERIOD/5));
			continue;
		}

		Camera.grab();
		Camera.retrieve(bgr_image);
		cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		image_vect.push_back(bgr_image);
		photos_taken ++;
		cout<<"...............TAKEN A PIC OUT OF LOOP................. NO:"<< photos_taken<< endl;
		//SLEEP FOR REST OF PERIOD

		cv::imwrite("results/imageTaken.jpg", rgb_image);
		cv::imwrite("results/imageVect.jpg", image_vect.back());

		this_thread::sleep_for(chrono::milliseconds(5000));

		auto t_end = std::chrono::system_clock::now();

		//COMPUTE SLEEP TIME
		auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
		sleep_time_ms = PRIORITY_3_PERIOD - time_diff_ms.count();
		//Sleep to complete period
		if (time_diff_ms.count() > 0)
			this_thread::sleep_for(chrono::milliseconds((long)sleep_time_ms));
		else
			this_thread::sleep_for(chrono::milliseconds(PRIORITY_3_PERIOD));

		if (finished){
			pthread_exit(NULL);
		}

	}


}




//PROCESSES THE IMAGE VECTOR
void* postProcessingThread(void * data){
	//vector <cv::Mat> image_vect = &((vector <cv::Mat>*)data);
	unsigned int cnt=0;
	bool a;

	objectInput data_str = *((objectInput*)(data));
	vector <cv::Mat>my_vect = *(data_str.image_vect_ptr);

	for (size_t i =0; i<my_vect.size(); i++){
		cout<< "VECTOR SIZE: "<< my_vect.size()<<endl;
		cv::Mat rgb_image;
		cout <<"PIC NO: "<<i<<endl;
		cv::cvtColor(my_vect[i], rgb_image, CV_RGB2BGR);
		//cv::imwrite("results/imageTaken.jpg", rgb_image);
		//cv::imwrite("results/vectImage.jpg", image_vect[i]);
		a = false;
		detect_object(*(data_str.queryImages_ptr), (*data_str.image_vect_ptr)[i], data_str, &a);
		if (a){
			cnt++;
		}
		//this_thread::sleep_for(chrono::milliseconds(5000));

		if ((data_str.queryImages_ptr)->size() <=0)
			break;
	}
	cout<< "FOUND: "<<cnt<<endl;
}



















int main ()
{
  char serial_loc[] = "/dev/ttyUSB0";

   try {
   memset(&act, 0, sizeof(act));
   act.sa_sigaction = sighandler;
   act.sa_flags = SA_SIGINFO;
   sigaction(SIGINT, &act, NULL);
    // raspicam::RaspiCam_Cv Camera;
    // cv::Mat rgb_image, bgr_image;
    // if (!Camera.open()) {
    //   cerr << "Error opening the camera" << endl;
    //   return -1;
    // }
    // cout << "Opened Camera" << endl;
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
	pthread_t thread_ids[NUMTHREADS];

	//
	// robot.sendDriveDirectCommand( 170, 30);
	// while (robot.wallSignal() <3){
	// 	//robot_ptr->sendDriveDirectCommand( RIGHTWHEELSPEED, LEFTWHEELSPEED);
	// }
	// robot.sendDriveDirectCommand(0,0);
	// printf("WALLSIGNAL IS: %d\n",robot.wallSignal() );
	// while(1){}

	/* PRECOMPUTE FEATURES */
	//READ IN THE IMAGES
	map<string, Mat> queryImages;

	queryImages["magic-lamp-600"] = imread("object-identification/query-image/low-resolution/magic-lamp-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["ancient-lamp-600"] = imread("object-identification/query-image/low-resolution/ancient-lamp-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["audio-cassette-600"] = imread("object-identification/query-image/low-resolution/audio-cassette-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["mammoth-600"] = imread("object-identification/query-image/low-resolution/mammoth-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["mayan-calandar-600"] = imread("object-identification/query-image/low-resolution/mayan-calendar-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["mjolnir-hammer-600"] = imread("object-identification/query-image/low-resolution/mjolnir-hammer-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["one-ring-600"] = imread("object-identification/query-image/low-resolution/one-ring-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["pueblo-pot-600"] = imread("object-identification/query-image/low-resolution/pueblo-pot-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["roman-glass-600"] = imread("object-identification/query-image/low-resolution/roman-glass-600.jpg", IMREAD_GRAYSCALE);
	// queryImages["willow-plate-600"] = imread("object-identification/query-image/low-resolution/willow-plate-600.jpg", IMREAD_GRAYSCALE);


	int minHessian = 100;
	int nOctaves = 4;
	int nOctaveLayers = 3;
	Ptr<SURF> detector = SURF::create(minHessian, nOctaves, nOctaveLayers, true);

	vector<KeyPoint> keypoints_query;
	Mat descriptors_query;
	map<string, vector<KeyPoint>> keypoints_query_map;
	map<string, Mat> descriptors_query_map;


	map<string, Mat>::iterator iter;
	for (iter = queryImages.begin(); iter != queryImages.end(); ++iter) {
		Mat img_query = iter->second;
		detector->detectAndCompute(
			img_query, Mat(), keypoints_query, descriptors_query);

		keypoints_query_map[iter->first] = keypoints_query;
		descriptors_query_map[iter->first] = descriptors_query;
	}


	objectInput object_input;
	object_input.keypoints_query_map_ptr = &keypoints_query_map;
	object_input.descriptors_query_map_ptr = &descriptors_query_map;
	object_input.queryImages_ptr = & queryImages;
	object_input.robot_ptr = &robot;





	//THREAD ATTRIBUTES TO SET PRIORITY
	pthread_attr_t attrSafety;				//SAFETY ATTRIBUTES
	sched_param paramSafety;
	pthread_attr_init (&attrSafety);
	pthread_attr_getschedparam(&attrSafety, &paramSafety);
	paramSafety.sched_priority = 4;
	pthread_attr_setschedparam(&attrSafety, &paramSafety);



	pthread_attr_t attrMotion;				//MOTION ATTRIBUTES
	sched_param paramMotion;
	pthread_attr_init (&attrMotion);
	pthread_attr_getschedparam(&attrMotion, &paramMotion);
	paramSafety.sched_priority = 3;
	pthread_attr_setschedparam(&attrMotion, &paramMotion);



	pthread_attr_t attrVision;				//VISION ATTRIBUTES
	sched_param paramVision;
	pthread_attr_init (&attrVision);
	pthread_attr_getschedparam(&attrVision, &paramVision);
	paramSafety.sched_priority = 2;
	pthread_attr_setschedparam(&attrVision, &paramVision);


	// while (!robot.playButton()){
	// 	fin
	// }
	// cout<< "FINISHED IS: "<<finished<<endl;



	//CREATE THE THEADS
	pthread_create(&thread_ids[0], &attrSafety, safetyThread,(void*)&robot);
	pthread_create(&thread_ids[1], &attrMotion, navigationThread,(void*)&robot);
	//pthread_create(&thread_ids[2], &attrMotion, mazePlotThread,(void*)&robot);
	//pthread_create(&thread_ids[1], &attrMotion, test, NULL );
	pthread_create(&thread_ids[3], &attrVision, objectIdentificationThread,(void*)&object_input);



	//Waits for nav to finish
	pthread_join(thread_ids[1], NULL);
	robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
	if (safe_abort){
		robot.sendLedCommand(Create::LED_NONE, Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
		cout<<"SAFE_ABORT"<<endl;
		cout<<"PRINTING TURN VECTOR"<<endl;
		for (size_t k =0; k<turn_vect.size(); k++){
			cout<< turn_vect[k]<< " ";
		}
		cout<<endl;
		exit(1);
	}


	cout<< ".....FINISHED NAV THREAD ......."<<endl;
	pthread_join(thread_ids[3], NULL);
	cout << "FINISHED VISION THREAD"<<endl;

	//if (safe_abort)


	queryImages.clear();

	//PERFORM OBJECT IDENTIFICATION ON REST OF IMAGES
	queryImages["ancient-lamp-600"] = imread("object-identification/query-image/low-resolution/ancient-lamp-600.jpg", IMREAD_GRAYSCALE);
	queryImages["audio-cassette-600"] = imread("object-identification/query-image/low-resolution/audio-cassette-600.jpg", IMREAD_GRAYSCALE);
	queryImages["mammoth-600"] = imread("object-identification/query-image/low-resolution/mammoth-600.jpg", IMREAD_GRAYSCALE);
	queryImages["mayan-calandar-600"] = imread("object-identification/query-image/low-resolution/mayan-calendar-600.jpg", IMREAD_GRAYSCALE);
	queryImages["mjolnir-hammer-600"] = imread("object-identification/query-image/low-resolution/mjolnir-hammer-600.jpg", IMREAD_GRAYSCALE);
	queryImages["one-ring-600"] = imread("object-identification/query-image/low-resolution/one-ring-600.jpg", IMREAD_GRAYSCALE);
	queryImages["pueblo-pot-600"] = imread("object-identification/query-image/low-resolution/pueblo-pot-600.jpg", IMREAD_GRAYSCALE);
	queryImages["roman-glass-600"] = imread("object-identification/query-image/low-resolution/roman-glass-600.jpg", IMREAD_GRAYSCALE);
	queryImages["willow-plate-600"] = imread("object-identification/query-image/low-resolution/willow-plate-600.jpg", IMREAD_GRAYSCALE);

	for (iter = queryImages.begin(); iter != queryImages.end(); iter++) {
		Mat img_query = iter->second;
		detector->detectAndCompute(
			img_query, Mat(), keypoints_query, descriptors_query);

		keypoints_query_map[iter->first] = keypoints_query;
		descriptors_query_map[iter->first] = descriptors_query;
	}




	//split the vector int two size
	vector<cv::Mat>  vector_b;
	size_t size = image_vect.size();
	for (size_t j =0; j< (size/2); j++){
		vector_b.push_back((image_vect.back()).clone());
		image_vect.pop_back();
	}
	cout<< "VECTOR SIZES ARE: "<<vector_b.size()<< " "<<image_vect.size()<<endl;



	objectInput object_input_b;
	object_input.keypoints_query_map_ptr = &keypoints_query_map;
	object_input.descriptors_query_map_ptr = &descriptors_query_map;
	object_input.queryImages_ptr = & queryImages;
	object_input.robot_ptr = NULL;
	object_input.image_vect_ptr = &image_vect;


	object_input_b.keypoints_query_map_ptr = &keypoints_query_map;
	object_input_b.descriptors_query_map_ptr = &descriptors_query_map;
	object_input_b.queryImages_ptr = & queryImages;
	object_input_b.robot_ptr = NULL;
	object_input_b.image_vect_ptr = &vector_b;

	//make two threads at the highest priority to do post processing
	//input is a vector of images pointer
	pthread_create(&thread_ids[0], &attrSafety, postProcessingThread,(void*)&object_input);
	pthread_create(&thread_ids[1], &attrSafety, postProcessingThread,(void*)&object_input_b);

	cout<<"PRINTING TURN VECTOR"<<endl;

	for (size_t k =0; k<turn_vect.size(); k++){
		cout<< turn_vect[k]<< " ";
	}
	cout<<endl;

	cout<< "STARTING POST PROCESSING"<<endl;
	//wait for both threads to finish
	pthread_join (thread_ids[0], NULL);
	pthread_join (thread_ids[1], NULL);
	cout<< "FINISHED POST PROCESSING"<<endl;

	exit (0);

	while (1){}



	bool a = false;
	unsigned int cnt=0;
	size = queryImages.size();
	//cout<< "VECTOR SIZE: "<< image_vect.size()<<endl;


	for (size_t i =0; i<image_vect.size(); i++){
		cout<< "VECTOR SIZE: "<< image_vect.size()<<endl;
		cv::Mat rgb_image;
		cout <<"PIC NO: "<<i<<endl;
		cv::cvtColor(image_vect[i], rgb_image, CV_RGB2BGR);
		cv::imwrite("results/imageTaken.jpg", rgb_image);
		cv::imwrite("results/vectImage.jpg", image_vect[i]);
		a = false;
		detect_object(queryImages, image_vect[i], object_input, &a);
		if (a){
			cnt++;
		}
		//this_thread::sleep_for(chrono::milliseconds(5000));

		if (queryImages.size() <=0)
			break;
	}
	cout<< "FINISHED PROCESSING"<<endl;
	cout<< "FOUND: "<<cnt<<" OUT OF "<<size<<endl;

	while(1){}














  //   while (!robot.playButton ())
  //   {
  // /Check the bump sensors
  // f (robot.bumpLeft () || robot.bumpRight ()) {
  // cout << "Bump !" << endl;
  // //Stop the robot
  // robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
  // //Create thread for led stuff
  // //Get mutex & set pic status
  //
  //
  // //take a picture
  // // Camera.grab();
  // // Camera.retrieve (bgr_image);
  // // cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
  // // cv::imwrite("irobot_image.jpg", rgb_image);
  // // cout << "Taking photo" << endl;
  //         	//this_thread::sleep_for(chrono::milliseconds(5000));
  //
  // //rotate by a random angle
  // int rand_sleep = 2800 + rand()%1900;
  // robot_mutex.lock();
  // robot.sendDriveCommand(107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
  // robot_mutex.unlock();
  // //robot.sendWaitAngleCommand (rand_angle);
  //
  // robot.sendDriveCommand (speed, Create::DRIVE_STRAIGHT);
  //
  //
  //
  // /Play the right tone
  // hort wallSignal = robot.wallSignal();
  // //cout << "Wall signal " << robot.wallSignal() << endl;
  // f (wallSignal > 0) {
  //  cout << "Wall signal " << robot.wallSignal() << endl;
  // //Play the right sound
  // if (wallSignal<4){
  // robot.sendPlaySongCommand((unsigned char)0);
  // }
  // else if(wallSignal < 8){
  //
  // robot.sendPlaySongCommand((unsigned char)1);
  // }
  // else{
  //
  // robot.sendPlaySongCommand((unsigned char)2);
  // }
  //
  // / You can add more commands here.
  // /this_thread::sleep_for(chrono::milliseconds(100));
  //   } //End of while loop
  //
  //
    cout << "Play button pressed, stopping Robot" << endl;
    //robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
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










/*PASSES IN MAP OF PHOTO NAMES->MATRICES OF PHOTOS, AND A MATRIX OF THE CURRENT SCENE*/
map<string, Mat> detect_object(map<string, Mat> &img_querys, Mat &img_scene_full, objectInput data_str, bool* found_lamp) {
	try {
		map<string, Mat>::iterator iter;
		for (iter = img_querys.begin();   (iter != img_querys.end())&&(img_querys.size()>0)   ; iter++) {

			string output_file = ("results/" + iter->first + ".png");
			// char output_file[255] = {0};
			// cout<< "iterator is: "<< iter->first<<endl;
			// string output
            //
			//sprintf(output_file, "results/%s.png", iter->first);


			Mat img_query = iter->second;

			/* USE FOR READING IN THE IMAGES*/
			//Mat img_query = imread(argv[1], IMREAD_GRAYSCALE);
			//Mat img_scene_full = imread(argv[2], IMREAD_GRAYSCALE);
			/*USE FOR READING IN IMAGES*/

			//char *output_file = argv[3];
			float keep_top_fraction = .85; // std::stof(argv[4]);
			cout<< "LOOKING FOR: "<< iter->first<<endl;

			if (!img_query.data || !img_scene_full.data) {
				cout << "Error reading images" << endl;
				return img_querys;
			}

			Mat img_scene;
			// Crop bottom
			// Images taken by mounting the camera on the robot will have some portion
			// of the side of the robot at the bottom. To reduce ambiguity during
			// detection and to speed up feature extraction, we crop it.
			// The fraction of cropping will be different depending on where the camera
			// is mounted on the robot. We find the useful portion of the picture is
			// the top 62.5% when camera mounted on front. When camera mounted on the
			// left side its the top 85% that contains useful information.
			cropBottom(img_scene_full, img_scene, keep_top_fraction);

			// Detect the keypoints and extract descriptors using SURF
			// Surf keypoint detector and descriptor.
			int minHessian = 100;
			int nOctaves = 4;
			int nOctaveLayers = 3;
			Ptr<SURF> detector = SURF::create(minHessian, nOctaves, nOctaveLayers, true);

			vector<KeyPoint> keypoints_query, keypoints_scene;
			Mat descriptors_query, descriptors_scene;

			// auto sttime = steady_clock::now();
			// detector->detectAndCompute(
			// 	img_query, Mat(), keypoints_query, descriptors_query);
			// cout << "Feature extraction query image "
			// 	<< (duration <double>(steady_clock::now() - sttime)).count()
			// 	<< " sec" << endl;

			keypoints_query = (*(data_str.keypoints_query_map_ptr))[iter->first];
			descriptors_query = (*(data_str.descriptors_query_map_ptr))[iter->first];


			auto sttime = steady_clock::now();
			detector->detectAndCompute(
				img_scene, Mat(), keypoints_scene, descriptors_scene);
			// cout << "Feature extraction scene image "
			// 	<< (duration <double>(steady_clock::now() - sttime)).count()
			// 	<< " sec" << endl;
			sttime = steady_clock::now();

			cout << keypoints_query.size() << endl;
			cout << keypoints_scene.size() << endl;
			if (keypoints_query.size() > 0 && keypoints_scene.size() > 0) {
				// Matching descriptor vectors using Brute Force matcher
				BFMatcher matcher(NORM_L2);
				vector<vector<DMatch>> matches;
				matcher.knnMatch(descriptors_query, descriptors_scene, matches, 2);

				vector<DMatch> good_matches;
				for (int i = 0; i < descriptors_query.rows; i++) {
					if (matches[i][0].distance < 0.75 * matches[i][1].distance)
						good_matches.push_back(matches[i][0]);
				}

				// Find the location of the query in the scene
				vector<Point2f> query;
				vector<Point2f> scene;
				for (size_t i = 0; i < good_matches.size(); i++) {
					query.push_back(keypoints_query[good_matches[i].queryIdx].pt);
					scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
				}

				vector<Point2f> scene_corners(4);
				bool res = alignPerspective(
					query, scene, img_query, img_scene, scene_corners);
				// cout << "Matching and alignment "
				// 	<< (duration <double>(steady_clock::now() - sttime)).count()
				// 	<< " sec" << endl;

				// Write output to file
				Mat img_matches;
				drawMatches(img_query, keypoints_query, img_scene, keypoints_scene,
					good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
					vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

				// Fill the extra area in almost white (Saves ink when printing)
				if (img_query.rows < img_scene.rows) {
					rectangle(img_matches, Point2f(0, img_query.rows),
						Point2f(img_query.cols - 1, img_scene.rows - 1),
						Scalar(255, 240, 240), CV_FILLED);
				}
				else if (img_scene.rows < img_query.rows) {
					rectangle(img_matches, Point2f(img_query.cols, img_scene.rows),
						Point2f(img_query.cols + img_scene.cols - 1, img_query.rows - 1),
						Scalar(255, 240, 240), CV_FILLED);
				}
				if (res) {
					cout << "<<<<<<<<<<<<<<<   Object found	>>>>>>>>>>>>>>>>>" << endl;
					drawProjection(img_matches, img_query, scene_corners);
					*found_lamp = true;
					// Write result to a file

					cv::imwrite(output_file, img_matches);
					//cout<< "OUTPUTFILE IS"<<  output_file<<endl;
					//cv::imwrite("results/output.png", img_matches);
					//cvSaveImage(output_file, (IplImage(img_matches)));
					iter = img_querys.erase(iter);

					//cout<< "iter: "<<img_querys.begin()<< " "<< img_querys.end()<< " "<< iter<<endl;


					if (iter != img_querys.begin()){
						iter--;
					}
					if (iter == img_querys.end())
						return img_querys;
					//if (img_querys.size() == 0)
						//iter = img_querys.end();
					//iter = img_querys.begin();
				}
				else {
					cout << "Object not found" << endl;
				}
			}
		}
	}
	catch (cv::Exception& e) {
		cout << "exception caught: " << e.what();
		return img_querys;
	}
	return img_querys;
}


void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction) {
	// Crop the lower part of the scene
	cv::Rect crop;
	crop.x = 0;
	crop.y = 0;
	crop.width = img_scene_full.size().width;
	crop.height = img_scene_full.size().height * crop_fraction;
	img_scene = img_scene_full(crop);
}


bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene,
	Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners) {
	Mat H = findHomography(query, scene, RANSAC);
	if (H.rows == 0 && H.cols == 0) {
		// cout << "Failed rule0: Empty homography" << endl;
		return false;
	}

	vector<Point2f> query_corners(4);
	query_corners[0] = cvPoint(0, 0);
	query_corners[1] = cvPoint(img_query.cols, 0);
	query_corners[2] = cvPoint(img_query.cols, img_query.rows);
	query_corners[3] = cvPoint(0, img_query.rows);

	perspectiveTransform(query_corners, scene_corners, H);

	float min_area = 32.0 * 32.0;
	double max_area = img_scene.rows * img_scene.cols;
	float ratio_inside = 0.75;
	float min_angle_sin = 0.173; // Minimum 10 degree angle required

	// Geometric verification heuristics
	// Rule 1: Must be a convex hull.
	// Rule 2: Area can�t be less than 32x32
	// Rule 3: The detected projection can�t have more than 100% area
	// Rule 4: Projection can't contain very small angle < 10 degree
	// Rule 5: More than 75% of the area of the detected projection should have
	// to be within image bounds

	// Rule 1: Must be a convex hull.
	vector<Point2f> sc_vec(4);
	// Generate 4 vectors from the 4 scene corners
	for (int i = 0; i < 4; i++) {
		sc_vec[i] = scene_corners[(i + 1) % 4] - scene_corners[i];
	}
	vector<float> sc_cross(4);
	// Calculate cross product of pairwise vectors
	for (int i = 0; i < 4; i++) {
		sc_cross[i] = sc_vec[i].cross(sc_vec[(i + 1) % 4]);
	}

	// Check for convex hull
	if (!(sc_cross[0] < 0 && sc_cross[1] < 0 && sc_cross[2] < 0 && sc_cross[3] < 0)
		&& !(sc_cross[0] > 0 && sc_cross[1] > 0 && sc_cross[2] > 0 && sc_cross[3] > 0)) {
		cout << "Failed rule1: Not a convex hull" << endl;
		return false;
	}

	// Rule 2: Area can�t be less than 32x32
	// Rule 3: The detected projection can�t have more than 100% area
	float area = (sc_cross[0] + sc_cross[2]) / 2.0;
	if (fabs(area) < min_area) {
		cout << "Failed rule2: Projection too small" << endl;
		return false;
	}
	else if (fabs(area) > max_area) {
		cout << "Failed rule3: Projection too large" << endl;
		return false;
	}

	// Rule 4: Can't contain very small angle < 10 degree inside projection.
	// Check for angles
	vector<float> sc_norm(4);
	for (int i = 0; i < 4; i++) {
		sc_norm[i] = norm(sc_vec[i]);
	}
	for (int i = 0; i < 4; i++) {
		float sint = sc_cross[i] / (sc_norm[i] * sc_norm[(i + 1) % 4]);
		if (fabs(sint) < min_angle_sin) {
			cout << "Failed rule4: Contains very small angle" << endl;
			return false;
		}
	}

	// Rule 5: More than 75% of the area of the detected projection should
	// have to be within image bounds.
	// Approximate mechanism by determining the bounding rectangle.
	cv::Rect bound = boundingRect(scene_corners);
	cv::Rect scene_rect(0.0, 0.0, img_scene.cols, img_scene.rows);
	cv::Rect isect = bound & scene_rect;
	if (isect.width * isect.height <  ratio_inside * bound.width * bound.height) {
		cout << "Failed rule5: Large proportion outside scene" << endl;
		return false;
	}
	return true;
}

// Show the projection
void drawProjection(Mat& img_matches, Mat& img_query,
	vector<Point2f>& scene_corners) {
	line(img_matches, scene_corners[0] + Point2f(img_query.cols, 0),
		scene_corners[1] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1] + Point2f(img_query.cols, 0),
		scene_corners[2] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2] + Point2f(img_query.cols, 0),
		scene_corners[3] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3] + Point2f(img_query.cols, 0),
		scene_corners[0] + Point2f(img_query.cols, 0), Scalar(0, 255, 0), 4);
}

string type2str(int type) {
	std::string r;
	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');
	return r;
}

void usage() {
	cout << " Usage: ./robovision <query-image> <scene-image> <output-image> <crop-bottom>" << endl;
}
