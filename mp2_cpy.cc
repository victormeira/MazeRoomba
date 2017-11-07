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

using namespace cv;
using namespace cv::xfeatures2d;

#define GREEN 0
#define RED 100
#define OFF 0
#define NUMTHREADS 4
#define CLIFFMAX 100
#define FULLSPEED 177
#define TURNSPEED 120
#define RANGE 2
#define SAMPLES 3
#define WALLSAMPLES 100
#define MIDINDEX 50
#define OCTHRESH 1
#define REVERSESPEED -220
#define REVERSESLEEPTIME 350
#define REVERSEDISTANCE -15
#define ROTATEANGLE 60
#define RIGHTWHEELSPEED 170
#define LEFTWHEELSPEED 30
#define ADJUST_SAMPLES 15


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


//FLAGS
volatile bool safe_flag = true;
volatile bool speed_set = false;
volatile bool _bumped = false;
volatile bool done = false;
volatile bool waypoint_change = false;


//GLOBAL VARS
volatile short rot_angle;


enum states {DRIVE, CALLIBRATE, LEFT_ADJUST, RIGHT_ADJUST, HARD_RIGHT};



bool alignPerspective(vector<Point2f>& query, vector<Point2f>& scene, Mat& img_query, Mat& img_scene, vector<Point2f>& scene_corners);
void cropBottom(Mat& img_scene_full, Mat& img_scene, float crop_fraction);
void drawProjection(Mat& img_matches, Mat& img_query, vector<Point2f>& scene_corners);
string type2str(int type);
void usage();
map<string, Mat> detect_object(map<string, Mat> img_querys, Mat img_scene_full) ;



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

		//CHECK IF PLAY BUTTON PRESSED
		if(robot_ptr->playButton())
			done = true;
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

			//TURN A LITTLE
			robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
			this_thread::sleep_for(chrono::milliseconds(300));
			robot_ptr->sendDriveCommand(REVERSESPEED, Create::DRIVE_STRAIGHT);
			this_thread::sleep_for(chrono::milliseconds(170));


			//robot_ptr->sendDriveCommand(TURNSPEED, Create::DRIVE_INPLACE_CLOCKWISE);
			wallSignal = robot_ptr->wallSignal();
			// while ((sampleWall>5) || (wallSignal>5)){
			// //while(1){
			// 	//cout<<"STOPPING SIGNAL IS: "<<wallSignal<<endl;
			// 	sampleWall = wallSignal;
			// 	wallSignal = robot_ptr->wallSignal();
			// }
			//this_thread::sleep_for(chrono::milliseconds(180));
			cout<<"STOPPING SIGNAL IS: "<<robot_ptr->wallSignal()<<endl;
			//robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);
			//robot_ptr->sendDriveCommand(REVERSESPEED, Create::DRIVE_STRAIGHT);
			//robot_ptr->sendWaitDistanceCommand(REVERSEDISTANCE);
			//robot_mutex.unlock();

			//this_thread::sleep_for(chrono::milliseconds(50));


			//wait until bump sensor is clear
			while (bump_flag){
				//robot_mutex.lock();
				bump_flag = robot_ptr->bumpLeft () || robot_ptr->bumpRight ();
				//robot_mutex.unlock();
			}

			//stop the robot
			//robot_mutex.lock();
			robot_ptr->sendDriveCommand(0, Create::DRIVE_STRAIGHT);


			//debugging
			//this_thread::sleep_for(chrono::milliseconds(2000));

			// robot_mutex.unlock();
			//
			//
			// robot_mutex.lock();
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

	//this_thread::sleep_for(chrono::milliseconds(10));

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
						cout<<"NEW WALL SIGNAL IS: "<<robot_ptr->wallSignal()<<endl;
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

	Create* robot_ptr = (Create*)data;

	//READ IN THE IMAGES
	map<string, Mat> queryImages;
	//Mat img_query = imread(argv[1], IMREAD_GRAYSCALE);
	queryImages["ancient-lamp-600"] = imread("object-identification/query-image/low-resolution/ancient-lamp-600.jpg", IMREAD_GRAYSCALE);
	queryImages["audio-cassette-600"] = imread("object-identification/query-image/low-resolution/audio-cassette-600.jpg", IMREAD_GRAYSCALE);
	queryImages["mammoth-600"] = imread("object-identification/query-image/low-resolution/mammoth-600.jpg", IMREAD_GRAYSCALE);
	queryImages["mayan-calandar-600"] = imread("object-identification/query-image/low-resolution/mayan-calandar-600.jpg", IMREAD_GRAYSCALE);
	queryImages["mjolnir-hammer-600"] = imread("object-identification/query-image/low-resolution/mjolnir-hammer-600.jpg", IMREAD_GRAYSCALE);
	queryImages["one-ring-600"] = imread("object-identification/query-image/low-resolution/one-ring-600.jpg", IMREAD_GRAYSCALE);
	queryImages["pueblo-pot-600"] = imread("object-identification/query-image/low-resolution/pueblo-pot-600.jpg", IMREAD_GRAYSCALE);
	queryImages["roman-glass-600"] = imread("object-identification/query-image/low-resolution/roman-glass-600.jpg", IMREAD_GRAYSCALE);
	queryImages["willow-plate-600"] = imread("object-identification/query-image/low-resolution/willow-plate-600.jpg", IMREAD_GRAYSCALE);
	queryImages["magic-lamp-600"] = imread("object-identification/query-image/low-resolution/magic-lamp-600.jpg", IMREAD_GRAYSCALE);

	while (queryImages.size() > 0 && safe_flag) {
		raspicam::RaspiCam_Cv Camera;
		//cv::Mat rgb_image, bgr_image;
		//if (!Camera.open()) {
		//	cerr << "Error opening the camera" << endl;
		//	break;
		//}
	    Camera.grab();
		Mat img_scene_full, rgb;
		Camera.retrieve(img_scene_full);
		cv::cvtColor(img_scene_full, rgb, CV_RGB2BGR);
		cv::imwrite("results/imageTaken.jpg", rgb);
		queryImages = detect_object(queryImages, img_scene_full);
	}
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
	pthread_t thread_ids[NUMTHREADS];

	//
	// robot.sendDriveDirectCommand( 170, 30);
	// while (robot.wallSignal() <3){
	// 	//robot_ptr->sendDriveDirectCommand( RIGHTWHEELSPEED, LEFTWHEELSPEED);
	// }
	// robot.sendDriveDirectCommand(0,0);
	// printf("WALLSIGNAL IS: %d\n",robot.wallSignal() );
	// while(1){}



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



	//CREATE THE THEADS
	pthread_create(&thread_ids[0], &attrSafety, safetyThread,(void*)&robot);
	pthread_create(&thread_ids[1], &attrMotion, wallFollowThread,(void*)&robot);
	pthread_create(&thread_ids[2], &attrMotion, mazePlotThread,(void*)&robot);
	//pthread_create(&thread_ids[3], NULL, objectIdentificationThread,(void*)&robot);



	while(1){
			// cout << "Main thread:" << robot.wallSignal() <<" "<<robot.bumpRight()<< endl;
			// this_thread::sleep_for(chrono::milliseconds(100));
	}














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










/*PASSES IN MAP OF PHOTO NAMES->MATRICES OF PHOTOS, AND A MATRIX OF THE CURRENT SCENE*/
map<string, Mat> detect_object(map<string, Mat> img_querys, Mat img_scene_full) {
	try {
		map<string, Mat>::iterator iter;
		for (iter = img_querys.begin(); iter != img_querys.end(); ++iter) {

			const char *output_file = ("results/" + iter->first + ".jpg").c_str();
			Mat img_query = iter->second;

			/* USE FOR READING IN THE IMAGES*/
			//Mat img_query = imread(argv[1], IMREAD_GRAYSCALE);
			//Mat img_scene_full = imread(argv[2], IMREAD_GRAYSCALE);
			/*USE FOR READING IN IMAGES*/

			//char *output_file = argv[3];
			float keep_top_fraction = .85; // std::stof(argv[4]);

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

			auto sttime = steady_clock::now();
			detector->detectAndCompute(
				img_query, Mat(), keypoints_query, descriptors_query);
			cout << "Feature extraction query image "
				<< (duration <double>(steady_clock::now() - sttime)).count()
				<< " sec" << endl;

			sttime = steady_clock::now();
			detector->detectAndCompute(
				img_scene, Mat(), keypoints_scene, descriptors_scene);
			cout << "Feature extraction scene image "
				<< (duration <double>(steady_clock::now() - sttime)).count()
				<< " sec" << endl;
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
				cout << "Matching and alignment "
					<< (duration <double>(steady_clock::now() - sttime)).count()
					<< " sec" << endl;

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
					cout << "Object found" << endl;
					drawProjection(img_matches, img_query, scene_corners);
					// Write result to a file
					cv::imwrite(output_file, img_matches);
					img_querys.erase(iter);
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
		cout << "Failed rule0: Empty homography" << endl;
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
