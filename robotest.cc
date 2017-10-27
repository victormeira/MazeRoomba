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


#define GREEN 0
#define RED 100
#define OFF 0

using namespace iRobot;
using namespace LibSerial;
using namespace std;

volatile int no_pic =1; //variable to see if picture has been take
mutex led_status_mutex;
mutex robot_mutex; 

struct thread_data{
	SerialStream istream;  
};

//A Thead runs this after hitting a bump 
void cycleLED(Create robot){
	int count =0;
	int status =1;	
	//Get mutex & check pic status 
	led_status_mutex.lock();
	status =no_pic;
	led_status_mutex.unlock();
	while(status){
		//Set the LED sequence
		switch(count){
		case 0:
			//GGO
			robot_mutex.lock();
          		robot.sendLedCommand (Create::LED_PLAY,Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL);	
			robot_mutex.unlock();
			cout<<"GGO"<<endl;
			break;
		case 1:
			//OGG
			robot_mutex.lock();
	       		robot.sendLedCommand (Create::LED_ALL,Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
			robot_mutex.unlock();
          		cout<<"OGG"<<endl;
       			break;	
		case 2:
			//ROG
			robot_mutex.lock();
			robot.sendLedCommand (Create::LED_ADVANCE,Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);
			robot_mutex.unlock();
          		cout<<"ROG"<<endl;
			break;	
		case 3:
			//RGO
			robot_mutex.lock();
          		robot.sendLedCommand (Create::LED_PLAY,Create::LED_COLOR_RED, Create::LED_INTENSITY_FULL);	
			robot_mutex.unlock();
          		cout<<"RGO"<<endl;
			break;	
		case 4:
			//OGG
			robot_mutex.lock();
          		robot.sendLedCommand (Create::LED_ALL,Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
			robot_mutex.unlock();
          		cout<<"OGG"<<endl;
          		break;		
		case 5:
			//GOG
			robot_mutex.lock();
          		robot.sendLedCommand (Create::LED_ADVANCE,Create::LED_COLOR_GREEN, Create::LED_INTENSITY_FULL);
			robot_mutex.unlock();
          		cout<<"GOG"<<endl;
			break;	
		default: 
			break;
		}
		//increment count
		count = ++count % 6;
		//Stay in state for 200 ms
          	this_thread::sleep_for(chrono::milliseconds(200));
		//Get new staus
		led_status_mutex.lock();
		status =no_pic;
		//cout<<"inside thread: "<<status<<" count ="<<count<<endl;
		led_status_mutex.unlock();
	} 
	//cout<<"ending thread, No pic ="<<status<<endl;
	//turn off all lights
        robot_mutex.lock();
	robot.sendLedCommand (Create::LED_NONE,Create::LED_COLOR_GREEN, Create::LED_INTENSITY_OFF);
	robot_mutex.unlock();
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

    //Set up songs 
    Create:: note_t note_1(pair<unsigned char, unsigned char>(85, 8));
    Create:: note_t note_2(pair<unsigned char, unsigned char>(85, 16));
    Create:: note_t note_3(pair<unsigned char, unsigned char>(85, 32));

    Create:: song_t song_slow, song_medium, song_fast;
    for (int i=0; i<1; i++){
	    song_slow.push_back(note_1);
	    song_medium.push_back(note_2);
	    song_fast.push_back(note_3);
    }

    
    robot.sendSongCommand((unsigned char)0, song_fast);
    robot.sendSongCommand((unsigned char)1, song_medium);
    robot.sendSongCommand((unsigned char)2, song_slow);


    // Let's stream some sensors.
    Create::sensorPackets_t sensors;
    sensors.push_back(Create::SENSOR_BUMPS_WHEELS_DROPS);
    sensors.push_back(Create::SENSOR_WALL_SIGNAL);
    sensors.push_back (Create::SENSOR_BUTTONS);
    robot.sendStreamCommand (sensors);
    cout << "Sent Stream Command" << endl;

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
		led_status_mutex.lock();
		no_pic=1;
		led_status_mutex.unlock();	
		//Make thread to handle the LEDs
		thread led_thread(cycleLED, robot);
		
		//back up 15 inches / 381 mm 	
		robot_mutex.lock();
		robot.sendDriveCommand(-165, Create::DRIVE_STRAIGHT);
		robot_mutex.unlock();
		//robot.sendWaitDistanceCommand(-381);
          	this_thread::sleep_for(chrono::milliseconds(2309));
		robot_mutex.lock();
		robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		robot_mutex.unlock();
		cout<<"just send drive commands"<<endl;
          	//this_thread::sleep_for(chrono::milliseconds(50009));
		
		//take a picture 
		Camera.grab();
		Camera.retrieve (bgr_image);
		cv::cvtColor(bgr_image, rgb_image, CV_RGB2BGR);
		cv::imwrite("irobot_image.jpg", rgb_image);
		cout << "Taking photo" << endl;
          	//this_thread::sleep_for(chrono::milliseconds(5000));
		
		//rotate by a random angle
		int rand_sleep = 2800 + rand()%1900;
		robot_mutex.lock(); 
		robot.sendDriveCommand(107, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
		robot_mutex.unlock();
		//robot.sendWaitAngleCommand (rand_angle);
		//Wait until some angle is done 
          	this_thread::sleep_for(chrono::milliseconds(rand_sleep));
		//Stop the robot
		robot_mutex.lock(); 
		robot.sendDriveCommand(0, Create::DRIVE_STRAIGHT);
		robot_mutex.unlock();
		
		//Stop the LED thread
		//Get mutex & set pic status to stop led thread 
		led_status_mutex.lock();
		no_pic=0;
		led_status_mutex.unlock();
		//wait for the led thread to end
		led_thread.join();
		//Start moving
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
