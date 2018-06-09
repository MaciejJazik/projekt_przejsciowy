/*
 * opencv_example_node.cpp
 * 
 * Copyright 2018 Maciej Jazikowski <maciej@maciej-N61Vg>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include "ros/ros.h" 
#include <iostream> 
#include "cv_bridge/cv_bridge.h" 
#include "sensor_msgs/Image.h" 
#include <opencv2/opencv.hpp> 
#include <image_transport/image_transport.h> 
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <stdio.h>
#include <vector>
#include <string>

using namespace std; 
using namespace cv; 

bool check=true;
cv_bridge::CvImagePtr cv_im_ptr;
cv::Mat dImg_gray;
cv::Mat image_HSV;
cv::Size size;
cv::Mat mask;
// OBSŁUGA DRONA ////////////
ros::Publisher topictakeoff;
ros::Publisher topiclanding;
ros::ServiceClient serviceflattrim;
ros::Publisher cmd_vel;
ros::Publisher topicinfo;
float logitud, lattitud,altitud;
vector<Vec3f> circles;//wektor przetrzymujący informacje o wykrytych kołach x y radius
 ////////////////////////////////
 // OBSŁUGA DRONA 
geometry_msgs::Twist changeTwist(float x, float y,float z, float turn)
{
	
	geometry_msgs::Twist msg_vel;
	msg_vel.angular.x = 0;
	msg_vel.angular.y = 0;
	msg_vel.angular.z = turn;
	msg_vel.linear.x = x;
	msg_vel.linear.y = y;
	msg_vel.linear.z = z;
	return(msg_vel);	
}

void takeoff(void)
{
	std_msgs::Empty empty;
	geometry_msgs::Twist msg_vel;
	topictakeoff.publish(empty);
	cout<<"getting takeoff..."<<endl;
	usleep(250000);
	msg_vel=changeTwist(0,0,0,0);
	cmd_vel.publish(msg_vel);
	
}
void land(void)
{
	std_msgs::Empty empty;
	topiclanding.publish(empty);
}
 void move(float x, float y, float z, float turn)
{
	geometry_msgs::Twist msg_vel;
	msg_vel=changeTwist(x,y,z,turn);
	cmd_vel.publish(msg_vel);
}
 
//konwersja obrazu(pętla główna)
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{ 
	
	//imshow("image", cv_bridge::toCvShare(msg, "bgr8")->image);
	//convert to opencv
	try
    {
      cv_im_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //double min = 0;
	//double max = 1000;
	//cv::Mat img_scaled_8u;
    //cv::Mat dImg =  cv_ptr->image;//do szarych
    //metoda hsv
    cv::Mat image = cv_im_ptr->image;
    namedWindow( "HSV transform", CV_WINDOW_AUTOSIZE );
    cvtColor(image, image_HSV, CV_BGR2HSV);
    size=image_HSV.size();
    cv::Mat l_red_mask=cvCreateMat(size.height, size.width, CV_8UC1);
    cv::Mat h_red_mask=cvCreateMat(size.height, size.width, CV_8UC1);
    //Scalar max = Scalar(200, 255, 255); //HSV VALUE
	//Scalar min = Scalar(150, 100, 100); //HSV VALUE
    inRange(image_HSV, cv::Scalar(0,100,30), cv::Scalar(8,255,255), l_red_mask);
    inRange(image_HSV, cv::Scalar(160,100,30), cv::Scalar(180,255,255), h_red_mask);
      cv::addWeighted(l_red_mask, 1.0, h_red_mask, 1.0, 0.0, mask);
    //dilate(mask,mask,getStructuringElement(MORPH_RECT, Size(21,21)));
    //erode(mask,mask,getStructuringElement(MORPH_RECT, Size(10,10)));
    
    dilate(mask,mask,getStructuringElement(MORPH_RECT, Size(11,11)));
    erode(mask,mask,getStructuringElement(MORPH_RECT, Size(5,5)));
    
    
    //wykrywanie kół za pomocą szarości (słaba metoda ale działa)
    //cvtColor( dImg, dImg_gray, CV_BGR2GRAY );//convert to gray
    GaussianBlur( mask, mask, Size(9, 9), 2, 2 );//reduction
    
    HoughCircles( mask, circles, CV_HOUGH_GRADIENT, 1.5, mask.rows/8, 200, 50, 0, 0 );//wykrywanie kół
    
    //HoughCircles( mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/8, 200, 100, 0, 1000 );//wykrywanie kół
    //cout<<circles.empty()<<endl;
	/*if(check)//test
    {
		if(!circles.empty())
		{
			std::cout<<"jest"<<std::endl;
			std::cout<<"kółko x:"<<circles[0][0]<<", y:"<<circles[0][1]<<", r:"<<circles[0][2]<<std::endl;
			check=false;
		}
			//std::cout<<"pusty"<<std::endl;
	}*/
    line(image,cvPoint(500,0),cvPoint(500,480),Scalar(255,0,0),3,8,0);//0,480
	line(image,cvPoint(180,0),cvPoint(180,480),Scalar(255,0,0),3,8,0);//0,480
	line(image,cvPoint(0,120),cvPoint(640,120),Scalar(255,0,0),3,8,0);//120,120
	line(image,cvPoint(0,240),cvPoint(640,240),Scalar(255,0,0),3,8,0);//360,360
    for( size_t i = 0; i < circles.size(); i++ )//draw detected circles
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( image, center, radius, Scalar(0,255,0), 3, 8, 0 );
      ////////////////////////////////////////
  
	 if(((image.cols/2)-center.x) > 180)
     {
		 //cout<<"aktualna pozycja x to :"<<center.x<< "przesuniecie w LEWO o " <<  (image.rows/2)-center.x<<endl;
		 cout<<"lewo ";
		 move(0,0.3,0,0);
     }
     else if(((image.cols/2)-center.x) < -180)
     {
		 cout<<"prawo ";
		 //cout<<"aktualna pozycja x to :"<<center.x<< "przesuniecie w PRAWO o " <<  (image.rows/2)-center.x<<endl;
		 move(0,-0.3,0,0);
     }
     else if(((image.rows/2)-center.y) > 60)//120
     {
		 cout<<"gora ";
       //cout<<"aktualna pozycja zto :" << center.y<< "przesuniecie w GÓREE"<< (image.cols/2)-center.y<<endl;
       move(0,0,0.3,0);
	 }
	 else if(((image.rows/2)-center.y) <-60)//120
	 {
		 cout<<"dol ";
		 //cout<<"aktualna pozycja z to :" << center.y<< "przesuniecie w DOL"<< (image.cols/2)-center.y<<endl;
		 move(0,0,-0.3,0);
	 }
	 else if (radius > 60)
	 {
		 cout<<"tyl ";
		 //cout << "aktualna wartosc promienia " << radius << endl;
		 move (-0.3,0,0,0);
	 }
	 else if (radius < 40)
	 {  
		 cout<<"przod ";
		 //cout << "aktualna wartosc promienia " << radius << endl;
		 move (0.3,0,0,0);
	     }
	 else 
	 {

		//cout<<"aktualne przesuniecie to x "<<(image.rows/2)-center.x<<" y "<<(image.cols/2)-center.y<<endl;
		move(0,0,0,0);
	 }
	 cout<<"X:"<<center.x<<" dX:"<<((image.cols/2)-center.x)<<" Y:"<<center.y<<" dY:"<<((image.rows/2)-center.y)<<" R:"<<radius<<endl;
      // cout<<"aktualna pozucja z to : << ....... << endl;
    //namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    ////////////////////////////////////////
    }
    if(circles.size()==0)//brak kola
    {
		move(0,0,0,0);
	}
		
    if(waitKey(30)==27)
    {
		cout << "zamykanie "<< endl;
		land();
		exit ( 0 );
		
	}
    
    imshow( "HSV transform", image );
    
    
	//imshow( "Hough Circle Transform Demo", dImg );//pokazywanie obrazu z wykrytymi kółkami
    

    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)//rysowanie przykładowego koła
    {
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	}*/
    
    
    //cv::imshow("image", cv_ptr->image);//pokaż obraz
    //cv::imshow("image", dImg);
	waitKey(30); 
} 


int main(int argc, char **argv) 
{ 
	ros::init(argc, argv, "cv_example"); //initialize node 
	ros::NodeHandle n; // node handler
	//convert to opencv
	image_transport::ImageTransport image_trans(n);
	image_transport::Subscriber image_sub;
	topictakeoff=n.advertise<std_msgs::Empty>("/ardrone/takeoff",1,true);
	topiclanding=n.advertise<std_msgs::Empty>("/ardrone/land",1,true);
	cmd_vel=n.advertise<geometry_msgs::Twist>("cmd_vel",1,true);
	//waitKey(100);
	takeoff();
	//kamera drona
	image_sub= image_trans.subscribe("/ardrone/front/image_raw", 1000, imageCallback);
	//kamera web
	//image_sub = image_trans.subscribe("/usb_cam/image_raw", 1000, imageCallback);
	//cv::namedWindow("image");
	
	 /*if(waitKey(30)==27)
    {
		cout << "zamykanie "<< endl;
		//land();
		exit( 0 ) ;
		
	}*/
	
	// subsribe topic 
	//ros::Subscriber sub = n.subscribe("/ardrone/front/image_raw", 1000, imageCallback); // if you use Kinect camera change the name of the topic 
	ros::spin();//ros loop !!!!
	return 0; 
} 
