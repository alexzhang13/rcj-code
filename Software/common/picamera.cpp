#include "picamera.h"

#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <ctime>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>
#include <opencv2/opencv.hpp>

_PiCamera_::_PiCamera_()
{
	m_max_len = 5;
	m_data = NULL;
        m_width = 720;
	m_height = 480;
}

_PiCamera_::~_PiCamera_()
{
	if(!m_data)
		delete m_data;
	m_data = NULL;
    m_camera.release();
}

bool _PiCamera_::cameraOpen(int32_t width, int32_t height)
{
	if(width > 0 && m_height > 0) {
		m_width = width;
		m_height = height;
	}

	m_camera.setFormat(raspicam::RASPICAM_FORMAT_GRAY);
    m_camera.setWidth ( m_width );
    m_camera.setHeight ( m_height );
    m_camera.setBrightness (60 );

    m_camera.setSharpness ( 0);
    m_camera.setContrast ( 0 );
    m_camera.setSaturation ( 0);
    m_camera.setShutterSpeed( 0 );
    m_camera.setISO ( 400 );

   //Open camera 
    printf("Opening Camera...\n");
    if ( !m_camera.open()) 
	{
		cerr<<"Error opening camera"<<endl;
		return false;
	}

	m_width = m_camera.getWidth();
	m_height = m_camera.getHeight();
    cout << "width= " << m_width << ", height =" << m_height << "\n";

	return true;
}

bool _PiCamera_::frameCapture(std::string fileName)
{
	cv::Mat img;
	m_camera.grab();
	//allocate memory
	m_data=new unsigned char[  m_camera.getImageTypeSize  ( raspicam::RASPICAM_FORMAT_RGB )];
	//extract the image in rgb format
	m_camera.retrieve ( m_data,raspicam::RASPICAM_FORMAT_IGNORE );//get camera image
	
	img = cv::Mat(m_height, m_width, CV_8UC3, m_data); 
	m_frames.push_back(img.clone());
	if(m_frames.size() > m_max_len)
		m_frames.erase(m_frames.begin());

        imwrite(fileName, img);
	delete m_data;
	return true;
}

bool _PiCamera_::videoCapture()
{
	return true;	
}

bool _PiCamera_::display()
{
	if(m_frames.size() < 1)
		return false;
	cv::Mat img = m_frames[m_frames.size()-1];

	cv::imshow("Display Image", img);
	cv::waitKey(50);

	return true;
}
