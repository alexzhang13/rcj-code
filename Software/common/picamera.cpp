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
    //capture
    Camera.grab();
    //allocate memory
    unsigned char *data=new unsigned char[Camera.getImageTypeSize (raspicam::RASPICAM_FORMAT_RGB)];
    //extract the image in rgb format
    Camera.retrieve (data, raspicam::RASPICAM_FORMAT_RGB);//get camera image
    //save
    std::ofstream outFile (fileName,std::ios::binary );
    outFile<<"P6\n" <<Camera.getWidth() << " " << Camera.getHeight() <<" 255\n";
    outFile.write((char*)data, Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB));
    cout<<"Image saved at raspicam_image.ppm"<<endl;
    delete data;
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
