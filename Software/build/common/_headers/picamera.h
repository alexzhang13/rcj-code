#ifndef _PICAMERA_RASPPI_h_
#define _PICAMERA_RASPPI_h_

/**
*/
#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <raspicam/raspicam.h>
#include <opencv2/opencv.hpp>
using namespace std;

class _PiCamera_ {

public:

	_PiCamera_();
	~_PiCamera_();

	bool cameraOpen(int32_t width, int32_t height);

	bool frameCapture();

	bool videoCapture();

	bool display();

	inline cv::Mat *getLatestImage() { return m_frames.size() == 0 ? NULL : &m_frames[m_frames.size()-1];}

	inline std::vector<cv::Mat> *getImageList() { return &m_frames;}
	inline std::vector<cv::Mat> getImageListCopy() {return m_frames;}
	inline void resetFrameBuffers() { m_frames.clear();}
	inline void close() { m_camera.release(); }

private:
	int32_t m_width;
	int32_t m_height;
	int32_t m_max_len;
	raspicam::RaspiCam m_camera; //Camera object
	unsigned char *m_data;
	std::vector<cv::Mat> m_frames; 
};

#endif
