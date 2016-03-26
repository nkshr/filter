#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_camcalib.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_camcalib.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_camcalib.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;

#define XMD_H

#ifdef SANYO_HD5400
#include <jpeglib.h>
#include <curl/curl.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "f_base.h"
#include "f_stereo_rectify.h"

bool f_stereo_rectify::write_pts(const char * fname)
{
	long long timg1, timg2;
	Mat img1 = m_pin1->get_img(timg1);
	Mat img2 = m_pin2->get_img(timg2);
	if(img1.empty() || img2.empty())
		return false;
	Size sz1(img1.cols, img1.rows), sz2(img2.cols, img2.rows);
	
	if(m_2dchsbd1.size() == 0 || m_2dchsbd2.size() == 0)
		return false;

	ofstream file(fname);
	if(!file.is_open()){
		cerr << "Cannot open file " << fname << endl;
		return false;
	}

	m_3dchsbd.resize(m_sz_chsbd.height*m_sz_chsbd.width);
	for(int i= 0; i < m_sz_chsbd.height; i++){
		for(int j = 0; j < m_sz_chsbd.width; j++){
			int ipt = m_sz_chsbd.width * i + j;
			m_3dchsbd[ipt].x = m_pitch_chsbd * i;
			m_3dchsbd[ipt].y = m_pitch_chsbd * j;
			m_3dchsbd[ipt].z = 0;
		}
	}

	//vector<vector<Point3f > > chsbd3d;
	//for(int i = 0; i < m_2dchsbd.size(); i++)
	//	chsbd3d.push_back(m_3dchsbd);

	file << "size1 " << sz1.width << " " << sz1.height << endl;
	for(int i = 0; i < m_2dchsbd1.size(); i++){
		file << "view " << m_2dchsbd1[i].size() << endl;
		for(int j = 0; j < m_2dchsbd1[i].size(); j++){
			file << m_2dchsbd1[i][j].x;
			file << " ";
			file << m_2dchsbd1[i][j].y;
			file << " ";
			file << m_3dchsbd[j].x;
			file << " ";
			file << m_3dchsbd[j].y;
			file << " ";
			file << m_3dchsbd[j].z;
			file << endl;
		}
	}

	file << "size2 " << sz2.width << " " << sz2.height << endl;
	for(int i = 0; i < m_2dchsbd2.size(); i++){
		file << "view " << m_2dchsbd2[i].size() << endl;
		for(int j = 0; j < m_2dchsbd2[i].size(); j++){
			file << m_2dchsbd2[i][j].x;
			file << " ";
			file << m_2dchsbd2[i][j].y;
			file << " ";
			file << m_3dchsbd[j].x;
			file << " ";
			file << m_3dchsbd[j].y;
			file << " ";
			file << m_3dchsbd[j].z;
			file << endl;
		}
	}
	return true;
}

bool f_stereo_rectify::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;
	int itok = 2;

	if(strcmp(args[itok], "rect") == 0){
		if(!rectify())
			return false;
		return true;
	}else if(strcmp(args[itok], "save") == 0){
		if(num_args != 5){
			cerr << "save <points file name> <camera parameters yml file name>" << endl;
			return false;
		}

		if(!write_pts(args[itok+1]) || !write_campars(args[itok+2]))
			return false;

		return true;
	}else if(strcmp(args[itok], "cba") == 0){
		if(num_args != 5){
			cerr << "cba <x size> <y size>" << endl;
			return false;
		}

		m_sz_chsbd.width = atoi(args[itok+1]);
		m_sz_chsbd.height = atoi(args[itok+2]);

		return true;
	}else if(strcmp(args[itok], "cbs") == 0){
		if(num_args != 4){
			cerr << "cbs <pitch>" << endl;
			return false;
		}
		m_pitch_chsbd = (float) atof(args[itok+1]);
		return true;
	}

	return f_base::cmd_proc(cmd);
}

bool f_stereo_rectify::proc()
{
	long long timg1;
	Mat img1 = m_pin1->get_img(timg1);
	long long timg2;
	Mat img2 = m_pin2->get_img(timg2);
	if(img1.empty() || img2.empty())
		return true;

	vector<Point2f> corners1, corners2;
	if(!findChessboardCorners(img1, m_sz_chsbd, corners1)){
		cout << "Cannot find chessboard in image1"  << endl;
		return true;
	}

	if(!findChessboardCorners(img2, m_sz_chsbd, corners2)){
		cout << "Cannot find chessboard in image2" << endl;
		return true;
	}
		
	static int num_det = 0;
	num_det++;
	cout << num_det << "th " 
	     << "Chessboards was detected" << endl;

	cornerSubPix(img1, corners1, Size(11, 11), Size(-1, -1),
		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	cornerSubPix(img2, corners2, Size(11, 11), Size(-1, -1),
		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	m_2dchsbd1.push_back(corners1);
	m_2dchsbd2.push_back(corners2);
	for(int i = 0; i < m_2dchsbd1.size(); i++){
		drawChessboardCorners(img1, m_sz_chsbd, m_2dchsbd1[i], true);
		drawChessboardCorners(img2, m_sz_chsbd, m_2dchsbd2[i], true);
	}

	m_pout1->set_img(img1, timg1);
	m_pout2->set_img(img2, timg2);
	return true;
}

bool f_stereo_rectify::rectify()
{
	long long timg1;
	Mat img1 = m_pin1->get_img(timg1);
	if(img1.empty())
		return false;
	
	m_3dchsbd.resize(m_sz_chsbd.height*m_sz_chsbd.width);
	for(int i= 0; i < m_sz_chsbd.height; i++){
		for(int j = 0; j < m_sz_chsbd.width; j++){
			int ipt = m_sz_chsbd.width * i + j;
			m_3dchsbd[ipt].x = m_pitch_chsbd * i;
			m_3dchsbd[ipt].y = m_pitch_chsbd * j;
			m_3dchsbd[ipt].z = 0;
		}
	}

	m_sterr = stereoCalibrate(m_3dchsbd, m_2dchsbd1, m_2dchsbd2, m_Mcam1, m_discoeff1, m_Mcam2, m_discoeff2, img1.size(), m_R, m_T, m_E, m_F, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5), CV_CALIB_FIX_INTRINSIC);

	stereoRectify(m_Mcam1, m_discoeff1, m_Mcam2, m_discoeff2, img1.size(), m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q);

	cout << "Camera Matrix1" << endl;
	cout << m_Mcam1 << endl;
	cout << "Camera Matrix2" << endl;
	cout << m_Mcam2 << endl;
	cout << "Distortion Coefficient1" << endl;
	cout << m_discoeff1 << endl;
	cout << "Distortion Coefficient2" << endl;
	cout << m_discoeff2 << endl;
	cout << "Reprojection err" << endl;
	cout << m_sterr << endl;
	cout << "Rotation Matrix" << endl;
	cout << m_R << endl;
	cout << "Translation vector" << endl;
	cout << m_T << endl;
	cout << "Essential Matrix" << endl;
	cout << m_E << endl;
	cout << "Fundamental Matrix" << endl;
	cout << m_F << endl;
	cout << "Rotation Matrix1" << endl;
	cout << m_R1 << endl;
	cout << "Rotation Matrix2" << endl;
	cout << m_R2 << endl;
	cout << "Projection Matrix1" << endl;
	cout << m_P1 << endl;
	cout << "Projection Matrix2" << endl;
	cout << m_P2 << endl;
	cout << "Disparity-to-Depth mapping Matrix" << endl;
	cout << m_Q << endl;
	
	
	return true;
}

bool f_stereo_rectify::write_campars(const char * fname)
{
	char buf[1024];
	snprintf(buf, 1024, "%s", fname);
	FileStorage fs(buf, FileStorage::WRITE);
	if(!fs.isOpened())
		return false;

	//save camera parameters
	fs << "CamInt1" << m_Mcam1;
	fs << "CamInt2" << m_Mcam2;
	fs << "CamDist1" << m_discoeff1;
	fs << "CamDist2" << m_discoeff2;
	fs << "R" << m_R;
	fs << "T" << m_T;
	fs << "E" << m_E;
	fs << "F" << m_F;
	fs << "R1" << m_R1;
	fs << "R2" << m_R2;
	fs << "P1" << m_P1;
	fs << "P2" << m_P2;
	fs << "Q" << m_Q;
	fs.release();
}

bool f_stereo_rectify::init_run()
{
	m_pin1 = dynamic_cast<ch_image*>(m_chin[0]);
	m_pin2 = dynamic_cast<ch_image*>(m_chin[1]);
	if(m_chin.size() != 2){
		cerr << m_name << "requires two input channel." << endl;
		return false;
	}
	if(m_pin1 == NULL || m_pin2 == NULL){
		cerr << m_name << "'s first and second input channel should be image channel." << endl;
		return false;
	}
	m_pout1 = dynamic_cast<ch_image*>(m_chout[0]);
	m_pout2 = dynamic_cast<ch_image*>(m_chout[1]);

	FileStorage fs(cam1_fname, FileStorage::READ);
	if(!fs.isOpened()){
		cerr << "Couldn't open " << cam1_fname << endl;
		return false;
	}
	fs["CamInt"] >> m_Mcam1;
	fs["CamDist"] >> m_discoeff1;
	
	fs.release();
	fs.open(cam2_fname, FileStorage::READ);
	if(!fs.isOpened()){
		cerr << "Couldnt open" << cam2_fname << endl;
		return false;
	}
	fs["CamInt"] >> m_Mcam2;
	fs["CamDist"] >> m_discoeff2;
	fs.release();
	return true;
}

void f_stereo_rectify::destroy_run()
{
}
