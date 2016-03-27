#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <map>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <GL/glew.h>

#include <GLFW/glfw3.h>
#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glut.h>
#include <GL/glu.h>

#include "../util/aws_sock.h"
#include "f_glfw_window.h"
 
//#include "../channel/ch_state.h"

#include <gear_transformation.h>
#include <gear_img_align.h>
using namespace gear;

#include "f_trckr.h"

f_trckr::f_trckr(const char *name): f_glfw_imview(name), m_src_rect(10,10,64,48), m_drag(false), m_lock(false), m_tc(TermCriteria::COUNT+TermCriteria::EPS, 30, 3.0e-3)
{
  m_target = PyrICIA(Aff2,1);
  register_fpar("ch_image", (ch_base**)&m_pin, typeid(ch_image).name(),
		"Image channel");
  register_fpar("epsilon", &m_tc.epsilon, "Threshold for Inverse Compositional Algorithm. If root mean square for delta parameters is less than epsilon, Inverse Compositional Algorithm is stopped. (default 3e-3)");
  register_fpar("maxCount", &m_tc.maxCount, "Threshold for Inverse Compositional Algorithm. If the number of iteration step is larger than maxCount, Inverse Compositional Algorithm is stopped. (default 30)");
}

bool f_trckr::init_run()
{
  if(!f_glfw_imview::init_run())
    return false;
  
  m_src_rect.width = m_sz_win.width/10;
  m_src_rect.height = m_sz_win.height/10;
  m_src_rect.x = m_sz_win.width/64;
  m_src_rect.y = m_sz_win.height/48;
  
  return true;
}

bool f_trckr::proc()
{
  glfwMakeContextCurrent(pwin());

  if(glfwWindowShouldClose(pwin()))
    return false;
  
  long long timg;
  Mat img;// = m_pin->get_img(timg);
  char img_name[256];
  static int count = 60; 
  sprintf(img_name, "/media/ubuntu/ssd0/2011_09_26/2011_09_26_drive_0001_extract/image_00/data/%010d.png", count);
  count++;
  img = imread(img_name);
  if(img.empty())
    return true;
  //if(m_timg == timg)
  //  return true;
  
  m_timg = timg;
  
  if(m_sz_win.width != img.cols || m_sz_win.height != img.rows){
    Mat tmp;
    resize(img, tmp, m_sz_win);
    img = tmp;
  }
  
  Scalar color;
  if(m_lock){
    m_mask = Mat::zeros(m_sz_win, CV_8U);
    rectangle(m_mask, m_src_rect, Scalar(1), -1);
    Mat init_pars = (Mat_<double>(6,1) 
		     << 0, 0, 0, 0,
		     m_src_rect.x + m_src_rect.width/2,
		     m_src_rect.y + m_src_rect.height/2);
    m_target.set_pars(init_pars, m_tc, 1);
    m_target.run(img, m_pre_img, m_mask);
    vector<Point2d> pts;
    Point2d pt;
    Mat pars;
    m_target.get_pars(pars);
    Mat Aff2;
    getAff2(pars, Aff2);
    double *pAff2 = Aff2.ptr<double>(0);
    pt.x = pAff2[0] * m_src_rect.x 
      + pAff2[1] * m_src_rect.y + pAff2[2];
    pt.y = pAff2[3] * m_src_rect.x 
      + pAff2[4] * m_src_rect.y + pAff2[5];
    pts.push_back(pt);
    pt.x = pAff2[0] * (m_src_rect.x+m_src_rect.width) 
      + pAff2[1] * m_src_rect.y + pAff2[2];
    pt.y = pAff2[3] * (m_src_rect.x+m_src_rect.width) 
      + pAff2[4] * m_src_rect.y + pAff2[5];
    pts.push_back(pt);
    pt.x = pAff2[0] * m_src_rect.x 
      + pAff2[1] * (m_src_rect.y+m_src_rect.height) + pAff2[2];
    pt.y = pAff2[3] * m_src_rect.x 
      + pAff2[4] * (m_src_rect.y+m_src_rect.height) + pAff2[5];
    pts.push_back(pt);
    pt.x = pAff2[0] * (m_src_rect.x+m_src_rect.width) 
      + pAff2[1] * (m_src_rect.y+m_src_rect.height) + pAff2[2];
    pt.y = pAff2[3] * (m_src_rect.x+m_src_rect.width) 
      + pAff2[4] * (m_src_rect.y+m_src_rect.height) + pAff2[5];
    pts.push_back(pt);
    m_src_rect = boundingRect(pts);
    color = Scalar(0, 212, 255);
  }
  else
    color = Scalar(255, 255, 255);
  rectangle(img, m_src_rect, color);
  glRasterPos2i(-1, -1);
	
  if(img.type() == CV_8U){
    glDrawPixels(img.cols, img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.data);
  }
  else{
    cnvCVBGR8toGLRGB8(img);
    glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);
  }

  float hfont = (float)(24. / (float) m_sz_win.height);
  float wfont = (float)(24. / (float) m_sz_win.height);
  float x = wfont - 1;
  float y = 1 - 2 * hfont;

  // show time
  drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  glfwSwapBuffers(pwin());
  glfwPollEvents();
  m_pre_img = img;
  return true;
}

void f_trckr::_key_callback(int key, int scancode, int action, int mods)
{
  switch(key){
  case GLFW_KEY_W:
    if(mods == GLFW_MOD_SHIFT)
      m_src_rect.width++;
    else
      m_src_rect.width--;
    break;
  case GLFW_KEY_H:
    if(mods == GLFW_MOD_SHIFT)
      m_src_rect.height++;
    else
      m_src_rect.height--;
    break;
    
    //enable reset process
  case GLFW_KEY_SPACE:
    m_src_rect.x = m_sz_win.width/64;
    m_src_rect.y = m_sz_win.height/48;
    m_lock = false;
    break;

    //lock target 
  case GLFW_KEY_ENTER:
    m_lock = true;
    break;
  default:
    break;
  }
}

void f_trckr::_cursor_position_callback(double xpos, double ypos)
{
  if(m_drag){
    m_src_rect.x = xpos - m_src_rect.width/2;
    m_src_rect.y = ypos - m_src_rect.height/2;
  }
}

void f_trckr::_mouse_button_callback(int button, int action, int mods)
{
  double xpos, ypos;
  glfwGetCursorPos(pwin(), &xpos, &ypos);
  if(action == GLFW_PRESS &&
     xpos > m_src_rect.x && xpos < m_src_rect.x + m_src_rect.width &&
     ypos > m_src_rect.y && ypos < m_src_rect.y + m_src_rect.height){
    m_drag = true;
  }
  else{
    m_drag = false;
  }
}

