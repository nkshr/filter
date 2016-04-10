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
#include <gear_util.h>
using namespace gear;

#include "f_trckr.h"

f_trckr::f_trckr(const char *name): f_glfw_imview(name), m_drag(false), m_lock(false), m_tc(TermCriteria::COUNT+TermCriteria::EPS, 30, 3.0e-3),m_pyr_lvl(0), m_src_rect(10, 10, 100, 100), m_thickness(1), m_sz_comp(640, 480), m_comp(false), m_group(Aff2)
{
  cout << "Entering constructor" << endl;
  register_fpar("ch_image", (ch_base**)&m_pin, typeid(ch_image).name(),
		"Image channel");
  register_fpar("epsilon", &m_tc.epsilon, "Threshold for Inverse Compositional Algorithm. If root mean square for delta parameters is less than epsilon, Inverse Compositional Algorithm is stopped. (default 3e-3)");
  register_fpar("maxCount", &m_tc.maxCount, "Threshold for Inverse Compositional Algorithm. If the number of iteration step is larger than maxCount, Inverse Compositional Algorithm is stopped. (default 30)");
  register_fpar("pyr_lvl", &m_pyr_lvl, "Pyramid level for Pyramid image in Inverse Composition(default 0)");
  register_fpar("thickness", &m_thickness, "Thickness of rectangle's line (default 0)");
  register_fpar("comp", &m_comp, "flag for image compression (default no)");
  register_fpar("comp_width", &m_sz_comp.width, "image width is resized to comp_width (default 640)");
  register_fpar("comp_height", &m_sz_comp.height, "image height is resized to comp_height");
  register_fpar("alpha0", &m_alpha_array[0], "alpha is gain for updating parameters.alpha0 (default 0)");
  register_fpar("alpha1", &m_alpha_array[1], "alpha1 (default 0)");
  register_fpar("alpha2", &m_alpha_array[2], "alpha2 (default 0)");
  register_fpar("alpha3", &m_alpha_array[3], "alpha3 (default 0)");
  register_fpar("alpha4", &m_alpha_array[4], "alpha4 (default 0)");
  register_fpar("alpha5", &m_alpha_array[5], "alpha5 (default 0)");
  //register_fpar("group", &m_group, "group which warping model belongs to. (default Aff2)");
  for(int i = 0; i < 6; ++i)
    m_alpha_array[i] = 0;
  cout <<"Exiting constructor" << endl;
}

bool f_trckr::init_run()
{
  if(!f_glfw_imview::init_run())
    return false;  
  return true;
}

bool f_trckr::proc()
{
  glfwMakeContextCurrent(pwin());

  if(glfwWindowShouldClose(pwin()))
    return false;
  
  long long timg;
  Mat img = m_pin->get_img(timg);
  
if(img.empty())
    return true;
  if(m_timg == timg)
    return true;
  
  m_timg = timg;
  flip(img, img, 0);
  if(m_comp){
    resize(img, img, m_sz_comp);
  }
  Scalar color;
  m_offset_x = img.cols/2;
  m_offset_y = img.rows/2;
  if(m_lock){
    imshow("template image", m_tmpl_img);
    waitKey(30);
    Mat mask = Mat(m_tmpl_img.size(), CV_8U, Scalar(255));
    Mat alpha; 
    switch(m_group){
    case SE2:
      alpha.create(3, 1, CV_64F);
      break;
    case Aff2:
      alpha.create(6, 1, CV_64F);       
      break;
    case Sim2:
      alpha.create(4, 1, CV_64F);
      break;
    }
    double *palpha = alpha.ptr<double>(0);
    for(int i = 0; i < alpha.rows; ++i)
      palpha[i] = m_alpha_array[i];

    m_target.set_pars(m_init_pars, alpha, m_tc, Aff2, m_pyr_lvl);
    m_target.run(img, m_tmpl_img, mask);
    vector<Point2d> pts;
    Point2f dest_pt, src_pt;
    Mat pars;
    m_target.get_pars(pars);
    m_init_pars = pars;
    Mat Aff2;
    getAff2(pars, Aff2);
    color = Scalar(0, 212, 255);
    draw_drapezoid(m_src_rect, Aff2, img, color, m_thickness);
  }
  else{
    color = Scalar(255, 255, 255);
    //check if m_src_rect is in window
    if(m_src_rect.x < 0 || m_src_rect.y < 0 
       || m_src_rect.x+m_src_rect.width >= img.cols 
       || m_src_rect.y+m_src_rect.height>=img.rows){
      cout << "rectangle is out of window" << endl;
    }
    else{
      m_tmpl_img = Mat(img, m_src_rect).clone();
      rectangle(img, m_src_rect, color, m_thickness);
    }
  }

  if(m_sz_win.width != img.cols || m_sz_win.height != img.rows)
    resize(img, img, m_sz_win);
  if(img.type() == CV_8UC3)
    cvtColor(img, img, CV_BGR2RGB);
  else
    cvtColor(img, img,CV_GRAY2RGB);

  glRasterPos2i(-1, -1);	
  glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);

  float hfont = (float)(24. / (float) m_sz_win.height);
  float wfont = (float)(24. / (float) m_sz_win.height);
  float x = wfont - 1;
  float y = 1 - 2 * hfont;

  // show time
  drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
  y-= 2*hfont;
  if(m_lock){
    char pars_str[256];
    double *pm_init_pars = m_init_pars.ptr<double>(0);
    sprintf(pars_str, "affine parameters : %05f %05f %05f", pm_init_pars[0], pm_init_pars[1], pm_init_pars[2]);
    drawGlText(x, y, pars_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
    y -= 2*hfont;
    sprintf(pars_str, "%05f %05f %05f", pm_init_pars[3], pm_init_pars[4], pm_init_pars[5]);
    drawGlText(x, y, pars_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
  } 
  else{
    char rect_pos_str[30];
    int rect_pos_x, rect_pos_y;
    rect_pos_x= m_src_rect.x +m_src_rect.width/2-  m_offset_x;
    rect_pos_y = m_src_rect.y + m_src_rect.height/2 - m_offset_y;
    sprintf(rect_pos_str, "rect pos : %03d %03d", rect_pos_x, rect_pos_y);
    drawGlText(x, y, rect_pos_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
    char rect_sz_str[30];
    y -= 2*hfont;
    sprintf(rect_sz_str, "rect size : %03d %03d", m_src_rect.width, m_src_rect.height);
    drawGlText(x, y, rect_sz_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
 }
  glfwSwapBuffers(pwin());
  glfwPollEvents();
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
  case GLFW_KEY_UP:
    if(mods == GLFW_MOD_SHIFT)
      m_src_rect.y += 3;
    else
      m_src_rect.y++;
    break;
  case GLFW_KEY_DOWN:
    if(mods == GLFW_MOD_SHIFT)
      m_src_rect.y -= 3;
    else
      m_src_rect.y--;
    break;
  case GLFW_KEY_RIGHT:
    if(mods == GLFW_MOD_SHIFT)
      m_src_rect.x += 3;
    else
      m_src_rect.x++;
    break;
  case GLFW_KEY_LEFT:
    if(mods == GLFW_MOD_SHIFT)
      m_src_rect.x -= 3;
    else
      m_src_rect.x--;
    break;

    //enable reset process
  case GLFW_KEY_SPACE:
    m_src_rect.x = 10;
    m_src_rect.y = 10;
    m_lock = false;
    break;

    //lock target 
  case GLFW_KEY_ENTER:
    m_lock = true;
    m_init_pars = (Mat_<double>(6,1) 
		     << 0, 0, 0, 0,
		     m_src_rect.x + m_src_rect.width/2 - m_offset_x,
		     m_src_rect.y + m_src_rect.height/2 - m_offset_y);

    break;
  default:
    break;
  }
}

void f_trckr::_cursor_position_callback(double xpos, double ypos)
{
  ypos *= -1;
  ypos += m_sz_win.height;
  
  if(m_drag){
    m_src_rect.x = xpos - m_src_rect.width/2;
    m_src_rect.y = ypos - m_src_rect.height/2;
  }
}

void f_trckr::_mouse_button_callback(int button, int action, int mods)
{
  double xpos, ypos;
  glfwGetCursorPos(pwin(), &xpos, &ypos);
  ypos *= -1;
  ypos += m_sz_win.height;
  if(action == GLFW_PRESS &&
     xpos > m_src_rect.x && xpos < m_src_rect.x + m_src_rect.width &&
     ypos > m_src_rect.y && ypos < m_src_rect.y + m_src_rect.height){
    m_drag = true;
  }
  else{
    m_drag = false;
  }
}

