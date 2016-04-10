#include "stdafx.h"

#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <GLFW/glfw3.h>

#include "f_base.h"
#include "f_aws1_ui.h"
#include "f_trckr_ui.h"
f_trckr_ui::f_trckr_ui(const char *name): f_aws1_ui(name){}

bool f_trckr_ui::init_run()
{
  if(!f_aws1_ui::init_run())
    return false;
  return true;
}

bool f_trckr_ui::proc()
{
  if(!f_aws1_ui::proc())
    return false;
  return true;
}

void f_trckr_ui::_key_callback(int key, int scancode, int action, int mods)
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
    break;

    //lock target 
  case GLFW_KEY_ENTER:
    vrect.push_back(m_src_rect);
    break;
  default:
    break;
  }
}


void f_trckr_ui::_cursor_position_callback(double xpos, double ypos)
{
  ypos *= -1;
  ypos += m_sz_win.height;
  
  if(m_drag){
    m_src_rect.x = xpos - m_src_rect.width/2;
    m_src_rect.y = ypos - m_src_rect.height/2;
  }
}

void f_trckr_ui::_mouse_button_callback(int button, int action, int mods)
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

