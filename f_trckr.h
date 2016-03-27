#ifndef _F_AWS1_TRCKR_UI_H_
#define _F_AWS1_TRCKR_UI_H_

class f_trckr: public f_glfw_imview
{
 private:
  Rect m_src_rect;
  bool m_drag, m_lock;
  PyrICIA m_target;
  Mat m_pre_img, m_mask;
  TermCriteria m_tc;
 public:
  f_trckr(const char * name);
  virtual bool init_run();
  virtual bool proc();
  virtual void _key_callback(int key, int scancode, int action, int mods);
  virtual void _cursor_position_callback(double xpos, double ypos);
  virtual void _mouse_button_callback(int button, int action, int mods);
};
#endif
