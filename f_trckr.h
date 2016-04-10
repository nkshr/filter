#ifndef _F_TRCKR_H_
#define _F_TRCKR_H_

class f_trckr: public f_glfw_imview
{
 private:
  Rect m_src_rect, m_cur_rect;
  bool m_drag, m_lock, m_comp;
  PyrICIA m_target;
  TermCriteria m_tc;
  int m_pyr_lvl, m_offset_x, m_offset_y, m_thickness;
  Mat m_tmpl_img, m_init_pars;
  Size m_sz_comp;
  double m_alpha_array[6];
  e_group m_group;
 public:
  f_trckr(const char * name);
  virtual bool init_run();
  virtual bool proc();
  virtual void _key_callback(int key, int scancode, int action, int mods);
  virtual void _cursor_position_callback(double xpos, double ypos);
  virtual void _mouse_button_callback(int button, int action, int mods);
};
#endif
