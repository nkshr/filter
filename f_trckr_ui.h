#ifndef _F_TRCKR_UI_H_
#define _F_TRCKR_UI_H_
class f_trckr_ui:protected f_aws1_ui
{
private:
  Rect m_src_rect;
  vector<Rect> vrect;
  bool m_drag;

public:
  f_trckr_ui(const char *name);
  virtual bool init_run();
  virtual bool proc();
  virtual void _key_callback(int key, int scancode, int action, int mods);
  virtual void _cursor_position_callback(double xpos, double ypos);
  virtual void _mouse_button_callback(int button, int action, int mods);
};
#endif


