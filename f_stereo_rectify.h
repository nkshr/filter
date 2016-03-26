// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_camcalib.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_camcalib.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_camcalib.h.  If not, see <http://www.gnu.org/licenses/>. 

class f_stereo_rectify: public f_base
{
protected:
	ch_image * m_pin1, * m_pin2;
	ch_image * m_pout1, * m_pout2;
	Size m_sz_chsbd;
	float m_pitch_chsbd;
	vector<Point3f> m_3dchsbd;
	vector<vector<Point2f > > m_2dchsbd1, m_2dchsbd2;
	Mat m_Mcam1, m_Mcam2, m_discoeff1, m_discoeff2,
		m_R, m_T, m_E, m_F, m_R1, m_R2, m_P1, m_P2, m_Q;
	char * cam1_fname, * cam2_fname;
	double m_err1, m_err2, m_sterr;

	bool write_pts(const char * fname);
public:
f_stereo_rectify(const char * name):f_base(name), m_pin1(NULL), m_pin2(NULL), 
		m_sz_chsbd(6, 9), m_pitch_chsbd(0.0254f/*meter*/)
	{
		register_fpar("p", &m_pitch_chsbd, "Pitch of the chesboard (0.0254m default)");
		register_fpar("v", &(m_sz_chsbd.height), "Number of vertical grids in the chessboard (6 default");
		register_fpar("h", &(m_sz_chsbd.width), "Number of horizontal grids in the chessboard (9 default)");
		cam1_fname = new char[1024];
		cam2_fname = new char[1024];
		register_fpar("cam1_fname", cam1_fname, "Camera1 parameters yml file");
		register_fpar("cam2_fname", cam2_fname, "Camera2 parameters yml file");
	}

	virtual ~f_stereo_rectify(){
	}

	virtual bool init_run();
	virtual void destroy_run();

	virtual bool check()
	{
		return m_chin[0] != NULL;
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();
	bool rectify();
	bool write_campars(const char * fname);
};
