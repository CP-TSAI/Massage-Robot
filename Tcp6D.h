#ifndef _TCP6D_H
#define _TCP6D_H

class Tcp6D
{
public:
	double x, y, z, a, b, c; // �����I(end-effector)��6�Ӧۥѫ�
	Tcp6D(double x_pos=0.65, double y_pos=0, double z_pos=0, double a_ori=-90, double b_ori=0, double c_ori=180):
		x(x_pos), y(y_pos), z(z_pos), a(a_ori), b(b_ori), c(c_ori){} // �غc�l



	// ��J���]�w���Ȫ��������
	void input( double x_pos, double y_pos, double z_pos, double a_ori, double b_ori, double c_ori) 
	{
		x = x_pos;
		y = y_pos;
		z = z_pos;
		a = a_ori;
		b = b_ori;
		c = c_ori;
	}
};

#endif _TCP6D_H