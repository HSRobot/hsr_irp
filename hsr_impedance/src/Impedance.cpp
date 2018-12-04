#include "Impedance.h"
//成员函数定义

Impedance::Impedance(){
	ts = 0.04;
	m_Mass[0] = 1.0;m_Mass[1] = 1.0;m_Mass[2] = 1.0;
	m_Mass[3] = 1.0;m_Mass[4] = 1.0;m_Mass[5] = 1.0;
}

void Impedance::setStiffness(double Stiffness[6])
{
	
	m_Stiffness[0] = Stiffness[0];
	m_Stiffness[1] = Stiffness[1];
	m_Stiffness[2] = Stiffness[2];
	m_Stiffness[3] = Stiffness[3];
	m_Stiffness[4] = Stiffness[4];
	m_Stiffness[5] = Stiffness[5];
	
}
void Impedance::setDamping(double Damping[6])
{
	for (int i = 0; i < 6; i++)
	{
		m_Damping[i] = Damping[i];
	}
}

void Impedance::getSensor( double Force[6] )	//输出Force
{
	//memccpy(Force, src, sizeof(double)*6);	//memccpy整段复制数组
}

void Impedance::reImpedance( std::vector<double> &Force,int &ERROR_ImpAcc, int &ERROR_ImpVel, std::vector<double> &Xa )	//或者*Xa
{	
	
	for (int i = 0; i < 6; i++)
	{
		ddXa[i] = ( Force[i] - m_Damping[i]*dXa_1[i] - m_Stiffness[i]*Xa_1[i] ) / m_Mass[i];
		//阻抗偏移加速度限制
		if(ddXa[i] > 10.0 || ddXa[i] < -10.0)
			ERROR_ImpAcc = i;
		
		
		dXa[i] = dXa_1[i] + ddXa[i] * ts;
		//阻抗偏移速度限制
		if(dXa[i] > 2.0 || dXa[i] < -2.0)
			ERROR_ImpVel = i;
		
		
		Xa[i] = Xa_1[i] + dXa[i] * ts;
		
		//变量回归
		dXa_1[i] = dXa[i];
		Xa_1[i] = Xa[i]; 
		//std::cout << "Xa----" << i << "=" << Xa[i] << std::endl;
	}
}


