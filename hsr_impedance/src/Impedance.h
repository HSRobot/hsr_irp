#ifndef __IMPEDANCE__H__
#define __IMPEDANCE__H__

#include <iostream>
#include <vector>
//类定义
class Impedance
{
	public:
	
		void reImpedance( std::vector<double> &Force,int &ERROR_ImpAcc, int &ERROR_ImpVel, std::vector<double> &Xa);		//相对偏移
		//void abImpedance( double Force[6] );								//绝对偏移
		void setStiffness( double Stiffness[6]);		//设置刚性
		void setDamping( double Damping[6]);			//设置阻尼
		void getSensor( double Force[6] );			//获取传感器数据
		
		double m_Mass[6];		//质量

		double m_Force[6];
		Impedance();			// 构造函数
		
	private:
		double m_Stiffness[6];		//刚性
		double m_Damping[6];		//阻尼
		double ts;
		double ddXa[6];	
		double dXa[6];
		double dXa_1[6];
		double Xa_1[6];
};

#endif
