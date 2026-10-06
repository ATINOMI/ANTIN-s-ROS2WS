/**
 * @file use-ikfom.hpp
 * @brief FAST-LIO2 的状态组合、IMU 连续过程模型及状态/噪声雅可比。状态 DIM=24、DOF=23，重力模长为 9.809。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

/**
 * @brief 生成 FAST-LIO2 组合状态。
 * @note pos、rot、vel、grav 使用同一惯性参考系；rot 将 IMU 向量转入该系。offset_R_L_I/offset_T_L_I 是 LiDAR 到 IMU 的旋转/平移。
 * @note 误差索引：pos 0、rot 3、外参旋转 6、外参平移 9、vel 12、bg 15、ba 18、grav 21（2 DOF）。过程导数中的 grav 占 3 维。
 */
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);

/**
 * @brief 生成 IMU 过程输入：acc 为 IMU 系加速度（m/s²），gyro 为角速度（rad/s）。
 */
MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);

/**
 * @brief 生成 12 维过程噪声：陀螺仪、加速度计噪声及两种偏置随机游走，按 ng、na、nbg、nba 排列。
 */
MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

/**
 * @brief 构造默认过程噪声对角协方差。
 * @return 12×12 矩阵；ng/na 对角值 1e-4，nbg/nba 对角值 1e-5，各交叉项为零。
 * @note 本函数不乘 dt；predict 中噪声雅可比乘 dt 后在两侧传播 Q。
 */
MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia 
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
/**
 * @brief 计算 IMU 连续时间过程导数。
 * @param s 当前状态，函数仅读取其值。
 * @param in IMU 系加速度和角速度输入。
 * @return 24 维导数：位置为 vel，姿态为 gyro-bg，速度为 rot·(acc-ba)+grav，其余分量为零。
 */
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	/* 加速度先在 IMU 系扣除偏置，再旋转并加惯性系重力；改变顺序会混用坐标系。 */
	vect3 a_inertial = s.rot * (in.acc-s.ba); 
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) =  omega[i]; 
		res(i + 12) = a_inertial[i] + s.grav[i]; 
	}
	return res;
}

/**
 * @brief 计算过程导数对 23 维状态局部误差的雅可比。
 * @param s 当前状态，提供姿态与重力局部基。
 * @param in IMU 输入，用 acc-ba 构造姿态耦合。
 * @return 24×23 矩阵；行按 DIM 排列、列按 DOF 排列。
 */
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	/* 重力模长固定，只估计方向：S2_Mx 将 2 维局部误差映射到 3 维重力变化，不能替换成 3×3 单位阵。 */
	s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 2>(12, 21) =  grav_matrix; 
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 
	return cov;
}


/**
 * @brief 计算过程导数对 12 维噪声的雅可比。
 * @param s 当前状态，提供加速度噪声到惯性系的旋转。
 * @param in 保留过程回调签名一致性，当前未使用。
 * @return 24×12 矩阵，噪声列按 ng、na、nbg、nba 排列。
 */
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
	return cov;
}

/**
 * @brief 将姿态四元数转换为显示用欧拉角。
 * @param orient 有效姿态四元数，系数顺序为 x、y、z、w。
 * @return roll、pitch、yaw，使用 57.3 近似乘数转为角度（deg）。
 * @note 与 RotMtoEuler 的弧度输出不同；接近俯仰 ±90° 时设 yaw=0。零四元数会导致除以 unit=0。
 */
vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif