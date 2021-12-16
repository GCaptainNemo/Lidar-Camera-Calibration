#include "ceres/ceres.h"
#include "Eigen/Core"
#include "ceres/rotation.h"

// #define UN_DISTORTION

double pt3_lst[] = 
{
	2.503, 0.406, 0.029,
	2.368, 0.397, -0.543,
	2.177, -0.367, -0.485,
	2.385, -0.379, 0.077,
	2.478, 0.394, 0.022,
	2.374, 0.4, -0.544,
	2.153, -0.36, -0.48,
	2.37, -0.374, 0.066
};

double pt2_lst[] = 
{
	661, 523.253,
	633, 968,
	1266.5, 964.157,
	1245.06, 500.534,
	660.657, 523.483,
	631.771, 967.607,
	1266.45, 964.192,
	1245.19, 500.664
};

struct CostFunctor
{
	public:
		// T pt3_[3];
		// T pt2_[2];
		// T fx, fy, cx, cy;
		// T k1, k2, p1, p2, k3;
		double pt3_[3];
		double pt2_[2];
		double fx_, fy_, cx_, cy_;
		double k1_, k2_, p1_, p2_, k3_;

	// CostFunctor(T pt_3[3], T pt_2[2])
	CostFunctor(double pt_3[3], double pt_2[2])
	{
		for(int i=0; i < 3; ++i){
			pt3_[i] = pt_3[i];
		}
		for(int i=0; i < 2; ++i){
			pt2_[i] = pt_2[i];
		}
		this->fx_ = 1.927941866078295e+03;
		this->fy_ = 1.929172443177110e+03;
		cx_ = 9.595316580313911e+02;
		cy_ = 5.144334699977888e+02;
		k1_ = -0.2272996302077433;
		k2_ = 0.244566784018986;
		p1_ = 4.524726839204985e-05;
		p2_ = -0.0008645881261561915;
		k3_ = 0.0;
	};
	
	// 输入euler角，平移坐标
	template <class T>
	bool operator ()(const T * const rot, const T * const trans, T *residual) const
	{
		T guiyihua[3];
		T pt3[3];
		pt3[0] = T(pt3_[0]);
		pt3[1] = T(pt3_[1]);
		pt3[2] = T(pt3_[2]);
		// rodriguez vector 使用李代数
		ceres::AngleAxisRotatePoint(rot, pt3, guiyihua);
		guiyihua[0] += trans[0];
		guiyihua[1] += trans[1];
		guiyihua[2] += trans[2];
		const T x = guiyihua[0] / guiyihua[2];
		const T y = guiyihua[1] / guiyihua[2];
		// 在归一化平面进行畸变矫正
		#ifdef UN_DISTORTION
		const T r2 = x * x + y * y;
		const T r4 = r2 * r2;
		const T undist_x = x + k1_ * r2 + k2_ * r4 + k3_ * r2 * r4 + (p1_ * x * y + p1_ * x * y) + p2_ * (r2 + x * x + x * x);
		const T undist_y = y + k1_ * r2 + k2_ * r4 + k3_ * r2 * r4 + (p2_ * x * y + p2_ * x * y) + p1_ * (r2 + y * y + y * y);

 		const T u = undist_x * fx_ + cx_;
    	const T v = undist_y * fy_ + cy_;
		#else
		const T u = x * fx_ + cx_;
		const T v = y * fy_ + cy_;
		#endif
		residual[0] = u - pt2_[0];
    	residual[1] = v - pt2_[1];
		return true;
	}
};



int main()
{	// ///////////////////////////////
	// ground truth
	// //////////////////////////////
	// Eigen::Matrix3d rot_mat;
	// rot_mat << -0.0468903,  -0.998525,  0.0273622, 
	// 			0.0452459,  -0.0294874,  -0.998541, 
	// 			0.997875,  -0.0455838,  0.0465618;
	// Eigen::AngleAxisd init_vec ;
	// init_vec.fromRotationMatrix(rot_mat);
	// std::cout << "angle = " << init_vec.angle() << init_vec.axis().transpose() << std::endl;
	// Eigen::Vector3d init_trans, init_rot; 
	// init_trans << 0.111005, -0.063236, 0.119411;
	// init_rot << 2.1117 * 0.555825, 2.1117 * -0.566065,  2.1117 * 0.608793;

	Eigen::Vector3d init_trans, init_rot; 
	init_rot << 1, 1, 1;
	init_trans << 1, 2, 3;

	double rot[3];
	double trans[3];
	for(int i=0; i < 3; ++i){
		rot[i] = init_rot[i];
		trans[i] = init_trans[i];
	}
		
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
  	options.minimizer_progress_to_stdout = true;

	// Step 4. 构建问题，添加残差块(p)
	ceres::Problem problem;
	int size = sizeof(pt2_lst) / sizeof(double) / 2;
	std::cout << "size = " << size << std::endl;
	for (int i = 0; i < size; ++i)
	{
		double _pt3[] = {pt3_lst[3 * i], pt3_lst[3 * i + 1], pt3_lst[3 * i + 2]};
		double _pt2[] = {pt2_lst[2 * i], pt2_lst[2 * i + 1]};
		problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 2, 3, 3>
		(new CostFunctor(_pt3, _pt2)), nullptr, rot, trans);
	}
	// Step 5. 构建问题输出报告(s)
	ceres::Solver::Summary summary;
	
	// Step 6. 开始求解（o-p-s）
	Solve(options, &problem, &summary);
	Eigen::Vector3d rotate_vec;
	rotate_vec << rot[0], rot[1], rot[2];
	Eigen::AngleAxisd roderigus_vec(rotate_vec.norm(), rotate_vec / rotate_vec.norm());
	std::cout << "rotate_matrix = " << "\n"<< roderigus_vec.matrix() << std::endl;

	std::cout << summary.BriefReport() << "\n";
	std::cout << "rot: " << rot[0] << rot[1] << rot[2]
				<< "trans: " << trans[0] << trans[1] << trans[2] << "\n";
	return 0;
}