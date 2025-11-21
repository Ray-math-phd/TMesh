#pragma once
#include <algorithm>
#include <charconv>
#include <map>
#include <iterator>
#include <set>

#include <easy3d/core/surface_mesh.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <Eigen/OrderingMethods>
#include <Eigen/IterativeLinearSolvers>

#include "Tspline.h"

namespace TMeshOptimizer {
	/**
	 * the basis optimizer class.
	 */
	class BasisOptimizer {
	public:
		BasisOptimizer(easy3d::SurfaceMesh* mesh, easy3d::SurfaceMesh* mesh_param, tspline::Tspline* mesh_tsp);
		~BasisOptimizer() {}
	public:
		void evaluate_single_tface_error(tspline::Tspline& tsp,
			tspline::Tspline::Face_iterator& fit,
			std::vector<easy3d::dvec3>& mesh,
			std::vector<easy3d::dvec3>& mesh_param,
			unsigned min_point_count,
			double& face_error);
		void evaluate_all_tface_error(tspline::Tspline& tsp,
			std::vector<easy3d::dvec3>& mesh,
			std::vector<easy3d::dvec3>& mesh_param,
			unsigned min_point_count,
			std::vector<double>& all_face_error);
		void evaluate_dis(Eigen::Vector3d& p1, Eigen::Vector3d& p2, double& result);
		void get_params_in_face(
			tspline::Tspline& tsp,
			tspline::Tspline::Face_iterator& fit,
			std::vector<easy3d::dvec3>& params,
			std::vector<int>& params_in_face);
		void update_n_vertex_in_tfaces(tspline::Tspline& tsp, std::vector<int>& n_param);
		void update_n_vertex_in_tfaces(tspline::Tspline& tsp, std::vector<easy3d::dvec3>& param_double);
		void update_all_tface_error(tspline::Tspline& tsp, std::vector<double>& all_tface_error);

		//========================================================
		void convert_vec3_2_dvec3(easy3d::SurfaceMesh* mesh, std::vector<easy3d::dvec3>& position_dvec3);

	public:
		easy3d::SurfaceMesh* mesh;
		easy3d::SurfaceMesh* mesh_param;
		tspline::Tspline* mesh_tsp;
	};
	
	class KKTSolver {
	public:
		KKTSolver(const Eigen::SparseMatrix<double>& K_, const Eigen::SparseMatrix<double>& A_,
			const Eigen::VectorXd& b_, const Eigen::VectorXd& c_,
			Eigen::VectorXd& x, Eigen::VectorXd& lambda);
		~KKTSolver() {}
	public:
		void Solve();
	public:
		const Eigen::SparseMatrix<double> K;//Coeff matrix
		const Eigen::SparseMatrix<double> A;//constrained matrix
		const Eigen::VectorXd b;
		const Eigen::VectorXd c;
		Eigen::VectorXd x;
		Eigen::VectorXd lambda;
	};

	class NewtonIte {
	public:
		NewtonIte(Eigen::VectorXd& x_, const int& max_iterations_);
		NewtonIte(Eigen::VectorXd& x_, const int& max_iterations_, Eigen::VectorXd& lambda);
		~NewtonIte() {}
	public:
		void Solve();
		void Solve_KKT();
	public:
		Eigen::VectorXd x;
		Eigen::VectorXd lambda;
		bool converged;
		int max_iterations;
		double final_residual;
	};

	//==================================================================================================
	/**
	 *  Local Optimizer.
	 * .
	 */
	class LocalTMeshOptimizer :public BasisOptimizer {
	public:
		LocalTMeshOptimizer(BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp), 
			epsilon(0.0), min_Point_count(0),
			parent(parent_) {}
		LocalTMeshOptimizer(double& epsilon_, unsigned& min_point_count_,
			BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp),
			epsilon(epsilon_), min_Point_count(min_point_count_),
			parent(parent_) {}

		~LocalTMeshOptimizer() {}
	public:
		void Local_LSM();

	protected:
		double epsilon;
		unsigned min_Point_count;
		BasisOptimizer* parent;
	};


	/**
	 *	Global Optimizer.
	 * .
	 */
	class GlobalTMeshOptimizer :public BasisOptimizer {
	public:
		GlobalTMeshOptimizer(BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp), 
			epsilon(0.0), min_Point_count(0),
			parent(parent_) {}
		GlobalTMeshOptimizer(double& epsilon_, unsigned& min_point_count_,
			BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp),
			epsilon(epsilon_), min_Point_count(min_point_count_),
			parent(parent_) {}
		~GlobalTMeshOptimizer() {}
	public:
		void Global_LSM();
	protected:
		double epsilon;
		unsigned min_Point_count;
		BasisOptimizer* parent;
	};

	//==================================================================================================
	/**
	 * KKT system.
	 * .
	 */
	class LocalTMeshOptimizer_KKT :public BasisOptimizer {
	public:
		LocalTMeshOptimizer_KKT(BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp),
			epsilon(0.0), min_point_count(0),
			parent(parent_) {}
		LocalTMeshOptimizer_KKT(double& epsilon_, unsigned& min_point_count_,
			BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp),
			epsilon(epsilon_), min_point_count(min_point_count_),
			parent(parent_) {}

		~LocalTMeshOptimizer_KKT() {}
	public:

		void Local_LSM_KKT();

	protected:
		double epsilon;
		unsigned min_point_count;
		BasisOptimizer* parent;
	};

	class GlobalTMeshOptimizer_KKT :public BasisOptimizer {
	public:
		GlobalTMeshOptimizer_KKT(BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp),
			epsilon(0.0), min_point_count(0),
			parent(parent_) {}
		GlobalTMeshOptimizer_KKT(double& epsilon_, unsigned& min_point_count_,
			BasisOptimizer* parent_) :
			BasisOptimizer(parent_->mesh, parent_->mesh_param, parent_->mesh_tsp),
			epsilon(epsilon_), min_point_count(min_point_count_),
			parent(parent_) {}
		~GlobalTMeshOptimizer_KKT() {}
	public:

		void Global_LSM_KKT();

	protected:
		double epsilon;
		unsigned min_point_count;
		BasisOptimizer* parent;
	};


	//==================================================================================================

}

