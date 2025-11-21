// TSkeletonTypes.inl - 模板实现文件
namespace skeleton {
	//=================================================================

	/**
	 * @brief 设置半边的参数坐标
	 * @param p 参数坐标点
	 */
	template<class Refs, class Traits>
	void skeleton_vertex<Refs, Traits>::set_param(const Point2d& p) {
		this->param = p;
	}
	template<class Refs, class Traits>
	void skeleton_halfedge<Refs, Traits>::set_d_gedestic(const double& d) {
		this->d_geodestic = d;
	}
	
	template<class Refs,class Traits>
	void skeleton_halfedge<Refs, Traits>::set_d_param(const double& d) {
		this->d_param = d;
	}


}


