#pragma once
namespace clash
{
	static constexpr size_t N_10E_3 = (size_t)1e3;
	static constexpr size_t N_10E_4 = (size_t)1e4;
	static constexpr size_t N_10E_5 = (size_t)1e5;
	static constexpr size_t N_10E_6 = (size_t)1e6;
	static constexpr size_t N_10E_7 = (size_t)1e7;
	static constexpr size_t N_10E_8 = (size_t)1e8;

	class Vertex3d
	{
	public:
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		Vertex3d() {}
		Vertex3d(double x_, double y_, double z_ = 0.0) :
			x(x_),
			y(y_),
			z(z_)
		{
		}
		inline bool operator==(const Vertex3d& rhs) const
		{
			return memcmp(this, &rhs, sizeof(Vertex3d)) == 0;
		}
		inline bool operator<(const Vertex3d& rhs) const
		{
			return memcmp(this, &rhs, sizeof(Vertex3d)) < 0;
		}
#ifdef EIGEN_WORLD_VERSION
		Vertex3d(const Eigen::Vector3d& vec) :
			x(vec.x()),
			y(vec.y()),
			z(vec.z())
		{
		}
		Vertex3d(const Eigen::Vector3f& vec) :
			x(vec.x()),
			y(vec.y()),
			z(vec.z())
		{
		}
		operator Eigen::Vector3d() const
		{
			return Eigen::Vector3d(x, y, z);
		}
		operator Eigen::Vector3f() const
		{
			return Eigen::Vector3f((float)x, (float)y, (float)z);
		}
		//auto operator[](int i)
		//{
		//    if (i == 0)
		//        return x;
		//}
#endif
	};
}

class Vec3d
{
public:
	double x;
	double y;
	double z;
	Vec3d() = default; //C++11 new feature
	Vec3d(double x, double y, double z = 0) : x(x), y(y), z(z)
	{
	}
	Vec3d(const Vec3d& vec) : x(vec.x), y(vec.y), z(vec.z) //拷贝构造
	{
	}

	Vec3d(Vec3d&& rhs) //移动构造
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
	}

	Vec3d operator=(const Vec3d& rhs)  //拷贝赋值
	{
		if (&rhs != this)
			memcpy(this, &rhs, sizeof(Vec3d));
			//x = rhs.x;
			//y = rhs.y;
			//z = rhs.z;
		return *this;
	}

	Vec3d operator=(Vec3d&& rhs)  //移动赋值
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		return *this;
	}
	//compare
	bool operator==(const Vec3d& vec) const
	{
		//return abs(vec.x - x) + abs(vec.y - y) + abs(vec.z - z) < epsF;
		return memcmp(this, &vec, sizeof(Vec3d)) == 0;
	}
	bool operator<(const Vec3d& vec) const
	{
		//return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z < x* x + y * y + z * z;
		return memcmp(this, &vec, sizeof(Vec3d)) < 0; // ==-1
	}
	//operator bool() const
	//{
	//	return (abs(x) + abs(y) + abs(z) < epsF);
	//}
	virtual ~Vec3d()
	{
	}
};

inline void print_triangle(const std::array<Eigen::Vector3d, 3>& T1) //not accurate
{
	std::cout << "trigon= " << std::endl << "(" << T1[0].x() << ", " << T1[0].y() << ", " << T1[0].z() << ")" << std::endl;
	std::cout << "(" << T1[1].x() << ", " << T1[1].y() << ", " << T1[1].z() << ")" << std::endl;
	std::cout << "(" << T1[2].x() << ", " << T1[2].y() << ", " << T1[2].z() << ")" << std::endl;
}

inline void print_triangle(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2)
{
	std::cout << "triPair=" << std::endl << "(" << T1[0].x() << ", " << T1[0].y() << ", " << T1[0].z() << ")" << std::endl;
	std::cout << "(" << T1[1].x() << ", " << T1[1].y() << ", " << T1[1].z() << ")" << std::endl;
	std::cout << "(" << T1[2].x() << ", " << T1[2].y() << ", " << T1[2].z() << ")" << std::endl;
	std::cout << "(" << T2[0].x() << ", " << T2[0].y() << ", " << T2[0].z() << ")" << std::endl;
	std::cout << "(" << T2[1].x() << ", " << T2[1].y() << ", " << T2[1].z() << ")" << std::endl;
	std::cout << "(" << T2[2].x() << ", " << T2[2].y() << ", " << T2[2].z() << ")" << std::endl;
}

// triangle distance
bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
void getSegmentsPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y, const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& Q, const Eigen::Vector3d& B);
double getTrianglesDistance(Eigen::Vector3d& P, Eigen::Vector3d& Q, const std::array<Eigen::Vector3d, 3>& S, const std::array<Eigen::Vector3d, 3>& T);

//preclash
/*
std::vector<std::tuple<BPEntityId, BPEntityId, bool>> BoundingClashDetectionLL(const std::tuple<Eigen::AlignedBox3d, std::map<BPEntityId, ModelInfo>>& model_info, double tolerance)
{
	long long entry_size = std::get<1>(model_info).size();
	std::vector<std::tuple<BPEntityId, std::tuple<bool, bool>, Eigen::AlignedBox<long long, 3>>> remap_box(entry_size);
	std::map<long long, std::tuple<std::vector<size_t>, std::vector<size_t>>> order_z; // min max
	auto iter_entry_info = std::get<1>(model_info).begin();
	Eigen::Vector3d world_size = std::get<0>(model_info).sizes();
	world_size.x() = world_size.x() < tolerance ? tolerance : world_size.x();
	world_size.y() = world_size.y() < tolerance ? tolerance : world_size.y();
	world_size.z() = world_size.z() < tolerance ? tolerance : world_size.z();
	Eigen::AlignedBox3d bounding_double;
	Eigen::AlignedBox<long long, 3> bounding_longlong, box_inter;
	Eigen::Vector3d d_min, d_max;
	for (long long i = 0; i < entry_size; i++)
	{
#ifdef USING_INSTANCE_GRAPHIC_VERSION
		bounding_double = iter_entry_info->second.bounding();
#else
		bounding_double = iter_entry_info->second.mesh_.bounding_;
#endif
		d_min = ((bounding_double.min() - std::get<0>(model_info).min()).cwiseQuotient(world_size)) * (MAXINT64 / 4) - Eigen::Vector3d(MAXINT64 / 8, MAXINT64 / 8, MAXINT64 / 8);
		d_max = ((bounding_double.max() - std::get<0>(model_info).min()).cwiseQuotient(world_size)) * (MAXINT64 / 4) - Eigen::Vector3d(MAXINT64 / 8, MAXINT64 / 8, MAXINT64 / 8);
		bounding_longlong = { Eigen::Vector<long long,3>(d_min.x(),d_min.y(),d_min.z()), Eigen::Vector<long long,3>(d_max.x(),d_max.y(),d_max.z()) };
		remap_box.at(i) = { iter_entry_info->first, iter_entry_info->second.select_, bounding_longlong };
		std::get<0>(order_z[bounding_longlong.min().z()]).push_back(i);
		std::get<1>(order_z[bounding_longlong.max().z()]).push_back(i);
		assert(!((bounding_longlong.max() - bounding_longlong.min()).array() < 0).any()); // 包围盒有效性检查
		iter_entry_info++;
	}
	Eigen::Vector3d box_tolerance(tolerance, tolerance, tolerance);
	if (tolerance == 0.0) //using extra threshold
		box_tolerance = Vector3d(epsF, epsF, epsF);
	Eigen::Vector3d tolerance_double = (box_tolerance.cwiseQuotient(world_size)) * (MAXINT64 / 4);
	//Eigen::Vector<long long, 3> tolerance_longlong(tolerance_double[0], tolerance_double[1], tolerance_double[2]);
	Eigen::Vector<long long, 3> tolerance_longlong(tolerance_double.x(), tolerance_double.y(), tolerance_double.z());
	//std::vector<std::tuple<BPEntityId, BPEntityId, bool>> pre_clash;
	std::vector<std::tuple<BPEntityId, BPEntityId, bool>> pre_clash; // 预碰撞结果
#pragma omp parallel for schedule(dynamic) // must signed
	for (long long i = 0; i < remap_box.size(); i++)
	{
		for (size_t j = 0; j < remap_box.size(); j++)
		{
			if (i >= j)
				continue;
			// the selected
			if ((std::get<0>(std::get<1>(remap_box.at(i))) && std::get<1>(std::get<1>(remap_box.at(j)))) || (std::get<0>(std::get<1>(remap_box.at(j))) && std::get<1>(std::get<1>(remap_box.at(i)))))
			{
				box_inter = std::get<2>(remap_box.at(i)).intersection(std::get<2>(remap_box.at(j))); //AlignedBox3l
				// hardclash and solfclash both related with tolerance
				if (((box_inter.sizes() + tolerance_longlong).array() < 0).any()) // hardclash tolerance must be negative
					continue;
				bool is_soft = tolerance > 0 && (box_inter.sizes().array() < 0).any(); // t>0 and self-box not intersect, to reduced hard-clash scale
				//Eigen::AlignedBox<long long, 3> bounding = std::get<2>(remap_box.at(i)).intersection(std::get<2>(remap_box.at(j))); //AlignedBox3l
				//// is separate judge
				//if (((bounding.min() - bounding.max() - tolerance_longlong).array() > 0).any()) // 所有的方向中，存在距离大于阈值的，即判断为无关
				//	continue;
				//bool is_soft = tolerance > 0 ? ((bounding.min() - bounding.max()).array() > 0).any() : false;
				//BPEntityId& entityidA = std::get<0>(remap_box.at(i));
				//BPEntityId& entityidB = std::get<0>(remap_box.at(j));
#pragma omp critical
				{
					// (is_sepa) is_soft==true, means box separate evenif tolerance
					pre_clash.push_back(std::tuple<BPEntityId, BPEntityId, bool>{ get<0>(remap_box.at(i)), get<0>(remap_box.at(j)), !is_soft });
				}
			}
		}
	}
	return pre_clash;
} // many warings
*/
