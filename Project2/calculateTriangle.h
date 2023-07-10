#pragma once
namespace psykronix
{
    // intersect of triangle
    DLLEXPORT bool isPointInTriangle(const Eigen::Vector2d& point, const std::array<Eigen::Vector2d, 3>& trigon);
    DLLEXPORT bool isPointInTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
    DLLEXPORT bool isTwoSegmentsIntersect(const std::array<Eigen::Vector3d, 2>& segmA, const std::array<Eigen::Vector3d, 2>& segmB, bool preBox = false);
    DLLEXPORT bool isEdgeCrossTriangle(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
    DLLEXPORT bool isSegmentCrossTriangleSurface(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isSegmentAndTriangleIntersctSAT(const std::array<Eigen::Vector3d, 2>& segment, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isTwoTrianglesIntersection(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
    DLLEXPORT bool isTwoTrianglesIntersect(const std::array<Eigen::Vector3d, 3>& triL, const std::array<Eigen::Vector3d, 3>& triR);
    DLLEXPORT bool isTwoTrianglesIntersectSAT(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
    DLLEXPORT bool isTwoTrianglesIntersectionSAT(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
    DLLEXPORT bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
	DLLEXPORT bool isPointRayAcrossTriangle(const Eigen::Vector3d& point, const std::array<Eigen::Vector3d, 3>& trigon);
	DLLEXPORT bool isPointInPolyfaceMesh(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);
	DLLEXPORT bool isPointContainedInPolyhedron(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& vbo, const std::vector<std::array<int, 3>>& ibo);

    // preprocess
    DLLEXPORT bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB);
    DLLEXPORT bool isTwoTrianglesBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& triA, const std::array<Eigen::Vector3d, 3>& triB, double tolerance);
    DLLEXPORT bool isTriangleAndBoundingBoxIntersect(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
    DLLEXPORT bool isTriangleAndBoundingBoxIntersectSAT(const std::array<Eigen::Vector3d, 3>& trigon, const Eigen::AlignedBox3d& box);
    DLLEXPORT std::tuple<Eigen::Vector3d, double> getTriangleBoundingCircle(const std::array<Eigen::Vector3d, 3>& trigon);
    // soft-clash
    DLLEXPORT void getSegmentsPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y, const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& Q, const Eigen::Vector3d& B);
    DLLEXPORT double getTrianglesDistance(Eigen::Vector3d& P, Eigen::Vector3d& Q, const std::array<Eigen::Vector3d, 3>& S, const std::array<Eigen::Vector3d, 3>& T);

}
namespace std
{
	inline void _wirteTrigonFile(const std::vector<std::array<uint64_t, 2>>& tris, const std::string& fileName)
	{
		//std::string fileName = "testTrisData.bin";
		size_t n = tris.size() * 2;
		uint64_t* arrayD = new uint64_t[n];
		for (size_t i = 0; i < tris.size(); ++i)
		{
			for (size_t j = 0; j < 2; ++j) // trigon pair
			{
				arrayD[i * 2 + j] = tris[i][j];
			}
		}
		ofstream out(fileName, ios::out | ios::binary);
		if (!out.is_open())
			cerr << "Error opening file" << endl;
		size_t num = tris.size();
		out.write(reinterpret_cast<char*>(&num), sizeof(size_t)); //
		out.write(reinterpret_cast<char*>(arrayD), n * sizeof(uint64_t));
		out.close();
		delete[] arrayD;
	}

	inline void _wirteTrigonFile(const std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>>& tris, const std::string& fileName)
	{
		//std::string fileName = "testTrisData.bin";
		size_t n = tris.size() * 2 * 3 * 3;
		double* arrayD = new double[n];
		size_t count = 0;
		for (size_t i = 0; i < tris.size(); ++i)
		{
			for (size_t j = 0; j < 2; ++j) // trigon pair
			{
				for (size_t t = 0; t < 3; ++t) //trigon vertex
				{
					arrayD[i * 18 + j * 9 + t * 3 + 0] = tris[i][j][t].x();
					arrayD[i * 18 + j * 9 + t * 3 + 1] = tris[i][j][t].y();
					arrayD[i * 18 + j * 9 + t * 3 + 2] = tris[i][j][t].z();
					count += 3;
				}
			}
		}
		ofstream out(fileName, ios::out | ios::binary);
		if (!out.is_open())
			cerr << "Error opening file" << endl;
		size_t num = tris.size();
		out.write(reinterpret_cast<char*>(&num), sizeof(size_t)); //
		out.write(reinterpret_cast<char*>(arrayD), n * sizeof(double));
		out.close();
		delete[] arrayD;

	}
	// manual serial
	inline std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> _readTrigonFile(const std::string& fileName)
	{
		//std::string fileName = "testTrisData.bin";
		ifstream in(fileName, ios::in | ios::binary);
		if (!in.is_open())
			return {};
		int n;
		in.read(reinterpret_cast<char*>(&n), sizeof(size_t));
		double* arrayD = new double[n * 2 * 3 * 3];
		in.read(reinterpret_cast<char*>(arrayD), n * 2 * 3 * 3 * sizeof(double));
		std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> tris;
		for (size_t i = 0; i < n; ++i)
		{
			std::array<std::array<Eigen::Vector3d, 3>, 2> triPair;
			for (size_t j = 0; j < 2; ++j) // trigon pair
			{
				array<Eigen::Vector3d, 3> trigon;
				for (size_t t = 0; t < 3; ++t) //trigon vertex
				{
					Eigen::Vector3d point = {
						arrayD[i * 18 + j * 9 + t * 3 + 0],
						arrayD[i * 18 + j * 9 + t * 3 + 1],
						arrayD[i * 18 + j * 9 + t * 3 + 2] };
					trigon[t] = point;
				}
				triPair[j] = trigon;
			}
			tris.push_back(triPair);
		}
		delete[] arrayD;
		return tris;
	}

	inline std::vector<std::array<uint64_t, 2>> _readEntityIDFile(const std::string& fileName)
	{
		ifstream in(fileName, ios::in | ios::binary);
		if (!in.is_open())
			return {};
		int n;
		in.read(reinterpret_cast<char*>(&n), sizeof(size_t));
		uint64_t* arrayD = new uint64_t[n * 2];
		in.read(reinterpret_cast<char*>(arrayD), n * 2 * sizeof(uint64_t));
		std::vector<std::array<uint64_t, 2>> tris;

		for (size_t i = 0; i < n; ++i)
		{
			tris.push_back({ arrayD[2 * i] ,arrayD[2 * i+1] });
		}
		delete[] arrayD;
		return tris;
	}

}

