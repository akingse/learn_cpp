#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

std::vector<TriMesh> ClashDetection::sm_meshStore;

//positive tolerance means magnify
static Eigen::AlignedBox3d boxExtendTolerance(const Eigen::AlignedBox3d& box, const double tolerance)
{
	Vector3d toleSize = Vector3d(tolerance, tolerance, tolerance);
	return Eigen::AlignedBox3d(box.min() - toleSize, box.max() + toleSize);
}

//tolerance keep with hardclash
static std::array<std::vector<int>, 2> trianglesAndCommonBoxPreclash(const TriMesh& meshA, const TriMesh& meshB, double tolerance)
{
//#define USING_SECOND_PRECLASH //test shows not faster
	Eigen::AlignedBox3d boxCom = meshA.bounding_.intersection(meshB.bounding_); // box common
    Eigen::AlignedBox3d boxMag = boxExtendTolerance(boxCom, -tolerance);
	Eigen::AlignedBox3d boxA;
	std::vector<int> indexA; 
	for (int i = 0; i < (int)meshA.ibo_.size(); ++i)
	{
		std::array<Eigen::Vector3d, 3> triA = { 
			meshA.vbo_[meshA.ibo_[i][0]],
			meshA.vbo_[meshA.ibo_[i][1]],
			meshA.vbo_[meshA.ibo_[i][2]] };
		if (isTriangleAndBoundingBoxIntersectSAT(triA, boxMag))
		{
			indexA.push_back(i);
#ifdef USING_SECOND_PRECLASH
			boxA.extend(triA[0]);
			boxA.extend(triA[1]);
			boxA.extend(triA[2]);
#endif
		}
	}
	if (indexA.empty())
		return {};
	Eigen::AlignedBox3d boxB;
	std::vector<int> indexB;
	for (int j = 0; j < (int)meshB.ibo_.size(); ++j)
	{
		std::array<Eigen::Vector3d, 3> triB= { 
			meshB.vbo_[meshB.ibo_[j][0]],
			meshB.vbo_[meshB.ibo_[j][1]],
			meshB.vbo_[meshB.ibo_[j][2]] };
		if (isTriangleAndBoundingBoxIntersectSAT(triB, boxMag))
		{
			indexB.push_back(j);
#ifdef USING_SECOND_PRECLASH
			boxB.extend(triB[0]);
			boxB.extend(triB[1]);
			boxB.extend(triB[2]);
#endif
		}
	}
	if (indexB.empty())
		return {};
#ifdef USING_SECOND_PRECLASH
	Eigen::AlignedBox3d boxMagB = boxExtendTolerance(boxB, -tolerance);
	std::vector<int> indexARes;
	for (const int& iA : indexA)
	{
		std::array<Eigen::Vector3d, 3> triA = {
			meshA.vbo_[meshA.ibo_[iA][0]],
			meshA.vbo_[meshA.ibo_[iA][1]],
			meshA.vbo_[meshA.ibo_[iA][2]] };
		if (isTriangleAndBoundingBoxIntersectSAT(triA, boxMagB))
			indexARes.push_back(iA);
	}
	Eigen::AlignedBox3d boxMagA = boxExtendTolerance(boxA, -tolerance);
	std::vector<int> indexBRes;
	for (const int& iB : indexB)
	{
		std::array<Eigen::Vector3d, 3> triB = {
			meshB.vbo_[meshB.ibo_[iB][0]],
			meshB.vbo_[meshB.ibo_[iB][1]],
			meshB.vbo_[meshB.ibo_[iB][2]] };
		if (isTriangleAndBoundingBoxIntersectSAT(triB, boxMagA))
			indexBRes.push_back(iB);
	}
	return { indexARes, indexBRes };
#endif
#ifndef USING_SECOND_PRECLASH
	return { indexA, indexB };
#endif
}

//hard clash without tolerance
static bool isMeshInsideOtherMesh(const TriMesh& meshIn, const TriMesh& meshOut) 
{
	for (int i = 0; i < (int)meshIn.vbo_.size(); ++i)
	{
		const Eigen::Vector3d& point = meshIn.vbo_[i];
		//isPointInsidePolyhedronROT
		if (!meshOut.bounding_.contains(point)) //include point on box edge
			return false;
		//one mesh inside other mesh, the bounding-box must inside other mesh
		Vector3d rayDir = Vector3d(0.0, 0.0, 1.0);
		int countInter = 0;
		auto _isRayAndTriangleIntersectParallel = [&point, &rayDir](std::array<Eigen::Vector3d, 3 >& trigon, const Eigen::Vector3d& normal)->bool
			{
				if ((point - trigon[0]).dot(normal) != 0.0) // not coplanar
					return false;
				// negetive direction ray cause cross product result opposite
				return
					((trigon[0] - point).cross(rayDir).dot(rayDir.cross(trigon[1] - point)) >= 0.0 && (trigon[0] - point).cross(rayDir).dot((trigon[0] - point).cross(trigon[1] - point)) >= 0.0) ||
					((trigon[1] - point).cross(rayDir).dot(rayDir.cross(trigon[2] - point)) >= 0.0 && (trigon[1] - point).cross(rayDir).dot((trigon[1] - point).cross(trigon[2] - point)) >= 0.0) ||
					((trigon[2] - point).cross(rayDir).dot(rayDir.cross(trigon[0] - point)) >= 0.0 && (trigon[2] - point).cross(rayDir).dot((trigon[2] - point).cross(trigon[0] - point)) >= 0.0);
			};
		while (true)
		{
			bool isNewRay = false;//new rayDir
			for (int i = 0; i < (int)meshOut.ibo_.size(); ++i)//(const auto& iter : ibo) // iterate every trigon
			{
				std::array<Eigen::Vector3d, 3> trigon = {
					meshOut.vbo_[meshOut.ibo_[i][0]],
					meshOut.vbo_[meshOut.ibo_[i][1]],
					meshOut.vbo_[meshOut.ibo_[i][2]] };
				const Eigen::Vector3d& normal = meshOut.fno_[i];
				if (isPointOnTriangleSurface(point, trigon))
					return false;// RelationOfPointAndMesh::SURFACE; // ray across is false
				double deno = rayDir.dot(normal); //ray.direction
				if (deno == 0.0)//ray direction is parallel, redo circulation
				{
					if (_isRayAndTriangleIntersectParallel(trigon, normal)) //coplanar
					{
						rayDir = Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff).normalized();//move RAND_MAX/2
						isNewRay = true;
						break;
					}
					continue;
				}
				double k = (trigon[0] - point).dot(normal) / deno;
				if (k < 0.0) // only positive direction
					continue;
				Vector3d local = point + k * rayDir;
				if (!isPointInTriangle(local, trigon))
					continue;
				if ((local - trigon[0]).isZero() ||
					(local - trigon[1]).isZero() ||
					(local - trigon[2]).isZero() ||
					(local - trigon[0]).cross(local - trigon[1]).isZero() ||
					(local - trigon[1]).cross(local - trigon[2]).isZero() ||
					(local - trigon[2]).cross(local - trigon[0]).isZero()) //singularity
				{
					rayDir = Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff).normalized();
					isNewRay = true;
					break;
				}
				countInter++; // ray across is true
			}
			if (!isNewRay)
				break;//end while
		}
		return (countInter % 2 == 1);
	}
	return false;
}

static bool _isTwoMeshsIntersectSAT(const TriMesh& meshA, const TriMesh& meshB, double tolerance = 0.0)
{
	const std::array<std::vector<int>, 2> indexAB = trianglesAndCommonBoxPreclash(meshA, meshB, tolerance);// first pre-judge
	if (!indexAB[0].empty() && !indexAB[1].empty())
	{
		for (const int& iA : indexAB[0])
		{
			std::array<Eigen::Vector3d, 3> triA = {
				meshA.vbo_[meshA.ibo_[iA][0]],
				meshA.vbo_[meshA.ibo_[iA][1]],
				meshA.vbo_[meshA.ibo_[iA][2]] };
			for (const int& iB : indexAB[1])
			{
				std::array<Eigen::Vector3d, 3> triB = {
					meshB.vbo_[meshB.ibo_[iB][0]],
					meshB.vbo_[meshB.ibo_[iB][1]],
					meshB.vbo_[meshB.ibo_[iB][2]] };
				if (!isTwoTrianglesBoundingBoxIntersect(triA, triB, tolerance)) // second pre-judge
				{
					continue;
				}
				if (isTwoTrianglesIntersectSAT(triA, triB))
				{
					return true;
				}
			}
		}
	}
	//judge whether mesh entirely inside other mesh
	if (meshA.bounding_.contains(meshB.bounding_))
	{
        if (isMeshInsideOtherMesh(meshB, meshA))
		{
			return true;
		}
	}
	else if (meshB.bounding_.contains(meshA.bounding_))
	{
        if (isMeshInsideOtherMesh(meshA, meshB))
		{
			return true;
		}
	}
	return false;
}

// tolerance > 0 is soft clash, include distance < tolerance
// tolerance <= 0 is hard clash, exclude abs(distance) < abs(tolerance)
std::vector<std::pair<int, int>> ClashDetection::executeAssignClashDetection(const std::vector<int>& indexesLeft, const std::vector<int>& indexesRight, 
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	if (sm_meshStore.empty())
		return {};
	std::vector<Polyface3d> polyfaceVct(sm_meshStore.size());// simpified TriMesh
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
		polyfaceVct[i].m_index = i; //keep order
		polyfaceVct[i].m_bound = sm_meshStore[i].bounding_;
	}
	bvh::BVHTree3d bvhtree(polyfaceVct);//create spatial search tree
	set<std::pair<int, int>> signRecord;
	std::vector<std::pair<int, int>> clashRes;
#ifdef _OPENMP 
	int numThreads = omp_get_max_threads() - 1;
	int maxThreads = std::max(numThreads, 1);
	omp_set_num_threads(maxThreads);
#endif //_OPENMP
#pragma omp parallel for //schedule(dynamic)
	for (int i = 0; i < (int)indexesLeft.size(); ++i)
	{
#pragma omp critical
		{
			if (callback != nullptr)
				callback(100.f * i / (float)indexesLeft.size(), (int)clashRes.size()); // progress bar | result count
		}
		const int iL = indexesLeft[i];
		if (iL < 0 || sm_meshStore.size() <= iL) //check
			continue;
		const std::vector<int> preClash = bvhtree.findIntersect(polyfaceVct[iL], tolerance);
		const TriMesh& meshL = sm_meshStore[iL];
		for (int j = 0; j < (int)indexesRight.size(); ++j)
		{
			const int iR = indexesRight[j];
			if (iR < 0 || sm_meshStore.size() <= iR || //check
				iL == iR || //skip self
				std::find(preClash.begin(), preClash.end(), iR) == preClash.end())
			{
				continue;
			}
			std::pair<int, int> current = (iL < iR) ? pair<int, int>{iL, iR} : pair<int, int>{ iR, iL };
			if (signRecord.find(current) != signRecord.end()) //avoid repeat
				continue;
#pragma omp critical
			{
				signRecord.insert(current);
			}
			const TriMesh& meshR = sm_meshStore[iR];
			//if (meshL.convex_ && meshR.convex_)
			if (!_isTwoMeshsIntersectSAT(meshL, meshR, tolerance))
				continue;
#pragma omp critical
			{
				if (meshL.index_ == -1 || meshR.index_ == -1) //invalid index
					clashRes.push_back(current);
				else
					clashRes.push_back({ meshL.index_, meshR.index_ });
			}
		}
	}
	return clashRes;
}

std::vector<std::pair<int, int>> ClashDetection::executeFullClashDetection(const std::vector<TriMesh>& meshVct, 
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	sm_meshStore = meshVct; //copy data
	std::vector<Polyface3d> polyfaceVct(sm_meshStore.size());// simpified TriMesh
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
		polyfaceVct[i].m_index = i; //keep order
		polyfaceVct[i].m_bound = sm_meshStore[i].bounding_;
	}
	bvh::BVHTree3d bvhtree(polyfaceVct);//create spatial search tree
	std::vector<std::pair<int, int>> clashRes;
#ifdef _OPENMP 
	int numThreads = omp_get_max_threads() - 1;
	int maxThreads = std::max(numThreads, 1);
	omp_set_num_threads(maxThreads);
#endif //_OPENMP
#pragma omp parallel for //schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
#pragma omp critical
		{
			if (callback != nullptr)
			{
				//callback(100.f * i / (float)sm_meshStore.size(), (int)clashRes.size()); // progress bar | result count
			}
		}
		const TriMesh& meshL = sm_meshStore[i];
		const std::vector<int> preClash = bvhtree.findIntersect(polyfaceVct[i], tolerance);
        for (const int& j : preClash)
		{
            if (j <= i) //also canbe exclude in findIntersect;
				continue;
			const TriMesh& meshR = sm_meshStore[j];
			if (!_isTwoMeshsIntersectSAT(meshL, meshR, tolerance))
				continue;
#pragma omp critical
			{
				if (meshL.index_ == -1 || meshR.index_ == -1) //invalid index
                    clashRes.push_back({ i,j });
				else
					clashRes.push_back({ meshL.index_, meshR.index_ });
			}
		}
	}
	sm_meshStore.clear(); //clearMeshs
	return clashRes;
}

std::vector<std::pair<int, int>> ClashDetection::executePairClashDetection(const std::vector<TriMesh>& meshsLeft, const std::vector<TriMesh>& meshsRight,
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	sm_meshStore = meshsLeft; //addMeshs
	sm_meshStore.insert(sm_meshStore.end(), meshsRight.begin(), meshsRight.end());
	std::vector<int> indexesLeft(meshsLeft.size());
	for (int i = 0; i < (int)meshsLeft.size(); ++i)
		indexesLeft[i] = i;
	std::vector<int> indexesRight(meshsRight.size());
	int begin = (int)indexesLeft.size();
	for (int i = 0; i < (int)meshsRight.size(); ++i)
		indexesRight[i] = i + begin;
	std::vector<std::pair<int, int>> res = ClashDetection::executeAssignClashDetection(indexesLeft, indexesRight, tolerance, callback);
	sm_meshStore.clear(); //clearMeshs
	return res;
}

//create a whole trigons tree, contain all meshs trigon (maybe there is wrong way)
std::vector<std::pair<int, int>> ClashDetection::executeFullClashDetectionBVH(const std::vector<TriMesh>& meshVct,
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	std::vector<RectBase3d> allTrgions;
//#pragma omp parallel for schedule(dynamic)
	int k = 0; //global index
	vector<vector<int>> meshTrigon;
	for (int i = 0; i < (int)meshVct.size(); ++i) //without mesh store
	{
		const TriMesh& mesh = meshVct[i];
		std::vector<RectBase3d> tempTri(mesh.ibo_.size()); //add ibo face to boundbox
		vector<int> tempId(mesh.ibo_.size());
		for (int j = 0; j < (int)mesh.ibo_.size(); ++j)
		{
			AlignedBox3d box;
			box.extend(mesh.vbo_[mesh.ibo_[j][0]]);
			box.extend(mesh.vbo_[mesh.ibo_[j][1]]);
			box.extend(mesh.vbo_[mesh.ibo_[j][2]]);
			tempTri[j].m_bound = box;
			tempTri[j].m_number = i; //mesh index
			tempTri[j].m_index = k++; //unique index
			tempId[j] = tempTri[j].m_index;
		}
		allTrgions.insert(allTrgions.end(), tempTri.begin(), tempTri.end());
		meshTrigon[i] = tempId;
	}
	//create tree
	bvh::RectBaseTree3d tree3d(allTrgions);
	vector<set<int>> meshInter;//mesh index - all intersect trigon of mesh
	for (int i = 0; i < (int)meshTrigon.size(); ++i)
	{
		for (int j = 0; j < (int)meshTrigon[i].size(); ++j)
		{
			const RectBase3d& iter = allTrgions[meshTrigon[i][j]];
            const std::vector<std::array<int, 2>> preInter = tree3d.findIntersect(iter, tolerance); //trigon index
			for (const auto& k : preInter)
                meshInter[i].insert(k[1]); //with trigon boundbox intersect here
		}
	}
	//only trigon intersect SAT
	for (int i = 0; i < (int)meshInter.size(); ++i)
	{

	}

	return {};
}

