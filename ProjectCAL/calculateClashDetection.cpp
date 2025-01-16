#include "pch.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace clash;

std::vector<ModelMesh> ClashDetection::sm_meshStore;

void ClashDetection::addMeshs(const std::vector<ModelMesh>& meshVct)
{
	sm_meshStore.insert(sm_meshStore.end(), meshVct.begin(), meshVct.end());
}

void ClashDetection::clearMeshs()
{
	sm_meshStore.clear();
}

static bool isMeshIntrusionTesting(const ModelMesh& meshA, const ModelMesh& meshB) //getIntersetAndPenetrationDepth
{
	// origin double for-loop process
	Eigen::Vector3d origin = -meshA.vbo_.front();
	for (size_t i = 0; i < meshA.ibo_.size(); i++)
	{
		for (size_t j = 0; j < meshB.ibo_.size(); j++)
		{
			std::array<Eigen::Vector3d, 3> triA = {
				meshA.vbo_[meshA.ibo_[i][0]] + origin,
				meshA.vbo_[meshA.ibo_[i][1]] + origin,
				meshA.vbo_[meshA.ibo_[i][2]] + origin };
			std::array<Eigen::Vector3d, 3> triB = {
				meshB.vbo_[meshB.ibo_[j][0]] + origin,
				meshB.vbo_[meshB.ibo_[j][1]] + origin,
				meshB.vbo_[meshB.ibo_[j][2]] + origin };
			if (isTwoTrianglesIntersectSAT(triA, triB))
			{
				return true;
			}
		}
	}
	return false;
}


// tolerance > 0 is soft clash, include distance < tolerance
// tolerance <= 0 is hard clash, exclude abs(distance) < abs(tolerance)
std::vector<std::pair<int, int>> ClashDetection::executeAssignClashDetection(const std::vector<int>& indexesLeft, const std::vector<int>& indexesRight, 
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	std::vector<Polyface3d> polyfaceVct(sm_meshStore.size());// simpified ModelMesh
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)sm_meshStore.size(); ++i)
	{
		polyfaceVct[i].m_index = i;
		polyfaceVct[i].m_bound = sm_meshStore[i].bounding_;
	}
	bvh::BVHTree3d bvhtree(polyfaceVct);//create spatial search tree
	set<std::pair<int, int>> signRecord;
	std::vector<std::pair<int, int>> clashRes;
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)indexesLeft.size(); ++i)
	{
#pragma omp critical
		{
			if (callback != nullptr)
				callback(100 * i / (float)indexesLeft.size(), (int)clashRes.size()); // progress bar | result count
		}
		const int iL = indexesLeft[i];
		if (iL < 0 || sm_meshStore.size() <= iL) //check
			continue;
		const std::vector<int> preClash = bvhtree.findIntersect(polyfaceVct[iL], tolerance);
		const ModelMesh& meshL = sm_meshStore[iL];
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
			const ModelMesh& meshR = sm_meshStore[iR];
			if (meshL.convex_ && meshR.convex_)
			{
			}
			if (isTwoMeshsIntersectSAT(meshL, meshR, tolerance))
				continue;
#pragma omp critical
			{
				clashRes.push_back(current);
			}
		}
	}
	return clashRes;
}

std::vector<std::pair<int, int>> ClashDetection::executeFullClashDetection(const std::vector<ModelMesh>& meshVct, 
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	sm_meshStore = meshVct;
	std::vector<int> indexes(meshVct.size());
	for (int i = 0; i < meshVct.size(); ++i)
		indexes[i] = i;
	return ClashDetection::executeAssignClashDetection(indexes, indexes, tolerance, callback);
}

std::vector<std::pair<int, int>> ClashDetection::executePairClashDetection(const std::vector<ModelMesh>& meshsLeft, const std::vector<ModelMesh>& meshsRight,
	const double tolerance /*= 0.0*/, const std::function<bool(float, int)>& callback /*= nullptr*/)
{
	sm_meshStore = meshsLeft;
	sm_meshStore.insert(sm_meshStore.end(), meshsRight.begin(), meshsRight.end());
	std::vector<int> indexesLeft(meshsLeft.size());
	for (int i = 0; i < meshsLeft.size(); ++i)
		indexesLeft[i] = i;
	std::vector<int> indexesRight(meshsRight.size());
	int begin = (int)indexesLeft.size();
	for (int i = 0; i < meshsRight.size(); ++i)
		indexesRight[i] = i + begin;
	return ClashDetection::executeAssignClashDetection(indexesLeft, indexesRight, tolerance, callback);
}

