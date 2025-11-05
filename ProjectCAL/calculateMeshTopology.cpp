#include "pch.h"
//#include "calculateMeshTopology.h"
using namespace std;
using namespace Eigen;
using namespace eigen;
using namespace games;
using namespace clash;
#define USING_HALFEDGE_STRUCTURE

// interconversion
HeMesh::HeMesh(const ModelMesh& mesh)
{
	const std::vector<Eigen::Vector3d>& vbo = mesh.vbo_;
	const std::vector<Eigen::Vector3i>& ibo = mesh.ibo_;
	m_vertexes.resize(vbo.size());
	for (int i = 0; i < mesh.vbo_.size(); ++i)
	{
		HeVertex* heVertex = new HeVertex;
		heVertex->m_index = i;
		heVertex->m_coord = mesh.vbo_[i];
		m_vertexes[i] = heVertex;
	}
	int countEdge = 0;
	map<array<int, 2>, int> uniqueEdge;
	m_faces.resize(ibo.size());
	m_edges.resize(3 * ibo.size());
	for (int i = 0; i < mesh.ibo_.size(); ++i)
	{
		HeFace* heFace = new HeFace;
		heFace->m_index = i;
		//heFace->m_normal = (vbo[ibo[i][1]] - vbo[ibo[i][0]]).cross(vbo[ibo[i][2]] - vbo[ibo[i][1]]);// with normalized
		heFace->m_normal = mesh.fno_[i];
		//mark thin triangle
		Triangle3d trigon = {
			mesh.vbo_[mesh.ibo_[i][0]],
			mesh.vbo_[mesh.ibo_[i][1]],
			mesh.vbo_[mesh.ibo_[i][2]] };
		constexpr double thin = 10.0 / 180 * M_PI;
		for (int j = 0; j < 3; ++j)
		{
			double angle = get_angle_of_two_vectors(trigon[(j + 1) % 3] - trigon[(j) % 3], trigon[(j + 2) % 3] - trigon[(j + 1) % 3]);
			if (angle < thin)
				heFace->m_isThin = true;
		}
		int firstEdge = countEdge;
		for (int j = 0; j < 3; ++j) //create 3 edges from 1 face
		{
			HeEdge* heEdge = new HeEdge;
			heEdge->m_index = countEdge++;
			heEdge->m_incFace = heFace;
			heEdge->m_oriVertex = m_vertexes[ibo[i][j]];
			m_vertexes[ibo[i][j]]->m_incEdge = heEdge;
			//m_edges.push_back(heEdge);
			m_edges[i * 3 + j] = heEdge;
		}
		array<int, 4> face = { ibo[i][0], ibo[i][1], ibo[i][2], ibo[i][0] };
		for (int j = 0; j < 3; ++j) // add adjacent relation after create edges
		{
			//m_prevEdge
			int prev = j - 1 < 0 ? 2 : j - 1;
			m_edges[firstEdge + j]->m_prevEdge = m_edges[firstEdge + prev];
			//m_nextEdge
			int next = j + 1 > 2 ? 0 : j + 1;
			m_edges[firstEdge + j]->m_nextEdge = m_edges[firstEdge + next];
			//m_twinEdge
			array<int, 2> edge = (face[j] < face[j + 1]) ?
				array<int, 2>{face[j], face[j + 1]} :
				array<int, 2>{face[j + 1], face[j]};
			if (uniqueEdge.find(edge) == uniqueEdge.end())
				uniqueEdge.emplace(edge, firstEdge + j);
			else //means twinEdge added
			{
				m_edges[firstEdge + j]->m_twinEdge = m_edges[uniqueEdge[edge]];
				m_edges[uniqueEdge[edge]]->m_twinEdge = m_edges[firstEdge + j];
			}
		}
		heFace->m_incEdge = m_edges[firstEdge];
		m_faces[i] = heFace;
	}
}

ModelMesh HeMesh::toMesh() const
{
	ModelMesh mesh;
	mesh.vbo_.resize(m_vertexes.size());
	for (int i = 0; i < m_vertexes.size(); ++i)
		mesh.vbo_[i] = m_vertexes[i]->m_coord;
	mesh.ibo_.resize(m_faces.size());
	mesh.fno_.resize(m_faces.size());
	for (int i = 0; i < m_faces.size(); ++i)
	{
		mesh.ibo_[i] = m_faces[i]->ibo();
		mesh.fno_[i] = m_faces[i]->m_normal;
	}
	return mesh;
}
ModelMesh HeMesh::toMeshs() const
{
	ModelMesh mesh;
	mesh.vbo_.resize(m_vertexes.size());
	for (int i = 0; i < m_vertexes.size(); ++i)
		mesh.vbo_[i] = m_vertexes[i]->m_coord;
    int max = m_edges.size() / 2;
	for (int i = 0; i < (int)m_faces.size(); ++i)
	{
		const HeFace* face = m_faces[i];
		if (face->m_isDel) //why not on backward line
			continue;
		std::vector<int> ibos = face->ibos(max);
        if (ibos.size() < 3)
			continue;
		mesh.ibos_.push_back(ibos);
        mesh.fno_.push_back(face->m_normal);
	}
	return mesh;
	//simplify
}

HeMesh::operator ModelMesh() const
{
	return toMesh();
}

void HeMesh::clear()
{
	//Reset();
	for (size_t i = 0; i != m_vertexes.size(); ++i)
	{
		if (m_vertexes[i])
		{
			delete m_vertexes[i];
			m_vertexes[i] = nullptr;
		}
	}
	for (size_t i = 0; i != m_edges.size(); ++i)
	{
		if (m_edges[i])
		{
			delete m_edges[i];
			m_edges[i] = nullptr;
		}
	}
	for (size_t i = 0; i != m_faces.size(); ++i)
	{
		if (m_faces[i])
		{
			delete m_faces[i];
			m_faces[i] = nullptr;
		}
	}
	m_vertexes.clear();
	m_edges.clear();
	m_faces.clear();
}

bool HeMesh::isValid() const //is mainfold mesh
{
	// without nullptr
	for (const auto& iter : m_vertexes)
	{
		if (iter->m_index == -1 || iter->m_incEdge == nullptr || std::isnan(iter->m_coord[0]))
			return false;
	}
	for (const auto& iter : m_edges)
	{
		if (iter->m_index == -1 || !iter->m_oriVertex || !iter->m_twinEdge || !iter->m_prevEdge || !iter->m_nextEdge || !iter->m_incFace)
			return false;
		if (iter->m_twinEdge->m_oriVertex != iter->m_nextEdge->m_oriVertex ||
			iter->m_twinEdge->m_nextEdge->m_oriVertex != iter->m_oriVertex)
			return false;
	}
	for (const auto& iter : m_faces)
	{
		if (iter->m_index == -1 || iter->m_incEdge == nullptr || iter->m_normal.isZero())
			return false;
		// other methods
	}

	return true;
}

// the diagonal vertex of face | the surround vertex of vertex
tuple<vector<Eigen::Vector3i>, vector<set<int>>> _getMeshVertexLinkedInfo(const ModelMesh& mesh)
{
	//get diagonal vertex index of each face-edge
	vector<Eigen::Vector3i> edgeDiag(mesh.ibo_.size(), { -1,-1,-1 }); //init
	// generate new middle vertex
	for (int i = 0; i < (int)mesh.ibo_.size(); ++i)
	{
		if (edgeDiag[i][0] != -1 && edgeDiag[i][1] != -1 && edgeDiag[i][2] != -1) //has been assemble
			continue;
		Eigen::Vector3i faceA = mesh.ibo_[i]; //copy
		std::sort(faceA.begin(), faceA.end());
		int find = 0;
		for (int j = 0; j < (int)mesh.ibo_.size(); ++j)
		{
			if (i >= j) //edgeDiag[j] been revised
				continue;
			Eigen::Vector3i faceB = mesh.ibo_[j]; //copy
			std::sort(faceB.begin(), faceB.end());
			if (faceB[2] < faceA[0] || faceB[0]> faceA[2]) //bound box
				continue;
			int coin = 0;
			for (const int vtB : faceB) //find two common vertex
			{
				if (vtB == faceA[0] || vtB == faceA[1] || vtB == faceA[2])
					coin++;
			}
			if (coin == 2) //means common edge
			{
				int diaA, diaB; // diagonal vertex index
				if (faceA[0] != faceB[0] && faceA[0] != faceB[1] && faceA[0] != faceB[2])
					diaA = faceA[0];
				else if (faceA[1] != faceB[0] && faceA[1] != faceB[1] && faceA[1] != faceB[2])
					diaA = faceA[1];
				else
					diaA = faceA[2];
				if (faceB[0] != faceA[0] && faceB[0] != faceA[1] && faceB[0] != faceA[2])
					diaB = faceB[0];
				else if (faceB[1] != faceA[0] && faceB[1] != faceA[1] && faceB[1] != faceA[2])
					diaB = faceB[1];
				else
					diaB = faceB[2];
				int indexI, indexJ; // faces origin index
				if (diaA == mesh.ibo_[i][0])
					indexI = 1;
				else if (diaA == mesh.ibo_[i][1])
					indexI = 2;
				else
					indexI = 0;
				if (diaB == mesh.ibo_[j][0])
					indexJ = 1;
				else if (diaB == mesh.ibo_[j][1])
					indexJ = 2;
				else
					indexJ = 0;
				//record
				edgeDiag[i][indexI] = diaB;
				edgeDiag[j][indexJ] = diaA;
				find++;
			}
			if (find == 3)
				break;
		}
	}
	// update origin vertex
	vector<set<int>> roundVct(mesh.vbo_.size());
	for (int i = 0; i < (int)mesh.vbo_.size(); ++i)
	{
		set<int> round;// vector<Vector3d> round;
		for (const auto& face : mesh.ibo_)
		{
			if (face[0] != i && face[1] != i && face[2] != i)
				continue;
			for (int j = 0; j < 3; j++)
			{
				if (face[j] != i)
					round.insert(face[j]);
			}
		}
		roundVct[i] = round;
	}
	return { edgeDiag, roundVct };
}

// every new vertex
Vector3d _getNewVertex(const Vector3d& A, const Vector3d& B, const Vector3d& C, const Vector3d& D)
{
	// AB on edge, CD is diagonal
	return 0.375 * (A + B) + 0.125 * (C + D);// 3/8*(A+B)+1/8*(A+D)
}

// update old vertex
Vector3d _updateOldVertex(const Vector3d& origin, const vector<Vector3d>& round)
{
	size_t n = round.size(); //vertex degree
	double u = (n == 3) ? 0.1875 : 3.0 / (8 * n); //double
	Vector3d sum = Vector3d::Zero(); //neighbor position sum
	for (const auto& iter : round)
		sum += iter;
	return (1 - n * u) * origin + u * sum;
}

// practice of games101
ModelMesh games::meshLoopSubdivision(const ModelMesh& mesh)
{
	// invent wheel and optimize wheel
	ModelMesh meshNew = mesh; //copy
	std::vector<Eigen::Vector3d>& vboNew = meshNew.vbo_;
	std::vector<Eigen::Vector3i> iboNew;// reload
	tuple<vector<Eigen::Vector3i>, vector<set<int>>> info = _getMeshVertexLinkedInfo(mesh);
	const vector<Eigen::Vector3i>& edgeDiag = get<0>(info);
	map<array<int, 2>, int> uniqueEdge;
	for (int i = 0; i < (int)mesh.ibo_.size(); ++i)
	{
		const Eigen::Vector3i& face = mesh.ibo_[i];
		int mid01, mid12, mid20; //the new vertex index of edge middle
		//the unique edge, small->large
		array<int, 2> edge01 = (face[0] < face[1]) ? array<int, 2>{ face[0], face[1] } : array<int, 2>{face[1], face[0] };
		if (uniqueEdge.find(edge01) != uniqueEdge.end())
		{
			mid01 = uniqueEdge.at(edge01);
		}
		else
		{
			mid01 = (int)vboNew.size();
			uniqueEdge.insert({ edge01, mid01 }); //index of new edge middle
			Vector3d pointNew = _getNewVertex(vboNew[face[0]], vboNew[face[1]], vboNew[face[2]], vboNew[edgeDiag[i][0]]);
			vboNew.push_back(pointNew);
		}
		array<int, 2> edge12 = (face[1] < face[2]) ? array<int, 2>{ face[1], face[2] } : array<int, 2>{face[2], face[1] };
		if (uniqueEdge.find(edge12) != uniqueEdge.end())
		{
			mid12 = uniqueEdge.at(edge12);
		}
		else
		{
			mid12 = (int)vboNew.size();
			uniqueEdge.insert({ edge12, mid12 });
			Vector3d pointNew = _getNewVertex(vboNew[face[1]], vboNew[face[2]], vboNew[face[0]], vboNew[edgeDiag[i][1]]);
			vboNew.push_back(pointNew);
		}
		array<int, 2> edge20 = (face[2] < face[0]) ? array<int, 2>{ face[2], face[0] } : array<int, 2>{face[0], face[2] };
		if (uniqueEdge.find(edge20) != uniqueEdge.end())
		{
			mid20 = uniqueEdge.at(edge20);
		}
		else
		{
			mid20 = (int)vboNew.size();
			uniqueEdge.insert({ edge20, mid20 });
			Vector3d pointNew = _getNewVertex(vboNew[face[2]], vboNew[face[0]], vboNew[face[1]], vboNew[edgeDiag[i][2]]);
			vboNew.push_back(pointNew);
		}
		iboNew.push_back({ face[0],mid01,mid20 });
		iboNew.push_back({ face[1],mid12,mid01 });
		iboNew.push_back({ face[2],mid20,mid12 });
		iboNew.push_back({ mid01,mid12,mid20 });
	}
	const vector<set<int>>& roundIndex = get<1>(info);
	for (int i = 0; i < (int)mesh.vbo_.size(); ++i)
	{
		vector<Vector3d> round;
		for (const auto& j : roundIndex[i])
			round.push_back(mesh.vbo_[j]);
		vboNew[i] = _updateOldVertex(mesh.vbo_[i], round);
	}
	meshNew.ibo_ = iboNew;
	return meshNew;
}

HeMesh games::meshLoopSubdivision(const HeMesh& mesh)
{
	HeMesh meshNew; //create new empty HeMesh
	int indexFace = 0;// mesh.m_faces.size();
	int indexVertex = 0;//mesh.m_vertexes.size();
	int indexEdge = 0;//mesh.m_edges.size();
	std::map<int, int> new2oldEdge; // the new outer edge | incident origin edge(two edge use one index)
	std::map<int, int> old2newEdge; // old origin edge index | two new statr and end edge (index-1, index)
	for (const auto& iter : mesh.m_faces)
	{
		// the face's index0 vertex
		HeVertex* vtFace0 = new HeVertex;
		vtFace0->m_index = indexVertex++;
		vtFace0->m_coord = iter->m_incEdge->m_oriVertex->m_coord;
		meshNew.m_vertexes.push_back(vtFace0);
		HeEdge* edgeFaceS0 = new HeEdge; // 1/2 start
		edgeFaceS0->m_index = indexEdge++;
		edgeFaceS0->m_oriVertex = vtFace0;
		vtFace0->m_incEdge = edgeFaceS0;
		meshNew.m_edges.push_back(edgeFaceS0);
		HeVertex* vtMiddle0 = new HeVertex;;
		vtMiddle0->m_index = indexVertex++;
		vtMiddle0->m_coord = _getNewVertex(
			iter->m_incEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_prevEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_twinEdge->m_prevEdge->m_oriVertex->m_coord);
		meshNew.m_vertexes.push_back(vtMiddle0);
		HeEdge* edgeFaceE0 = new HeEdge; // 2/2 end
		edgeFaceE0->m_index = indexEdge++;
		edgeFaceE0->m_oriVertex = vtMiddle0;
		meshNew.m_edges.push_back(edgeFaceE0);
		vtMiddle0->m_incEdge = edgeFaceE0;
		new2oldEdge.emplace(indexEdge, iter->m_incEdge->m_index);
		old2newEdge.emplace(iter->m_incEdge->m_index, indexEdge);
		// the face's index1 vertex
		HeVertex* vtFace1 = new HeVertex;
		vtFace1->m_index = indexVertex++;
		vtFace1->m_coord = iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord;
		meshNew.m_vertexes.push_back(vtFace1);
		HeEdge* edgeFaceS1 = new HeEdge; // 1/2 start
		edgeFaceS1->m_index = indexEdge++;
		edgeFaceS1->m_oriVertex = vtFace1;
		vtFace1->m_incEdge = edgeFaceS1;
		meshNew.m_edges.push_back(edgeFaceS1);
		HeVertex* vtMiddle1 = new HeVertex;;
		vtMiddle1->m_index = indexVertex++;
		vtMiddle1->m_coord = _getNewVertex(
			iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_prevEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_nextEdge->m_twinEdge->m_prevEdge->m_oriVertex->m_coord);
		meshNew.m_vertexes.push_back(vtMiddle1);
		HeEdge* edgeFaceE1 = new HeEdge; // 2/2 end
		edgeFaceE1->m_index = indexEdge++;
		edgeFaceE1->m_oriVertex = vtMiddle1;
		meshNew.m_edges.push_back(edgeFaceE1);
		vtMiddle1->m_incEdge = edgeFaceE1;
		new2oldEdge.emplace(indexEdge, iter->m_incEdge->m_nextEdge->m_index);
		old2newEdge.emplace(iter->m_incEdge->m_nextEdge->m_index, indexEdge);
		// the face's index2 vertex
		HeVertex* vtFace2 = new HeVertex;
		vtFace2->m_index = indexVertex++;
		vtFace2->m_coord = iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord;
		edgeFaceS1->m_oriVertex = vtFace2;
		meshNew.m_vertexes.push_back(vtFace2);
		HeEdge* edgeFaceS2 = new HeEdge; // 1/2 start
		edgeFaceS2->m_index = indexEdge++;
		edgeFaceS2->m_oriVertex = vtFace2;
		vtFace2->m_incEdge = edgeFaceS2;
		meshNew.m_edges.push_back(edgeFaceS2);
		HeVertex* vtMiddle2 = new HeVertex;;
		vtMiddle2->m_index = indexVertex++;
		vtMiddle2->m_coord = _getNewVertex(
			iter->m_incEdge->m_prevEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_nextEdge->m_oriVertex->m_coord,
			iter->m_incEdge->m_prevEdge->m_twinEdge->m_prevEdge->m_oriVertex->m_coord);
		meshNew.m_vertexes.push_back(vtMiddle2);
		HeEdge* edgeFaceE2 = new HeEdge; // 2/2 end
		edgeFaceE2->m_index = indexEdge++;
		edgeFaceE2->m_oriVertex = vtMiddle2;
		meshNew.m_edges.push_back(edgeFaceE2);
		vtMiddle2->m_incEdge = edgeFaceE2;
		new2oldEdge.emplace(indexEdge, iter->m_incEdge->m_prevEdge->m_index);
		old2newEdge.emplace(iter->m_incEdge->m_prevEdge->m_index, indexEdge);
		// create 4 triangles of half-edge
		// new triangle-0
		HeEdge* edgeMidO0 = new HeEdge; //outer
		HeEdge* edgeMidI0 = new HeEdge; //inner
		edgeMidO0->m_index = indexEdge++;
		edgeMidI0->m_index = indexEdge++;
		edgeMidO0->m_twinEdge = edgeMidI0;
		edgeMidI0->m_twinEdge = edgeMidO0;
		edgeMidO0->m_oriVertex = vtMiddle0;
		edgeMidI0->m_oriVertex = vtMiddle2;
		edgeMidO0->m_prevEdge = edgeFaceS0;
		edgeMidO0->m_nextEdge = edgeFaceE2;
		meshNew.m_edges.push_back(edgeMidO0);
		meshNew.m_edges.push_back(edgeMidI0);
		HeFace* face0 = new HeFace;
		face0->m_index = indexFace++;
		face0->m_normal = (vtMiddle0->m_coord - vtFace0->m_coord).cross(vtMiddle2->m_coord - vtFace0->m_coord);
		face0->m_incEdge = edgeMidO0;
		meshNew.m_faces.push_back(face0);
		edgeMidO0->m_incFace = face0;
		edgeFaceS0->m_incFace = face0;
		edgeFaceE2->m_incFace = face0;
		// new triangle-1
		HeEdge* edgeMidO1 = new HeEdge; //outer
		HeEdge* edgeMidI1 = new HeEdge; //inner
		edgeMidO1->m_index = indexEdge++;
		edgeMidI1->m_index = indexEdge++;
		edgeMidO1->m_twinEdge = edgeMidI1;
		edgeMidI1->m_twinEdge = edgeMidO1;
		edgeMidO1->m_oriVertex = vtMiddle1;
		edgeMidI1->m_oriVertex = vtMiddle0;
		edgeMidO1->m_prevEdge = edgeFaceS1;
		edgeMidO1->m_nextEdge = edgeFaceE0;
		meshNew.m_edges.push_back(edgeMidO1);
		meshNew.m_edges.push_back(edgeMidI1);
		HeFace* face1 = new HeFace;
		face1->m_index = indexFace++;
		face1->m_normal = (vtMiddle1->m_coord - vtFace1->m_coord).cross(vtMiddle0->m_coord - vtFace1->m_coord);
		face1->m_incEdge = edgeMidO1;
		meshNew.m_faces.push_back(face1);
		edgeMidO1->m_incFace = face1;
		edgeFaceS1->m_incFace = face1;
		edgeFaceE0->m_incFace = face1;
		// new triangle-2
		HeEdge* edgeMidO2 = new HeEdge; //outer
		HeEdge* edgeMidI2 = new HeEdge; //inner
		edgeMidO2->m_index = indexEdge++;
		edgeMidI2->m_index = indexEdge++;
		edgeMidO2->m_twinEdge = edgeMidI2;
		edgeMidI2->m_twinEdge = edgeMidO2;
		edgeMidO2->m_oriVertex = vtMiddle2;
		edgeMidI2->m_oriVertex = vtMiddle1;
		edgeMidO2->m_prevEdge = edgeFaceS2;
		edgeMidO2->m_nextEdge = edgeFaceE1;
		meshNew.m_edges.push_back(edgeMidO2);
		meshNew.m_edges.push_back(edgeMidI2);
		HeFace* face2 = new HeFace;
		face2->m_index = indexFace++;
		face2->m_normal = (vtMiddle2->m_coord - vtFace2->m_coord).cross(vtMiddle1->m_coord - vtFace2->m_coord);
		face2->m_incEdge = edgeMidO0;
		meshNew.m_faces.push_back(face2);
		edgeMidO2->m_incFace = face2;
		edgeFaceS2->m_incFace = face2;
		edgeFaceE1->m_incFace = face2;
		// new triangle-inner
		HeFace* faceIn = new HeFace;
		faceIn->m_index = indexFace++;
		faceIn->m_normal = (vtMiddle1->m_coord - vtMiddle0->m_coord).cross(vtMiddle2->m_coord - vtMiddle2->m_coord);
		faceIn->m_incEdge = edgeMidI0;
		meshNew.m_faces.push_back(faceIn);
		//relation of inner triangle's edge
		edgeMidI0->m_prevEdge = edgeMidI2;
		edgeMidI0->m_nextEdge = edgeMidI1;
		edgeMidI0->m_incFace = faceIn;
		edgeMidI1->m_prevEdge = edgeMidI0;
		edgeMidI1->m_nextEdge = edgeMidI2;
		edgeMidI1->m_incFace = faceIn;
		edgeMidI2->m_prevEdge = edgeMidI1;
		edgeMidI2->m_nextEdge = edgeMidI0;
		edgeMidI2->m_incFace = faceIn;
		//relation of origin face triangle's edge
		edgeFaceS0->m_prevEdge = edgeFaceE2;
		edgeFaceS0->m_nextEdge = edgeMidO0;
		edgeFaceE0->m_prevEdge = edgeMidO1;
		edgeFaceE0->m_nextEdge = edgeFaceS1;
		edgeFaceS1->m_prevEdge = edgeFaceE0;
		edgeFaceS1->m_nextEdge = edgeMidO1;
		edgeFaceE1->m_prevEdge = edgeMidO2;
		edgeFaceE1->m_nextEdge = edgeFaceS2;
		edgeFaceS2->m_prevEdge = edgeFaceE1;
		edgeFaceS2->m_nextEdge = edgeMidO2;
		edgeFaceE2->m_prevEdge = edgeMidO0;
		edgeFaceE2->m_nextEdge = edgeFaceS0;
	}
	for (const auto& edge : new2oldEdge) // edge pair
	{
		HeEdge* edgeS = meshNew.m_edges[edge.first - 1];
		HeEdge* edgeE = meshNew.m_edges[edge.first];
		edgeS->m_twinEdge = meshNew.m_edges[old2newEdge[edge.second]];
		edgeE->m_twinEdge = meshNew.m_edges[old2newEdge[edge.second - 1]];
		// not using double assign
	}
	return meshNew;
}

//ModelMesh meshGeneralSubdivision(const ModelMesh& mesh) //catmull-clark subdivision
//{
//	return mesh;
//}
////without update
//ModelMesh meshQuadricErrorMetricsSimplification(const ModelMesh& mesh, size_t collapseEdgeCount /*= 0*/) //edge collapse and quadirc error metrics
//{
//	if (collapseEdgeCount == 0)
//		collapseEdgeCount = mesh.ibo_.size() / 2;
//	if (collapseEdgeCount >= mesh.ibo_.size())
//		return {};
//	//get edge
//	set<array<int, 2>> uniqueEdge;
//	for (const auto& iter : mesh.ibo_)
//	{
//		array<int, 4> tri = { iter[0], iter[1], iter[2], iter[0] };
//		for (int i = 0; i < 3; i++)
//		{
//			array<int, 2> edge = (tri[i] < tri[i + 1]) ?
//				array<int, 2>{tri[i], tri[i + 1]} : 
//				array<int, 2>{tri[i + 1], tri[i]};
//			uniqueEdge.insert(edge);
//		}
//	}
//	map<array<int, 2>, array<vector<int>, 2>> edgeNeighborFace; // edge vertex index | two vertex NeighborFace index
//	map<array<int, 2>, array<int, 2>> edgeOnFace; // edge vertex index | two faces index
//	for (const auto& edge : uniqueEdge)
//	{
//		//get edgeNeighborFace
//		vector<int> edge0, edge1;
//		for (int i = 0; i < mesh.ibo_.size(); i++)
//		{
//			const array<int, 3>& face = mesh.ibo_[i];
//			if (face[0] == edge[0] || face[1] == edge[0] || face[2] == edge[0])
//				edge0.push_back(i);
//			if (face[0] == edge[1] || face[1] == edge[1] || face[2] == edge[1])
//				edge1.push_back(i);
//		}
//		edgeNeighborFace.insert({ edge, { edge0, edge1 } });
//		// get edgeOnFace
//		array<int, 2> faceTwo;
//		int find = 0;
//		for (int i = 0; i < mesh.ibo_.size(); i++)
//		{
//			int coin = 0;
//			const array<int, 3>& face = mesh.ibo_[i];
//			for (const int& vt : face) //find two common vertex
//			{
//				if (vt == edge[0] || vt == edge[1])
//					coin++;
//			}
//			if (coin == 2) //means common edge
//			{
//				faceTwo[find] = i;
//				find++;
//			}
//			if (find == 2)
//			{
//				edgeOnFace.insert({ edge, faceTwo });
//				break;
//			}
//		}
//	}
//	//process come on
//	ModelMesh meshNew = mesh;//copy
//	std::vector<Eigen::Vector3d>& vbo = meshNew.vbo_;
//	std::vector<Eigen::Vector3i>& ibo = meshNew.ibo_;
//	auto _computeEdgeError = [&](const Vector3d& v, const array<vector<int>, 2>& neighbor)->double
//		{
//			double totalError = 0.0;
//			for (const auto& faces : neighbor) //two vertex neighbor face
//			{
//				for (const int& iter : faces)
//				{
//					Vector3d v0 = vbo[ibo[iter][0]];
//					Vector3d v1 = vbo[ibo[iter][1]];
//					Vector3d v2 = vbo[ibo[iter][2]];
//					Vector3d n = (v1 - v0).cross(v2 - v1);
//					totalError += (v0 - v).dot(n) * ((v0 - v).dot(n)) / n.dot(n);
//				}
//			}
//			return totalError;
//		};
//	std::priority_queue<QEMEdge> collapseEdgeQueue;
//	auto _updateCollapseEdgeQueue = [&]()->void
//		{
//			for (const auto& iter : uniqueEdge)
//			{
//				QEMEdge edge;
//				edge.m_edge = iter;
//				edge.m_vertex = 0.5 * (vbo[iter[0]] + vbo[iter[1]]);
//				edge.m_error = _computeEdgeError(edge.m_vertex, edgeNeighborFace[iter]);
//				collapseEdgeQueue.push(edge);
//			}
//		};
//	_updateCollapseEdgeQueue();
//	//contract edge
//	size_t collaCout = 0;
//	while (collaCout < collapseEdgeCount)
//	{
//		QEMEdge edge = collapseEdgeQueue.top();
//		collapseEdgeQueue.pop(); //delete edge
//		uniqueEdge.erase(edge.m_edge);
//		ibo[edgeOnFace[edge.m_edge][0]] = { -1,-1,-1 };
//		ibo[edgeOnFace[edge.m_edge][1]] = { -1,-1,-1 };
//		vbo[edge.m_edge[0]] = edge.m_vertex;
//		vbo[edge.m_edge[1]] = gVecNaN;
//		// change record map
//		for (auto& face : ibo)
//		{
//			for (int i = 0; i < 3; i++)
//			{
//				//if (face[i] == edge.m_edge[0]) continue;
//				if (face[i] == edge.m_edge[1])
//					face[i] = edge.m_edge[0];
//			}
//		}
//		_updateCollapseEdgeQueue();
//		collaCout++;
//	}
//	ModelMesh meshSim;
//	//map<int, int> indexMap;
//	vector<int> indexMap;
//	int j = 0;
//	for (int i = 0; i < vbo.size(); i++)
//	{
//		indexMap.push_back(j);
//		if (!isnan(vbo[i][0]))
//		{
//			meshSim.vbo_.push_back(vbo[i]);
//			j++;
//		}
//	}
//	for (const auto& iter : ibo)
//	{
//		if (iter[0] != -1) //valid
//		{
//			Eigen::Vector3i face = { indexMap[iter[0]], indexMap[iter[1]], indexMap[iter[2]] };
//			meshSim.ibo_.push_back(face);
//		}
//	}
//	return meshSim;
//}

// all function of QEM
static void _getPlaneCoefficient(const array<Vector3d, 3>& trigon, double& a, double& b, double& c, double& d)
{
	// a*x + b*y + c*z + d = 0
	//Vector3d N = (A - B).cross(C - B); // N_x*(x-A_x) + N_y*(y-A_y) + N_z*(y-A_z) = 0
	//assert(!N.isZero());
	Vector3d normal = (trigon[0] - trigon[1]).cross(trigon[2] - trigon[1]);
	a = normal[0];
	b = normal[1];
	c = normal[2];
	d = -normal.dot(trigon[0]);
	double len = std::sqrt(a * a + b * b + c * c); //a^2 + b^2 + c^2 = 1
	a /= len;
	b /= len;
	c /= len;
	d /= len;
	//return;
}

// calculate Q of every vertex
static Matrix4d _getQMatrixOfVertex(const std::vector<Eigen::Vector3i>& ibo, const std::vector<Eigen::Vector3d>& vbo, int i)
{
	Matrix4d Q = Eigen::Matrix4d::Zero();
	for (const auto& face : ibo)
	{
		if (face[0] != i && face[1] != i && face[2] != i) //find all adjacent faces 
			continue;
		double a, b, c, d;
		_getPlaneCoefficient({ vbo[face[0]], vbo[face[1]], vbo[face[2]] }, a, b, c, d);
		Vector4d p(a, b, c, d);
		Q += p * p.transpose(); //Kp matrix sigma
	}
	return Q;
}

static Matrix4d _getQMatrixOfVertex(const HeMesh& mesh, int i)
{
	Matrix4d Q = Eigen::Matrix4d::Zero();
	for (const auto& face : mesh.m_faces)
	{
		if (!face->isinclude(mesh.m_vertexes[i]))
			continue;
		double a, b, c, d;
		_getPlaneCoefficient(face->triangle(), a, b, c, d);
		Vector4d p(a, b, c, d);
		Q += p * p.transpose(); //Kp matrix sigma
	}
	return Q;
}

static QEMEdge _getCostAndVbarOfEdge(const std::vector<Matrix4d>& Qs, const std::vector<Vector3d>& vbo, const array<int, 2 >& i)
{
	QEMEdge edge;
	edge.m_edge = i;
	//calculate cost and v_bar
	Eigen::Matrix4d Q_bar = Qs[i[0]] + Qs[i[1]];
	Eigen::Matrix4d Q_h = Q_bar;//homogeneous
	Q_h.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
	Eigen::Matrix4d inverse;
	bool invertible;
	Q_h.computeInverseWithCheck(inverse, invertible);
	Eigen::Vector4d b(0, 0, 0, 1);
	Eigen::Vector4d v_bar = (invertible) ?
		v_bar = inverse * b : //v_bar = (1 / v_bar[3]) * v_bar; //hnormalized
		v_bar = (0.5 * (vbo[i[0]] + vbo[i[0]])).homogeneous();
	double error = v_bar.transpose() * Q_bar * v_bar; // the cost
	edge.m_vbar = v_bar;
	edge.m_error = error;
	return edge;
}

static QEMEdge _getCostAndVbarOfEdge(const std::vector<Matrix4d>& Qs, const HeEdge* i)
{
	QEMEdge edge;
	edge.m_index = i->m_index;
	//calculate cost and v_bar
	Eigen::Matrix4d Q_bar = Qs[i->m_oriVertex->m_index] + Qs[i->m_nextEdge->m_oriVertex->m_index]; //get vertex index
	Eigen::Matrix4d Q_h = Q_bar;//homogeneous
	Q_h.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
	Eigen::Matrix4d inverse;
	bool invertible;
	Q_h.computeInverseWithCheck(inverse, invertible);
	Eigen::Vector4d b(0, 0, 0, 1);
	Eigen::Vector4d v_bar = (invertible) ?
		v_bar = inverse * b : //v_bar = (1 / v_bar[3]) * v_bar; //hnormalized
		v_bar = (0.5 * (i->m_oriVertex->m_coord + i->m_nextEdge->m_oriVertex->m_coord)).homogeneous(); //get vertex coord
	double error = v_bar.transpose() * Q_bar * v_bar; // the cost
	edge.m_vbar = v_bar;
	edge.m_error = error;
	return edge;
}

//#ifndef USING_HALFEDGE_STRUCTURE
ModelMesh games::meshQEMSimplification(const ModelMesh& mesh, size_t collapseEdgeCount /*= 0*/)
{
	//get edge
	set<array<int, 2>> uniqueEdge;
	for (const auto& iter : mesh.ibo_)
	{
		array<int, 4> tri = { iter[0], iter[1], iter[2], iter[0] };
		for (int i = 0; i < 3; i++)
		{
			array<int, 2> edge = (tri[i] < tri[i + 1]) ?
				array<int, 2>{tri[i], tri[i + 1]} : array<int, 2>{tri[i + 1], tri[i]};
			uniqueEdge.insert(edge);
		}
	}
	//get edge located face2
	map<array<int, 2>, array<int, 2>> edgeOnFace; // edge vertex index | two faces index
	for (const auto& edge : uniqueEdge)
	{
		array<int, 2> faceTwo;
		int find = 0;
		for (int i = 0; i < mesh.ibo_.size(); i++)
		{
			int coin = 0;
			const Eigen::Vector3i& face = mesh.ibo_[i];
			for (const int& vt : face) //find two common vertex
			{
				if (vt == edge[0] || vt == edge[1])
					coin++;
			}
			if (coin == 2) //means common edge
			{
				faceTwo[find] = i;
				find++;
			}
			if (find == 2)
			{
				edgeOnFace.emplace(edge, faceTwo);
				break;
			}
		}
	}
	std::vector<Eigen::Matrix4d> Qs;
	for (int i = 0; i < mesh.vbo_.size(); ++i)
	{
		Qs.push_back(_getQMatrixOfVertex(mesh.ibo_, mesh.vbo_, i));
	}
	// place edge into heap
	std::priority_queue<QEMEdge> heap;
	for (const auto& iter : uniqueEdge)
	{
		QEMEdge edge = _getCostAndVbarOfEdge(Qs, mesh.vbo_, iter);
		heap.push(edge);
	}
	ModelMesh meshC = mesh;//copy
	std::vector<Eigen::Vector3d>& vbo = meshC.vbo_;
	std::vector<Eigen::Vector3i>& ibo = meshC.ibo_;
	auto _updateCollapseEdgeHeap = [&](int i_bar) ->void
		{
			set<int> adjacentVertex;
			set<array<int, 2>> adjacentEdge;
			//Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
			for (const auto& face : ibo)
			{
				if (face[0] != i_bar && face[1] != i_bar && face[2] != i_bar) //find the adjacent faces 
					continue;
				if (face[0] == i_bar)
				{
					adjacentVertex.insert(face[1]);
					adjacentVertex.insert(face[2]);
					(face[0] < face[1]) ? adjacentEdge.insert({ face[0], face[1] }) : adjacentEdge.insert({ face[1], face[0] });
					(face[0] < face[2]) ? adjacentEdge.insert({ face[0], face[2] }) : adjacentEdge.insert({ face[2], face[0] });
				}
				else if (face[1] == i_bar)
				{
					adjacentVertex.insert(face[0]);
					adjacentVertex.insert(face[2]);
					(face[1] < face[0]) ? adjacentEdge.insert({ face[1], face[0] }) : adjacentEdge.insert({ face[0], face[1] });
					(face[1] < face[2]) ? adjacentEdge.insert({ face[1], face[2] }) : adjacentEdge.insert({ face[2], face[1] });
				}
				else//if (face[2] == i_bar)
				{
					adjacentVertex.insert(face[1]);
					adjacentVertex.insert(face[0]);
					(face[2] < face[1]) ? adjacentEdge.insert({ face[2], face[1] }) : adjacentEdge.insert({ face[1], face[2] });
					(face[2] < face[0]) ? adjacentEdge.insert({ face[2], face[0] }) : adjacentEdge.insert({ face[0], face[2] });
				}
			}
			// update Q of adjacent Vertex
			Qs[i_bar] = _getQMatrixOfVertex(ibo, vbo, i_bar);
			for (const int& iter : adjacentVertex)
			{
				Qs[iter] = _getQMatrixOfVertex(ibo, vbo, iter);
			}
			// add new edges
			for (const auto& iter : adjacentEdge)
			{
				QEMEdge edge = _getCostAndVbarOfEdge(Qs, vbo, iter);
				heap.push(edge);
			}
		};
	//contract edge
	size_t collaCout = 0;
	while (collaCout < collapseEdgeCount)
	{
		const QEMEdge& edge = heap.top();
		ibo[edgeOnFace[edge.m_edge][0]] = { -1,-1,-1 }; //delete two face
		ibo[edgeOnFace[edge.m_edge][1]] = { -1,-1,-1 };
		vbo[edge.m_edge[0]] = edge.m_vbar.hnormalized();
		vbo[edge.m_edge[1]] = gVecNaN; //delete one vertex
		for (auto& face : ibo) //change face who own deleted vertex
		{
			for (int i = 0; i < 3; i++)
			{
				if (face[i] == edge.m_edge[1])
					face[i] = edge.m_edge[0];
			}
		}
		heap.pop(); //delete one edge
		_updateCollapseEdgeHeap(edge.m_edge[0]); //新的edge的error会不会小于受vbar影响而改变的edge
		collaCout++;
	}
	ModelMesh meshSim; //new mesh
	vector<int> indexMap; // origin face's vertex index -> new index
	int j = 0;
	for (int i = 0; i < vbo.size(); i++)
	{
		indexMap.push_back(j);
		if (!isnan(vbo[i][0]))
		{
			meshSim.vbo_.push_back(vbo[i]);
			j++;
		}
	}
	for (const auto& iter : ibo)
	{
		if (iter[0] != -1) //valid
		{
			Eigen::Vector3i face = { indexMap[iter[0]], indexMap[iter[1]], indexMap[iter[2]] };
			meshSim.ibo_.push_back(face);
		}
	}
	return meshSim;
}
//#endif

// mesh operation with halfedge
#ifdef USING_HALFEDGE_STRUCTURE
HeMesh games::meshQEMSimplification(const HeMesh& mesh, size_t edgeCollapseTarget /*= 0*/)
{
	std::vector<Eigen::Matrix4d> Qs;
	for (int i = 0; i < mesh.m_vertexes.size(); ++i)
	{
		Qs.push_back(_getQMatrixOfVertex(mesh, i));
	}
	// place edge into heap
	std::priority_queue<QEMEdge> heap;
	std::set<int> uniqueEdge;
	for (const auto& iter : mesh.m_edges) // only unique edge, halfedge means double amount
	{
		if (uniqueEdge.find(iter->m_index) == uniqueEdge.end() &&
			uniqueEdge.find(iter->m_twinEdge->m_index) == uniqueEdge.end())
		{
			uniqueEdge.insert(iter->m_index);
			QEMEdge edge = _getCostAndVbarOfEdge(Qs, iter);
			heap.push(edge);
		}
	}
	size_t edgeCollapseCurrent = 0;
	HeMesh meshC = mesh;//copy
	auto _updateCollapseEdgeHeap = [&](const HeVertex* vbar)->void
		{
			//update neibor vertex qme-value
			set<int> adjacentVertex;
			for (const auto iter : meshC.m_faces)
			{
				if (iter->isinclude(vbar))
				{
					const Eigen::Vector3i& ibo = iter->ibo();
					for (const auto i : ibo)
						adjacentVertex.insert(i); //include vbar->m_index self
				}
			}
			for (const auto iter : adjacentVertex)
				Qs[iter] = _getQMatrixOfVertex(meshC, iter);
			//update neibor edge qme-value
			for (const auto iter : meshC.m_edges)
			{
				if (iter->m_oriVertex == vbar) //keep half amout
				{
					QEMEdge edge = _getCostAndVbarOfEdge(Qs, iter);
					heap.push(edge);
				}
			}
		};
	//contract edge
	while (edgeCollapseCurrent < edgeCollapseTarget)
	{
		const QEMEdge& mini = heap.top();
		HeEdge* heEdge = meshC.m_edges[mini.m_index];
		//update first vertex, delete second vertex
		heEdge->m_oriVertex->m_coord = mini.m_vbar.hnormalized(); //update merged vertex
		for (auto iter : mesh.m_edges)
		{
			if (iter->m_oriVertex->m_index == heEdge->m_twinEdge->m_oriVertex->m_index)
				iter->m_oriVertex = heEdge->m_oriVertex;
		}
		if (heEdge->m_oriVertex->m_incEdge == heEdge) //avoid heVertex hold nullptr
			heEdge->m_oriVertex->m_incEdge = heEdge->m_prevEdge->m_twinEdge;
		if (heEdge->m_twinEdge->m_oriVertex)
		{
			delete heEdge->m_twinEdge->m_oriVertex; //also heEdge->m_nextEdge->m_oriVertex
			heEdge->m_twinEdge->m_oriVertex = nullptr;
		}
		//change the twin edge of deleted face
		int merge0 = heEdge->m_nextEdge->m_twinEdge->m_index;
		int merge1 = heEdge->m_prevEdge->m_twinEdge->m_index;
		meshC.m_edges[merge0]->m_twinEdge = meshC.m_edges[merge1];
		meshC.m_edges[merge1]->m_twinEdge = meshC.m_edges[merge0];
		merge0 = heEdge->m_twinEdge->m_nextEdge->m_twinEdge->m_index;
		merge1 = heEdge->m_twinEdge->m_prevEdge->m_twinEdge->m_index;
		meshC.m_edges[merge0]->m_twinEdge = meshC.m_edges[merge1];
		meshC.m_edges[merge1]->m_twinEdge = meshC.m_edges[merge0];
		// delete two face
		if (heEdge->m_incFace)
		{
			delete heEdge->m_incFace;
			heEdge->m_incFace = nullptr;
		}
		if (heEdge->m_twinEdge->m_incFace)
		{
			delete heEdge->m_twinEdge->m_incFace;
			heEdge->m_twinEdge->m_incFace = nullptr;
		}
		_updateCollapseEdgeHeap(heEdge->m_oriVertex);
		//delete one edge
		if (heEdge)
		{
			delete heEdge;
			heEdge = nullptr;
		}
		if (heEdge->m_twinEdge)
		{
			delete heEdge->m_twinEdge;
			heEdge->m_twinEdge = nullptr;
		}
		heap.pop();
		edgeCollapseCurrent++;
	}
	HeMesh meshSim; //new mesh
	int j = 0;
	for (int i = 0; i < meshC.m_vertexes.size(); i++)
	{
		if (meshC.m_vertexes[i] != nullptr)
		{
			meshC.m_vertexes[i]->m_index = j;
			meshSim.m_vertexes.emplace_back(meshC.m_vertexes[i]);
			j++;
		}
	}
	j = 0;
	for (int i = 0; i < meshC.m_edges.size(); i++)
	{
		if (meshC.m_edges[i] != nullptr)
		{
			meshC.m_edges[i]->m_index = j;
			meshSim.m_edges.emplace_back(meshC.m_edges[i]);
			j++;
		}
	}
	j = 0;
	for (int i = 0; i < meshC.m_faces.size(); i++)
	{
		if (meshC.m_faces[i] != nullptr)
		{
			meshC.m_faces[i]->m_index = j;
			meshSim.m_faces.emplace_back(meshC.m_faces[i]);
			j++;
		}
	}
	return meshSim;
}

#endif //USING_HALFEDGE_STRUCTURE

clash::ModelMesh games::meshMergeFacesBaseonNormal(const clash::ModelMesh& mesh, double toleAngle)
{
	MACRO_EXPANSION_TIME_DEFINE;
	MACRO_EXPANSION_TIME_START;
	HeMesh hesh = HeMesh(mesh);
	MACRO_EXPANSION_TIME_END("time_mesh2hesh");
    auto _topo_merge_and_mark = [](HeEdge* edge, HeEdge* edgeTw) //merge
		{
			edge->m_prevEdge->m_nextEdge = edgeTw->m_nextEdge;
			edgeTw->m_nextEdge->m_prevEdge = edge->m_prevEdge;
			edge->m_nextEdge->m_prevEdge = edgeTw->m_prevEdge;
			edgeTw->m_prevEdge->m_nextEdge = edge->m_nextEdge;
			//edge->m_incFace->m_normal = 0.5 * (edge->m_incFace->m_normal + edgeTw->m_incFace->m_normal); //average
			edge->m_isDel = true;
			edgeTw->m_isDel = true;
			//if (delFace)
			edgeTw->m_incFace->m_isDel = true; //only accelerate convert mesh
		};
	auto _inner_remove_and_mark = [](HeEdge* last, HeEdge* edge) //merge
		{
			last->m_prevEdge->m_nextEdge = edge;
			edge->m_prevEdge->m_nextEdge = last;
			HeEdge* swap = last->m_prevEdge;
			last->m_prevEdge = edge->m_prevEdge;
			edge->m_prevEdge = swap;
		};
	MACRO_EXPANSION_TIME_START;
	for (size_t i = 0; i < hesh.m_edges.size(); i++)
	{
		HeEdge* edge = hesh.m_edges[i];
		if (edge->m_isDel) 
			continue;
		HeEdge* edgeTw = edge->m_twinEdge;
		if (edgeTw == nullptr || edgeTw->m_isDel)//boundary twin-edge is null
			continue;
		if (edge->m_nextEdge == edgeTw || edge->m_prevEdge == edgeTw || //single backward
			fabs(1.0 - edge->m_incFace->m_normal.dot(edgeTw->m_incFace->m_normal)) <= toleAngle) //small angle
		{
			_topo_merge_and_mark(edge, edgeTw);
		}
	}
	for (size_t i = 0; i < hesh.m_edges.size(); i++) //for multi backward
	{
		HeEdge* edge = hesh.m_edges[i];
		if (edge->m_isDel)
			continue;
		HeEdge* edgeTw = edge->m_twinEdge;
		if (edgeTw == nullptr || edgeTw->m_isDel)//boundary twin-edge is null
			continue;
		while (edge->m_nextEdge == edgeTw && !edge->m_isDel && edge->m_twinEdge == edgeTw)
		{
			_topo_merge_and_mark(edge, edgeTw);
			edge = edge->m_prevEdge;
			edgeTw = edgeTw->m_nextEdge;
			test::DataRecordSingleton::dataCountAppend("count_BackWard0");
		}
		while (edge->m_prevEdge == edgeTw && !edge->m_isDel && edge->m_twinEdge == edgeTw)
		{
			_topo_merge_and_mark(edge, edgeTw);
			edge = edge->m_nextEdge;
			edgeTw = edgeTw->m_prevEdge;
			test::DataRecordSingleton::dataCountAppend("count_BackWard1"); //notCCW
		}
	}
	//process self intersect
	//for (size_t i = 0; i < hesh.m_faces.size(); i++)
	//{
	//	const HeFace* face = hesh.m_faces[i];
	//	if (face->m_isDel)
	//		continue;
 //       //int max = hesh.m_edges.size() / 2;
	//	std::vector<int> ibos = face->ibos();
	//	if (ibos.size() < 6) //min inner triangle
	//		continue;
	//	std::unordered_set<int> seen;// unique;
	//	std::unordered_set<int> breakset;//repeat
	//	for (const int& j : ibos)
	//	{
	//		if (seen.find(j) != seen.end())
	//			breakset.insert(j);
	//		seen.insert(j);
	//	}
	//	if (ibos.size() == seen.size())
	//		continue; //check
	//	//re-topo
	//	//std::unordered_set<HeVertex*> breakset;//repeat
	//	HeEdge* edge = face->m_incEdge;
	//	HeEdge* first = edge;
	//	std::stack<HeEdge*> edgeRec; //record
	//	edge = face->m_incEdge;
	//	first = edge;
	//	int count = 0;
	//	do {
	//		count++;
	//		if (breakset.find(edge->m_oriVertex->m_index) == breakset.end())
	//		{
	//			edge = edge->m_nextEdge;
	//			continue;
	//		}
	//		if (edgeRec.empty())
	//		{
	//			edgeRec.push(edge);
	//			edge = edge->m_nextEdge;
	//			continue;
	//		}
 //           if (edge->m_oriVertex != edgeRec.top()->m_oriVertex)
	//			edgeRec.push(edge);
	//		else
	//		{
	//			HeEdge*& last = edgeRec.top();
	//			//if (edge == last->m_prevEdge->m_twinEdge)
	//			//	_topo_merge_and_mark(edge, last->m_prevEdge->m_twinEdge);
	//			_inner_remove_and_mark(last, edge);
	//			edgeRec.pop();
	//		}
	//		edge = edge->m_nextEdge;
	//	} while (edge != first);
	//}

	MACRO_EXPANSION_TIME_END("time_calMerge");
	MACRO_EXPANSION_TIME_START;
	ModelMesh res = hesh.toMeshs();
	MACRO_EXPANSION_TIME_END("time_hesh2Meshs");
	hesh.clear();
	return res;
}

clash::ModelMesh games::meshMergeFacesToQuadrangle(const clash::ModelMesh& mesh, double toleAngle)
{
	MACRO_EXPANSION_TIME_DEFINE;
	MACRO_EXPANSION_TIME_START;
	HeMesh hesh = HeMesh(mesh);
	MACRO_EXPANSION_TIME_END("time_mesh2hesh");
	auto _topo_merge_and_mark = [](HeEdge* edge, HeEdge* edgeTw) //merge
		{
			edge->m_prevEdge->m_nextEdge = edgeTw->m_nextEdge;
			edgeTw->m_nextEdge->m_prevEdge = edge->m_prevEdge;
			edge->m_nextEdge->m_prevEdge = edgeTw->m_prevEdge;
			edgeTw->m_prevEdge->m_nextEdge = edge->m_nextEdge;
			edge->m_isDel = true;
			edgeTw->m_isDel = true;
			edgeTw->m_incFace->m_isDel = true; //only accelerate convert mesh
			edge->m_incFace->m_isMark = true;
			edgeTw->m_incFace->m_isMark = true;
		};
	MACRO_EXPANSION_TIME_START;
	for (size_t i = 0; i < hesh.m_edges.size(); i++)
	{
		HeEdge* edge = hesh.m_edges[i];
		if (edge->m_isDel)
			continue;
		HeEdge* edgeTw = edge->m_twinEdge;
		if (edgeTw == nullptr || edgeTw->m_isDel)//boundary twin-edge is null
			continue;
		if (edge->m_incFace->m_isMark == true || edgeTw->m_incFace->m_isMark == true)
			continue;
		if (edge->m_nextEdge == edgeTw || edge->m_prevEdge == edgeTw || //all edge ccw
			fabs(1.0 - edge->m_incFace->m_normal.dot(edgeTw->m_incFace->m_normal)) <= toleAngle)
		{
			_topo_merge_and_mark(edge, edgeTw);
		}
	}
	MACRO_EXPANSION_TIME_END("time_calMerge");
	MACRO_EXPANSION_TIME_START;
	ModelMesh res = hesh.toMeshs();
	MACRO_EXPANSION_TIME_END("time_hesh2Meshs");
	hesh.clear();
	return res;
}

clash::ModelMesh games::meshMergeFacesSideEdgeOnly(const clash::ModelMesh& mesh, double toleAngle)
{
	MACRO_EXPANSION_TIME_DEFINE;
	MACRO_EXPANSION_TIME_START;
	HeMesh hesh = HeMesh(mesh);
	MACRO_EXPANSION_TIME_END("time_mesh2hesh");
	//统计顶点使用次数
	//map<HeVertex*, int> vertCount;
	//for (size_t i = 0; i < hesh.m_edges.size(); i++)
	//{
	//	HeEdge* edge = hesh.m_edges[i];
	//	HeVertex* vert = edge->m_oriVertex;
	//	if (vertCount.find(vert) == vertCount.end())
	//		vertCount.emplace(vert, 1);
	//	else
	//		vertCount.at(vert) += 1;
	//}
	//vector<pair<HeVertex*, int>> vertCountVct;
	//for (const auto& iter : vertCount)
	//{
	//	if (iter.second > 10)
	//		vertCountVct.push_back(iter);
	//}
	auto _topo_merge_and_mark = [](HeEdge* edge, HeEdge* edgeTw, double toleAngle) //merge
		{
			if (edge->m_nextEdge == edgeTw || edge->m_prevEdge == edgeTw)
				test::DataRecordSingleton::dataCountAppend("count_BackWardLine");
			edge->m_nextEdge->m_isSide = false;
			edgeTw->m_prevEdge->m_isSide = false;
			edge->m_prevEdge->m_isSide = false;
			edgeTw->m_nextEdge->m_isSide = false;
			if (fabs(1.0 - edge->m_nextEdge->vector().dot(edgeTw->m_prevEdge->vector())) <= toleAngle)
			{
				edge->m_prevEdge->m_isSide = true;
				edgeTw->m_nextEdge->m_isSide = true;
			}
			if (fabs(1.0 - edge->m_prevEdge->vector().dot(edgeTw->m_nextEdge->vector())) <= toleAngle)
			{
				edge->m_nextEdge->m_isSide = true;
				edgeTw->m_prevEdge->m_isSide = true;
			}
			edge->m_prevEdge->m_nextEdge = edgeTw->m_nextEdge;
			edgeTw->m_nextEdge->m_prevEdge = edge->m_prevEdge;
			edge->m_nextEdge->m_prevEdge = edgeTw->m_prevEdge;
			edgeTw->m_prevEdge->m_nextEdge = edge->m_nextEdge;
			edge->m_isDel = true;
			edgeTw->m_isDel = true;
			edgeTw->m_incFace->m_isDel = true; //only accelerate convert mesh
		};
	MACRO_EXPANSION_TIME_START;
	for (size_t i = 0; i < hesh.m_edges.size(); i++)
	{
		HeEdge* edge = hesh.m_edges[i];
		if (edge->m_isDel)
			continue;
		HeEdge* edgeTw = edge->m_twinEdge;
		if (edgeTw == nullptr || edgeTw->m_isDel)//boundary twin-edge is null
			continue;
        if (edge->m_isSide == false || edgeTw->m_isSide == false)
			continue;
		if (fabs(1.0 - edge->m_incFace->m_normal.dot(edgeTw->m_incFace->m_normal)) <= toleAngle)
		{
            _topo_merge_and_mark(edge, edgeTw, 2 * toleAngle);
		}
	}
	MACRO_EXPANSION_TIME_END("time_calMerge");
	MACRO_EXPANSION_TIME_START;
	ModelMesh res = hesh.toMeshs();
	MACRO_EXPANSION_TIME_END("time_hesh2Meshs");
	hesh.clear();
	return res;
}
