#pragma once
/*******************************************************************************
* Author    :  akingse		                                                   *
* Date      :  from June 2023												   *
* Website   :  https://github.com/akingse                                      *
* Copyright :  All rights reserved											   *
* Purpose   :  Some common mesh topology calculation methods				   *
* License   :  MIT									                           *
*******************************************************************************/
#ifndef CALCULATE_MESHTOPOLOGY_H
#define CALCULATE_MESHTOPOLOGY_H

namespace games
{
	//------------------------------------------------------------------------------------------------
	//			Quadratic error measure
	//------------------------------------------------------------------------------------------------

	struct QEMVertex
	{
		Eigen::Vector3d m_vertex;
		double m_error = 0.0;
		//Vertex(double x, double y, double z) : x(x), y(y), z(z), error(0.0) {}
		bool operator<(const QEMVertex& rhs) const
		{
			return m_error > rhs.m_error; //for small root heap
		}
	};

	struct QEMEdge
	{
		std::array<int, 2> m_edge; //unique index
		int m_index; //index of HeMesh
		Eigen::Vector3d m_vertex; //using in first average method
		Eigen::Vector4d m_vbar; //using in QEM
		double m_error = 0.0;
		bool operator<(const QEMEdge& rhs) const
		{
			return m_error > rhs.m_error; //for small root heap
		}
	};

	//struct CompareVertex //using for priority_queue template
	//{
	//	bool operator()(const Vertex& v1, const Vertex& v2) 
	//	{
	//		return v1.m_error < v2.m_error;
	//	}
	//};
	////operator<
	//bool cmpless_QuadricError(const Vertex& vA, const Vertex& vB) //using as function-pointer
	//{
	//	return vA.m_error < vB.m_error;
	//}

	//------------------------------------------------------------------------------------------------
	//			The half edge data structure
	//------------------------------------------------------------------------------------------------

	struct HeEdge;
	struct HeFace;

	struct HeVertex
	{
		HeVertex() = default;
		//HeVertex(const Eigen::Vector3d& v) :m_coord(v) {}
		int m_index = -1;
		Eigen::Vector3d m_coord = Eigen::Vector3d(std::nan("0"), 0, 0);// start vertex coordinate of HalfEdge
		HeEdge* m_incEdge = nullptr; //incident edge, the start of edge
	};

	struct HeEdge //the half edge
	{
		HeEdge() = default;
		//HeEdge(HeVertex* v0, HeVertex* v1) :m_oriVertex(v0), m_desVertex(v1) {}
		//HeVertex* m_desVertex = nullptr; //end destination vertex, get by nextEdge
		int m_index = -1;
		HeVertex* m_oriVertex = nullptr; //start origin vertex
		HeEdge* m_twinEdge = nullptr; //means pair opposite halfedge, get by index
		HeEdge* m_prevEdge = nullptr;
		HeEdge* m_nextEdge = nullptr;
		HeFace* m_incFace = nullptr; //incident face
		bool m_isDel = false;
		bool m_isSide = true;
		inline Eigen::Vector3d vector() const //direction
		{
			return (m_nextEdge->m_oriVertex->m_coord - m_oriVertex->m_coord).normalized();
		}
	};

	struct HeFace
	{
		HeFace() = default;
		//HeFace(HeEdge* edge) : m_incEdge(edge) {}
		int m_index = -1;
		HeEdge* m_incEdge = nullptr; //any one of three edges
		Eigen::Vector3d m_normal = Eigen::Vector3d::Zero();
		bool m_isDel = false;
		bool m_isMark = false;
		bool m_isThin = false;

		inline Eigen::Vector3i ibo() const
		{
			Eigen::Vector3i face = {
				m_incEdge->m_oriVertex->m_index,
				m_incEdge->m_nextEdge->m_oriVertex->m_index,
				m_incEdge->m_prevEdge->m_oriVertex->m_index };
			return face;
		}
		inline std::vector<int> ibos(int max) const
		{
			std::vector<int> face;
			int count = 0;
			HeEdge* recrod = m_incEdge;
			while (recrod->m_isDel)
			{
				recrod = recrod->m_nextEdge;
				if (3 < count++)
					return {};
			}
			count = 0;
			HeEdge* iter = recrod;
			do {
				if (iter->m_isDel)
					return {};
				face.push_back(iter->m_oriVertex->m_index);
				iter->m_isDel = true;//isUsed
				iter = iter->m_nextEdge;
				if (max < count++) //avoid endlessloop
					return {}; //topo error
			} while (iter != recrod);
			return face;
		}
		inline std::array<Eigen::Vector3d, 3> triangle() const
		{
			std::array<Eigen::Vector3d, 3> face = {
				m_incEdge->m_oriVertex->m_coord,
				m_incEdge->m_nextEdge->m_oriVertex->m_coord,
				m_incEdge->m_prevEdge->m_oriVertex->m_coord };
			return face;
		}
		inline bool isinclude(const HeVertex* vt) const
		{
			return //using pointer judge
				vt == m_incEdge->m_oriVertex ||
				vt == m_incEdge->m_nextEdge->m_oriVertex ||
				vt == m_incEdge->m_prevEdge->m_oriVertex;
		}

	};

#define USING_POINTER_VERION
	struct HeMesh
	{
#ifdef USING_POINTER_VERION
		std::vector<HeVertex*> m_vertexes; //vbo
		std::vector<HeEdge*> m_edges;
		std::vector<HeFace*> m_faces; //ibo
#else //using_vector_index
		std::vector<HeVertex> m_vertexes;
		std::vector<HeEdge> m_edges;
		std::vector<HeFace> m_faces;
#endif
		bool isManifold = true;
		//std::vector<bool> is_border;
		HeMesh() = default;
		void clear();
		~HeMesh()
		{
			clear();
		}
		//convert
		HeMesh(const clash::ModelMesh& mesh); //fromTriangleMesh
		clash::ModelMesh toMesh() const;
		clash::ModelMesh toMeshs() const; //support polygon
		operator clash::ModelMesh() const; //toTriangleMesh
		bool isValid() const; //is mainfold mesh

		//CRUD
		//std::vector<HeEdge*> findEdges(const HeVertex*) const;
		//std::vector<HeEdge*> findEdges(const HeFace*) const;
		//std::vector<HeFace*> findFaces(const HeEdge*) const;
		////function
		//void update();
		//void rearrangeFaceIncEdge();
		//std::vector<int> findConnectedFaces(size_t fid);

	};

	//utility
	clash::ModelMesh meshLoopSubdivision(const clash::ModelMesh& mesh);
	HeMesh meshLoopSubdivision(const HeMesh& mesh);
	//ModelMesh meshQuadricErrorMetricsSimplification(const ModelMesh& mesh, size_t collapseEdgeCount = 0);
	clash::ModelMesh meshQEMSimplification(const clash::ModelMesh& mesh, size_t collapseEdgeCount = 0);
	HeMesh meshQEMSimplification(const HeMesh& mesh, size_t edgeCollapseTarget = 0);
	DLLEXPORT_CAL clash::ModelMesh meshMergeFacesBaseonNormal(const clash::ModelMesh& mesh, double toleAngle = 1e-6);
	DLLEXPORT_CAL clash::ModelMesh meshMergeFacesToQuadrangle(const clash::ModelMesh& mesh, double toleAngle = 1e-6);
	DLLEXPORT_CAL clash::ModelMesh meshMergeFacesSideEdgeOnly(const clash::ModelMesh& mesh, double toleAngle = 1e-6);

}
#endif// CALCULATE_MESHTOPOLOGY_H
