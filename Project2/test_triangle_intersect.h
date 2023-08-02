#pragma once
#include <afx.h>
inline std::string getExePath() // include<afx.h>
{
	TCHAR buff[MAX_PATH];
	GetModuleFileNameW(NULL, buff, MAX_PATH);
	CString path = buff;
	path = path.Left(path.ReverseFind('\\')); // delete exename
	return (CStringA)path;
}

bool TriangularIntersectionTest(const std::array<Eigen::Vector3d, 3>& T1, const std::array<Eigen::Vector3d, 3>& T2);
// triangle distance
void getSegmentsPoints(Eigen::Vector3d& VEC, Eigen::Vector3d& X, Eigen::Vector3d& Y, const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& Q, const Eigen::Vector3d& B);
double getTrianglesDistance(Eigen::Vector3d& P, Eigen::Vector3d& Q, const std::array<Eigen::Vector3d, 3>& S, const std::array<Eigen::Vector3d, 3>& T);