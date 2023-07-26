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

