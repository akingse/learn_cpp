//写于2000年的经典代码
#include "pch.h"
class HE_vert; //顶点
class HE_face; //半面
class HE_edge; //半边


class HE_edge
{
    HE_vert* vert;   // vertex at the end of the half-edge
    HE_edge* pair;   // oppositely oriented adjacent half-edge 
    HE_face* face;   // face the half-edge borders
    HE_edge* next;   // next half-edge around the face

};

class HE_vert
{
    float x;
    float y;
    float z;
    HE_edge* edge;  // one of the half-edges emantating from the vertex

};

class HE_face
{
    HE_edge* edge;  // one of the half-edges bordering the face
};


