#include "pch.h"
using namespace psykronix;

bool _isPointInTriangular(const BPParaVec& point, const std::array<BPParaVec, 3>& trigon)
{
	BPParaVec pA = trigon[0];
	BPParaVec pB = trigon[1];
	BPParaVec pC = trigon[2];
	BPParaVec sdA = (point - pA) ^ (pB - pA);
	BPParaVec sdB = (point - pB) ^ (pC - pB);
	BPParaVec sdC = (point - pC) ^ (pA - pC);
	return abs(norm(sdA) * norm(sdB) - (sdA * sdB)) < PL_Length && abs(norm(sdA) * norm(sdC) - (sdA * sdC)) < PL_Length;
}

BPParaTransform _getMatrixFromThreePoints(const std::array<BPParaVec, 3>& points)
{
    BPParaVec vecX = unitize(points[1]- points[0]);
    BPParaVec vecY = points[2]- points[0];
    BPParaVec vecZ = unitize(vecX ^ vecY); // been collinear judge
    vecY = (vecZ ^ vecX);
    return setMatrixByColumnVectors(vecX, vecY, vecZ, points[0]);
}

BPParaVec _getIntersectPointOfSegmentPlane(const BPParaVec& pA, const BPParaVec& pB, const BPParaVec& pOri, const BPParaVec& normal)
{
    double div = (normal * (pB - pA)); //
    if (abs(div) < PL_Length)
    {
        return pA;
    }
    double k = (normal * (pA - pOri)) / div;
    return pA + k * (pA - pB);
}

bool _isTwoTriangularIntersection(const std::array<BPParaVec, 3>& tBase, const std::array<BPParaVec, 3>& tLine)
{
    //include coplanar, zero distance
    //std::array<BPParaVec, 2> edgeA = { tLine[0], tLine[1] };
    //std::array<BPParaVec, 2> edgeB = { tLine[1], tLine[2] };
    //std::array<BPParaVec, 2> edgeC = { tLine[2], tLine[0] };
	//BPParaVec vecA = tLine[0] - tLine[1];
	//BPParaVec vecB = tLine[1] - tLine[2];
	//BPParaVec vecC = tLine[2] - tLine[0];
	//if (abs((normal * (tLine[0] - tBase[0])) * (normal * (tLine[1] - tBase[0]))) < PL_Length) 
 //   {
 //       // special handling
 //   }
	BPParaVec normal = (tBase[1] - tBase[0]) ^ (tBase[2] - tBase[0]);
    BPParaVec pOri = tBase[0];
    BPParaVec pL0 = tLine[0];
    BPParaVec pL1 = tLine[1];
    BPParaVec pL2 = tLine[2];
    //through the triangular plane
    double dotA = (normal * (pL0 - pOri)) * (normal * (pL0 - pOri));
    double dotB = (normal * (pL1 - pOri)) * (normal * (pL2 - pOri));
    double dotC = (normal * (pL2 - pOri)) * (normal * (pL0 - pOri));


    if (abs(dotA) < PL_Length || dotA < 0.0) // first filter
    {
        //calculate shadow
        //BPParaTransform mat = _getMatrixFromThreePoints(tBase);
        //BPParaTransform inv = inverseOrth(mat);
        //std::array<BPParaVec, 3> trigon = { BPParaVec(), inv * tBase[1], inv * tBase[2] };
        //get_intersect_point_of_line_plane
        double div = (normal * (pL0 - pL1));
        if (abs(div) < PL_Length)
        {
            if (_isPointInTriangular(pL0, tBase) || _isPointInTriangular(pL1, tBase))
                return true;
        }
        double k = (normal * (pL0 - pOri)) / div;
        BPParaVec locate = pL0 + k * (pL1 - pL0);  // paramater formula vector
        if (_isPointInTriangular(locate, tBase))
            return true;
    }
    if (abs(dotB) < PL_Length || dotB < 0.0) // first filter
    {
        double div = (normal * (pL1 - pL2));
        if (abs(div) < PL_Length)
        {
            if (_isPointInTriangular(pL1, tBase) || _isPointInTriangular(pL2, tBase))
                return true;
        }
        double k = (normal * (pL1 - pOri)) / div;
        BPParaVec locate = pL1 + k * (pL2 - pL1);  // paramater formula vector
        if (_isPointInTriangular(locate, tBase))
            return true;
    }
    if (abs(dotC) < PL_Length || dotB < 0.0) // first filter
    {
        double div = (normal * (pL2 - pL0));
        if (abs(div) < PL_Length)
        {
            if (_isPointInTriangular(pL2, tBase) || _isPointInTriangular(pL0, tBase))
                return true;
        }
        double k = (normal * (pL2 - pOri)) / div;
        BPParaVec locate = pL2 + k * (pL0 - pL2);  // paramater formula vector
        if (_isPointInTriangular(locate, tBase))
            return true;
    }
    return false;
}


static void _test1()
{



}