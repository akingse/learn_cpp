#include "pch.h"
using namespace std;
using namespace para;

// mouse catch algorithm
typedef pair<short, pair<double, double>> pair_ParaCube;
bool cmp_less_vec(pair_ParaCube a, pair_ParaCube b) // operator<
{
    double aNorm = a.second.first;
    double bNorm = b.second.first;
    double aDeep = a.second.second;
    double bDeep = b.second.second;
    if (abs(aNorm - bNorm) > PL_Length)
        return aNorm < bNorm;
    else
        return aDeep > bDeep;
}
BPPropertyID getNearestCatchIndexCube(const GeoCube& cube, const BPParaVec& mouse, const BPParaTransform& viewport)
{
    BPParaTransform matC = cube.getTransform();
    vector<BPParaVec> vertexes = getCubeAllVertexes(matC);
    vector<PGSegment> edges = getCubeAllEdges(matC);
    map<short, pair<double, double>> catchDict; //<distance, depth>
    vector<pair_ParaCube> sortVect; //using for sort
    double k = 0.05;
    double r = k * norm(vertexes[6] - vertexes[0]);
    // executed shadow
    BPParaVec axisZ = getMatrixsAxisZ(inverseOrth(viewport));
    BPParaTransform mat = viewport * shadowVectorMatrix2D((-1.0) * axisZ);
    BPParaVec mouseSha = mat * mouse;
    // push calculate result
    for (int i = 0; i < 8; i++)  //calculate vertex
    {
        double d = norm(mouseSha - mat * vertexes[i]);
        double h = (viewport * vertexes[i]).m_imp.z;
        catchDict.insert(pair_ParaCube((i + GeoCube::enCubeVertex1), pair<double, double>(d, h))); // ParaCube refer name table
    }
    for (int i = 0; i < 12; i++)  //calculate edge
    {
        PGSegment segmSha = PGSegment(mat * edges[i].start(), mat * edges[i].end());
        double d = getDistanceOfPointLine(mouseSha, segmSha) + r;
        BPParaVec middle = 0.5 * (edges[i].start() + edges[i].end()); //using middle point of segment
        double h = (viewport * middle).m_imp.z;
        catchDict.insert(pair_ParaCube((i + GeoCube::enCubeEdge1), pair<double, double>(d, h)));
    }
    // sort vector
    for (auto iter = catchDict.begin(); iter != catchDict.end(); iter++)
    {
        sortVect.push_back(pair_ParaCube(iter->first, iter->second));
    }
    sort(sortVect.begin(), sortVect.end(), cmp_less_vec);
    short param = sortVect.front().first;
    return BPPropertyID(param);
}


BPPropertyID getNearestCatchIndexSphere(const GeoSphere& sphere, const BPParaVec& mouse, const BPParaTransform& viewport)
{
    //const GeoSphere* sphere = dynamic_cast<const GeoSphere*>(&_sphere);
    //if (!_sphere.is<GeoSphere>())
    //    return BPPropertyID();
    //GeoSphere sphere = _sphere.as<GeoSphere>();
    double k = 0.2; //set a default rate
    BPParaVec axisZ = getMatrixsAxisZ(inverseOrth(viewport));
    BPParaTransform mat = viewport * shadowVectorMatrix2D((-1.0) * axisZ);
    BPParaVec mouseSha = mat * mouse;
    BPParaVec centerSha = mat * (sphere.getCenter());
    short param = (norm(mouseSha - centerSha) < k * sphere.getRadius()) ? GeoSphere::enSphereCenter : GeoSphere::enSphereRadius;
    return BPPropertyID(param);
}


BPPropertyID getNearestCatchIndexCone(const GeoCone& cone, const BPParaVec& mouse, const BPParaTransform& viewport)
{
    double k = 0.5; //set a default rate
    BPParaVec axisZ = getMatrixsAxisZ(inverseOrth(viewport));
    // the matrix transfer to viewport relative plane
    BPParaTransform mat = viewport * shadowVectorMatrix2D((-1.0) * axisZ);
    BPParaVec mouseSha = mat * mouse;
    BPParaVec pBotSha = mat * cone.getBottomPoint();
    BPParaVec pTopSha = mat * cone.getTopPoint();
    double rBot = cone.getBottomRadius();
    double rTop = cone.getTopRadius();
    BPParaTransform poseB = getMatrixFromTwoPoints(cone.getBottomPoint(), cone.getTopPoint());
    BPParaTransform poseT = getMatrixFromTwoPoints(cone.getTopPoint(), cone.getBottomPoint());
    bool inB = isPointOnArc(mouseSha, mat * poseB * scale(rBot));
    bool inT = isPointOnArc(mouseSha, mat * poseT * scale(rTop));
    double hB = (viewport * cone.getBottomPoint()).m_imp.z;
    double hT = (viewport * cone.getTopPoint()).m_imp.z;
    short param;
    //whlie point out of the shadow ellipse of cone arc
    if (!inB && !inT)
        param = GeoCone::enConeSegmentNorm;
    else if (inB && !inT) //in Bot && out Top
        param = (norm(mouseSha - pBotSha) < k * rBot) ? GeoCone::enConeBottomPoint : GeoCone::enConeBottomRadius;
    else if (!inB && inT) //out Bot && in Top
        param = (norm(mouseSha - pTopSha) < k * rTop) ? GeoCone::enConeTopPoint : GeoCone::enConeTopRadius;
    else //both in
        param = (hB < hT) ? GeoCone::enConeTopRadius : GeoCone::enConeBottomRadius;
    return BPPropertyID(param);
}
