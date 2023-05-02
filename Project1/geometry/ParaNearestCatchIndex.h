#pragma once

__declspec(dllexport) BPPropertyID getNearestCatchIndexCube(const GeoCube& cube, const BPParaVec& mouse, const BPParaTransform& viewport);
__declspec(dllexport) BPPropertyID getNearestCatchIndexSphere(const GeoSphere& sphere, const BPParaVec& mouse, const BPParaTransform& viewport);
__declspec(dllexport) BPPropertyID getNearestCatchIndexCone(const GeoCone& cone, const BPParaVec& mouse, const BPParaTransform& viewport);
