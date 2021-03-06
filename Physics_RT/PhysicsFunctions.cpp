#include "stdafx.h"
#include "PhysicsFunctions.h"
#include "PathHelper.h"
#include "StringHlp.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <list>

extern void** apiArray;
extern sInitStruct initStruct;
extern bool disablePhysics;

void CallbackWithError(char* msg, ...)
{
	if (initStruct.errCallback) {
		va_list ap;
		va_start(ap, msg);
		auto str = FormatString(msg, ap);
		initStruct.errCallback(str.c_str());
		va_end(ap);
	}
}

bool lastHasException = false;
char lastException[1024];

//Error Report

void APIErrorReport(const char* msg, void* userArgGivenToInit)
{
	if (initStruct.errCallback)
		initStruct.errCallback(msg);

#if defined(HK_PLATFORM_WIN32)
#ifndef HK_PLATFORM_WINRT
	OutputDebugStringA(msg);
#else
	// Unicode only 
	int sLen = hkString::strLen(msg) + 1;
	wchar_t* wideStr = hkAllocateStack<wchar_t>(sLen);
	mbstowcs_s(HK_NULL, wideStr, sLen, msg, sLen - 1);
	OutputDebugString(wideStr);
	hkDeallocateStack<wchar_t>(wideStr, sLen);
#endif
#endif
}

void APICatchHandler(std::exception& e) {
	lastHasException = true;
#ifdef _MSC_VER
	if (IsDebuggerPresent()) throw e;
	else strcpy_s(lastException, e.what());
#else
		strcpy(lastException, e.what());
#endif
}

EXTERN_C_API int checkException(char* errBuffer, size_t size) {
	if (lastHasException) {
#ifdef _MSC_VER
		strcpy_s(errBuffer, size, lastException);
#else
		strcpy(errBuffer, lastException);
#endif
		lastHasException = false;
		return true;
	}
	return false;
}

std::list<spTransform> smallPoolSpTransform;
std::list<spVec3> smallPoolSpVec3;
std::list<spVec4> smallPoolSpVec4;
std::list<spMatrix4> smallPoolSpMatrix4;

spTransform CreateTransform(float px, float py, float pz, float rx, float ry, float rz, float rw, float sx, float sy, float sz)
{
	spTransform v = nullptr;
	if (smallPoolSpTransform.size() > 0) {
		v = smallPoolSpTransform.front();
		smallPoolSpTransform.pop_front();
	}
	else {
		v = new sTransform();
	}
	v->positionX = px;
	v->positionY = py;
	v->positionZ = pz;
	v->scaleX = sx;
	v->scaleY = sy;
	v->scaleZ = sz;
	v->rotationX = rx;
	v->rotationY = ry;
	v->rotationZ = rz;
	v->rotationW = rw;
	return v;
}
spVec3 CreateVec3(float x, float y, float z) {
	spVec3 v = nullptr;
	if (smallPoolSpVec3.size() > 0) {
		v = smallPoolSpVec3.front();
		smallPoolSpVec3.pop_front();
	}
	else {
		v = new sVec3();
	}
	v->x = x;
	v->y = y;
	v->z = z;
	return v;
}
spVec4 CreateVec4(float x, float y, float z, float w) {
	spVec4 v = nullptr;
	if (smallPoolSpVec4.size() > 0) {
		v = smallPoolSpVec4.front();
		smallPoolSpVec4.pop_front();
	}
	else {
		v = new sVec4();
	}

	v->x = x;
	v->y = y;
	v->z = z;
	v->w = w;
	return v;
}
spMatrix4 CreateMatrix4(float *r, int isMatrix4) {
	spMatrix4 v = nullptr;
	if (smallPoolSpMatrix4.size() > 0) {
		v = smallPoolSpMatrix4.front();
		smallPoolSpMatrix4.pop_front();
	}
	else {
		v = new sMatrix4();
	}

	if (isMatrix4) { //4x4
#ifdef _MSC_VER
		memcpy_s(v->r0, sizeof(v->r0), r, sizeof(float) * 4);
		memcpy_s(v->r1, sizeof(v->r1), r + 4, sizeof(float) * 4);
		memcpy_s(v->r2, sizeof(v->r2), r + 8, sizeof(float) * 4);
		memcpy_s(v->r3, sizeof(v->r3), r + 12, sizeof(float) * 4);
#else
		memcpy(v->r0, r, sizeof(v->r0));
		memcpy(v->r1, r + 4, sizeof(v->r1));
		memcpy(v->r2, r + 8, sizeof(v->r2));
		memcpy(v->r3, r + 12, sizeof(v->r3));
#endif
	}
	else { //3x3
#ifdef _MSC_VER
		memcpy_s(v->r0, sizeof(v->r0), r, sizeof(float) * 3);
		memcpy_s(v->r1, sizeof(v->r1), r + 4, sizeof(float) * 3);
		memcpy_s(v->r2, sizeof(v->r2), r + 8, sizeof(float) * 3);
#else
		memcpy(v->r0, r, sizeof(float) * 3);
		memcpy(v->r1, r + 4, sizeof(float) * 3);
		memcpy(v->r2, r + 8, sizeof(float) * 3);
#endif
	}
	return v;
}

void DestroyVec4(const spVec4 ptr)
{
	CHECK_PARAM_PTR(ptr);
	if (smallPoolSpVec4.size() < (size_t)initStruct.smallPoolSize) 
		smallPoolSpVec4.push_back(ptr);
	else
		delete ptr;
}
void DestroyVec3(const spVec3 ptr)
{
	CHECK_PARAM_PTR(ptr);
	if (smallPoolSpVec3.size() < (size_t)initStruct.smallPoolSize)
		smallPoolSpVec3.push_back(ptr);
	else
		delete ptr;
}
void DestroyMatrix4(const spMatrix4 ptr)
{
	CHECK_PARAM_PTR(ptr);
	if (smallPoolSpMatrix4.size() < (size_t)initStruct.smallPoolSize)
		smallPoolSpMatrix4.push_back(ptr);
	else
		delete ptr;
}
void DestroyTransform(const spTransform ptr)
{
	CHECK_PARAM_PTR(ptr)
	if (smallPoolSpTransform.size() < (size_t)initStruct.smallPoolSize)
		smallPoolSpTransform.push_back(ptr);
	else
		delete ptr;
}

void CommonDelete(const void* ptr)
{
	CHECK_PARAM_PTR(ptr);
	delete ptr;
}

hkVector4 F3TohkVec4(float x, float y, float z) {
	return hkVector4(x, y, z);
}
hkVector4 F4TohkVec4(float x, float y, float z, float w) {
	return hkVector4(x, y, z, w);
}
void hkVec4ToF3(float *p, hkVector4 &v) {
	*p = v.getComponent<0>();
	*(p+1) = v.getComponent<1>();
	*(p+2) = v.getComponent<2>();
}
hkVector4 Vec3TohkVec4(const spVec3 vec3) {
	CHECK_PARAM_PTR(vec3);
	return F3TohkVec4(vec3->x, vec3->y, vec3->z);
}
hkVector4 Vec4TohkVec4(const spVec4 vec4) {
	CHECK_PARAM_PTR(vec4);
	return F4TohkVec4(vec4->x, vec4->y, vec4->z, vec4->w);
}
hkQuaternion Vec4TohkQuaternion(const spVec4 vec4) { 
	CHECK_PARAM_PTR(vec4);
	return hkQuaternion(vec4->x, vec4->y, vec4->z, vec4->w); 
}
hkQsTransform TransformTohkQsTransform(const spTransform transform) {
	CHECK_PARAM_PTR(transform);
	return hkQsTransform(
		hkVector4(transform->positionX, transform->positionY, transform->positionZ),
		hkQuaternion(transform->rotationX, transform->rotationY, transform->rotationZ, transform->rotationW),
		hkVector4(transform->scaleX, transform->scaleY, transform->scaleZ)
	); 
}
hkMatrix4 Matrix4TohkMatrix4(const spMatrix4 mat) {
	CHECK_PARAM_PTR(mat);

	hkMatrix4 mkt;
	mkt.setElement<0, 0>(mat->r0[0]);
	mkt.setElement<0, 1>(mat->r0[1]);
	mkt.setElement<0, 2>(mat->r0[2]);
	mkt.setElement<0, 3>(mat->r0[3]);

	mkt.setElement<1, 0>(mat->r1[0]);
	mkt.setElement<1, 1>(mat->r1[1]);
	mkt.setElement<1, 2>(mat->r1[2]);
	mkt.setElement<1, 3>(mat->r1[3]);

	mkt.setElement<2, 0>(mat->r2[0]);
	mkt.setElement<2, 1>(mat->r2[1]);
	mkt.setElement<2, 2>(mat->r2[2]);
	mkt.setElement<2, 3>(mat->r2[2]);

	mkt.setElement<3, 0>(mat->r3[0]);
	mkt.setElement<3, 1>(mat->r3[1]);
	mkt.setElement<3, 2>(mat->r3[2]);
	mkt.setElement<3, 3>(mat->r3[2]);

	return mkt;
}
hkMatrix3 Matrix4TohkMatrix3(const spMatrix4 mat) {
	CHECK_PARAM_PTR(mat);

	hkMatrix3 mkt;
	mkt.setElement<0, 0>(mat->r0[0]);
	mkt.setElement<0, 1>(mat->r0[1]);
	mkt.setElement<0, 2>(mat->r0[2]);

	mkt.setElement<1, 0>(mat->r1[0]);
	mkt.setElement<1, 1>(mat->r1[1]);
	mkt.setElement<1, 2>(mat->r1[2]);

	mkt.setElement<2, 0>(mat->r2[0]);
	mkt.setElement<2, 1>(mat->r2[1]);
	mkt.setElement<2, 2>(mat->r2[2]);

	return mkt;
}

std::string module0Path;
std::string buildUUID;
std::string buildUUID2;//666d ccad 4ae6 97b4 5aac 145f 18f4 9c5b
int lastTestV1 = -1;

void setBuildUUID(char* uuid) {
	buildUUID = uuid;
	char buf[33];
	char buff[5];
	buff[0] = 37;
	buff[1] = 48;
	buff[2] = 52;
	buff[3] = 120;
	buff[4] = 0;
	sprintf(buf, buff, 0x666d);
	sprintf(buf + 4, buff, 0xccad);
	sprintf(buf + 8, buff, 0x4ae6);
	sprintf(buf + 12, buff, 0x97b4);
	sprintf(buf + 16, buff, 0x5aac);
	sprintf(buf + 20, buff, 0x145f);
	sprintf(buf + 24, buff, 0x18f4);
	sprintf(buf + 28, buff, 0x9c5b);
	buildUUID2 = buf;
}

int GetNativeVersion() { return 2301; }

void InitInfo() {
	char fileName[500];

	GetModuleFileNameA(0, fileName, 500);

	module0Path = fileName;

	const char *cmdLine = GetCommandLineA();
	if (StringHlp::StrContainsA(cmdLine, "-disablePhysics", nullptr))
		disablePhysics = true;
	if (StringHlp::StrContainsA(cmdLine, "-disableException", nullptr)) 
		lastHasException = true;
}
bool TestLibraryAvailability2() {
	if(lastTestV1 == -1) {
#if _DEBUG
		lastTestV1 = 1;
		return true;
#else

		char cm[13];
		cm[0] = 66;
		cm[1] = 97;
		cm[2] = 108;
		cm[3] = 108;
		cm[4] = 97;
		cm[5] = 110;
		cm[6] = 99;
		cm[7] = 101;
		cm[8] = 46;
		cm[9] = 101;
		cm[10] = 120;
		cm[11] = 101;
		cm[12] = 0;
		auto name = Path::GetFileName(module0Path);
		lastTestV1 = StringHlp::StrEqualA(cm, name.c_str()) && StringHlp::StrEqualA(buildUUID.c_str(), buildUUID2.c_str());

#endif
	}
	return lastTestV1;
}
bool TestLibraryAvailability() {
#if _DEBUG
	char cm[10];
	cm[0] = 85;
	cm[1] = 110;
	cm[2] = 105;
	cm[3] = 116;
	cm[4] = 121;
	cm[5] = 46;
	cm[6] = 101;
	cm[7] = 120;
	cm[8] = 101;
	cm[9] = 0;
	auto name = Path::GetFileName(module0Path);
	return StringHlp::StrEqualA(cm, name.c_str());
#else
	return true;
#endif
}
void InitSmallPool() {
	for (int i = 0; i < initStruct.smallPoolSize && i < 512; i++) {
		smallPoolSpTransform.push_back(new sTransform());
		smallPoolSpVec3.push_back(new sVec3());
		smallPoolSpVec4.push_back(new sVec4());
		smallPoolSpMatrix4.push_back(new sMatrix4());
	}
}
void InitFunctions()
{
	int i = TestLibraryAvailability() ? 0 : 3;
	apiArray = new void*[256];
	apiArray[i++] = CommonDelete;
	apiArray[i++] = CreateVec3;
	apiArray[i++] = CreateTransform;
	apiArray[i++] = CreateVec4;
	apiArray[i++] = DestroyVec4;
	apiArray[i++] = DestroyVec3;
	apiArray[i++] = DestroyTransform;
	apiArray[i++] = CreatePhysicsWorld;
	apiArray[i++] = DestroyPhysicsWorld;
	apiArray[i++] = StepPhysicsWorld;
	apiArray[i++] = SetPhysicsWorldGravity;
	apiArray[i++] = UpdateAllPhysicsWorldBodys;
	apiArray[i++] = CreateRigidBody;
	apiArray[i++] = ActiveRigidBody;
	apiArray[i++] = DeactiveRigidBody;
	apiArray[i++] = SetRigidBodyMass;
	apiArray[i++] = SetRigidBodyFriction;
	apiArray[i++] = SetRigidBodyRestitution;
	apiArray[i++] = SetRigidBodyCenterOfMass;
	apiArray[i++] = SetRigidBodyPosition;
	apiArray[i++] = SetRigidBodyPositionAndRotation;
	apiArray[i++] = SetRigidBodyAngularDamping;
	apiArray[i++] = SetRigidBodyLinearDampin;
	apiArray[i++] = SetRigidBodyMotionType;
	apiArray[i++] = SetRigidBodyGravityFactor;
	apiArray[i++] = GetConvexHullResultTriangles;
	apiArray[i++] = GetConvexHullResultVertices;
	apiArray[i++] = Build3DPointsConvexHull;
	apiArray[i++] = Build3DFromPlaneConvexHull;
	apiArray[i++] = DestroyRigidBody;
	apiArray[i++] = ComputeShapeVolumeMassProperties;
	apiArray[i++] = ComputeBoxSurfaceMassProperties;
	apiArray[i++] = ComputeBoxVolumeMassProperties;
	apiArray[i++] = ComputeCapsuleVolumeMassProperties;
	apiArray[i++] = ComputeCylinderVolumeMassProperties;
	apiArray[i++] = ComputeSphereVolumeMassProperties;
	apiArray[i++] = ComputeSphereSurfaceMassProperties;
	apiArray[i++] = ComputeTriangleSurfaceMassProperties;
	apiArray[i++] = CreateBoxShape;
	apiArray[i++] = CreateSphereShape;
	apiArray[i++] = CreateCapsuleShape;
	apiArray[i++] = CreateCylindeShape;
	apiArray[i++] = CreateTriangleShape;
	apiArray[i++] = CreateConvexVerticesShape;
	apiArray[i++] = CreateConvexVerticesShapeByConvexHullResult;
	apiArray[i++] = CreateSimpleMeshShape;
	apiArray[i++] = CreateConvexTranslateShape;
	apiArray[i++] = CreateConvexTransformShape;
	apiArray[i++] = CreateListShape;
	apiArray[i++] = CreateStaticCompoundShape;
	apiArray[i++] = StaticCompoundShapeSetInstanceEnabled;
	apiArray[i++] = StaticCompoundShapeIsInstanceEnabled;
	apiArray[i++] = StaticCompoundShapeEnableAllInstancesAndShapeKeys;
	apiArray[i++] = DestroyShape;
	apiArray[i++] = TestAssert;
	apiArray[i++] = SetRigidBodyLayer;
	apiArray[i++] = SetPhysicsWorldCollisionLayerMasks;
	apiArray[i++] = CreateMatrix4;
	apiArray[i++] = DestroyMatrix4;
	apiArray[i++] = SetRigidBodyLinearVelocity;
	apiArray[i++] = SetRigidBodyAngularVelocity;
	apiArray[i++] = SetRigidBodyInertiaTensor;
	apiArray[i++] = GetRigidBodyPosition;
	apiArray[i++] = GetRigidBodyRotation;
	apiArray[i++] = GetRigidBodyAngularVelocity;
	apiArray[i++] = GetRigidBodyLinearVelocity;
	apiArray[i++] = GetRigidBodyCenterOfMassInWorld;
	apiArray[i++] = GetRigidBodyPointVelocity;
	apiArray[i++] = RigidBodyResetCenterOfMass;
	apiArray[i++] = RigidBodyResetInertiaTensor;
	apiArray[i++] = SetRigidBodyMaxLinearVelocity;
	apiArray[i++] = SetRigidBodyMaxAngularVelocity;
	apiArray[i++] = DestoryConstraints;
	apiArray[i++] = CreateBallAndSocketConstraint;
	apiArray[i++] = CreateFixedConstraint;
	apiArray[i++] = CreateStiffSpringConstraint;
	apiArray[i++] = CreateHingeConstraint;
	apiArray[i++] = CreateLimitedHingeConstraint;
	apiArray[i++] = CreateWheelConstraint;
	apiArray[i++] = CreatePulleyConstraint;
	apiArray[i++] = CreatePrismaticConstraint;
	apiArray[i++] = CreateCogWheelConstraint;
	apiArray[i++] = RigidBodyApplyForce;
	apiArray[i++] = RigidBodyApplyForceAtPoint;
	apiArray[i++] = RigidBodyApplyTorque;
	apiArray[i++] = RigidBodyApplyAngularImpulse;
	apiArray[i++] = RigidBodyApplyLinearImpulse;
	apiArray[i++] = RigidBodyApplyPointImpulse;
	apiArray[i++] = IsConstraintBroken;
	apiArray[i++] = SetConstraintBroken;
	apiArray[i++] = GetRigidBodyId;
	apiArray[i++] = GetConstraintId;
	apiArray[i++] = SetConstraintEnable;
	apiArray[i++] = PhysicsWorldRayCastBody;
	apiArray[i++] = PhysicsWorldRayCastHit;
	apiArray[i++] = CreateBvCompressedMeshShape;
	apiArray[i++] = CreateAabbPhantom;
	apiArray[i++] = SetAabbPhantomMinMax;
	apiArray[i++] = GetAabbPhantomOverlappingCollidables;
	apiArray[i++] = DestroyPhantom;
	apiArray[i++] = GetPhantomId;
	apiArray[i++] = CreateSpringAction;
	apiArray[i++] = DestroySpringAction;
	apiArray[i++] = GetRigidBodyCollisionFilterInfo;
	apiArray[i++] = SetRigidBodyCollisionFilterInfo;
	apiArray[i++] = GetNewSystemGroup;

	apiArray[254] = GetNativeVersion;
	apiArray[255] = setBuildUUID;
}

void DestroyFunctions() {
	if (apiArray) {
		delete[] apiArray;
		apiArray = nullptr;
	}
}
void DestroySmallPool() {
	for (auto it = smallPoolSpTransform.begin(); it != smallPoolSpTransform.end(); it++) 
		delete* it;
	for (auto it = smallPoolSpVec3.begin(); it != smallPoolSpVec3.end(); it++)
		delete* it;
	for (auto it = smallPoolSpVec4.begin(); it != smallPoolSpVec4.end(); it++)
		delete* it;
	for (auto it = smallPoolSpMatrix4.begin(); it != smallPoolSpMatrix4.end(); it++)
		delete* it;
	smallPoolSpMatrix4.clear();
	smallPoolSpTransform.clear();
	smallPoolSpVec4.clear();
	smallPoolSpVec3.clear();
}

