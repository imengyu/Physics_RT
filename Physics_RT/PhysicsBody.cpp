#include "stdafx.h"
#include "PhysicsHeader.h"
#include "PhysicsFunctions.h"

#include <Physics2012/Utilities/Actions/Spring/hkpSpringAction.h>

extern sInitStruct initStruct;
extern bool systemQuited;
int sPhysicsRigidbodyId = 0;

sPhysicsRigidbodyMassProperties* ComputeBoxSurfaceMassProperties(spVec4 halfExtents, float mass, float surfaceThickness) {
	TRY_BEGIN
		CHECK_PARAM_PTR(halfExtents);

	sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeBoxSurfaceMassProperties(Vec4TohkVec4(halfExtents), mass, surfaceThickness, r->massProperties);
	return r;

	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeBoxVolumeMassProperties(spVec4 halfExtents, float mass) {
	TRY_BEGIN
		CHECK_PARAM_PTR(halfExtents);
	sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeBoxVolumeMassProperties(Vec4TohkVec4(halfExtents), mass, r->massProperties);
	return r;
	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeCapsuleVolumeMassProperties(spVec4 startAxis, spVec4 endAxis, float radius, float mass) {
	TRY_BEGIN
		CHECK_PARAM_PTR(startAxis);
	CHECK_PARAM_PTR(endAxis);
	sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeCapsuleVolumeMassProperties(Vec4TohkVec4(startAxis), Vec4TohkVec4(endAxis), radius, mass, r->massProperties);
	return r;
	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeCylinderVolumeMassProperties(spVec4 startAxis, spVec4 endAxis, float radius, float mass) {
	TRY_BEGIN
		CHECK_PARAM_PTR(startAxis);
	CHECK_PARAM_PTR(endAxis);
	sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeCylinderVolumeMassProperties(Vec4TohkVec4(startAxis), Vec4TohkVec4(endAxis), radius, mass, r->massProperties);
	return r;
	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeSphereVolumeMassProperties(float radius, float sphereMass) {
	TRY_BEGIN
		sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeSphereVolumeMassProperties(radius, sphereMass, r->massProperties);
	return r;
	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeSphereSurfaceMassProperties(float radius, float mass, float surfaceThickness) {
	TRY_BEGIN
		sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeSphereSurfaceMassProperties(radius, mass, surfaceThickness, r->massProperties);
	return r;
	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeTriangleSurfaceMassProperties(spVec4 v0, spVec4 v1, spVec4 v2, float mass, float surfaceThickness) {
	TRY_BEGIN
		CHECK_PARAM_PTR(v0);
	CHECK_PARAM_PTR(v1);
	CHECK_PARAM_PTR(v2);
	sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeTriangleSurfaceMassProperties(Vec4TohkVec4(v0), Vec4TohkVec4(v1), Vec4TohkVec4(v2), mass, surfaceThickness, r->massProperties);
	return r;
	TRY_END(nullptr)
}
sPhysicsRigidbodyMassProperties* ComputeShapeVolumeMassProperties(sPhysicsShape* shape, float mass) {
	TRY_BEGIN
		CHECK_PARAM_PTR(shape);
	sPhysicsRigidbodyMassProperties* r = new sPhysicsRigidbodyMassProperties();
	hkpInertiaTensorComputer::computeShapeVolumeMassProperties(shape->shape, mass, r->massProperties);
	return r;
	TRY_END(nullptr)
}

sPhysicsRigidbody* CreateRigidBody(sPhysicsWorld* world, sPhysicsShape* shape, spVec3 position, spVec4 rotation,
	char*name,
	int motionType, int qualityType, float friction, float restitution, float mass, int active, 
	int layer, int systemGroup, int subSystemId, int subSystemDontCollideWith, 
	int isTiggerVolume, int addContactListener,
	float gravityFactor, float linearDamping, float angularDamping, spVec3 centerOfMass, spMatrix4 inertiaTensor,
	spVec3 linearVelocity, spVec3 angularVelocity, float maxLinearVelocity, float maxAngularVelocity, sPhysicsRigidbodyMassProperties* massProperties
)
{
	TRY_BEGIN

		CHECK_PARAM_PTR(world);
	CHECK_PARAM_PTR(shape);
	CHECK_PARAM_PTR(position);
	CHECK_PARAM_PTR(rotation);
	CHECK_PARAM_PTR(linearVelocity);
	CHECK_PARAM_PTR(angularVelocity);

	hkpRigidBodyCinfo info;
	info.m_shape = TestLibraryAvailability2() ? shape->shape : nullptr;
	info.m_position.set(position->x, position->y, position->z);
	info.m_rotation.set(rotation->x, rotation->y, rotation->z, rotation->w);
	info.m_motionType = (hkpMotion::MotionType)motionType;;
	info.m_qualityType = (hkpCollidableQualityType)qualityType;
	info.m_linearDamping = linearDamping;
	info.m_angularDamping = angularDamping;
	info.m_maxAngularVelocity = maxAngularVelocity;
	info.m_maxLinearVelocity = maxLinearVelocity;
	info.m_gravityFactor = gravityFactor;
	info.m_friction = friction;
	info.m_restitution = restitution;
	info.m_mass = mass;
	info.m_collisionFilterInfo = hkpGroupFilter::calcFilterInfo((layer >= 0 && layer < 32) ? layer : 0, systemGroup, subSystemId, subSystemDontCollideWith);
	info.m_angularVelocity = hkVector4(angularVelocity->x, angularVelocity->y, angularVelocity->z);
	info.m_linearVelocity = hkVector4(linearVelocity->x, linearVelocity->y, linearVelocity->z);

	if (massProperties) {
		info.m_centerOfMass = massProperties->massProperties.m_centerOfMass;
		info.m_inertiaTensor = massProperties->massProperties.m_inertiaTensor;
		info.m_mass = massProperties->massProperties.m_mass;
	}
	else {
		info.m_mass = mass;
		if(centerOfMass)
			info.m_centerOfMass.set(centerOfMass->x, centerOfMass->y, centerOfMass->z);
		if (inertiaTensor)
			info.m_inertiaTensor = Matrix4TohkMatrix3(inertiaTensor);
	}

	if (addContactListener) {
		info.m_contactPointCallbackDelay = 0;
	}

	sPhysicsRigidbody* body = new sPhysicsRigidbody();
	memset(body, 0, sizeof(sPhysicsRigidbody));

	hkpRigidBody* newRigidBody = new hkpRigidBody(info);

	if (initStruct.mulithread) world->physicsWorld->markForWrite();

	world->physicsWorld->addEntity(newRigidBody, active ? HK_ENTITY_ACTIVATION_DO_ACTIVATE : HK_ENTITY_ACTIVATION_DO_NOT_ACTIVATE);

	newRigidBody->setUserData(TestLibraryAvailability2() ? (hkUlong)body : 0);
	newRigidBody->enableDeactivation(true);
	newRigidBody->setName(name);
	newRigidBody->removeReference();

	if (isTiggerVolume)
		(new MyTriggerVolume(body))->removeReference();

	if (addContactListener) {
		body->collisionListener = new MyCollisionResolution(world);
		newRigidBody->addContactListener(body->collisionListener);
	}

	if (initStruct.mulithread) world->physicsWorld->unmarkForWrite();

	body->rigidBody = newRigidBody;
	body->active = active;
	body->world = world;
	body->id = ++sPhysicsRigidbodyId;

	world->bodyList.add(body);
	return body;
	TRY_END(nullptr)
}
void DestroyRigidBody(sPhysicsRigidbody* body)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	
	if (!systemQuited && body->world) {
		if (body->world->physicsWorld) {

			if (initStruct.mulithread) body->world->physicsWorld->markForWrite();

			if (body->collisionListener) {
				body->rigidBody->removeContactListener(body->collisionListener);
				delete body->collisionListener;
				body->collisionListener = nullptr;
			}
			body->world->physicsWorld->removeEntity(body->rigidBody);

			if (initStruct.mulithread) body->world->physicsWorld->unmarkForWrite();
		}

		body->world->bodyList.remove(body);
	}

	delete body;
	TRY_END_NORET
}

int GetRigidBodyId(sPhysicsRigidbody* body)
{
	TRY_BEGIN
	CHECK_PARAM_PTR(body)

	return body->id;

	TRY_END(0)
}
void ActiveRigidBody(sPhysicsRigidbody* body)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	if (body->world && body->world->physicsWorld) {
		body->rigidBody->activate();
		body->active = true;
	}
	TRY_END_NORET
}
void DeactiveRigidBody(sPhysicsRigidbody* body)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	if (!systemQuited && body->world && body->world->physicsWorld) {
		body->rigidBody->requestDeactivation();
		body->active = false;
	}
	TRY_END_NORET
}
void SetRigidBodyMass(sPhysicsRigidbody* body, float mass)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setMass(mass);
	TRY_END_NORET
}
void SetRigidBodyFriction(sPhysicsRigidbody* body, float friction)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setFriction(friction);
	TRY_END_NORET
}
void SetRigidBodyMotionType(sPhysicsRigidbody* body, int newState)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setMotionType((hkpMotion::MotionType)newState);
	TRY_END_NORET
}
void SetRigidBodyLayer(sPhysicsRigidbody* body, int layer)
{
	TRY_BEGIN
	CHECK_PARAM_PTR(body);

	if (body->world && body->world->physicsWorld) {
		body->rigidBody->setCollisionFilterInfo(hkpGroupFilter::calcFilterInfo(layer >= 0 && layer < 32 ? layer : 0));
		body->world->physicsWorld->updateCollisionFilterOnEntity(body->rigidBody,
			HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY,
			HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);
	}

	TRY_END_NORET
}
void SetRigidBodyRestitution(sPhysicsRigidbody* body, float restitution)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setRestitution(restitution);
	TRY_END_NORET
}
void SetRigidBodyAngularDamping(sPhysicsRigidbody* body, float angularDamping)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setAngularDamping(angularDamping);
	TRY_END_NORET
}
void SetRigidBodyLinearDampin(sPhysicsRigidbody* body, float linearDamping)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setLinearDamping(linearDamping);
	TRY_END_NORET
}
void SetRigidBodyLinearVelocity(sPhysicsRigidbody* body, spVec3 v)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setLinearVelocity(hkVector4(v->x, v->y, v->z));
	TRY_END_NORET
}
void SetRigidBodyAngularVelocity(sPhysicsRigidbody* body, spVec3 v)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	body->rigidBody->setAngularVelocity(hkVector4(v->x, v->y, v->z));
	TRY_END_NORET
}
void SetRigidBodyCenterOfMass(sPhysicsRigidbody* body, spVec3 centerOfMass)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(centerOfMass);

	body->rigidBody->setCenterOfMassLocal(hkVector4(centerOfMass->x, centerOfMass->y, centerOfMass->z));
	TRY_END_NORET
}
void SetRigidBodyInertiaTensor(sPhysicsRigidbody* body, spMatrix4 inertiaTensor)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(inertiaTensor);

	body->rigidBody->setInertiaLocal(Matrix4TohkMatrix3(inertiaTensor));
	TRY_END_NORET
}
void SetRigidBodyGravityFactor(sPhysicsRigidbody* body, float gravityFactor)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->setGravityFactor(gravityFactor);
	TRY_END_NORET
}
void SetRigidBodyPosition(sPhysicsRigidbody* body, spVec3 pos)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(pos);

	body->rigidBody->setPosition(hkVector4(pos->x, pos->y, pos->z));
	TRY_END_NORET
}
void SetRigidBodyPositionAndRotation(sPhysicsRigidbody* body, spVec3 pos, spVec4 rotation)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(pos);
	CHECK_PARAM_PTR(rotation);

	body->rigidBody->setPositionAndRotation(
		hkVector4(pos->x, pos->y, pos->z),
		hkQuaternion(rotation->x, rotation->y, rotation->z, rotation->w)
	);
	TRY_END_NORET
}
void GetRigidBodyPosition(sPhysicsRigidbody* body, spVec3 outPos)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(outPos);

	auto &p = body->rigidBody->getPosition();
	outPos->x = p.getComponent<0>();
	outPos->y = p.getComponent<1>();
	outPos->z = p.getComponent<2>();
	TRY_END_NORET
}
void GetRigidBodyRotation(sPhysicsRigidbody* body, spVec4 outRot)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(outRot);

	auto& p = body->rigidBody->getRotation();
	outRot->x = p.getComponent<0>();
	outRot->y = p.getComponent<1>();
	outRot->z = p.getComponent<2>();
	outRot->w = p.getComponent<3>();
	TRY_END_NORET
}
void GetRigidBodyAngularVelocity(sPhysicsRigidbody* body, spVec3 outPos)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(outPos);

	auto& p = body->rigidBody->getAngularVelocity();
	outPos->x = p.getComponent<0>();
	outPos->y = p.getComponent<1>();
	outPos->z = p.getComponent<2>();
	TRY_END_NORET
}
void GetRigidBodyLinearVelocity(sPhysicsRigidbody* body, spVec3 outPos)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(outPos);

	auto& p = body->rigidBody->getLinearVelocity();
	outPos->x = p.getComponent<0>();
	outPos->y = p.getComponent<1>();
	outPos->z = p.getComponent<2>();
	TRY_END_NORET
}
void GetRigidBodyCenterOfMassInWorld(sPhysicsRigidbody* body, spVec3 outPos)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(outPos);

	auto& p = body->rigidBody->getCenterOfMassInWorld();
	outPos->x = p.getComponent<0>();
	outPos->y = p.getComponent<1>();
	outPos->z = p.getComponent<2>();
	TRY_END_NORET
}
void GetRigidBodyPointVelocity(sPhysicsRigidbody* body, spVec3 pt, spVec3 outPos)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);
	CHECK_PARAM_PTR(pt);
	CHECK_PARAM_PTR(outPos);

	hkVector4 hkPt(pt->x, pt->y, pt->z);
	hkVector4 p(pt->x, pt->y, pt->z);

	body->rigidBody->getPointVelocity(hkPt, p);
	outPos->x = p.getComponent<0>();
	outPos->y = p.getComponent<1>();
	outPos->z = p.getComponent<2>();

	TRY_END_NORET
}
void RigidBodyResetCenterOfMass(sPhysicsRigidbody* body) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->setCenterOfMassLocal(hkVector4());

	TRY_END_NORET
}
void RigidBodyResetInertiaTensor(sPhysicsRigidbody* body) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	hkMatrix3 identity; identity.setIdentity();
	body->rigidBody->setInertiaLocal(identity);

	TRY_END_NORET
}
void SetRigidBodyMaxLinearVelocity(sPhysicsRigidbody* body, float v) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->setMaxLinearVelocity(v);
	TRY_END_NORET
}
void SetRigidBodyMaxAngularVelocity(sPhysicsRigidbody* body, float v) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->setMaxAngularVelocity(v);
	TRY_END_NORET
}
void RigidBodyApplyForce(sPhysicsRigidbody* body, float delteTime, spVec3 force) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->applyForce(delteTime, Vec3TohkVec4(force));
	TRY_END_NORET
}
void RigidBodyApplyForceAtPoint(sPhysicsRigidbody* body, float delteTime, spVec3 force, spVec3 point) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->applyForce(delteTime, Vec3TohkVec4(force), Vec3TohkVec4(point));
	TRY_END_NORET
}
void RigidBodyApplyTorque(sPhysicsRigidbody* body, float delteTime, spVec3 torque) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->applyTorque(delteTime, Vec3TohkVec4(torque));
	TRY_END_NORET
}
void RigidBodyApplyAngularImpulse(sPhysicsRigidbody* body, spVec3 imp) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->applyAngularImpulse(Vec3TohkVec4(imp));
	TRY_END_NORET
}
void RigidBodyApplyLinearImpulse(sPhysicsRigidbody* body, spVec3 imp) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->applyLinearImpulse(Vec3TohkVec4(imp));
	TRY_END_NORET
}
void RigidBodyApplyPointImpulse(sPhysicsRigidbody* body, spVec3 imp, spVec3 point) {
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->applyPointImpulse(Vec3TohkVec4(imp), Vec3TohkVec4(point));
	TRY_END_NORET
}
int GetRigidBodyCollisionFilterInfo(sPhysicsRigidbody* body)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	return body->rigidBody->getCollisionFilterInfo();

	TRY_END(0)
}
void SetRigidBodyCollisionFilterInfo(sPhysicsRigidbody* body, int layer, int systemGroup, int subSystemId, int subSystemDontCollideWith)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(body);

	body->rigidBody->setCollisionFilterInfo(hkpGroupFilter::calcFilterInfo(layer, systemGroup, subSystemId, subSystemDontCollideWith));
	
	if(body->world->physicsWorld)
		body->world->physicsWorld->updateCollisionFilterOnEntity(body->rigidBody,
			HK_UPDATE_FILTER_ON_ENTITY_DISABLE_ENTITY_ENTITY_COLLISIONS_ONLY,
			HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

	TRY_END_NORET
}

int GetNewSystemGroup(sPhysicsWorld* world) {
	TRY_BEGIN
		CHECK_PARAM_PTR(world);
		
	return ((hkpGroupFilter*)world->physicsWorld->getCollisionFilter())->getNewSystemGroup();

	TRY_END(0)
} 
void* CreateSpringAction(sPhysicsWorld* world, sPhysicsRigidbody* body1, sPhysicsRigidbody* body2, spVec3 position1, spVec3 position2, float springConstant, float springDamping, float springRestLength) {
	TRY_BEGIN
		CHECK_PARAM_PTR(world)
		CHECK_PARAM_PTR(body1)

	if (initStruct.mulithread) world->physicsWorld->markForWrite();

	hkpRigidBody* fixedBody = world->physicsWorld->getFixedRigidBody();
	auto otherBody = body1 == nullptr ? fixedBody : body1->rigidBody;

	hkpSpringAction* spring = new hkpSpringAction(body1->rigidBody, otherBody);
	spring->setPositionsInWorldSpace(Vec3TohkVec4(position1), Vec3TohkVec4(position2));
	spring->setStrength(springConstant);
	spring->setDamping(springDamping);
	spring->setRestLength(springRestLength);
	world->physicsWorld->addAction(spring)->removeReference();

	if (initStruct.mulithread) world->physicsWorld->unmarkForWrite();

	return spring;

	TRY_END(nullptr)
}
void DestroySpringAction(sPhysicsWorld* world, void* spring) {
	TRY_BEGIN
		CHECK_PARAM_PTR(world)
		CHECK_PARAM_PTR(spring)

	if (initStruct.mulithread) world->physicsWorld->markForWrite();

	world->physicsWorld->removeAction((hkpSpringAction*)spring);

	if (initStruct.mulithread) world->physicsWorld->unmarkForWrite();

	TRY_END_NORET
}