#include "stdafx.h"
#include "PhysicsHeader.h"
#include "PhysicsFunctions.h"

#include <Physics/Constraint/Data/BallAndSocket/hkpBallAndSocketConstraintData.h>
#include <Physics/Constraint/Data/Fixed/hkpFixedConstraintData.h>
#include <Physics/Constraint/Data/Hinge/hkpHingeConstraintData.h>
#include <Physics/Constraint/Data/LimitedHinge/hkpLimitedHingeConstraintData.h>
#include <Physics/Constraint/Data/PointToPath/hkpPointToPathConstraintData.h>
#include <Physics/Constraint/Data/PointToPlane/hkpPointToPlaneConstraintData.h>
#include <Physics/Constraint/Data/Prismatic/hkpPrismaticConstraintData.h>
#include <Physics/Constraint/Data/Pulley/hkpPulleyConstraintData.h>
#include <Physics/Constraint/Data/Wheel/hkpWheelConstraintData.h>
#include <Physics/Constraint/Data/WheelFriction/hkpWheelFrictionConstraintData.h>
#include <Physics/Constraint/Data/CogWheel/hkpCogWheelConstraintData.h>
#include <Physics/Constraint/Data/StiffSpring/hkpStiffSpringConstraintData.h>
#include <Physics/Constraint/Motor/Position/hkpPositionConstraintMotor.h>
#include <Physics/Constraint/Motor/LimitedForce/hkpLimitedForceConstraintMotor.h>
#include <Physics/Constraint/Motor/SpringDamper/hkpSpringDamperConstraintMotor.h>

void SetupWorldConstraintListener(sPhysicsWorld* world) {
	world->breakableListener = new MyBreakableListener(world);
	world->physicsWorld->addConstraintListener(world->breakableListener);
}
void DestroyWorldConstraintListener(sPhysicsWorld* world) {
	if (world->breakableListener) {
		world->physicsWorld->removeConstraintListener(world->breakableListener);
		delete world->breakableListener;
		world->breakableListener = nullptr;
	}
}

int sPhysicsConstraintsId = 0;
extern bool systemQuited;

void CreateConstraintCheck(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody) {
	CHECK_PARAM_PTR(body);

	if (!body->world->physicsWorld)
		throw std::exception("physicsWorld is destroyed");
	if (otherBody != nullptr && body->world->physicsWorld != otherBody->world->physicsWorld)
		throw std::exception("Two bodies belong to different worlds");
}
sPhysicsConstraints* CreateConstraint(sPhysicsWorld *world, hkpRigidBody* body, hkpRigidBody* otherBody, hkpConstraintData* data, sConstraintBreakData* breakable) {
	auto rs = new sPhysicsConstraints();

	if (breakable) {

		hkpBreakableConstraintData* breaker = new hkpBreakableConstraintData(data);
		breaker->setThreshold(breakable->threshold);
		breaker->setRemoveWhenBroken(false); 
		breaker->setMaximumAngularImpulse(breakable->maximumAngularImpulse);
		breaker->setMaximumLinearImpulse(breakable->maximumLinearImpulse);

		rs->instance = world->physicsWorld->createAndAddConstraintInstance(body, otherBody, breaker);
		rs->breaker = breaker;
	}
	else {
		rs->instance = world->physicsWorld->createAndAddConstraintInstance(body, otherBody, data);
	}

	rs->instance->setUserData((hkUlong)rs);
	rs->id = sPhysicsConstraintsId++;
	rs->world = world;
	rs->breakable = breakable;

	rs->instance->removeReference();
	data->removeReference();

	return rs;
}
hkpConstraintMotor* CreateConstraintMotor(sConstraintMotorData* motorData) {
	if (motorData) {
		if (motorData->spring) {
			hkpSpringDamperConstraintMotor* motor = new hkpSpringDamperConstraintMotor();
			motor->m_springDamping = motorData->m_springDamping;
			motor->m_springConstant = motorData->m_springConstant;
			motor->m_minForce = motorData->m_minForce;
			motor->m_maxForce = motorData->m_maxForce;
			return motor;
		}
		else {
			hkpPositionConstraintMotor* motor = new hkpPositionConstraintMotor();
			motor->m_tau = motorData->m_tau;
			motor->m_damping = motorData->m_damping;
			motor->m_proportionalRecoveryVelocity = motorData->m_proportionalRecoveryVelocity;
			motor->m_constantRecoveryVelocity = motorData->m_constantRecoveryVelocity;
			motor->m_minForce = motorData->m_minForce;
			motor->m_maxForce = motorData->m_maxForce;
			return motor;
		}
	}
	return nullptr;
}

void DestoryConstraints(sPhysicsConstraints* constraint) {
	TRY_BEGIN
		CHECK_PARAM_PTR(constraint);

	if (!systemQuited && !constraint->world->physicsWorld) {
		delete constraint;
		return;
	}

	constraint->world->physicsWorld->removeConstraint(constraint->instance);
	delete constraint;

	TRY_END_NORET
}
int IsConstraintBroken(sPhysicsConstraints* constraint)
{
	if (constraint && constraint->breakable)
		return constraint->breaker->getIsBroken(constraint->instance);
	return false;
}
int GetConstraintId(sPhysicsConstraints* constraint)
{
	if (constraint)
		return constraint->id;
	return 0;
}
void SetConstraintBroken(sPhysicsConstraints* constraint, int broken, float force)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(constraint)

	if (constraint->breakable)
		return constraint->breaker->setBroken(constraint->instance, broken, force);
	else 
		throw std::exception("constraint is not breakable");

	TRY_END_NORET
}
void SetConstraintEnable(sPhysicsConstraints* constraint, int enable)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(constraint)

		return constraint->instance->setEnabled(enable);

	TRY_END_NORET
}
sPhysicsConstraints* CreateBallAndSocketConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, sConstraintBreakData* breakable)
{
	TRY_BEGIN
		
	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpBallAndSocketConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), Vec3TohkVec4(povit));

	return CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);

	TRY_END(nullptr)
}
sPhysicsConstraints* CreateFixedConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, sConstraintBreakData* breakable)
{
	TRY_BEGIN
		
	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpFixedConstraintData(); 
	hkTransform tpivot;
	tpivot.setIdentity();
	tpivot.setTranslation(Vec3TohkVec4(povit));
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), tpivot);

	return CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);

	TRY_END(nullptr)
}
sPhysicsConstraints* CreateStiffSpringConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povitAW, spVec3 povitBW, float springMin, float springMax, sConstraintBreakData* breakable)
{
	TRY_BEGIN
		
	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpStiffSpringConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), Vec3TohkVec4(povitAW), Vec3TohkVec4(povitBW));
	data->setSpringLength(springMin, springMax);

	return CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);

	TRY_END(nullptr)
}
sPhysicsConstraints* CreateHingeConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, spVec3 axis, sConstraintBreakData* breakable)
{
	TRY_BEGIN
		
	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpHingeConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), Vec3TohkVec4(povit), Vec3TohkVec4(axis));

	return CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);

	TRY_END(nullptr)
}
sPhysicsConstraints* CreateLimitedHingeConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, spVec3 axis, float agularLimitMin, float agularLimitMax, sConstraintBreakData* breakable, sConstraintMotorData* motorData)
{
	TRY_BEGIN

	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpLimitedHingeConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), Vec3TohkVec4(povit), Vec3TohkVec4(axis));
	data->setMinAngularLimit(agularLimitMin);
	data->setMaxAngularLimit(agularLimitMax);

	if (motorData) {
		auto motor = CreateConstraintMotor(motorData);
		data->setMotor(motor);
		motor->removeReference();
	}

	auto instance = CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);
	if (motorData)
		data->setMotorEnabled(instance->instance->getRuntime(), true);
	return instance;

	TRY_END(nullptr)
}
sPhysicsConstraints* CreateWheelConstraint(sPhysicsRigidbody* wheelRigidBody, sPhysicsRigidbody* chassis, spVec3 povit, spVec3 axle, spVec3 suspension, spVec3 steering, float suspensionLimitMin, float suspensionLimitMax, float suspensionStrength, float suspensionDamping, sConstraintBreakData* breakable)
{
	TRY_BEGIN

	CreateConstraintCheck(wheelRigidBody, chassis);

	hkpRigidBody* fixedBody = wheelRigidBody->world->physicsWorld->getFixedRigidBody();

	auto rs = new sPhysicsConstraints();

	// Create the constraint
	auto data = new hkpWheelConstraintData();
	data->setInWorldSpace(wheelRigidBody->rigidBody->getTransform(), chassis == nullptr ? fixedBody->getTransform() : chassis->rigidBody->getTransform(), Vec3TohkVec4(povit), Vec3TohkVec4(axle), Vec3TohkVec4(suspension), Vec3TohkVec4(steering));
	data->setSuspensionMaxLimit(suspensionLimitMin);
	data->setSuspensionMinLimit(suspensionLimitMax);
	data->setSuspensionStrength(suspensionStrength);
	data->setSuspensionDamping(suspensionDamping);

	return CreateConstraint(wheelRigidBody->world, chassis->rigidBody, chassis == nullptr ? fixedBody : chassis->rigidBody, data, breakable);

	TRY_END(nullptr)
}
sPhysicsConstraints* CreatePulleyConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 bodyPivot0, spVec3 bodyPivots1, spVec3 worldPivots0, spVec3 worldPivots1, float leverageRatio, sConstraintBreakData* breakable)
{
	TRY_BEGIN

	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpPulleyConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), Vec3TohkVec4(bodyPivot0), Vec3TohkVec4(bodyPivots1), Vec3TohkVec4(worldPivots0), Vec3TohkVec4(worldPivots1), leverageRatio);

	return CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);

	TRY_END(nullptr)
}
sPhysicsConstraints* CreatePrismaticConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, spVec3 axis, int allowRotationAroundAxis, float mmax, float mmin, float mag, sConstraintBreakData* breakable, sConstraintMotorData* motorData)
{
	TRY_BEGIN

	CreateConstraintCheck(body, otherBody);

	hkpRigidBody* fixedBody = body->world->physicsWorld->getFixedRigidBody();

	// Create the constraint
	auto data = new hkpPrismaticConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody == nullptr ? fixedBody->getTransform() : otherBody->rigidBody->getTransform(), Vec3TohkVec4(povit), Vec3TohkVec4(axis));
	data->setMinLinearLimit(mmin);
	data->setMaxLinearLimit(mmax);
	data->setMaxFrictionForce(mag);
	data->allowRotationAroundAxis(allowRotationAroundAxis);

	if (motorData) {
		auto motor = CreateConstraintMotor(motorData);
		data->setMotor(motor);
		motor->removeReference();
	}

	auto instance = CreateConstraint(body->world, body->rigidBody, otherBody == nullptr ? fixedBody : otherBody->rigidBody, data, breakable);

	if (motorData)
		data->setMotorEnabled(instance->instance->getRuntime(), true);
	return instance;

	TRY_END(nullptr)
}
sPhysicsConstraints* CreateCogWheelConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 rotationPivotA, spVec3 rotationAxisA, float radiusA, spVec3 rotationPivotB, spVec3 rotationAxisB, float radiusB, sConstraintBreakData* breakable)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(otherBody)

	CreateConstraintCheck(body, otherBody);

	// Create the constraint
	auto data = new hkpCogWheelConstraintData();
	data->setInWorldSpace(body->rigidBody->getTransform(), otherBody->rigidBody->getTransform(), Vec3TohkVec4(rotationPivotA), Vec3TohkVec4(rotationAxisA), radiusA, Vec3TohkVec4(rotationPivotB), Vec3TohkVec4(rotationAxisB), radiusB);

	return CreateConstraint(body->world, body->rigidBody, otherBody->rigidBody, data, breakable);

	TRY_END(nullptr)
}