#pragma once
#include "PhysicsHeader.h"
#include "PhysicsCommon.h"

struct sPhysicsConstraints {
	hkpConstraintInstance* instance;
	hkpBreakableConstraintData* breaker;
	sPhysicsWorld* world;
	bool breakable;
	int id;
};

struct sConstraintBreakData {
	float threshold;
	float maximumAngularImpulse;
	float maximumLinearImpulse;
};
struct sConstraintMotorData {
	int spring;
	float m_tau;
	float m_damping;
	float m_proportionalRecoveryVelocity;
	float m_constantRecoveryVelocity;
	float m_minForce;
	float m_maxForce;
	float m_springConstant;
	float m_springDamping;
};

void SetupWorldConstraintListener(sPhysicsWorld* world);
void DestroyWorldConstraintListener(sPhysicsWorld* world);

void DestoryConstraints(sPhysicsConstraints* constraint);
int IsConstraintBroken(sPhysicsConstraints* constraint);
int GetConstraintId(sPhysicsConstraints* constraint);
void SetConstraintBroken(sPhysicsConstraints* constraint, int broken, float force);
void SetConstraintEnable(sPhysicsConstraints* constraint, int enable);
sPhysicsConstraints* CreateBallAndSocketConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, sConstraintBreakData* breakable, int priority);
sPhysicsConstraints* CreateFixedConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, sConstraintBreakData* breakable, int priority);
sPhysicsConstraints* CreateStiffSpringConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povitAW, spVec3 povitBW, float springMin, float springMax, sConstraintBreakData* breakable, int priority);
sPhysicsConstraints* CreateHingeConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, spVec3 axis, sConstraintBreakData* breakable, int priority);
sPhysicsConstraints* CreateLimitedHingeConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, spVec3 axis, float agularLimitMin, float agularLimitMax, sConstraintBreakData* breakable, sConstraintMotorData* motorData, int priority);
sPhysicsConstraints* CreateWheelConstraint(sPhysicsRigidbody* wheelRigidBody, sPhysicsRigidbody* chassis, spVec3 povit, spVec3 axle, spVec3 suspension, spVec3 steering, float suspensionLimitMin, float suspensionLimitMax, float suspensionStrength, float suspensionDamping, sConstraintBreakData* breakable, int priority);
sPhysicsConstraints* CreatePulleyConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 bodyPivot0, spVec3 bodyPivots1, spVec3 worldPivots0, spVec3 worldPivots1, float leverageRatio, sConstraintBreakData* breakable, int priority);
sPhysicsConstraints* CreatePrismaticConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 povit, spVec3 axis, int allowRotationAroundAxis, float mmax, float mmin, float mag, sConstraintBreakData* breakable, sConstraintMotorData* motorData, int priority);
sPhysicsConstraints* CreateCogWheelConstraint(sPhysicsRigidbody* body, sPhysicsRigidbody* otherBody, spVec3 rotationPivotA, spVec3 rotationAxisA, float radiusA, spVec3 rotationPivotB, spVec3 rotationAxisB, float radiusB, sConstraintBreakData* breakable, int priority);