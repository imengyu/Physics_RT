#pragma once
#include "PhysicsHeader.h"
#include "PhysicsCommon.h"

struct sPhysicsPhantom {
	int id;
	hkpPhantom* phantom;
	sPhysicsWorld* world;
	MyPhantomOverlapListener* listener;
};

sPhysicsPhantom* CreateAabbPhantom(sPhysicsWorld* world, spVec3 min, spVec3 max, int enableListener, int layer);
void SetAabbPhantomMinMax(sPhysicsPhantom* phantom, spVec3 min, spVec3 max);
int* GetAabbPhantomOverlappingCollidables(sPhysicsPhantom* phantom, int* outLen);
void DestroyPhantom(sPhysicsPhantom* ptr);
int GetPhantomId(sPhysicsPhantom* ptr);
