#include "stdafx.h"
#include "PhysicsHeader.h"
#include "PhysicsFunctions.h"

#include <Physics2012/Dynamics/Phantom/hkpAabbPhantom.h>

extern sInitStruct initStruct;
extern bool systemQuited;
int sPhantomId = 0;

sPhysicsPhantom* CreateAabbPhantom(sPhysicsWorld* world, spVec3 m_min, spVec3 m_max, int enableListener, int layer) {
	TRY_BEGIN

	CHECK_PARAM_PTR(world)
	CHECK_PARAM_PTR(m_min)
	CHECK_PARAM_PTR(m_max)

	if (!world->physicsWorld)
		throw std::exception("physicsWorld is not create!");

	hkAabb aabb(Vec3TohkVec4(m_min), Vec3TohkVec4(m_max));

	if (initStruct.mulithread) world->physicsWorld->markForWrite();

	sPhysicsPhantom* s = new sPhysicsPhantom();
	s->world = world;
	s->id = sPhantomId++;
	s->phantom = new hkpAabbPhantom(aabb, (layer >= 0 && layer < 32) ? hkpGroupFilter::calcFilterInfo(layer) : 0);
	s->phantom->setUserData((hkUlong)s);
	s->listener = nullptr;

	if (enableListener) {
		s->listener = new MyPhantomOverlapListener(world);
		s->phantom->addPhantomOverlapListener(s->listener);
	}
	world->physicsWorld->addPhantom(s->phantom);
	s->phantom->removeReference();

	if (initStruct.mulithread) world->physicsWorld->unmarkForWrite();

	return s;

	TRY_END(nullptr)
}
void SetAabbPhantomMinMax(sPhysicsPhantom* phantom, spVec3 m_min, spVec3 m_max) {
	TRY_BEGIN

	CHECK_PARAM_PTR(phantom)
	CHECK_PARAM_PTR(m_min)
	CHECK_PARAM_PTR(m_max)

	hkAabb aabb(Vec3TohkVec4(m_min), Vec3TohkVec4(m_max));

	(dynamic_cast<hkpAabbPhantom*>(phantom->phantom))->setAabb(aabb);

	TRY_END_NORET
}
int* GetAabbPhantomOverlappingCollidables(sPhysicsPhantom* phantom, int * outLen) {
	TRY_BEGIN
	CHECK_PARAM_PTR(phantom)

	auto &arr = (dynamic_cast<hkpAabbPhantom*>(phantom->phantom))->getOverlappingCollidables();
	int* bodyIdArr = new int[arr.getSize()]; int i = 0;
	for (auto it = arr.begin(); it != arr.end(); it++) {
		auto body = (hkpEntity*)((*it)->getOwner());
		if (body)
			bodyIdArr[i++] = ((sPhysicsRigidbody*)body->getUserData())->id;
	}

	*outLen = i;
	return bodyIdArr;
		
	TRY_END(nullptr)
}
void DestroyPhantom(sPhysicsPhantom* ptr) {
	TRY_BEGIN
		CHECK_PARAM_PTR(ptr)

	if (ptr->listener) {
		if (ptr->phantom)
			ptr->phantom->removePhantomOverlapListener(ptr->listener);
		ptr->listener->removeReference();
		ptr->listener = nullptr;
	}
	if (!systemQuited && ptr->world && ptr->world->physicsWorld) {
		if (initStruct.mulithread) {
			ptr->world->physicsWorld->markForWrite();
			ptr->world->physicsWorld->removePhantom(ptr->phantom);
			ptr->world->physicsWorld->unmarkForWrite();
		}
		else {
			ptr->world->physicsWorld->removePhantom(ptr->phantom);
		}
	}

	delete ptr;

	TRY_END_NORET
}
int GetPhantomId(sPhysicsPhantom* ptr) {
	TRY_BEGIN
	CHECK_PARAM_PTR(ptr)
	return ptr->id;
	TRY_END(0)
}
