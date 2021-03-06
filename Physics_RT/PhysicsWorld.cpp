#include "PhysicsHeader.h"
#include "PhysicsFunctions.h"
#include <list>
#include <Physics2012/Collide/Query/Collector/RayCollector/hkpAllRayHitCollector.h> 
#include <Physics2012/Collide/Query/Collector/RayCollector/hkpClosestRayHitCollector.h> 
#include <Physics2012/Dynamics/Constraint/Util/hkpConstraintStabilizationUtil.h>

extern hkJobQueue* jobQueue;
extern hkJobThreadPool* threadPool;
extern sInitStruct initStruct;

std::list<sPhysicsWorld*> lateDeleteWord;

void lateDeleteWordInfo() {
	for (auto it = lateDeleteWord.begin(); it != lateDeleteWord.end(); it++)
		delete* it;
	lateDeleteWord.clear();
}

sPhysicsWorld* CreatePhysicsWorld(spVec3 gravity, int solverIterationCount, float broadPhaseWorldSize, float collisionTolerance, 
	bool bContinuous, bool bVisualDebugger, int layerMask, int *layerToMask, int stableSolverOn,
	fnOnConstraintBreakingCallback onConstraintBreakingCallback, fnOnBodyTriggerEventCallback onBodyTriggerEeventCallback,
	fnOnBodyContactEventCallback onBodyContactEventCallback, fnOnPhantomOverlapCallback onPhantomOverlapCallback)
{
	TRY_BEGIN

#if defined(HK_COMPILER_HAS_INTRINSICS_IA32) && HK_CONFIG_SIMD == HK_CONFIG_SIMD_ENABLED
	// Flush all denormal/subnormal numbers (2^-1074 to 2^-1022) to zero.
	// Typically operations on denormals are very slow, up to 100 times slower than normal numbers.
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
#endif

	CHECK_PARAM_PTR(gravity);
	CHECK_PARAM_PTR(layerToMask);

	hkpWorld* physicsWorld = nullptr;
	{
		// The world cinfo contains global simulation parameters, including gravity, solver settings etc.
		hkpWorldCinfo worldInfo;

		worldInfo.m_gravity = hkVector4(gravity->x, gravity->y, gravity->z);
		worldInfo.m_solverIterations = solverIterationCount;
		worldInfo.m_collisionTolerance = collisionTolerance;
		worldInfo.m_contactRestingVelocity = (hkReal)(gravity->y <= -50 ? 10 : 1);
		worldInfo.m_fireCollisionCallbacks = true;
		worldInfo.m_simulationType = bContinuous ? 
			(initStruct.mulithread ? hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED : hkpWorldCinfo::SIMULATION_TYPE_CONTINUOUS) : 
			hkpWorldCinfo::SIMULATION_TYPE_DISCRETE;
		worldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_FIX_ENTITY;
		worldInfo.setBroadPhaseWorldSize(broadPhaseWorldSize);

		if (initStruct.mulithread) {

			// Set the simulation type of the world to multi-threaded.
			worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED;
			physicsWorld = new hkpWorld(worldInfo);

			// When the simulation type is SIMULATION_TYPE_MULTITHREADED, in the debug build, the sdk performs checks
			// to make sure only one thread is modifying the world at once to prevent multithreaded bugs. Each thread
			// must call markForRead / markForWrite before it modifies the world to enable these checks.
			physicsWorld->markForWrite();

			// Register all collision agents, even though only box - box will be used in this particular example.
			// It's important to register collision agents before adding any entities to the world.
			hkpAgentRegisterUtil::registerAllAgents(physicsWorld->getCollisionDispatcher());

			// We need to register all modules we will be running multi-threaded with the job queue
			physicsWorld->registerWithJobQueue(jobQueue);
		}
		else {
			physicsWorld = new hkpWorld(worldInfo);

			// Register all collision agents, even though only box - box will be used in this particular example.
			// It's important to register collision agents before adding any entities to the world.
			{
				hkpAgentRegisterUtil::registerAllAgents(physicsWorld->getCollisionDispatcher());
			}
		}
	}

	sPhysicsWorld* def = new sPhysicsWorld();

	if (bVisualDebugger) {
		//
		// Initialize the VDB
		//
		hkArray<hkProcessContext*> contexts;


		// <PHYSICS-ONLY>: Register physics specific visual debugger processes
		// By default the VDB will show debug points and lines, however some products such as physics and cloth have additional viewers
		// that can show geometries etc and can be enabled and disabled by the VDB app.
		hkpPhysicsContext* context;
		{
			// The visual debugger so we can connect remotely to the simulation
			// The context must exist beyond the use of the VDB instance, and you can make
			// whatever contexts you like for your own viewer types.
			context = new hkpPhysicsContext();
			hkpPhysicsContext::registerAllPhysicsProcesses(); // all the physics viewers
			context->addWorld(physicsWorld); // add the physics world so the viewers can see it
			contexts.pushBack(context);
		}

		hkVisualDebugger* vdb = new hkVisualDebugger(contexts);
		vdb->serve();

		def->vdb = vdb;
		def->context = context;
	}

	hkpGroupFilter* filter = new hkpGroupFilter();

	//Set layer masks
	for (int i = 1; i < 32; i++) {
		int mask = layerToMask[i];
		for (int j = i; j < 32; j++) {
			if ((mask >> j) & 0x01)
				filter->enableCollisionsBetween(i, j);
			else
				filter->disableCollisionsBetween(i, j);
		}
	}

	hkpConstraintAtom::SolvingMethod method = stableSolverOn ? hkpConstraintAtom::METHOD_STABILIZED : hkpConstraintAtom::METHOD_OLD;
	hkpConstraintStabilizationUtil::setConstraintsSolvingMethod(physicsWorld, method);

	physicsWorld->setCollisionFilter(filter, true);
	filter->removeReference();

	def->filter = filter;
	def->physicsWorld = physicsWorld;
	def->callbacks.onConstraintBreakingCallback = onConstraintBreakingCallback;
	def->callbacks.onBodyContactEventCallback = onBodyContactEventCallback;
	def->callbacks.onBodyTriggerEeventCallback = onBodyTriggerEeventCallback;
	def->callbacks.onPhantomOverlapCallback = onPhantomOverlapCallback;

	SetupWorldConstraintListener(def);

	if(initStruct.mulithread)
		physicsWorld->unmarkForWrite();// Now we have finished modifying the world, release our write marker.

	return def;

	TRY_END(nullptr)
}
void DestroyPhysicsWorld(sPhysicsWorld* world) {
	TRY_BEGIN
		CHECK_PARAM_PTR(world);

		if (world->physicsWorld) {
			if (initStruct.mulithread)
				world->physicsWorld->markForWrite();

			DestroyWorldConstraintListener(world);

			world->physicsWorld->removeReference();
			world->physicsWorld = nullptr;
		}
		
		if (world->vdb) {
			world->vdb->removeReference();
			world->vdb = nullptr;
		}

		if (world->context) {
			// Contexts are not reference counted at the base class level by the VDB as
			// they are just interfaces really. So only delete the context after you have
			// finished using the VDB.
			world->context->removeReference();
			world->context = nullptr;
		}
		world->bodyList.clear();
		lateDeleteWord.push_back(world);
	
	TRY_END_NORET
}
void SetPhysicsWorldGravity(sPhysicsWorld* world, spVec3 gravity)
{
	TRY_BEGIN
		CHECK_PARAM_PTR(world);

	if (initStruct.mulithread) world->physicsWorld->markForWrite();
	
	world->physicsWorld->setGravity(hkVector4(gravity->x, gravity->y, gravity->z));


	if (initStruct.mulithread) world->physicsWorld->unmarkForWrite();
	TRY_END_NORET
}
void SetPhysicsWorldCollisionLayerMasks(sPhysicsWorld* world, unsigned int layerId, unsigned int toMask, int enable, int forceUpdate) {
	TRY_BEGIN
		CHECK_PARAM_PTR(world);

	if (initStruct.mulithread) world->physicsWorld->markForWrite();
		
	if (enable)
		world->filter->enableCollisionsUsingBitfield(1 << layerId, toMask);
	else
		world->filter->disableCollisionsUsingBitfield(1 << layerId, toMask);

	if (forceUpdate)
		world->physicsWorld->updateCollisionFilterOnWorld(HK_UPDATE_FILTER_ON_WORLD_FULL_CHECK, HK_UPDATE_COLLECTION_FILTER_PROCESS_SHAPE_COLLECTIONS);

	if (initStruct.mulithread) world->physicsWorld->unmarkForWrite();

	TRY_END_NORET
}

void StepPhysicsWorld(sPhysicsWorld* world, float timestep) {

	TRY_BEGIN
	CHECK_PARAM_PTR(world);
	if (initStruct.mulithread) {

		// <PHYSICS-ONLY>:
		// Step the physics world. This single call steps using this thread and all threads
		// in the threadPool. For other products you add jobs, call process all jobs and wait for completion.
		// See the multithreading chapter in the user guide for details
		{
			world->physicsWorld->stepMultithreaded(jobQueue, threadPool, timestep);
		}

		if (world->vdb) {
			// Step the visual debugger. We first synchronize the timer data
			world->context->syncTimers(threadPool);
			world->vdb->step();
		}

		// Clear accumulated timer data in this thread and all slave threads
		hkMonitorStream::getInstance().reset();
		threadPool->clearTimerData();
	}
	else {
		world->physicsWorld->stepDeltaTime(timestep);
		if (world->vdb) {
			// Step the debugger
			world->vdb->step();
		}
		// Reset internal profiling info for next frame
		hkMonitorStream::getInstance().reset();
	}

	TRY_END_NORET
}
void UpdateAllPhysicsWorldBodys(sPhysicsWorld* world)
{
	TRY_BEGIN
	CHECK_PARAM_PTR(world);

	auto* ptr = world->bodyList.begin;
	while (ptr != nullptr) {

		if (ptr->active) {
			auto& pos = ptr->rigidBody->getPosition();
			auto& rot = ptr->rigidBody->getRotation();

			ptr->pos[0] = pos.getComponent(0);
			ptr->pos[1] = pos.getComponent(1);
			ptr->pos[2] = pos.getComponent(2);

			ptr->rot[0] = rot.getComponent<0>();
			ptr->rot[1] = rot.getComponent<1>();
			ptr->rot[2] = rot.getComponent<2>();
			ptr->rot[3] = rot.getComponent<3>();
		}

		ptr = ptr->next;
	}

	TRY_END_NORET
}

void SetRayCastResult(sRayCastResult* rs, hkpWorldRayCastInput &ray, const hkpWorldRayCastOutput &result) {
	hkpRigidBody* rb = hkpGetRigidBody(result.m_rootCollidable);
	rs->body = (sPhysicsRigidbody*)rb->getUserData();
	rs->hitFraction = result.m_hitFraction;

	hkVector4 position; position.setInterpolate4(ray.m_from, ray.m_to, result.m_hitFraction);
	rs->pos[0] = position.getComponent<0>();
	rs->pos[1] = position.getComponent<1>();
	rs->pos[2] = position.getComponent<2>();

	rs->normal[0] = result.m_normal.getComponent<0>();
	rs->normal[1] = result.m_normal.getComponent<1>();
	rs->normal[2] = result.m_normal.getComponent<2>();
	rs->bodyId = rs->body->id;
}
int PhysicsWorldRayCastBody(sPhysicsWorld* world, spVec3 from, spVec3 to, int rayLayer, sRayCastResult** outResult) {
	TRY_BEGIN
	CHECK_PARAM_PTR(world)

	hkpWorldRayCastInput ray;
	ray.m_from = Vec3TohkVec4(from);
	ray.m_to = Vec3TohkVec4(to);
	if (rayLayer >= 0) {
		ray.m_enableShapeCollectionFilter = true;
		ray.m_filterInfo = hkpGroupFilter::calcFilterInfo(rayLayer);
	}
	hkpWorldRayCastOutput result;
	sRayCastResult* rs = new sRayCastResult();
	world->physicsWorld->castRay(ray, result);

	if (result.hasHit())
	{
		SetRayCastResult(rs, ray, result);
		*outResult = rs; 
		return 1;
	}
	return 0;

	TRY_END(0)
}
int PhysicsWorldRayCastHit(sPhysicsWorld* world, spVec3 from, spVec3 to, int rayLayer, int castAll, sRayCastResult** outResult) {
	TRY_BEGIN
	CHECK_PARAM_PTR(world)

	hkpWorldRayCastInput ray;
	ray.m_from = Vec3TohkVec4(from);
	ray.m_to = Vec3TohkVec4(to);
	if (rayLayer >= 0) {
		ray.m_enableShapeCollectionFilter = true;
		ray.m_filterInfo = hkpGroupFilter::calcFilterInfo(rayLayer);
	}

	if (castAll) {
		hkpAllRayHitCollector collector;
		world->physicsWorld->castRay(ray, collector);

		auto &hits = collector.getHits();
		if (hits.getSize() > 0)
		{
			sRayCastResult** rs = new sRayCastResult*[hits.getSize()];

			int i = 0;
			for (auto it = hits.begin(); it != hits.end(); it++, i++) {
				auto result = *it;
				hkpRigidBody* rb = hkpGetRigidBody(result.m_rootCollidable);
				SetRayCastResult(rs[i], ray, result);
			}

			*outResult = (sRayCastResult*)rs;
			return hits.getSize();
		}
		else
		{
			return 0;
		}
	}
	else {
		hkpClosestRayHitCollector collector;
		world->physicsWorld->castRay(ray, collector);

		const hkBool didHit = collector.hasHit();
		if (didHit)
		{
			const hkpWorldRayCastOutput& result = collector.getHit();
			sRayCastResult* rs = new sRayCastResult();
			SetRayCastResult(rs, ray, result);
			*outResult = rs;

			return 1;
		}
	}
	return 0;

	TRY_END(0)
}

void TestAssert() {
	TRY_BEGIN

	HK_ASSERT2(0x76867893, false, "TestAssert");

	TRY_END_NORET
}


