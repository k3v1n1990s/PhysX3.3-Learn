#ifndef SCENE_SIMULATE_FILTER_SHADER_H
#define	SCENE_SIMULATE_FILTER_SHADER_H

#include "PhysX-3.3-mini/Include/PxPhysicsAPI.h"

physx::PxFilterFlags SceneSimulationFilterShader(
	physx::PxFilterObjectAttributes attributes0,
	physx::PxFilterData filterData0,
	physx::PxFilterObjectAttributes attributes1,
	physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags,
	const void* constantBlock,
	physx::PxU32 constantBlockSize);

class SceneSimulateFilterCallback : public physx::PxSimulationFilterCallback {
	// if numThreads > 0, use thread of PxSceneDesc.cpuDispatcher exec...
public:
	physx::PxFilterFlags pairFound(physx::PxU32 pairID,
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, const physx::PxActor* a0, const physx::PxShape* s0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, const physx::PxActor* a1, const physx::PxShape* s1,
		physx::PxPairFlags& pairFlags);

	void pairLost(physx::PxU32 pairID,
		physx::PxFilterObjectAttributes attributes0,
		physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1,
		physx::PxFilterData filterData1,
		bool objectRemoved);

	bool statusChange(physx::PxU32& pairID, physx::PxPairFlags& pairFlags, physx::PxFilterFlags& filterFlags);
};

#endif
