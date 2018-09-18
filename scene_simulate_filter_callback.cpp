#include "scene_simulate_filter_callback.h"
#include <stdio.h>

physx::PxFilterFlags SceneSimulationFilterShader(
	physx::PxFilterObjectAttributes attributes0,
	physx::PxFilterData filterData0,
	physx::PxFilterObjectAttributes attributes1,
	physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags,
	const void* constantBlock,
	physx::PxU32 constantBlockSize)
{
	pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT | physx::PxPairFlag::eTRIGGER_DEFAULT | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
	//pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
	//pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
	//pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
	//return physx::PxFilterFlag::eCALLBACK;
	return physx::PxFilterFlag::eDEFAULT;
}

physx::PxFilterFlags SceneSimulateFilterCallback::pairFound(physx::PxU32 pairID,
	physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0, const physx::PxActor* a0, const physx::PxShape* s0,
	physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1, const physx::PxActor* a1, const physx::PxShape* s1,
	physx::PxPairFlags& pairFlags)
{
	printf("pairFound <actor:%u>, <actor:%u>, <flag:%u>\n", filterData0.word0, filterData1.word0, (unsigned int)pairFlags);
	return physx::PxFilterFlag::eNOTIFY;
}

void SceneSimulateFilterCallback::pairLost(physx::PxU32 pairID,
	physx::PxFilterObjectAttributes attributes0,
	physx::PxFilterData filterData0,
	physx::PxFilterObjectAttributes attributes1,
	physx::PxFilterData filterData1,
	bool objectRemoved)
{
	printf("pairLost <actor:%u>, <actor:%u>\n", filterData0.word0, filterData1.word0);
}

bool SceneSimulateFilterCallback::statusChange(physx::PxU32& pairID, physx::PxPairFlags& pairFlags, physx::PxFilterFlags& filterFlags) {
	return false;
}
