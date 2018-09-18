#ifndef SCENE_SIMULATE_EVENT_CALLBACK_H
#define	SCENE_SIMULATE_EVENT_CALLBACK_H

#include "PhysX-3.3-mini/Include/PxPhysicsAPI.h"

class SceneSimulateEventCallback : public physx::PxSimulationEventCallback {
	// scene->fetchResults() will call these functions...
public:
	void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count);
	void onWake(physx::PxActor** actors, physx::PxU32 count);
	void onSleep(physx::PxActor** actors, physx::PxU32 count);
	void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs);
	void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count);
};

#endif // !SIMULATE_EVENT_CALLBACK_H
