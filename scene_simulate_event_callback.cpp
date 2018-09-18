#include "scene_simulate_event_callback.h"
#include <stdio.h>

void SceneSimulateEventCallback::onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) {
	puts("onConstraintBreak");
}

void SceneSimulateEventCallback::onWake(physx::PxActor** actors, physx::PxU32 count) {
	puts("onWake");
}

void SceneSimulateEventCallback::onSleep(physx::PxActor** actors, physx::PxU32 count) {
	puts("onSleep");
}

void SceneSimulateEventCallback::onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) {
	for (physx::PxU32 i = 0; i < nbPairs; ++i) {
		physx::PxFilterData filter_data[2] = {
			pairs[i].shapes[0]->getSimulationFilterData(),
			pairs[i].shapes[1]->getSimulationFilterData()
		};
		const char* event_name;
		if (pairs[i].events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND) {
			event_name = "touch found";
			printf("onContact %s <actor:%u>, <actor:%u>", event_name, filter_data[0].word0, filter_data[1].word0);

			physx::PxContactPairPoint* contact_points = (physx::PxContactPairPoint*)alloca(pairs[i].contactCount * sizeof(physx::PxContactPairPoint));
			physx::PxU32 contact_points_num = pairs[i].extractContacts(contact_points, pairs[i].contactCount);
			if (contact_points_num) {
				printf("<contact points>:");
			}
			for (physx::PxU32 i = 0; i < contact_points_num; ++i) {
				printf("[%f,%f,%f]", contact_points[i].position.x, contact_points[i].position.y, contact_points[i].position.z);
			}
			putchar('\n');
		}
		else if (pairs[i].events & physx::PxPairFlag::eNOTIFY_TOUCH_LOST) {
			event_name = "touch lost";
			printf("onContact %s <actor:%u>, <actor:%u>\n", event_name, filter_data[0].word0, filter_data[1].word0);
		}
		else if (pairs[i].events & physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS) {
			event_name = "touch persists";
			printf("onContact %s <actor:%u>, <actor:%u>\n", event_name, filter_data[0].word0, filter_data[1].word0);
		}
		else {
			event_name = "unknow";
			printf("onContact %s <actor:%u>, <actor:%u>\n", event_name, filter_data[0].word0, filter_data[1].word0);
		}
	}
}

void SceneSimulateEventCallback::onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) {
	for (physx::PxU32 i = 0; i < count; ++i) {
		physx::PxFilterData filter_data[2] = {
			pairs[i].triggerShape->getSimulationFilterData(),
			pairs[i].otherShape->getSimulationFilterData()
		};
		physx::PxRigidActor* rigid_actor[2] = {
			pairs[i].triggerActor,
			pairs[i].otherActor
		};
		const char* event_name;
		if (pairs[i].status & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND) {
			event_name = "touch found";
		}
		else if (pairs[i].status & physx::PxPairFlag::eNOTIFY_TOUCH_LOST) {
			event_name = "touch lost";
		}
		else {
			event_name = "unknow";
		}
		printf("onTrigger %s <actor:%u>, <actor:%u>\n", event_name, filter_data[0].word0, filter_data[1].word0);
	}
}