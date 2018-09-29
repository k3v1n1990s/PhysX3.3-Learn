#include "util/c/syslib/assert.h"
#include "util/c/syslib/process.h"
#include "util/c/syslib/terminal.h"
#include "util/c/syslib/time.h"
#include "util/c/syslib/math.h"
#include "PhysX-3.3-mini/Include/PxPhysicsAPI.h"
#include "scene_simulate_event_callback.h"
#include "scene_simulate_filter_callback.h"
#include "scene_xml.h"
#include "convex_mesh.h"
#include <stdio.h>
#include <iostream>

#define	LIBRARY_BASIC_PATH	""

#if defined(_WIN64)
	#ifdef _DEBUG
		#define	PHYSX_ENABLE_PVD

		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3DEBUG_x64.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3ExtensionsDEBUG.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3CommonDEBUG_x64.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3CharacterKinematicDEBUG_x64.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3CookingDEBUG_x64.lib")
		#ifdef PHYSX_ENABLE_PVD
			#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysXVisualDebuggerSDKDEBUG.lib")
		#endif
	#else
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3_x64.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3Extensions.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3Common_x64.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3CharacterKinematic_x64.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win64/PhysX3Cooking_x64.lib")
	#endif
#elif defined(_WIN32)
	#ifdef _DEBUG
		#define	PHYSX_ENABLE_PVD

		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3DEBUG_x86.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3ExtensionsDEBUG.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3CommonDEBUG_x86.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3CharacterKinematicDEBUG_x86.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3CookingDEBUG_x86.lib")
		#ifdef PHYSX_ENABLE_PVD
			#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysXVisualDebuggerSDKDEBUG.lib")
		#endif
	#else
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3_x86.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3Extensions.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3Common_x86.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3CharacterKinematic_x86.lib")
		#pragma comment(lib, LIBRARY_BASIC_PATH"PhysX-3.3-mini/Lib/vc14win32/PhysX3Cooking_x86.lib)
	#endif
#else
	#ifdef	_DEBUG
		#define	PHYSX_ENABLE_PVD
	#endif
#endif

static physx::PxDefaultAllocator gDefaultAllocatorCallback;
static physx::PxDefaultErrorCallback gDefaultErrorCallback;
static physx::PxCooking* gCooking;
static SceneSimulateEventCallback gSimulateEventCallback;
static SceneXML gSceneXML;
//static SceneSimulateFilterCallback gSimulateFilterCallback;

const unsigned int UINT_FRAME_INTERVAL_MSEC = 20;
long long UINT64_FRAME_NEXT_MSEC;

physx::PxVec3 shapeGlobalPose(const physx::PxVec3& local_pose, const physx::PxQuat& quat) {
	physx::PxQuat q(local_pose.x, local_pose.y, local_pose.z, 0.0f);
	q = quat * q * quat.getConjugate();
	return physx::PxVec3(q.x, q.y, q.z);
}

physx::PxShape* actor_move(physx::PxRigidDynamic* actor, const physx::PxVec3& dir, physx::PxReal speed) {
	static const physx::PxReal MIN_DISTANCE = 0.005f;
	physx::PxReal real_speed = speed;
	physx::PxScene* scene = actor->getScene();
	if (!scene) {
		return NULL;
	}
	//
	if (actor->getNbShapes()) {
		physx::PxU32 sweephitcnt = actor->getNbShapes() + 1;
		physx::PxSweepHit* hit = (physx::PxSweepHit*)alloca(sizeof(physx::PxSweepHit) * sweephitcnt);
		physx::PxSweepBuffer hitbuffer(hit, sweephitcnt);
		physx::PxShape** shapes = (physx::PxShape**)alloca(sizeof(physx::PxShape*) * actor->getNbShapes());
		physx::PxU32 shape_cnt = actor->getShapes(shapes, sizeof(physx::PxShape*) * actor->getNbShapes());

		for (physx::PxU32 i = 0; i < shape_cnt; ++i) {
			physx::PxTransform pos = actor->getGlobalPose();
			pos.p += shapeGlobalPose(shapes[i]->getLocalPose().p, pos.q);
			if (!scene->sweep(shapes[i]->getGeometry().any(), pos, dir, speed, hitbuffer,
				physx::PxHitFlags(physx::PxHitFlag::eDISTANCE)))
			{
				continue;
			}
			const physx::PxU32 anyhitcnt = hitbuffer.getNbAnyHits();
			for (physx::PxU32 j = 0; j < anyhitcnt; ++j) {
				const physx::PxSweepHit& sh = hitbuffer.getAnyHit(j);
				if (sh.actor == actor) {
					continue;
				}
				if (sh.distance <= MIN_DISTANCE) {
					return sh.shape;
				}
				else if (speed + MIN_DISTANCE > sh.distance)
				{
					physx::PxReal test_speed = sh.distance - MIN_DISTANCE;
					if (test_speed < real_speed)
						real_speed = test_speed;
					break;
				}
				else if (speed + MIN_DISTANCE < sh.distance) {
					continue;
				}
				else {
					return sh.shape;
				}
			}
		}
	}
	physx::PxTransform pos = actor->getGlobalPose();
	pos.p += dir * real_speed;
	if (actor->getRigidDynamicFlags() & physx::PxRigidDynamicFlag::eKINEMATIC)
		actor->setKinematicTarget(pos);
	else
		actor->setGlobalPose(pos);
	return NULL;
}

int main(int argc, char** argv) {
	// init
	assertTRUE(PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback));
	assertTRUE(PxCreatePhysics(PX_PHYSICS_VERSION, PxGetFoundation(), physx::PxTolerancesScale()));
	assertTRUE(PxInitExtensions(PxGetPhysics()));
	assertTRUE(gCooking = PxCreateCooking(PX_PHYSICS_VERSION, PxGetFoundation(), physx::PxCookingParams(PxGetPhysics().getTolerancesScale())));
#ifdef PHYSX_ENABLE_PVD
	if (!PxGetPhysics().getPvdConnectionManager()) {
		fputs("PVD connect manager isn't exist\n", stderr);
		getchar();
		return 1;
	}
	physx::PxVisualDebuggerConnection* px_debug_connection = physx::PxVisualDebuggerExt::createConnection(PxGetPhysics().getPvdConnectionManager(), "127.0.0.1", 5425, 5000, physx::PxVisualDebuggerExt::getAllConnectionFlags());
	if (!px_debug_connection) {
		fputs("warnning: PVD connect failure\n", stderr);
	}
#endif

	// create scene
	physx::PxSceneDesc scene_desc(PxGetPhysics().getTolerancesScale());
	scene_desc.gravity = physx::PxVec3(0.0f, -9.8f, 0.0f);
	scene_desc.filterShader = physx::PxDefaultSimulationFilterShader;//SceneSimulationFilterShader;
	scene_desc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(0);
	//scene_desc.filterCallback = &gSimulateFilterCallback;
	//scene_desc.simulationEventCallback = &gSimulateEventCallback;
	physx::PxScene* scene;
	assertTRUE(scene = PxGetPhysics().createScene(scene_desc));
	physx::PxControllerManager* ctrlmgr = NULL;
	assertTRUE(ctrlmgr = PxCreateControllerManager(*scene));

	// create material
	physx::PxMaterial* material;
	assertTRUE(material = PxGetPhysics().createMaterial(0.15f, 0.05f, 0.0f));

	//////////////////////////////////
	if (!gSceneXML.loadfile("my.xml")) {
		fputs("load scene xml file error\n", stderr);
		getchar();
		return 1;
	}
	gSceneXML.loadscene(scene, material);
	/*
	const physx::PxVec3 convex_verts[] = {
		physx::PxVec3(10,2,0),
		physx::PxVec3(-10,2,0),
		physx::PxVec3(0,2,10),
		physx::PxVec3(0,2,-10),
	};
	physx::PxConvexMeshDesc convex_desc;
	convex_desc.points.count = sizeof(convex_verts) / sizeof(convex_verts[0]);
	convex_desc.points.stride = sizeof(convex_verts[0]);
	convex_desc.points.data = convex_verts;
	convex_desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;
	physx::PxDefaultMemoryOutputStream outbuf;
	assertTRUE(gCooking->cookConvexMesh(convex_desc, outbuf));
	physx::PxDefaultMemoryInputData inputdata(outbuf.getData(), outbuf.getSize());
	physx::PxConvexMesh* convex_mesh;
	assertTRUE(convex_mesh = PxGetPhysics().createConvexMesh(inputdata));

	float euler_angle[3] = {
		0.0f,
		0.0f,
		mathDegToRad(180.0)
	};
	float quat[4];
	mathEulerAnglesToQuaternion(euler_angle, quat);
	physx::PxTransform convex_transform(physx::PxVec3(-10.0f, 0.0f, 0.0f), physx::PxQuat(quat[0], quat[1], quat[2], quat[3]));
	physx::PxRigidDynamic* convex_actor;
	assertTRUE(convex_actor = PxGetPhysics().createRigidDynamic(convex_transform));
	//convex_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
	physx::PxShape* convex_shape;
	assertTRUE(convex_shape = convex_actor->createShape(physx::PxConvexMeshGeometry(convex_mesh), *material));
	scene->addActor(*convex_actor);
	*/

	physx::PxTransform cylinder_transform(physx::PxVec3(-10.0f, 10.0f, 0.0f));
	physx::PxRigidDynamic* cylinder_actor;
	assertTRUE(cylinder_actor = PxGetPhysics().createRigidDynamic(cylinder_transform));
	cylinder_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
	physx::PxTriangleMesh* cylinder_mesh;
	assertTRUE(cylinder_mesh = createCylinderMesh(gCooking, 1.0f, 2.0f, 20));
	cylinder_actor->createShape(physx::PxTriangleMeshGeometry(cylinder_mesh), *material);
	scene->addActor(*cylinder_actor);

	/*
	// create plane actor
	physx::PxTransform plane_transform(physx::PxVec3(0.0f, 0.0f, 0.0f), physx::PxQuat(q[0], q[1], q[2], q[3]));
	physx::PxRigidStatic* plane_actor;
	assertTRUE(plane_actor = PxGetPhysics().createRigidStatic(plane_transform));
	assertTRUE(plane_actor->createShape(physx::PxPlaneGeometry(), *material));
	// create capsule actor
	physx::PxTransform capsule_transform(physx::PxVec3(3.5f, 1.501f, 2.0f), physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0.0f, 0.0f, 1.0f).getNormalized()));
	physx::PxRigidDynamic* capsule_actor;
	assertTRUE(capsule_actor = PxGetPhysics().createRigidDynamic(capsule_transform));
	capsule_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
	assertTRUE(capsule_actor->createShape(physx::PxCapsuleGeometry(0.5f, 0.5f), *material));
	assertTRUE(capsule_actor->createShape(physx::PxBoxGeometry(0.25f, 0.25f, 0.25f), *material, physx::PxTransform(1.25f, 0.0f, 0.0f)));
	assertTRUE(capsule_actor->createShape(physx::PxBoxGeometry(0.1f, 0.2f, 0.1f), *material, physx::PxTransform(0.4f, 0.7f, 0.0f)));
	assertTRUE(capsule_actor->createShape(physx::PxBoxGeometry(0.1f, 0.2f, 0.1f), *material, physx::PxTransform(0.4f, -0.7f, 0.0f)));
	assertTRUE(capsule_actor->createShape(physx::PxBoxGeometry(0.25f, 0.1f, 0.1f), *material, physx::PxTransform(-1.25f, 0.25f, 0.0f)));
	assertTRUE(capsule_actor->createShape(physx::PxBoxGeometry(0.25f, 0.1f, 0.1f), *material, physx::PxTransform(-1.25f, -0.25f, 0.0f)));
	// create box
	physx::PxRigidDynamic* box_actor;
	assertTRUE(box_actor = PxGetPhysics().createRigidDynamic(physx::PxTransform(0.0f, 0.0f, 0.0f)));
	box_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
	physx::PxShape* box_shape;
	assertTRUE(box_shape = box_actor->createShape(physx::PxBoxGeometry(0.5f, 0.5f, 0.5f), *material));
	box_shape->setSimulationFilterData(physx::PxFilterData(1, 0, 0, 0));

	box_actor->setGlobalPose(physx::PxTransform(physx::PxVec3(-3.5f, 0.5f, 0.0f)));

	// scene add all actors
	scene->addActor(*plane_actor);
	scene->addActor(*capsule_actor);
	scene->addActor(*box_actor);

	// create controller
	//physx::PxCapsuleControllerDesc capsule_ctrl_desc;
	//capsule_ctrl_desc.material = material;
	//capsule_ctrl_desc.radius = 1.0f;
	//capsule_ctrl_desc.height = 4.0f;
	//physx::PxCapsuleController* capsule_controller;
	//assert_true(capsule_controller = (physx::PxCapsuleController*)ctrlmgr->createController(capsule_ctrl_desc));
	//capsule_controller->setFootPosition(physx::PxExtendedVec3(0.0, 0.0, 0.0));
	*/
	// scene loop
	const physx::PxReal max_wait_sec = UINT_FRAME_INTERVAL_MSEC * 0.001f;
	UINT64_FRAME_NEXT_MSEC = gmtimeMillisecond() + UINT_FRAME_INTERVAL_MSEC;
	while (1) {
		scene->simulate(max_wait_sec);
		if (!scene->fetchResults(true)) {
			continue;
		}

		static size_t frame_times;
		++frame_times;
	/*
		// self control
		actor_move(capsule_actor, physx::PxVec3(0.0f, -1.0f, 0.0f), 0.5f);
		do {
			if (!terminalKbhit()) {
				break;
			}
			int key = terminalGetch();
			if (27 == key) { // esc
				goto end;
			}
			else if (32 == key) { // space
				actor_move(capsule_actor, physx::PxVec3(0.0f, 1.0f, 0.0f), 5.0f);
				break;
			}
			else if (224 != key) {
				break;
			}

			if (!terminalKbhit()) {
				break;
			}
			//
			physx::PxShape* obstacle_shape = NULL;
			physx::PxReal x = 0.0f, z = 0.0f;
			switch (terminalGetch()) {
				// right
				case 77:
					//capsule_controller->move(physx::PxVec3(1.0f, 0.0f, 0.0f), 0.05f, 0, physx::PxControllerFilters());
					obstacle_shape = actor_move(capsule_actor, physx::PxVec3(1.0f, 0.0f, 0.0f), 0.5f);
					x = 0.4f;
					break;

				// left
				case 75:
					//capsule_controller->move(physx::PxVec3(-1.0f, 0.0f, 0.0f), 0.05f, 0, physx::PxControllerFilters());
					obstacle_shape = actor_move(capsule_actor, physx::PxVec3(-1.0f, 0.0f, 0.0f), 0.5f);
					x = -0.4f;
					break;

				// up
				case 72:
					//capsule_controller->move(physx::PxVec3(0.0f, 0.0f, -1.0f), 0.05f, 0, physx::PxControllerFilters());
					obstacle_shape = actor_move(capsule_actor, physx::PxVec3(0.0f, 0.0f, -1.0f), 0.5f);
					z = -0.5f;
					break;

				// down
				case 80:
					//capsule_controller->move(physx::PxVec3(0.0f, 0.0f, 1.0f), 0.05f, 0, physx::PxControllerFilters());
					obstacle_shape = actor_move(capsule_actor, physx::PxVec3(0.0f, 0.0f, 1.0f), 0.5f);
					z = 0.5f;
					break;
			}
			//
			if (obstacle_shape) {
				if (1 == obstacle_shape->getSimulationFilterData().word0) {
					physx::PxTransform pos = capsule_actor->getGlobalPose();
					pos.p.x += x;
					pos.p.y += 1.01f;
					pos.p.z += z;
					if (capsule_actor->getRigidDynamicFlags() & physx::PxRigidDynamicFlag::eKINEMATIC)
						capsule_actor->setKinematicTarget(pos);
					else
						capsule_actor->setGlobalPose(pos);
				}
			}
		} while (0);
		*/
		// calculate simulate time
		long long cur_msec = gmtimeMillisecond();
		if (UINT64_FRAME_NEXT_MSEC >= cur_msec) {
			if (UINT64_FRAME_NEXT_MSEC > cur_msec) {
				threadSleepMillsecond(UINT64_FRAME_NEXT_MSEC - cur_msec);
			}
		}
		UINT64_FRAME_NEXT_MSEC += UINT_FRAME_INTERVAL_MSEC;
	}

	// destroy
end:
	if (ctrlmgr) {
		ctrlmgr->release();
	}
	scene->release();
#ifdef PHYSX_ENABLE_PVD
	if (px_debug_connection) {
		px_debug_connection->disconnect();
		px_debug_connection->release();
	}
#endif
	PxCloseExtensions();
	PxGetPhysics().release();
	PxGetFoundation().release();

	// end
	getchar();
	return 0;
}
