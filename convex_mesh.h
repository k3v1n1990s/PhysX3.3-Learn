#ifndef PHYSX_CONVEX_MESH_H
#define	PHYSX_CONVEX_MESH_H

#include "PhysX-3.3-mini/Include/PxPhysicsAPI.h"
#include <stdlib.h>

class PxCylinderMesh {
public:
	static PxCylinderMesh* createCylinderMesh(PxCylinderMesh* cylinder, physx::PxCooking* cooking, physx::PxReal radius, physx::PxReal height);

public:
/* IN */
	physx::PxU32 sidecnt;
/* OUT */
	physx::PxU32 meshcnt;
	physx::PxTriangleMesh** mesh;
	/*
	struct Mesh {
		physx::PxTriangleMesh* side[2];
		physx::PxTriangleMesh* top;
		physx::PxTriangleMesh* bottom;
	} *mesh;
	*/

	PxCylinderMesh(physx::PxU32 sidecnt = 20) :
		sidecnt(sidecnt),
		mesh(NULL)
	{}
	~PxCylinderMesh(void) {
		free(mesh);
	}

	PxCylinderMesh* create(physx::PxCooking* cooking, physx::PxReal radius, physx::PxReal height) {
		return createCylinderMesh(this, cooking, radius, height);
	}
};



#endif // !PHYSX_CONVEX_MESH_H
