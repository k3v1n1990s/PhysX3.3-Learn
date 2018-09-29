#include "util/c/syslib/math.h"
#include "convex_mesh.h"
#include <stdlib.h>

PxCylinderMesh* PxCylinderMesh::createCylinderMesh(PxCylinderMesh* cylinder, physx::PxCooking* cooking, physx::PxReal radius, physx::PxReal height) {
	cylinder->meshcnt = cylinder->sidecnt * 4;
	cylinder->mesh = (physx::PxTriangleMesh**)calloc(1, sizeof(physx::PxTriangleMesh*) * cylinder->meshcnt);
	if (!cylinder->mesh)
		return NULL;

	physx::PxU32 side_i, mesh_i = 0;
	physx::PxReal rad, step = physx::PxTwoPi / (physx::PxReal)(cylinder->sidecnt);
	for (side_i = 0, rad = 0.0f; side_i < cylinder->sidecnt; ++side_i, rad += step) {
		physx::PxReal x0 = radius * physx::PxCos(rad);
		physx::PxReal z0 = radius * physx::PxSin(rad);
		physx::PxReal x1 = radius * physx::PxCos(rad + step);
		physx::PxReal z1 = radius * physx::PxSin(rad + step);

		physx::PxTriangleMesh* top_triangle_mesh = NULL;
		do {
			physx::PxVec3 triangle_verts[] = {
				physx::PxVec3(0, height, 0),
				physx::PxVec3(x1, height, z1),
				physx::PxVec3(x0, height, z0),
			};
			physx::PxU32 triangle_indices[] = { 0, 1, 2 };
			physx::PxTriangleMeshDesc triangle_desc;
			triangle_desc.points.count = sizeof(triangle_verts) / sizeof(triangle_verts[0]);
			triangle_desc.points.stride = sizeof(triangle_verts[0]);
			triangle_desc.points.data = triangle_verts;
			triangle_desc.triangles.count = sizeof(triangle_indices) / (sizeof(triangle_indices[0]) * 3);
			triangle_desc.triangles.stride = sizeof(triangle_indices[0]) * 3;
			triangle_desc.triangles.data = triangle_indices;
			physx::PxDefaultMemoryOutputStream outbuf;
			if (!cooking->cookTriangleMesh(triangle_desc, outbuf))
				break;
			physx::PxDefaultMemoryInputData inputdata(outbuf.getData(), outbuf.getSize());
			top_triangle_mesh = PxGetPhysics().createTriangleMesh(inputdata);
		} while (0);
		if (!top_triangle_mesh)
			break;
		cylinder->mesh[mesh_i++] = top_triangle_mesh;

		physx::PxTriangleMesh* bottom_triangle_mesh = NULL;
		do {
			physx::PxVec3 triangle_verts[] = {
				physx::PxVec3(x0, -height, z0),
				physx::PxVec3(x1, -height, z1),
				physx::PxVec3(0, -height, 0)
			};
			physx::PxU32 triangle_indices[] = { 0, 1, 2 };
			physx::PxTriangleMeshDesc triangle_desc;
			triangle_desc.points.count = sizeof(triangle_verts) / sizeof(triangle_verts[0]);
			triangle_desc.points.stride = sizeof(triangle_verts[0]);
			triangle_desc.points.data = triangle_verts;
			triangle_desc.triangles.count = sizeof(triangle_indices) / (sizeof(triangle_indices[0]) * 3);
			triangle_desc.triangles.stride = sizeof(triangle_indices[0]) * 3;
			triangle_desc.triangles.data = triangle_indices;
			physx::PxDefaultMemoryOutputStream outbuf;
			if (!cooking->cookTriangleMesh(triangle_desc, outbuf))
				break;
			physx::PxDefaultMemoryInputData inputdata(outbuf.getData(), outbuf.getSize());
			bottom_triangle_mesh = PxGetPhysics().createTriangleMesh(inputdata);
		} while (0);
		if (!bottom_triangle_mesh)
			break;
		cylinder->mesh[mesh_i++] = bottom_triangle_mesh;

		physx::PxTriangleMesh* side0_triangle_mesh = NULL;
		do {
			physx::PxVec3 triangle_verts[] = {
				physx::PxVec3(x1, -height, z1),
				physx::PxVec3(x0, -height, z0),
				physx::PxVec3(x0, height, z0)
			};
			physx::PxU32 triangle_indices[] = { 0, 1, 2 };
			physx::PxTriangleMeshDesc triangle_desc;
			triangle_desc.points.count = sizeof(triangle_verts) / sizeof(triangle_verts[0]);
			triangle_desc.points.stride = sizeof(triangle_verts[0]);
			triangle_desc.points.data = triangle_verts;
			triangle_desc.triangles.count = sizeof(triangle_indices) / (sizeof(triangle_indices[0]) * 3);
			triangle_desc.triangles.stride = sizeof(triangle_indices[0]) * 3;
			triangle_desc.triangles.data = triangle_indices;
			physx::PxDefaultMemoryOutputStream outbuf;
			if (!cooking->cookTriangleMesh(triangle_desc, outbuf))
				break;
			physx::PxDefaultMemoryInputData inputdata(outbuf.getData(), outbuf.getSize());
			side0_triangle_mesh = PxGetPhysics().createTriangleMesh(inputdata);
		} while (0);
		if (!side0_triangle_mesh)
			break;
		cylinder->mesh[mesh_i++] = side0_triangle_mesh;

		physx::PxTriangleMesh* side1_triangle_mesh = NULL;
		do {
			physx::PxVec3 triangle_verts[] = {
				physx::PxVec3(x0, height, z0),
				physx::PxVec3(x1, height, z1),
				physx::PxVec3(x1, -height, z1)
			};
			physx::PxU32 triangle_indices[] = { 0, 1, 2 };
			physx::PxTriangleMeshDesc triangle_desc;
			triangle_desc.points.count = sizeof(triangle_verts) / sizeof(triangle_verts[0]);
			triangle_desc.points.stride = sizeof(triangle_verts[0]);
			triangle_desc.points.data = triangle_verts;
			triangle_desc.triangles.count = sizeof(triangle_indices) / (sizeof(triangle_indices[0]) * 3);
			triangle_desc.triangles.stride = sizeof(triangle_indices[0]) * 3;
			triangle_desc.triangles.data = triangle_indices;
			physx::PxDefaultMemoryOutputStream outbuf;
			if (!cooking->cookTriangleMesh(triangle_desc, outbuf))
				break;
			physx::PxDefaultMemoryInputData inputdata(outbuf.getData(), outbuf.getSize());
			side1_triangle_mesh = PxGetPhysics().createTriangleMesh(inputdata);
		} while (0);
		if (!side1_triangle_mesh)
			break;
		cylinder->mesh[mesh_i++] = side1_triangle_mesh;
	}
	if (side_i != cylinder->sidecnt) {
		return NULL;
	}
	return cylinder;
}
