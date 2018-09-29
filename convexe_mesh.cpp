#include "util/c/syslib/math.h"
#include "convex_mesh.h"
#include <stdio.h>
#include <stdlib.h>

/*
void debug(physx::PxVec3* triangle_verts, physx::PxU32 triangle_verts_cnt, physx::PxU32* triangle_indices, physx::PxU32 triangle_indices_cnt) {
	physx::PxU32 i;
	puts("verts:");
	for (i = 0; i < triangle_verts_cnt; ++i) {
		printf("%f, %f, %f\n", triangle_verts[i].x, triangle_verts[i].y, triangle_verts[i].z);
	}
	puts("indices:");
	for (i = 0; i < triangle_indices_cnt * 3; i += 3) {
		printf("%d, %d, %d\n", triangle_indices[i], triangle_indices[i+1], triangle_indices[i+2]);
	}
}
*/

physx::PxTriangleMesh* createCylinderMesh(physx::PxCooking* cooking, physx::PxReal radius, physx::PxReal height, physx::PxU32 sidecnt) {
	physx::PxU32 triangle_verts_cnt = sidecnt * 2 + 2;
	physx::PxU32 triangle_indices_cnt = sidecnt * 4;
	physx::PxU32 memsize = sizeof(physx::PxVec3) * triangle_verts_cnt + sizeof(physx::PxU32) * 3 * triangle_indices_cnt;
	char* membuf = (char*)malloc(memsize);
	if (!membuf)
		return NULL;

	physx::PxVec3* triangle_verts = (physx::PxVec3*)membuf;
	physx::PxU32* triangle_indices = (physx::PxU32*)(membuf + sizeof(physx::PxVec3) * triangle_verts_cnt);

	physx::PxReal rad = 0.0f, step = physx::PxTwoPi / (physx::PxReal)(sidecnt);
	for (physx::PxU32 i = 0; i < sidecnt; ++i, rad += step) {
		physx::PxReal x = radius * physx::PxCos(rad);
		physx::PxReal z = radius * physx::PxSin(rad);
		triangle_verts[i * 2] = physx::PxVec3(x, height, z);
		triangle_verts[i * 2 + 1] = physx::PxVec3(x, -height, z);
	}
	triangle_verts[sidecnt * 2] = physx::PxVec3(0.0f, height, 0.0f);
	triangle_verts[sidecnt * 2 + 1] = physx::PxVec3(0.0f, -height, 0.0f);

	physx::PxU32 ind_i = 0;
	for (physx::PxU32 i = 0; i < sidecnt; ++i) {
		physx::PxU32 vert_i = i * 2;
		if (i + 1 == sidecnt) {
			triangle_indices[ind_i++] = 0;
			triangle_indices[ind_i++] = vert_i;
			triangle_indices[ind_i++] = triangle_verts_cnt - 2;

			triangle_indices[ind_i++] = triangle_verts_cnt - 1;
			triangle_indices[ind_i++] = vert_i + 1;
			triangle_indices[ind_i++] = 1;

			triangle_indices[ind_i++] = 1;
			triangle_indices[ind_i++] = vert_i + 1;
			triangle_indices[ind_i++] = vert_i;

			triangle_indices[ind_i++] = 0;
			triangle_indices[ind_i++] = 1;
			triangle_indices[ind_i++] = vert_i;
		}
		else {
			triangle_indices[ind_i++] = vert_i + 2;
			triangle_indices[ind_i++] = vert_i;
			triangle_indices[ind_i++] = triangle_verts_cnt - 2;

			triangle_indices[ind_i++] = triangle_verts_cnt - 1;
			triangle_indices[ind_i++] = vert_i + 1;
			triangle_indices[ind_i++] = vert_i + 3;

			triangle_indices[ind_i++] = vert_i + 3;
			triangle_indices[ind_i++] = vert_i + 1;
			triangle_indices[ind_i++] = vert_i;

			triangle_indices[ind_i++] = vert_i + 2;
			triangle_indices[ind_i++] = vert_i + 3;
			triangle_indices[ind_i++] = vert_i;
		}
	}

	physx::PxTriangleMeshDesc triangle_desc;
	triangle_desc.points.count = triangle_verts_cnt;
	triangle_desc.points.stride = sizeof(*triangle_verts);
	triangle_desc.points.data = triangle_verts;
	triangle_desc.triangles.count = triangle_indices_cnt;
	triangle_desc.triangles.stride = sizeof(*triangle_indices) * 3;
	triangle_desc.triangles.data = triangle_indices;

	physx::PxTriangleMesh* mesh;
	physx::PxDefaultMemoryOutputStream outbuf;
	if (cooking->cookTriangleMesh(triangle_desc, outbuf)) {
		physx::PxDefaultMemoryInputData inputdata(outbuf.getData(), outbuf.getSize());
		mesh = PxGetPhysics().createTriangleMesh(inputdata);
	}
	else {
		mesh = NULL;
	}
	free(membuf);
	return mesh;
}
