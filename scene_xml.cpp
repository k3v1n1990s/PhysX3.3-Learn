#include "util/c/syslib/file.h"
#include "util/c/syslib/math.h"
#include "scene_xml.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool SceneXML::loadfile(const char* path) {
	m_root = cXML_ParseFromFile(path);
	return m_root != NULL;
}

void SceneXML::loadscene(physx::PxScene* scene, physx::PxMaterial* material) {
	for (cXML_t* gameobject_node = cXML_FirstChild(m_root, "gameObjects");
		gameobject_node;
		gameobject_node = cXML_NextChild(gameobject_node))
	{
		cXMLAttr_t* name_attr = cXML_GetAttr(gameobject_node, "name");
		if (!name_attr)
			continue;

		cXML_t* transform_node = cXML_FirstChild(gameobject_node, "transform");
		if (!transform_node)
			continue;

		cXML_t* position_node = cXML_FirstChild(transform_node, "position");
		if (!position_node)
			continue;
		cXML_t* position_x_node = cXML_FirstChild(position_node, "x");
		if (!position_x_node)
			continue;
		cXML_t* position_y_node = cXML_FirstChild(position_node, "y");
		if (!position_y_node)
			continue;
		cXML_t* position_z_node = cXML_FirstChild(position_node, "z");
		if (!position_z_node)
			continue;

		cXML_t* rotation_node = cXML_FirstChild(transform_node, "rotation");
		if (!rotation_node)
			continue;
		cXML_t* rotation_x_node = cXML_FirstChild(rotation_node, "x");
		if (!rotation_x_node)
			continue;
		cXML_t* rotation_y_node = cXML_FirstChild(rotation_node, "y");
		if (!rotation_y_node)
			continue;
		cXML_t* rotation_z_node = cXML_FirstChild(rotation_node, "z");
		if (!rotation_z_node)
			continue;

		cXML_t* scale_node = cXML_FirstChild(transform_node, "scale");
		if (!scale_node)
			continue;
		cXML_t* scale_x_node = cXML_FirstChild(scale_node, "x");
		if (!scale_x_node)
			continue;
		cXML_t* scale_y_node = cXML_FirstChild(scale_node, "y");
		if (!scale_y_node)
			continue;
		cXML_t* scale_z_node = cXML_FirstChild(scale_node, "z");
		if (!scale_z_node)
			continue;

		float euler[3] = {
			mathDegToRad(-atof(rotation_x_node->content)),
			mathDegToRad(atof(rotation_y_node->content)),
			mathDegToRad(atof(rotation_z_node->content) + 90.0f)
		};
		float quat[4];
		mathEulerAnglesToQuaternion(euler, quat);
		physx::PxTransform transform(
			physx::PxVec3(
				atof(position_x_node->content),
				atof(position_y_node->content),
				-atof(position_z_node->content)
			),
			physx::PxQuat(quat[0], quat[1], quat[2], quat[3])
		);

		physx::PxRigidActor* actor;
		if (strstr(name_attr->value, "Plane")) {
			physx::PxRigidStatic* plane_actor;
			plane_actor = PxGetPhysics().createRigidStatic(transform);
			plane_actor->createShape(physx::PxPlaneGeometry(), *material);
			///////////////
			actor = plane_actor;
		}
		else if (strstr(name_attr->value, "Cube")) {
			physx::PxRigidDynamic* box_actor;
			box_actor = PxGetPhysics().createRigidDynamic(transform);
			box_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
			physx::PxShape* box_shape;
			physx::PxVec3 scale(
				atof(scale_x_node->content) * 0.5f,
				atof(scale_y_node->content) * 0.5f,
				atof(scale_z_node->content) * 0.5f
			);
			box_shape = box_actor->createShape(physx::PxBoxGeometry(scale), *material);
			///////////////
			actor = box_actor;
		}
		else if (strstr(name_attr->value, "Capsule")) {
			physx::PxRigidDynamic* capsule_actor;
			capsule_actor = PxGetPhysics().createRigidDynamic(transform);
			capsule_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
			physx::PxShape* box_shape;
			physx::PxCapsuleGeometry gem(
				atof(scale_x_node->content) * 0.5f,
				atof(scale_y_node->content) * 0.5f
			);
			box_shape = capsule_actor->createShape(gem, *material);
			//////////////
			actor = capsule_actor;
		}
		else if (strstr(name_attr->value, "Sphere")) {
			physx::PxRigidDynamic* sphere_actor;
			sphere_actor = PxGetPhysics().createRigidDynamic(transform);
			sphere_actor->setRigidDynamicFlag(physx::PxRigidDynamicFlag::eKINEMATIC, true);
			physx::PxShape* sphere_shape;
			physx::PxSphereGeometry gem(atof(scale_x_node->content));
			sphere_shape = sphere_actor->createShape(gem, *material);
			//////////////
			actor = sphere_actor;
		}
		else {
			actor = NULL;
		}

		if (actor) {
			scene->addActor(*actor);
		}
	}
}
