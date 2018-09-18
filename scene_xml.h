#ifndef SCENE_XML_H
#define	SCENE_XML_H

#include "util/c/component/cXML.h"
#include "PhysX-3.3-mini/Include/PxPhysicsAPI.h"

class SceneXML {
public:
	SceneXML(void) : m_root(NULL) {}
	~SceneXML(void) { cXML_Delete(m_root); }

	bool loadfile(const char* path);
	void loadscene(physx::PxScene* scene, physx::PxMaterial* material);

private:
	struct cXML_t* m_root;
};

#endif // !SCENE_XML_IMPORT_H
