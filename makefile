SOURCE_FILE += $(shell find util/c/ -name "*.c" && find -maxdepth 1 -name "*.cpp")

TARGET := run.exe
#COMPILE_OPTION := -std=c++11
DEFAULT_LINK := -lpthread -lm -lrt -ldl -lcrypto -luuid
PHYSX_INCLUDE_PATH := -I ./PhysX-3.3-mini/Include

PHYSX_STATIC_LINK_PATH := -L ./PhysX-3.3-mini/Lib/linux64
PHYSX_DYNAMIC_LINK_PATH := -Wl,-rpath ./PhysX-3.3-mini/Bin/linux64 -L ./PhysX-3.3-mini/Bin/linux64
PHYSX_DYNAMIC_DEBUG_LINK := -lPhysX3DEBUG_x64 -lPhysX3CommonDEBUG_x64 -lPhysX3CharacterKinematicDEBUG_x64
PHYSX_STATIC_DEBUG_LINK := -lPhysX3ExtensionsDEBUG -lPhysXVisualDebuggerSDKDEBUG
PHYSX_DYNAMIC_RELEASE_LINK := -lPhysX3_x64 -lPhysX3Common_x64 -lPhysX3CharacterKinematic_x64
PHYSX_STATIC_RELEASE_LINK := -lPhysX3Extensions -lPhysXVisualDebuggerSDK

debug:
	-rm $(TARGET)
	-ctags -R *
	g++ $(COMPILE_OPTION) -g -D_DEBUG $(PHYSX_INCLUDE_PATH) $(SOURCE_FILE) -o $(TARGET) $(DEFAULT_LINK) $(PHYSX_DYNAMIC_LINK_PATH) $(PHYSX_DYNAMIC_DEBUG_LINK) $(PHYSX_STATIC_LINK_PATH) $(PHYSX_STATIC_DEBUG_LINK)
release:
	-rm $(TARGET)
	-ctags -R *
	g++ $(COMPILE_OPTION) -DNDEBUG $(PHYSX_INCLUDE_PATH) $(SOURCE_FILE) -o $(TARGET) $(DEFAULT_LINK) $(PHYSX_DYNAMIC_LINK_PATH) $(PHYSX_DYNAMIC_RELEASE_LINK) $(PHYSX_STATIC_LINK_PATH) $(PHYSX_STATIC_RELEASE_LINK)

