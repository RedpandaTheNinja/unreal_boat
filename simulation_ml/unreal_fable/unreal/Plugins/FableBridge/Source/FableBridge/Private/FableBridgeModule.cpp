#include "Modules/ModuleManager.h"

class FFableBridgeModule : public IModuleInterface
{
public:
	virtual void StartupModule() override {}
	virtual void ShutdownModule() override {}
};

IMPLEMENT_MODULE(FFableBridgeModule, FableBridge)
