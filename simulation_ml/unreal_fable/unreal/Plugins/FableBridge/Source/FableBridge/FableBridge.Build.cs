using UnrealBuildTool;

public class FableBridge : ModuleRules
{
    public FableBridge(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core", "CoreUObject", "Engine", "InputCore",
            "Sockets", "Networking",
            "Json", "JsonUtilities",
            "PhysicsCore",
            "ChaosVehicles", "ChaosVehiclesCore"
        });

        PrivateDependencyModuleNames.AddRange(new string[] { });
    }
}
