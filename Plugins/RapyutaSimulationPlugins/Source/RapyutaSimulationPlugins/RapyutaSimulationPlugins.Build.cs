using UnrealBuildTool;
public class RapyutaSimulationPlugins : ModuleRules
{
    public RapyutaSimulationPlugins(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "CoreUObject", "Engine", "PhysicsCore", "ChaosVehicles" });
        PrivateDependencyModuleNames.AddRange(new[] { "Sockets", "Networking", "Json", "InputCore", "ImageWrapper", "RenderCore", "RHI", "Chaos", "ChaosVehiclesCore" });
    }
}
