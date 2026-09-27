using UnrealBuildTool;
public class BoatPhysics : ModuleRules {
 public BoatPhysics(ReadOnlyTargetRules Target) : base(Target) {
  PCHUsage=PCHUsageMode.UseExplicitOrSharedPCHs;
  PublicDependencyModuleNames.AddRange(new[]{"Core","CoreUObject","Engine","PhysicsCore"});
  // BoatLab v2 adds ImageWrapper/RenderCore/RHI for the camera (same set the F1 car plugin compiles with).
  PrivateDependencyModuleNames.AddRange(new[]{"Chaos","InputCore","Sockets","Networking","Json","ImageWrapper","RenderCore","RHI"});
 }
}
