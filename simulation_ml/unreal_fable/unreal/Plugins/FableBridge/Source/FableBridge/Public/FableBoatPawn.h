// FableBoatPawn.h - twin-thruster surface vessel with its own hydrodynamics.
//
// Unreal has no boat physics worth calibrating, so this pawn applies the same
// force model as the Python mock (Fossen 3-DOF: thrust, linear+quadratic drag in
// surge/sway/yaw, wind, current) onto a rigid body, plus a point-buoyancy model for
// heave/roll/pitch so the hull sits on the water and rocks in waves. Every
// coefficient comes from vehicle_params.json through FableApplyParams.
//
// Added mass is not modelled here (a rigid body cannot have direction-dependent
// inertia); it IS in the mock. Expect the Unreal boat to accelerate slightly
// faster in sway - fit both to your real logs and the fitter will tell you which
// is closer.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "FableTypes.h"
#include "FableBoatPawn.generated.h"

class UStaticMeshComponent;

UCLASS()
class FABLEBRIDGE_API AFableBoatPawn : public APawn, public IFableVehicle
{
	GENERATED_BODY()

public:
	AFableBoatPawn();

	UPROPERTY(VisibleAnywhere, Category = "Fable") UStaticMeshComponent* Hull = nullptr;
	UPROPERTY(EditAnywhere, Category = "Fable") FFableVehicleParams Params;
	UPROPERTY(EditAnywhere, Category = "Fable") bool bDrawDebugForces = false;

	// --- IFableVehicle
	virtual void FableApplyCommand(const TMap<FString, double>& U) override;
	virtual void FableApplyParams(const FFableVehicleParams& P) override;
	virtual void FableResetTo(double x_m, double y_m, double yaw_rad) override;
	virtual void FableSetEnv(const FFableEnv* InEnv) override { Env = InEnv; }
	virtual void FableGetState(FFableState& Out, float Dt) override;
	virtual FString FableVehicleType() const override { return TEXT("boat"); }

	virtual void Tick(float DeltaSeconds) override;
	virtual void BeginPlay() override;
	virtual void NotifyHit(UPrimitiveComponent* MyComp, AActor* Other, UPrimitiveComponent* OtherComp, bool bSelfMoved,
	                       FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

private:
	const FFableEnv* Env = nullptr;
	double SimT = 0.0;

	struct FCmd { double L = 0, R = 0; };
	TArray<FCmd> DelayLine;
	int32 DelayHead = 0;
	FCmd Applied;
	float ThrustL = 0.f, ThrustR = 0.f;     // N, after first-order lag
	FVector PrevVelWorld = FVector::ZeroVector;
	FVector AccBodyRos = FVector::ZeroVector;

	TArray<FString> Contacts;
	bool bHitThisTick = false;

	FCmd PushDelay(const FCmd& In);
	float ThrustFor(double cmd) const;
	void ApplyHydro(float Dt);
};
