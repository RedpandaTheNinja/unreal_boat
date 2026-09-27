// FableCarPawn.h - Ackermann car on Chaos Vehicles, parameterised from vehicle_params.json.
//
// Setup (once, in the editor): make BP_FableCar from this class, assign a skeletal
// mesh with four wheel bones (Epic's free Vehicle Template "SportsCar" works, scaled
// down; any RC-car mesh with wheel bones works), set WheelBoneNames FL FR RL RR.
// Everything else - mass, wheel radius, steer angle, speed, torque - is written by
// FableApplyParams so the Blueprint never needs editing again. See docs/UNREAL_SETUP.md.
//
// Longitudinal control is a governor: the Python command is throttle in [-1, 1]
// meaning "fraction of max speed"; the pawn computes the acceleration the mock
// model would produce ((v_ref - v) / tau, clamped) and asks Chaos for the wheel
// force that delivers it. So the calibrated parameters mean the same thing in
// Unreal and in the pure-Python model, which is the whole point of having both.
#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "ChaosVehicleWheel.h"
#include "FableTypes.h"
#include "FableCarPawn.generated.h"

UCLASS()
class FABLEBRIDGE_API UFableFrontWheel : public UChaosVehicleWheel
{
	GENERATED_BODY()
public:
	UFableFrontWheel();
};

UCLASS()
class FABLEBRIDGE_API UFableRearWheel : public UChaosVehicleWheel
{
	GENERATED_BODY()
public:
	UFableRearWheel();
};

UCLASS()
class FABLEBRIDGE_API AFableCarPawn : public AWheeledVehiclePawn, public IFableVehicle
{
	GENERATED_BODY()

public:
	AFableCarPawn();

	// Bone names on the skeletal mesh: FL, FR, RL, RR. Leave a name empty to place
	// that wheel by offset from WheelbaseM/TrackWidthM instead of a bone.
	UPROPERTY(EditAnywhere, Category = "Fable") TArray<FName> WheelBoneNames = { TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR") };

	UPROPERTY(EditAnywhere, Category = "Fable") FFableVehicleParams Params;

	// --- IFableVehicle
	virtual void FableApplyCommand(const TMap<FString, double>& U) override;
	virtual void FableApplyParams(const FFableVehicleParams& P) override;
	virtual void FableResetTo(double x_m, double y_m, double yaw_rad) override;
	virtual void FableSetEnv(const FFableEnv* InEnv) override { Env = InEnv; }
	virtual void FableGetState(FFableState& Out, float Dt) override;
	virtual FString FableVehicleType() const override { return TEXT("car"); }

	virtual void Tick(float DeltaSeconds) override;
	virtual void BeginPlay() override;
	virtual void NotifyHit(UPrimitiveComponent* MyComp, AActor* Other, UPrimitiveComponent* OtherComp, bool bSelfMoved,
	                       FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

private:
	const FFableEnv* Env = nullptr;

	// command pipeline: delay line -> servo model -> governor -> Chaos inputs
	struct FCmd { double Throttle = 0, Steer = 0, Brake = 0; };
	TArray<FCmd> DelayLine;
	int32 DelayHead = 0;
	FCmd Applied;
	float SteerAngle = 0.f;                 // rad, after slew limit
	FVector PrevVelWorld = FVector::ZeroVector;

	TArray<FString> Contacts;
	bool bHitThisTick = false;

	void ConfigureChaos();
	FCmd PushDelay(const FCmd& In);
	class UChaosWheeledVehicleMovementComponent* Chaos() const;
};
