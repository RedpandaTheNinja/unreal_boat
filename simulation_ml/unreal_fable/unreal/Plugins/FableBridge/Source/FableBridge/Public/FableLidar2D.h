// FableLidar2D.h - planar line-trace lidar. Cheap (540 traces at 10 Hz is nothing)
// and returns ranges in metres in the ROS convention (angle 0 = forward, +CCW).
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "FableLidar2D.generated.h"

UCLASS(ClassGroup = (Fable), meta = (BlueprintSpawnableComponent))
class FABLEBRIDGE_API UFableLidar2D : public USceneComponent
{
	GENERATED_BODY()

public:
	UFableLidar2D();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") int32 NumRays = 360;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") float FovRad = 6.283f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") float RangeMaxM = 20.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") float NoiseStdM = 0.02f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") float RateHz = 10.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") bool bDrawDebug = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fable") TEnumAsByte<ECollisionChannel> Channel = ECC_Visibility;

	// Runs a scan if it is time (SimTime advances with the fixed tick). Returns true if updated.
	bool Update(double SimTime);

	// Last scan, metres, RangeMaxM where nothing was hit. Index 0 = AngleMin.
	const TArray<float>& Ranges() const { return LastRanges; }
	float AngleMin() const { return -FovRad * 0.5f; }
	float AngleMax() const { return FovRad * 0.5f; }

private:
	TArray<float> LastRanges;
	double LastScanTime = -1e9;
	FRandomStream Rng;
};
