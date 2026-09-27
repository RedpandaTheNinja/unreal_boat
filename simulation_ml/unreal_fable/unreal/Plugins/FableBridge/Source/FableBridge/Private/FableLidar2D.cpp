#include "FableLidar2D.h"
#include "FableConv.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

UFableLidar2D::UFableLidar2D()
{
	PrimaryComponentTick.bCanEverTick = false;
	Rng.Initialize(1234);
}

bool UFableLidar2D::Update(double SimTime)
{
	if (RateHz > 0.f && SimTime - LastScanTime < 1.0 / RateHz - 1e-6 && LastRanges.Num() == NumRays)
		return false;
	LastScanTime = SimTime;
	LastRanges.SetNumUninitialized(NumRays);

	UWorld* World = GetWorld();
	if (!World) return false;

	const FVector Origin = GetComponentLocation();
	const FRotator Rot = GetComponentRotation();
	FCollisionQueryParams Params(SCENE_QUERY_STAT(FableLidar), false, GetOwner());
	Params.bTraceComplex = false;

	const float RangeCm = RangeMaxM * Fable::M2CM;
	for (int32 i = 0; i < NumRays; ++i)
	{
		// ROS angle (CCW positive) -> UE yaw offset (CW positive)
		const float aRos = AngleMin() + (NumRays > 1 ? FovRad * i / (NumRays - 1) : 0.f);
		const float yawDeg = Rot.Yaw - FMath::RadiansToDegrees(aRos);
		const FVector Dir = FRotator(0.f, yawDeg, 0.f).Vector();
		const FVector End = Origin + Dir * RangeCm;

		FHitResult Hit;
		float r = RangeMaxM;
		if (World->LineTraceSingleByChannel(Hit, Origin, End, Channel, Params))
		{
			r = Hit.Distance * Fable::CM2M;
			if (NoiseStdM > 0.f)
				r = FMath::Clamp(r + Rng.FRandRange(-1.f, 1.f) * NoiseStdM * 1.7f, 0.02f, RangeMaxM);
		}
		LastRanges[i] = r;
		if (bDrawDebug)
			DrawDebugLine(World, Origin, Origin + Dir * r * Fable::M2CM, r < RangeMaxM ? FColor::Red : FColor(60, 60, 60), false, 1.f / FMath::Max(RateHz, 1.f));
	}
	return true;
}
