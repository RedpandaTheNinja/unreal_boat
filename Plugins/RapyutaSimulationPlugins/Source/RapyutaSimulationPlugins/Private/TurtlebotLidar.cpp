// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
// Modified 2026: synchronous range-only Windows adaptation of RR2DLidarComponent.
// Apache-2.0. See NOTICE for differences from upstream.
#include "TurtlebotBurger.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Dom/JsonObject.h"

URR2DLidarComponent::URR2DLidarComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
}
void URR2DLidarComponent::TickComponent(float Dt, ELevelTick Type, FActorComponentTickFunction* Function)
{
    Super::TickComponent(Dt,Type,Function);
    const double Now=GetWorld()->GetTimeSeconds();
    if (ScanTime>=0 && Now-ScanTime < 1.f/FMath::Clamp(PublicationFrequencyHz,1.f,60.f)) return;
    FCollisionQueryParams Query(SCENE_QUERY_STAT(TurtlebotLidar),true);
    // The full rigid assembly must be excluded, not just this sensor's robot.
    AActor* Assembly=GetOwner()->GetParentActor();
    Query.AddIgnoredActor(GetOwner());
    if (Assembly)
    {
        Query.AddIgnoredActor(Assembly);
        TArray<AActor*> Children;
        Assembly->GetAllChildActors(Children,true);
        Query.AddIgnoredActors(Children);
    }
    ScanOrigin=GetComponentLocation();
    ScanRotation=GetComponentQuat();
    const int32 Count=FMath::Clamp(NSamplesPerScan,4,720);
    const float Near=FMath::Max(0.f,MinRange);
    const float Far=FMath::Max(Near+1.f,MaxRange);
    RangesM.SetNum(Count);
    for(int32 I=0; I<Count; ++I)
    {
        // Public scan angles are counterclockwise, +X forward, +Y left.
        const float Angle=2.f*PI*I/Count;
        const FVector Direction=ScanRotation.RotateVector(FVector(FMath::Cos(Angle),-FMath::Sin(Angle),0));
        FHitResult Hit;
        const bool bHit=GetWorld()->LineTraceSingleByChannel(Hit,ScanOrigin+Near*Direction,
            ScanOrigin+Far*Direction,ECC_Visibility,Query);
        RangesM[I]=bHit ? (Near+Hit.Distance)*0.01f : -1.f;
        if (bShowLidarRays && bHit)
            DrawDebugPoint(GetWorld(),Hit.ImpactPoint,4.f,FColor::Green,false,
                1.f/FMath::Clamp(PublicationFrequencyHz,1.f,60.f),0);
    }
    ScanTime=Now;
    ++ScanSequence;
}
TSharedPtr<FJsonObject> URR2DLidarComponent::ScanJson(const FString& FrameId) const
{
    auto Out=MakeShared<FJsonObject>();
    Out->SetStringField(TEXT("frame_id"),FrameId);
    Out->SetStringField(TEXT("model"),TEXT("ideal_range_only"));
    Out->SetNumberField(TEXT("sequence"),ScanSequence);
    Out->SetNumberField(TEXT("stamp_s"),ScanTime);
    Out->SetBoolField(TEXT("valid"),ScanTime>=0 && RangesM.Num()>0);
    Out->SetNumberField(TEXT("angle_min_rad"),0);
    Out->SetNumberField(TEXT("angle_increment_rad"),RangesM.Num()>0?2.*PI/RangesM.Num():0);
    Out->SetNumberField(TEXT("range_min_m"),FMath::Max(0.f,MinRange)*.01);
    Out->SetNumberField(TEXT("range_max_m"),FMath::Max(FMath::Max(0.f,MinRange)+1.f,MaxRange)*.01);
    Out->SetNumberField(TEXT("time_increment_s"),0); // snapshot, no rolling-scan distortion
    auto Pose=MakeShared<FJsonObject>();
    Pose->SetNumberField(TEXT("x_m"),(ScanOrigin.X-10000.)*.01);
    Pose->SetNumberField(TEXT("y_m"),-ScanOrigin.Y*.01);
    Pose->SetNumberField(TEXT("z_m"),ScanOrigin.Z*.01);
    // Axial-vector handedness conversion for orientation quaternion.
    Pose->SetNumberField(TEXT("qx"),-ScanRotation.X);
    Pose->SetNumberField(TEXT("qy"),ScanRotation.Y);
    Pose->SetNumberField(TEXT("qz"),-ScanRotation.Z);
    Pose->SetNumberField(TEXT("qw"),ScanRotation.W);
    Out->SetObjectField(TEXT("sensor_pose_ground_truth"),Pose);
    TArray<TSharedPtr<FJsonValue>> Values;
    for(float R:RangesM)
    {
        if (R>=0) Values.Add(MakeShared<FJsonValueNumber>(R));
        else Values.Add(MakeShared<FJsonValueNull>());
    }
    Out->SetArrayField(TEXT("ranges_m"),Values);
    return Out;
}
