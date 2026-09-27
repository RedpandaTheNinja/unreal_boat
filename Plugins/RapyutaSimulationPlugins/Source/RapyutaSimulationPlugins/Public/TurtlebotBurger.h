// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
// Modified 2026: physics-only Windows port. Licensed under Apache-2.0.
#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/SceneComponent.h"
#include "TurtlebotBurger.generated.h"
class UStaticMeshComponent;
class UMeshComponent;
class UPhysicsConstraintComponent;

// Windows range-only adaptation of upstream RR2DLidarComponent ray geometry.
// Noise/intensity/ROS publishing are excluded; JSON labels these ideal scans.
UCLASS(ClassGroup=Simulation, meta=(BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URR2DLidarComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    URR2DLidarComponent();
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    UPROPERTY(Category="Lidar", EditAnywhere, BlueprintReadWrite) int32 NSamplesPerScan = 360;
    UPROPERTY(Category="Lidar", EditAnywhere, BlueprintReadWrite) float MinRange = 12.f;
    UPROPERTY(Category="Lidar", EditAnywhere, BlueprintReadWrite) float MaxRange = 350.f;
    UPROPERTY(Category="Lidar", EditAnywhere, BlueprintReadWrite) bool bShowLidarRays = true;
    UPROPERTY(Category="Lidar", EditAnywhere, BlueprintReadWrite) float PublicationFrequencyHz = 30.f;
    TSharedPtr<class FJsonObject> ScanJson(const FString& FrameId) const;
private:
    TArray<float> RangesM;
    double ScanTime = -1;
    FVector ScanOrigin = FVector::ZeroVector;
    FQuat ScanRotation = FQuat::Identity;
    uint32 ScanSequence = 0;
};

UCLASS(Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurgerBase : public APawn
{
    GENERATED_BODY()
public:
    ATurtlebotBurgerBase();
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UStaticMeshComponent* Base;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UStaticMeshComponent* LidarSensor;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UStaticMeshComponent* WheelLeft;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UStaticMeshComponent* WheelRight;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UStaticMeshComponent* CasterBack;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) URR2DLidarComponent* LidarComponent;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere) UPhysicsConstraintComponent* Base_LidarSensor;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere) UPhysicsConstraintComponent* Base_CasterBack;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere) USceneComponent* DefaultRoot;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere) UMeshComponent* BaseMeshComp;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadWrite) FString RobotModelName;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadOnly) float MaxForce = 1000.f;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadOnly) float WheelRadius = 3.3f;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadOnly) float WheelSeparationHalf = 7.9f;
};

UCLASS(Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurger : public ATurtlebotBurgerBase
{
    GENERATED_BODY()
public:
    ATurtlebotBurger();
    virtual void PostInitializeComponents() override;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UPhysicsConstraintComponent* Base_WheelLeft;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UPhysicsConstraintComponent* Base_WheelRight;
    // SI convention: positive yaw is counterclockwise in the external XY plane.
    UFUNCTION(BlueprintCallable, Category="TurtleBot") void SetVelocitySI(float ForwardMps, float YawRadps);
};
