#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TandemTurtleBot.generated.h"
class UChildActorComponent;
class UPhysicsConstraintComponent;
class FSocket;
class ATurtlebotBurger;
class URRTurtlebotVehicleSensors;

UCLASS(Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API ATandemTurtleBot : public AActor
{
    GENERATED_BODY()
public:
    ATandemTurtleBot();
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    UFUNCTION(BlueprintCallable, Category="TurtleBot") void SetVelocitySI(float ForwardMps, float YawRadps);
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UChildActorComponent* Front;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UChildActorComponent* Rear;
    UPROPERTY(Category="TurtleBot", VisibleAnywhere, BlueprintReadOnly) UPhysicsConstraintComponent* RigidLink;
    UPROPERTY(Category="Sensors", VisibleAnywhere, BlueprintReadOnly) URRTurtlebotVehicleSensors* VehicleSensors;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadWrite, meta=(ClampMin="0.20")) float AxleSpacingM = 0.30f;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadWrite) int32 CommandPort = 7447;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadWrite) bool bKeyboardControl = true;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadWrite) float ManualSpeedMps = 0.15f;
    UPROPERTY(Category="TurtleBot", EditAnywhere, BlueprintReadWrite) float ManualYawRadps = 0.5f;
private:
    FSocket* Socket = nullptr;
    float LastCommandTime = -100.f;
    float TargetV = 0.f;
    float TargetW = 0.f;
    ATurtlebotBurger* FrontBot() const;
    ATurtlebotBurger* RearBot() const;
};
