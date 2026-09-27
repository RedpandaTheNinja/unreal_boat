#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "ChaosVehicleWheel.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "MiniF1Car.generated.h"
class UStaticMeshComponent;
class UChaosWheeledVehicleMovementComponent;
class URR2DLidarComponent;
class URRTurtlebotVehicleSensors;
class USpringArmComponent;
class UCameraComponent;
class FSocket;
class FJsonObject;

// Per-vehicle force suspension for a miniature static-mesh chassis; no global
// Chaos cvar changes and no skeletal suspension-constraint dependency.
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API UMiniF1Movement : public UChaosWheeledVehicleMovementComponent
{
    GENERATED_BODY()
public:
    virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() override;
protected:
    virtual void FixupSkeletalMesh() override;
};

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API UMiniF1FrontWheel : public UChaosVehicleWheel
{
    GENERATED_BODY()
public: UMiniF1FrontWheel();
};
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API UMiniF1RearWheel : public UMiniF1FrontWheel
{
    GENERATED_BODY()
public: UMiniF1RearWheel();
};

UCLASS(Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API AMiniF1Car : public APawn
{
    GENERATED_BODY()
public:
    AMiniF1Car();
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="F1") UStaticMeshComponent* Chassis;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="F1") UChaosWheeledVehicleMovementComponent* Vehicle;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="F1") URRTurtlebotVehicleSensors* VehicleSensors;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="F1") URR2DLidarComponent* FrontLidar;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="F1") URR2DLidarComponent* RearLidar;
    UPROPERTY(VisibleAnywhere, Category="F1") USpringArmComponent* ChaseArm;
    UPROPERTY(VisibleAnywhere, Category="F1") UCameraComponent* ChaseCamera;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="F1") int32 CommandPort=7449;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="F1") bool bKeyboardControl=true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="F1") float ManualSpeedMps=.6f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="F1", meta=(ClampMin="0.01", ClampMax="138.5824")) float MaxSpeedMps=138.5824f;
    UPROPERTY(VisibleAnywhere, Category="F1") TArray<UStaticMeshComponent*> WheelVisuals;
    UPROPERTY(VisibleAnywhere, Category="F1") TArray<UStaticMeshComponent*> Links;
private:
    FSocket* Socket=nullptr;
    double LastCommand=-100;
    float SpeedTarget=0, SteeringTarget=0, ThrottleTarget=0, BrakeTarget=1;
    bool bDirectInputs=false;
    bool bSetView=false;
    float FrontGripMultiplier=1.f, RearGripMultiplier=1.f;
    FTransform EpisodeStart;
    int32 EpisodeId=0;
    FString LastResetToken;
    TArray<FVector> BestRacingLine;
    UStaticMeshComponent* AddPart(const TCHAR* Name,const TCHAR* Asset,USceneComponent* Parent);
    void AnimateWheels();
    void PollCommands();
    TSharedPtr<FJsonObject> Telemetry(bool Sensors) const;
};
