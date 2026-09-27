#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "TurtlebotVehicleSensors.generated.h"
class UPrimitiveComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class FJsonObject;

// Ideal Windows sensors; no ROS dependency and no additional physics bodies.
UCLASS(ClassGroup=Simulation, meta=(BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRTurtlebotVehicleSensors : public USceneComponent
{
    GENERATED_BODY()
public:
    URRTurtlebotVehicleSensors();
    virtual void BeginPlay() override;
    virtual void TickComponent(float Dt, ELevelTick Type, FActorComponentTickFunction* Function) override;
    UPROPERTY(EditAnywhere, Category="Sensors") int32 CameraWidth = 640;
    UPROPERTY(EditAnywhere, Category="Sensors") int32 CameraHeight = 480;
    UPROPERTY(EditAnywhere, Category="Sensors") float CameraHorizontalFovDeg = 90.f;
    UPROPERTY(EditAnywhere, Category="Sensors") FVector CameraOffsetCm = FVector(8,0,22);
    UPROPERTY(EditAnywhere, Category="Sensors") float CameraPitchDeg = -12.f;
    UPROPERTY(EditAnywhere, Category="Sensors") float CameraMaxHz = 10.f;
    UPROPERTY(EditAnywhere, Category="Sensors") float ImuMaxHz = 100.f;
    void SetBody(UPrimitiveComponent* InBody);
    TSharedPtr<FJsonObject> ImuJson() const;
    TSharedPtr<FJsonObject> CameraJson() const;
    bool CaptureRgb();
    TSharedPtr<FJsonObject> CameraChunk(double Sequence, int32 Index) const;
private:
    UPROPERTY(Transient) TObjectPtr<UPrimitiveComponent> Body;
    UPROPERTY(Transient) TObjectPtr<USceneCaptureComponent2D> Capture;
    UPROPERTY(Transient) TObjectPtr<UTextureRenderTarget2D> Target;
    FVector LastVelocity = FVector::ZeroVector;
    FVector SpecificForce = FVector::ZeroVector;
    FVector Gyro = FVector::ZeroVector;
    FQuat ImuOrientation = FQuat::Identity;
    FQuat PreviousOrientation = FQuat::Identity;
    FTransform CameraPose;
    double ImuStamp = -1, PreviousStamp = -1, CameraStamp = -1;
    double ImuDt = 0;
    uint32 ImuSequence = 0, CameraSequence = 0;
    TArray64<uint8> Jpeg;
    static constexpr int32 ChunkBytes = 24000;
};
