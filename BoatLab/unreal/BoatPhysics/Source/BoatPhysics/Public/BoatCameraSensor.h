#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "BoatCameraSensor.generated.h"
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class FJsonObject;

// BoatLab forward camera: RGB (JPEG) + ideal metric depth (16-bit PNG, millimetres along the
// optical axis) captured on request and served in 24 kB chunks over the BoatDynamics UDP port.
// Pattern proven in RRTurtlebotVehicleSensors (same engine build). Ideal sensor: no noise,
// no rolling shutter, no ZED stereo artefacts - label results accordingly.
UCLASS(ClassGroup=(Boat), meta=(BlueprintSpawnableComponent))
class BOATPHYSICS_API UBoatCameraSensor : public USceneComponent {
 GENERATED_BODY()
public:
 UBoatCameraSensor();
 UPROPERTY(EditAnywhere,Category="Camera") int32 Width=640;
 UPROPERTY(EditAnywhere,Category="Camera") int32 Height=480;
 UPROPERTY(EditAnywhere,Category="Camera") float HorizontalFovDeg=90.f;
 UPROPERTY(EditAnywhere,Category="Camera") float PitchDownDeg=8.f;
 UPROPERTY(EditAnywhere,Category="Camera") float MaxHz=10.f;
 UPROPERTY(EditAnywhere,Category="Camera") bool bDepth=true;
 UPROPERTY(EditAnywhere,Category="Camera") int32 DepthWidth=320;
 UPROPERTY(EditAnywhere,Category="Camera") int32 DepthHeight=240;
 // Creates render targets and scene captures. Call once after RegisterComponent().
 void Init();
 // Capture now (rate-limited by MaxHz). Returns true when a valid JPEG is available.
 bool Capture(double Now);
 TSharedPtr<FJsonObject> RgbMeta() const;
 TSharedPtr<FJsonObject> DepthMeta() const;
 TSharedPtr<FJsonObject> Chunk(bool bDepthStream,double InSequence,int32 Index) const;
private:
 UPROPERTY(Transient) TObjectPtr<USceneCaptureComponent2D> RgbCapture;
 UPROPERTY(Transient) TObjectPtr<UTextureRenderTarget2D> RgbTarget;
 UPROPERTY(Transient) TObjectPtr<USceneCaptureComponent2D> DepthCapture;
 UPROPERTY(Transient) TObjectPtr<UTextureRenderTarget2D> DepthTarget;
 TArray64<uint8> Jpeg;
 TArray64<uint8> DepthPng;
 double Stamp=-1;
 uint32 Sequence=0;
 bool bInitialized=false;
 static constexpr int32 ChunkBytes=24000;
};
