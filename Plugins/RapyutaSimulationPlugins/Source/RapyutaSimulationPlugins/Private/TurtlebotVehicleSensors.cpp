#include "TurtlebotVehicleSensors.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "Engine/World.h"
#include "Dom/JsonObject.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Misc/Base64.h"

namespace
{
TArray<TSharedPtr<FJsonValue>> Numbers(std::initializer_list<double> Values)
{
    TArray<TSharedPtr<FJsonValue>> Result;
    for (double V : Values) Result.Add(MakeShared<FJsonValueNumber>(V));
    return Result;
}
TSharedPtr<FJsonObject> Vec(const FVector& V)
{
    auto O=MakeShared<FJsonObject>();
    O->SetNumberField(TEXT("x"),V.X); O->SetNumberField(TEXT("y"),V.Y); O->SetNumberField(TEXT("z"),V.Z);
    return O;
}
TSharedPtr<FJsonObject> Quat(const FQuat& Q)
{
    auto O=Vec(FVector(-Q.X,Q.Y,-Q.Z)); O->SetNumberField(TEXT("w"),Q.W); return O;
}
}
URRTurtlebotVehicleSensors::URRTurtlebotVehicleSensors()
{
    PrimaryComponentTick.bCanEverTick=true;
    PrimaryComponentTick.TickGroup=TG_PostPhysics;
}
void URRTurtlebotVehicleSensors::SetBody(UPrimitiveComponent* InBody)
{
    Body=InBody;
    AttachToComponent(InBody,FAttachmentTransformRules::SnapToTargetNotIncludingScale);
    PreviousStamp=ImuStamp=-1; ImuSequence=0;
}
void URRTurtlebotVehicleSensors::BeginPlay()
{
    Super::BeginPlay();
    CameraWidth=FMath::Clamp(CameraWidth,64,1280);
    CameraHeight=FMath::Clamp(CameraHeight,64,960);
    CameraHorizontalFovDeg=FMath::Clamp(CameraHorizontalFovDeg,30.f,140.f);
    Target=NewObject<UTextureRenderTarget2D>(this);
    Target->RenderTargetFormat=RTF_RGBA8;
    Target->InitAutoFormat(CameraWidth,CameraHeight);
    Target->UpdateResourceImmediate(true);
    Capture=NewObject<USceneCaptureComponent2D>(GetOwner(),TEXT("RgbSensorCapture"));
    Capture->SetupAttachment(this);
    Capture->SetRelativeLocation(CameraOffsetCm);
    Capture->SetRelativeRotation(FRotator(CameraPitchDeg,0,0));
    Capture->TextureTarget=Target;
    Capture->FOVAngle=CameraHorizontalFovDeg;
    Capture->CaptureSource=SCS_FinalColorLDR;
    Capture->bCaptureEveryFrame=false;
    Capture->bCaptureOnMovement=false;
    Capture->bAlwaysPersistRenderingState=true;
    Capture->ShowFlags.SetMotionBlur(false);
    Capture->RegisterComponent();
}
void URRTurtlebotVehicleSensors::TickComponent(float Dt,ELevelTick Type,FActorComponentTickFunction* Function)
{
    Super::TickComponent(Dt,Type,Function);
    if (!Body) return;
    const double Now=GetWorld()->GetTimeSeconds();
    if (PreviousStamp>=0 && Now-PreviousStamp<1./FMath::Clamp(ImuMaxHz,1.f,400.f)) return;
    const FVector Velocity=Body->GetPhysicsLinearVelocityAtPoint(GetComponentLocation());
    const FQuat Q=GetComponentQuat();
    if (PreviousStamp>=0 && Now>PreviousStamp)
    {
        ImuDt=Now-PreviousStamp;
        const FVector A=(Velocity-LastVelocity)/ImuDt;
        // Accelerometer measures specific force a-g, not gravity-free acceleration.
        const FVector Local=Q.UnrotateVector(A-FVector(0,0,GetWorld()->GetGravityZ()))*.01;
        SpecificForce=FVector(Local.X,-Local.Y,Local.Z);
        // Measure realized rotation between samples. Chaos solver angular
        // velocity can diverge from realized motion during constraint projection.
        FQuat Delta=(Q*PreviousOrientation.Inverse()).GetNormalized();
        if (Delta.W<0) Delta=FQuat(-Delta.X,-Delta.Y,-Delta.Z,-Delta.W);
        FVector Axis; double Angle=0;
        Delta.ToAxisAndAngle(Axis,Angle);
        const FVector W=Q.UnrotateVector(Axis*(Angle/ImuDt));
        // Angular velocity is an axial vector under UE -> right-handed reflection.
        Gyro=FVector(-W.X,W.Y,-W.Z);
        ImuOrientation=Q; ImuStamp=Now; ++ImuSequence;
    }
    LastVelocity=Velocity; PreviousOrientation=Q; PreviousStamp=Now;
}
TSharedPtr<FJsonObject> URRTurtlebotVehicleSensors::ImuJson() const
{
    auto O=MakeShared<FJsonObject>();
    O->SetBoolField(TEXT("valid"),ImuStamp>=0);
    O->SetStringField(TEXT("frame_id"),TEXT("front/imu"));
    O->SetStringField(TEXT("model"),TEXT("ideal_specific_force_and_gyro"));
    O->SetStringField(TEXT("axes"),TEXT("x_forward_y_left_z_up"));
    O->SetNumberField(TEXT("sequence"),ImuSequence);
    O->SetNumberField(TEXT("stamp_s"),ImuStamp);
    O->SetNumberField(TEXT("sample_dt_s"),ImuDt);
    O->SetNumberField(TEXT("requested_max_hz"),ImuMaxHz);
    O->SetObjectField(TEXT("linear_acceleration_mps2"),Vec(SpecificForce));
    O->SetObjectField(TEXT("angular_velocity_radps"),Vec(Gyro));
    O->SetObjectField(TEXT("orientation_ground_truth_xyzw"),Quat(ImuOrientation));
    return O;
}
bool URRTurtlebotVehicleSensors::CaptureRgb()
{
    if (!Capture || !Target || !Body) return false;
    const double Now=GetWorld()->GetTimeSeconds();
    if (CameraStamp>=0 && Now-CameraStamp<1./FMath::Clamp(CameraMaxHz,1.f,30.f)) return Jpeg.Num()>0;
    Capture->CaptureScene();
    TArray<FColor> Pixels;
    FReadSurfaceDataFlags Flags(RCM_UNorm);
    Flags.SetLinearToGamma(false);
    if (!Target->GameThread_GetRenderTargetResource()->ReadPixels(Pixels,Flags)) return false;
    auto& Module=FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
    auto Wrapper=Module.CreateImageWrapper(EImageFormat::JPEG);
    if (!Wrapper->SetRaw(Pixels.GetData(),Pixels.Num()*sizeof(FColor),CameraWidth,CameraHeight,ERGBFormat::BGRA,8)) return false;
    Jpeg=Wrapper->GetCompressed(90);
    if (!Jpeg.Num()) return false;
    CameraPose=Capture->GetComponentTransform(); CameraStamp=Now; ++CameraSequence;
    return true;
}
TSharedPtr<FJsonObject> URRTurtlebotVehicleSensors::CameraJson() const
{
    auto O=MakeShared<FJsonObject>();
    O->SetBoolField(TEXT("valid"),CameraStamp>=0 && Jpeg.Num()>0);
    O->SetStringField(TEXT("frame_id"),TEXT("front/rgb_optical"));
    O->SetStringField(TEXT("encoding"),TEXT("jpeg"));
    O->SetStringField(TEXT("decoded_encoding"),TEXT("rgb8"));
    O->SetStringField(TEXT("optical_axes"),TEXT("x_right_y_down_z_forward"));
    O->SetStringField(TEXT("capture_mode"),TEXT("on_request_synchronous"));
    O->SetNumberField(TEXT("sequence"),CameraSequence); O->SetNumberField(TEXT("stamp_s"),CameraStamp);
    O->SetNumberField(TEXT("width"),CameraWidth); O->SetNumberField(TEXT("height"),CameraHeight);
    O->SetNumberField(TEXT("horizontal_fov_deg"),CameraHorizontalFovDeg);
    const double F=CameraWidth/(2.*FMath::Tan(FMath::DegreesToRadians(CameraHorizontalFovDeg)*.5));
    O->SetArrayField(TEXT("K"),Numbers({F,0,CameraWidth*.5,0,F,CameraHeight*.5,0,0,1}));
    O->SetArrayField(TEXT("distortion"),Numbers({0,0,0,0,0}));
    O->SetArrayField(TEXT("mount_position_body_flu_m"),Numbers({CameraOffsetCm.X*.01,-CameraOffsetCm.Y*.01,CameraOffsetCm.Z*.01}));
    O->SetObjectField(TEXT("mount_rotation_body_flu_xyzw"),Quat(FRotator(CameraPitchDeg,0,0).Quaternion()));
    O->SetStringField(TEXT("mount_rotation_axes"),TEXT("camera_forward_left_up_to_body_forward_left_up; optical=( -left,-up,forward )"));
    auto P=Vec(FVector((CameraPose.GetLocation().X-10000)*.01,-CameraPose.GetLocation().Y*.01,CameraPose.GetLocation().Z*.01));
    O->SetObjectField(TEXT("position_ground_truth_m"),P);
    O->SetObjectField(TEXT("rotation_ground_truth_camera_flu_xyzw"),Quat(CameraPose.GetRotation()));
    O->SetNumberField(TEXT("byte_count"),Jpeg.Num());
    O->SetNumberField(TEXT("chunk_count"),(Jpeg.Num()+ChunkBytes-1)/ChunkBytes);
    return O;
}
TSharedPtr<FJsonObject> URRTurtlebotVehicleSensors::CameraChunk(double Sequence,int32 Index) const
{
    auto O=MakeShared<FJsonObject>();
    O->SetNumberField(TEXT("rgb_sequence"),CameraSequence); O->SetNumberField(TEXT("rgb_chunk"),Index);
    const int64 Offset=int64(Index)*ChunkBytes;
    const bool Valid=Sequence==CameraSequence && Index>=0 && Offset<Jpeg.Num();
    O->SetBoolField(TEXT("valid"),Valid);
    if (Valid) O->SetStringField(TEXT("data_base64"),FBase64::Encode(Jpeg.GetData()+Offset,int32(FMath::Min<int64>(ChunkBytes,Jpeg.Num()-Offset))));
    else O->SetStringField(TEXT("error"),TEXT("frame_replaced_or_invalid_chunk; request_rgb_again"));
    return O;
}
