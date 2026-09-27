#include "BoatCameraSensor.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Dom/JsonObject.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Misc/Base64.h"

namespace {
TArray<TSharedPtr<FJsonValue>> Numbers(std::initializer_list<double> Values) {
 TArray<TSharedPtr<FJsonValue>> Out;
 for(double V:Values) Out.Add(MakeShared<FJsonValueNumber>(V));
 return Out;
}
}

UBoatCameraSensor::UBoatCameraSensor() {
 PrimaryComponentTick.bCanEverTick=false;
}

void UBoatCameraSensor::Init() {
 if(bInitialized) return;
 Width=FMath::Clamp(Width,64,1920); Height=FMath::Clamp(Height,64,1080);
 DepthWidth=FMath::Clamp(DepthWidth,32,1280); DepthHeight=FMath::Clamp(DepthHeight,32,720);
 HorizontalFovDeg=FMath::Clamp(HorizontalFovDeg,30.f,140.f);
 RgbTarget=NewObject<UTextureRenderTarget2D>(this);
 RgbTarget->RenderTargetFormat=RTF_RGBA8;
 RgbTarget->InitAutoFormat(Width,Height);
 RgbTarget->UpdateResourceImmediate(true);
 RgbCapture=NewObject<USceneCaptureComponent2D>(GetOwner(),TEXT("BoatLabRgbCapture"));
 RgbCapture->SetupAttachment(this);
 RgbCapture->SetRelativeRotation(FRotator(-PitchDownDeg,0,0));
 RgbCapture->TextureTarget=RgbTarget;
 RgbCapture->FOVAngle=HorizontalFovDeg;
 RgbCapture->CaptureSource=SCS_FinalColorLDR;
 RgbCapture->bCaptureEveryFrame=false;
 RgbCapture->bCaptureOnMovement=false;
 RgbCapture->bAlwaysPersistRenderingState=true;
 RgbCapture->ShowFlags.SetMotionBlur(false);
 RgbCapture->HiddenActors.Add(GetOwner());          // never see our own hull
 RgbCapture->RegisterComponent();
 if(bDepth) {
  DepthTarget=NewObject<UTextureRenderTarget2D>(this);
  DepthTarget->RenderTargetFormat=RTF_RGBA32f;
  DepthTarget->bForceLinearGamma=true;
  DepthTarget->InitAutoFormat(DepthWidth,DepthHeight);
  DepthTarget->UpdateResourceImmediate(true);
  DepthCapture=NewObject<USceneCaptureComponent2D>(GetOwner(),TEXT("BoatLabDepthCapture"));
  DepthCapture->SetupAttachment(this);
  DepthCapture->SetRelativeRotation(FRotator(-PitchDownDeg,0,0));
  DepthCapture->TextureTarget=DepthTarget;
  DepthCapture->FOVAngle=HorizontalFovDeg;
  DepthCapture->CaptureSource=SCS_SceneDepth;        // R = scene depth in cm along the view axis
  DepthCapture->bCaptureEveryFrame=false;
  DepthCapture->bCaptureOnMovement=false;
  DepthCapture->bAlwaysPersistRenderingState=true;
  DepthCapture->HiddenActors.Add(GetOwner());
  DepthCapture->RegisterComponent();
 }
 bInitialized=true;
}

bool UBoatCameraSensor::Capture(double Now) {
 if(!bInitialized || !RgbCapture || !RgbTarget) return false;
 if(Stamp>=0 && Now-Stamp<1./FMath::Clamp(MaxHz,1.f,30.f)) return Jpeg.Num()>0;
 RgbCapture->CaptureScene();
 TArray<FColor> Pixels;
 FReadSurfaceDataFlags Flags(RCM_UNorm);
 Flags.SetLinearToGamma(false);
 if(!RgbTarget->GameThread_GetRenderTargetResource()->ReadPixels(Pixels,Flags)) return false;
 auto& Module=FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
 auto Wrapper=Module.CreateImageWrapper(EImageFormat::JPEG);
 if(!Wrapper->SetRaw(Pixels.GetData(),Pixels.Num()*sizeof(FColor),Width,Height,ERGBFormat::BGRA,8)) return false;
 Jpeg=Wrapper->GetCompressed(90);
 DepthPng.Reset();
 if(bDepth && DepthCapture && DepthTarget) {
  DepthCapture->CaptureScene();
  TArray<FLinearColor> Depth;
  if(DepthTarget->GameThread_GetRenderTargetResource()->ReadLinearColorPixels(Depth) && Depth.Num()==DepthWidth*DepthHeight) {
   TArray<uint16> Mm;
   Mm.SetNumUninitialized(Depth.Num());
   for(int32 I=0;I<Depth.Num();++I) {
    const double Millimetres=double(Depth[I].R)*10.0;       // cm -> mm
    Mm[I]=(FMath::IsFinite(Millimetres) && Millimetres>0 && Millimetres<65535.0) ? uint16(Millimetres) : 0;
   }
   auto Png=Module.CreateImageWrapper(EImageFormat::PNG);
   if(Png->SetRaw(Mm.GetData(),Mm.Num()*sizeof(uint16),DepthWidth,DepthHeight,ERGBFormat::Gray,16)) DepthPng=Png->GetCompressed();
  }
 }
 Stamp=Now; ++Sequence;
 return Jpeg.Num()>0;
}

TSharedPtr<FJsonObject> UBoatCameraSensor::RgbMeta() const {
 auto O=MakeShared<FJsonObject>();
 O->SetBoolField(TEXT("valid"),Stamp>=0 && Jpeg.Num()>0);
 O->SetStringField(TEXT("encoding"),TEXT("jpeg"));
 O->SetStringField(TEXT("model"),TEXT("ideal_pinhole_scene_capture"));
 O->SetNumberField(TEXT("sequence"),Sequence);
 O->SetNumberField(TEXT("stamp_s"),Stamp);
 O->SetNumberField(TEXT("width"),Width);
 O->SetNumberField(TEXT("height"),Height);
 O->SetNumberField(TEXT("horizontal_fov_deg"),HorizontalFovDeg);
 O->SetNumberField(TEXT("pitch_down_deg"),PitchDownDeg);
 const double F=Width/(2.*FMath::Tan(FMath::DegreesToRadians(HorizontalFovDeg)*.5));
 O->SetArrayField(TEXT("K"),Numbers({F,0,Width*.5,0,F,Height*.5,0,0,1}));
 const FVector L=GetRelativeLocation();
 // UE body (x fwd, y right, z up, cm) -> FLU metres
 O->SetArrayField(TEXT("mount_position_body_flu_m"),Numbers({L.X*.01,-L.Y*.01,L.Z*.01}));
 O->SetNumberField(TEXT("byte_count"),Jpeg.Num());
 O->SetNumberField(TEXT("chunk_count"),(Jpeg.Num()+ChunkBytes-1)/ChunkBytes);
 return O;
}

TSharedPtr<FJsonObject> UBoatCameraSensor::DepthMeta() const {
 auto O=MakeShared<FJsonObject>();
 O->SetBoolField(TEXT("valid"),Stamp>=0 && DepthPng.Num()>0);
 O->SetStringField(TEXT("encoding"),TEXT("png_uint16_mm"));
 O->SetStringField(TEXT("model"),TEXT("ideal_scene_depth_not_stereo"));
 O->SetNumberField(TEXT("sequence"),Sequence);
 O->SetNumberField(TEXT("width"),DepthWidth);
 O->SetNumberField(TEXT("height"),DepthHeight);
 O->SetNumberField(TEXT("byte_count"),DepthPng.Num());
 O->SetNumberField(TEXT("chunk_count"),(DepthPng.Num()+ChunkBytes-1)/ChunkBytes);
 return O;
}

TSharedPtr<FJsonObject> UBoatCameraSensor::Chunk(bool bDepthStream,double InSequence,int32 Index) const {
 const TArray64<uint8>& Data=bDepthStream?DepthPng:Jpeg;
 const FString Prefix=bDepthStream?TEXT("depth"):TEXT("rgb");
 auto O=MakeShared<FJsonObject>();
 O->SetNumberField(Prefix+TEXT("_sequence"),Sequence);
 O->SetNumberField(Prefix+TEXT("_chunk"),Index);
 const int64 Offset=int64(Index)*ChunkBytes;
 const bool Valid=uint32(InSequence)==Sequence && Index>=0 && Offset<Data.Num();
 O->SetBoolField(TEXT("valid"),Valid);
 if(Valid) O->SetStringField(TEXT("data_base64"),FBase64::Encode(Data.GetData()+Offset,int32(FMath::Min<int64>(ChunkBytes,Data.Num()-Offset))));
 else O->SetStringField(TEXT("error"),TEXT("frame_replaced_or_invalid_chunk; request rgb again"));
 return O;
}
