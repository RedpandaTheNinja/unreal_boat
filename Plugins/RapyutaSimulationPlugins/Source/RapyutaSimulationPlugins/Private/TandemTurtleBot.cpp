#include "TandemTurtleBot.h"
#include "TurtlebotBurger.h"
#include "TurtlebotVehicleSensors.h"
#include "Components/ChildActorComponent.h"
#include "Components/StaticMeshComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "GameFramework/PlayerController.h"
#include "Engine/World.h"
#include "InputCoreTypes.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Serialization/JsonSerializer.h"
#include "Policies/CondensedJsonPrintPolicy.h"

ATandemTurtleBot::ATandemTurtleBot()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("AssemblyOrigin"));
    Front = CreateDefaultSubobject<UChildActorComponent>(TEXT("Front"));
    Rear = CreateDefaultSubobject<UChildActorComponent>(TEXT("Rear"));
    Front->SetupAttachment(RootComponent);
    Rear->SetupAttachment(RootComponent);
    static ConstructorHelpers::FClassFinder<ATurtlebotBurger> BP(TEXT("/RapyutaSimulationPlugins/Robots/Turtlebot3/Physics/BP_TurtlebotBurger"));
    if (BP.Succeeded()) { Front->SetChildActorClass(BP.Class); Rear->SetChildActorClass(BP.Class); }
    RigidLink = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RigidLink"));
    RigidLink->SetupAttachment(RootComponent);
    RigidLink->SetLinearXLimit(LCM_Locked,0);
    RigidLink->SetLinearYLimit(LCM_Locked,0);
    RigidLink->SetLinearZLimit(LCM_Locked,0);
    RigidLink->SetAngularSwing1Limit(ACM_Locked,0);
    RigidLink->SetAngularSwing2Limit(ACM_Locked,0);
    RigidLink->SetAngularTwistLimit(ACM_Locked,0);
    RigidLink->SetDisableCollision(true);
    VehicleSensors=CreateDefaultSubobject<URRTurtlebotVehicleSensors>(TEXT("VehicleSensors"));
    VehicleSensors->SetupAttachment(RootComponent);
}
ATurtlebotBurger* ATandemTurtleBot::FrontBot() const { return Cast<ATurtlebotBurger>(Front->GetChildActor()); }
ATurtlebotBurger* ATandemTurtleBot::RearBot() const { return Cast<ATurtlebotBurger>(Rear->GetChildActor()); }
void ATandemTurtleBot::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    Front->SetRelativeLocation(FVector(AxleSpacingM*50.f,0,0));
    Rear->SetRelativeLocation(FVector(-AxleSpacingM*50.f,0,0));
}
void ATandemTurtleBot::BeginPlay()
{
    Super::BeginPlay();
    if (FrontBot()) VehicleSensors->SetBody(FrontBot()->Base);
    if (FrontBot() && RearBot())
        RigidLink->SetConstrainedComponents(FrontBot()->Base,NAME_None,RearBot()->Base,NAME_None);
    else UE_LOG(LogTemp, Error, TEXT("TandemTurtleBot: original Burger Blueprint failed to load"));
    Socket = FUdpSocketBuilder(TEXT("TandemTurtleCommands")).AsNonBlocking()
        .BoundToEndpoint(FIPv4Endpoint(FIPv4Address(127,0,0,1),CommandPort));
    if (!Socket) UE_LOG(LogTemp, Error, TEXT("TandemTurtleBot: UDP port %d unavailable"),CommandPort);
}
void ATandemTurtleBot::SetVelocitySI(float ForwardMps,float YawRadps)
{
    if (!FMath::IsFinite(ForwardMps) || !FMath::IsFinite(YawRadps)) return;
    TargetV=ForwardMps; TargetW=YawRadps;
    LastCommandTime=GetWorld()->GetTimeSeconds();
}
void ATandemTurtleBot::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    uint32 Pending=0;
    // Bounded per-frame work. Replies are actual Chaos body poses, never commanded poses.
    for (int32 N=0; Socket && N<16 && Socket->HasPendingData(Pending); ++N)
    {
        uint8 Buffer[4096]; int32 Read=0;
        auto Sender=ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        if (!Socket->RecvFrom(Buffer,sizeof(Buffer)-1,Read,*Sender)) break;
        Buffer[Read]=0;
        TSharedPtr<FJsonObject> Message;
        auto Reader=TJsonReaderFactory<>::Create(FString(UTF8_TO_TCHAR(reinterpret_cast<char*>(Buffer))));
        double V,W;
        if (FJsonSerializer::Deserialize(Reader,Message) && Message.IsValid()
            && Message->TryGetNumberField(TEXT("v_mps"),V) && Message->TryGetNumberField(TEXT("yaw_radps"),W))
            SetVelocitySI(V,W);
        if (FrontBot() && RearBot())
        {
            const FVector P=(FrontBot()->Base->GetComponentLocation()+RearBot()->Base->GetComponentLocation())*0.5;
            const FVector Vel=FrontBot()->Base->GetPhysicsLinearVelocity();
            const float Yaw=FrontBot()->Base->GetComponentRotation().Yaw;
            const float Separation=FVector::Distance(FrontBot()->Base->GetComponentLocation(),RearBot()->Base->GetComponentLocation())/100.f;
            FString Reply=FString::Printf(TEXT("{\"x_m\":%.6f,\"y_m\":%.6f,\"z_m\":%.6f,\"yaw_rad\":%.6f,\"vx_mps\":%.6f,\"vy_mps\":%.6f,\"separation_m\":%.6f,\"time_s\":%.6f}"),
                (P.X-10000.f)/100.f,-P.Y/100.f,P.Z/100.f,-FMath::DegreesToRadians(Yaw),Vel.X/100.f,-Vel.Y/100.f,Separation,GetWorld()->GetTimeSeconds());
            bool bSensors=false;
            if (Message.IsValid() && Message->TryGetBoolField(TEXT("sensors"),bSensors) && bSensors)
            {
                TSharedPtr<FJsonObject> Data;
                FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Reply),Data);
                Data->SetStringField(TEXT("pose_source"),TEXT("chaos_ground_truth_not_encoder_odometry"));
                Data->SetStringField(TEXT("schema"),TEXT("tandem_sensors_v1"));
                Data->SetObjectField(TEXT("front_scan"),FrontBot()->LidarComponent->ScanJson(TEXT("front/base_scan")));
                Data->SetObjectField(TEXT("rear_scan"),RearBot()->LidarComponent->ScanJson(TEXT("rear/base_scan")));
                Data->SetObjectField(TEXT("imu"),VehicleSensors->ImuJson());
                Data->SetObjectField(TEXT("rgb"),VehicleSensors->CameraJson());
                Reply.Empty();
                auto Writer=TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Reply);
                FJsonSerializer::Serialize(Data.ToSharedRef(),Writer);
            }
            bool bRgb=false;
            double ChunkIndex=0,RequestedSequence=0;
            TSharedPtr<FJsonObject> Extra;
            if (Message.IsValid() && Message->TryGetBoolField(TEXT("rgb"),bRgb) && bRgb)
            {
                const bool Captured=VehicleSensors->CaptureRgb();
                Extra=MakeShared<FJsonObject>();
                Extra->SetBoolField(TEXT("valid"),Captured);
                Extra->SetObjectField(TEXT("rgb"),VehicleSensors->CameraJson());
                Extra->SetObjectField(TEXT("imu"),VehicleSensors->ImuJson());
            }
            else if (Message.IsValid() && Message->TryGetNumberField(TEXT("rgb_chunk"),ChunkIndex)
                && Message->TryGetNumberField(TEXT("rgb_sequence"),RequestedSequence))
                Extra=VehicleSensors->CameraChunk(RequestedSequence,int32(FMath::Clamp(ChunkIndex,-1.,10000.)));
            if (Extra.IsValid())
            {
                Reply.Empty();
                auto Writer=TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Reply);
                FJsonSerializer::Serialize(Extra.ToSharedRef(),Writer);
            }
            FTCHARToUTF8 Bytes(*Reply); int32 Sent;
            Socket->SendTo(reinterpret_cast<const uint8*>(Bytes.Get()),Bytes.Length(),Sent,*Sender);
        }
    }
    if (bKeyboardControl)
    {
        if (auto* PC=GetWorld()->GetFirstPlayerController())
        {
            const float V=(PC->IsInputKeyDown(EKeys::I)?1.f:0.f)-(PC->IsInputKeyDown(EKeys::K)?1.f:0.f);
            const float W=(PC->IsInputKeyDown(EKeys::J)?1.f:0.f)-(PC->IsInputKeyDown(EKeys::L)?1.f:0.f);
            if (V!=0 || W!=0 || PC->IsInputKeyDown(EKeys::SpaceBar)) SetVelocitySI(V*ManualSpeedMps,W*ManualYawRadps);
        }
    }
    if (GetWorld()->GetTimeSeconds()-LastCommandTime>0.25f) { TargetV=0; TargetW=0; }
    if (FrontBot()) FrontBot()->SetVelocitySI(TargetV,TargetW);
    if (RearBot()) RearBot()->SetVelocitySI(TargetV,TargetW);
}
void ATandemTurtleBot::EndPlay(const EEndPlayReason::Type Reason)
{
    if (FrontBot()) FrontBot()->SetVelocitySI(0,0);
    if (RearBot()) RearBot()->SetVelocitySI(0,0);
    if (Socket) { Socket->Close(); ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket); Socket=nullptr; }
    Super::EndPlay(Reason);
}
