#include "MiniF1Car.h"
#include "TurtlebotBurger.h"
#include "TurtlebotVehicleSensors.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/PlayerController.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "InputCoreTypes.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Serialization/JsonSerializer.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Chaos/ChaosEngineInterface.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "DrawDebugHelpers.h"

namespace {
constexpr float F1WheelbaseCm=37.f, F1HalfTrackCm=10.f, F1WheelLimitDeg=30.f;
constexpr float F1CommandLimitMps=138.5824f; // 310 mph; command ceiling, not performance rating.
float MaxVirtualSteeringRad()
{
    return FMath::Atan(F1WheelbaseCm/(F1WheelbaseCm/FMath::Tan(FMath::DegreesToRadians(F1WheelLimitDeg))+F1HalfTrackCm));
}
}

class FMiniF1Simulation : public UChaosWheeledVehicleSimulation
{
public:
    virtual void ProcessSteering(const FControlInputs& Inputs) override
    {
        // Geometric Ackermann, independent of Chaos's fixed 10 cm rack stroke.
        const float Input=FMath::Clamp(Inputs.SteeringInput,-1.f,1.f);
        const float Delta=Input*MaxVirtualSteeringRad();
        for(int I=0;I<PVehicle->Wheels.Num();++I)
        {
            auto& Wheel=PVehicle->Wheels[I]; float Angle=0;
            if(Wheel.SteeringEnabled && FMath::Abs(Delta)>1.e-6f)
            {
                const float Radius=F1WheelbaseCm/FMath::Tan(FMath::Abs(Delta));
                const float Side=PVehicle->Suspension[I].GetLocalRestingPosition().Y;
                Angle=FMath::Sign(Delta)*FMath::RadiansToDegrees(FMath::Atan(F1WheelbaseCm/(Radius-FMath::Sign(Delta)*Side)));
            }
            Wheel.SetSteeringAngle(FMath::Clamp(Angle,-F1WheelLimitDeg,F1WheelLimitDeg));
        }
    }
    virtual void ApplySuspensionForces(float Dt,TArray<FWheelTraceParams>& Params) override
    {
        for(int I=0;I<PVehicle->Suspension.Num();++I)
        {
            auto& Spring=PVehicle->Suspension[I]; auto& Wheel=PVehicle->Wheels[I];
            if(Wheel.InContact())
            {
                Spring.SetSuspensionLength(WheelState.TraceResult[I].Distance,Wheel.GetEffectiveRadius());
                Spring.SetLocalVelocity(WheelState.LocalWheelVelocity[I]); Spring.Simulate(Dt);
                const float Force=FMath::Max(0.f,Spring.GetSuspensionForce());
                AddForceAtPosition(VehicleState.VehicleUpAxis*Force,
                    WheelState.WheelWorldLocation[I]+Spring.Setup().SuspensionForceOffset);
                Wheel.SetWheelLoadForce(Force);
                Wheel.SetMassPerWheel(RigidHandle->M()/PVehicle->Wheels.Num());
            }
            else { Spring.SetSuspensionLength(Spring.GetTraceLength(Wheel.GetEffectiveRadius()),Wheel.GetEffectiveRadius()); Wheel.SetWheelLoadForce(0); }
        }
    }
};
TUniquePtr<Chaos::FSimpleWheeledVehicle> UMiniF1Movement::CreatePhysicsVehicle()
{
    VehicleSimulationPT=MakeUnique<FMiniF1Simulation>();
    return UChaosVehicleMovementComponent::CreatePhysicsVehicle();
}
void UMiniF1Movement::FixupSkeletalMesh() {}

namespace {
FVector WheelPosition(int I) { return FVector(I<2?18.5:-18.5,I%2==0?-10:10,0); }
void LinkTransform(UStaticMeshComponent* C,const FVector& A,const FVector& B,float Radius=.22f)
{
    C->SetRelativeLocation((A+B)*.5);
    C->SetRelativeRotation(FRotationMatrix::MakeFromZ(B-A).Rotator());
    C->SetRelativeScale3D(FVector(Radius/50,Radius/50,(B-A).Size()/100));
}
}
UMiniF1FrontWheel::UMiniF1FrontWheel()
{
    WheelRadius=4.5f; WheelWidth=5.f; WheelMass=.08f;
    AxleType=EAxleType::Front; bAffectedBySteering=true; MaxSteerAngle=F1WheelLimitDeg;
    bAffectedByBrake=true; bAffectedByEngine=false; bAffectedByHandbrake=false;
    MaxBrakeTorque=.25f; MaxHandBrakeTorque=.35f;
    CorneringStiffness=.4f; FrictionForceMultiplier=1.4f; SideSlipModifier=.8f;
    WheelLoadRatio=1; SuspensionMaxRaise=1.5f; SuspensionMaxDrop=1.5f;
    // UE 5.8 FillSuspensionSetup multiplies this by 100: internal k=1200.
    SpringRate=12; SpringPreload=0; SuspensionDampingRatio=.8f; RollbarScaling=0;
    SweepShape=ESweepShape::Raycast; SweepType=ESweepType::ComplexSweep;
    bABSEnabled=true; bTractionControlEnabled=true;
}
UMiniF1RearWheel::UMiniF1RearWheel()
{
    AxleType=EAxleType::Rear; bAffectedBySteering=false; MaxSteerAngle=0;
    bAffectedByEngine=true; bAffectedByHandbrake=true;
}
UStaticMeshComponent* AMiniF1Car::AddPart(const TCHAR* Name,const TCHAR* Asset,USceneComponent* Parent)
{
    auto* C=CreateDefaultSubobject<UStaticMeshComponent>(Name);
    C->SetupAttachment(Parent);
    C->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,Asset));
    C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->SetGenerateOverlapEvents(false);
    return C;
}
AMiniF1Car::AMiniF1Car()
{
    PrimaryActorTick.bCanEverTick=true; PrimaryActorTick.TickGroup=TG_PostPhysics;
    AutoPossessPlayer=EAutoReceiveInput::Disabled;
    Chassis=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Chassis"));
    RootComponent=Chassis;
    Chassis->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Game/Fable/F1Prototype/SM_F1_Chassis.SM_F1_Chassis")));
    Chassis->SetCollisionProfileName(TEXT("Vehicle"));
    Chassis->SetSimulatePhysics(true); Chassis->SetEnableGravity(true);
    Chassis->SetLinearDamping(.02f); Chassis->SetAngularDamping(.05f);
    Chassis->BodyInstance.bUseCCD=true;
    for(const auto* Name:{TEXT("Floor"),TEXT("Wings"),TEXT("Cockpit"),TEXT("Halo"),TEXT("Accents")})
        AddPart(Name,*FString::Printf(TEXT("/Game/Fable/F1Prototype/SM_F1_%s.SM_F1_%s"),Name,Name),Chassis);
    auto* Metal=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Fable/F1Prototype/M_F1_Metal.M_F1_Metal"));
    for(int I=0;I<4;++I)
    {
        auto* W=AddPart(*FString::Printf(TEXT("Wheel%d"),I),TEXT("/Game/Fable/F1Prototype/SM_F1_Tire.SM_F1_Tire"),Chassis);
        WheelVisuals.Add(W);
        AddPart(*FString::Printf(TEXT("Rim%d"),I),TEXT("/Game/Fable/F1Prototype/SM_F1_Rim.SM_F1_Rim"),W);
        AddPart(*FString::Printf(TEXT("Hub%d"),I),TEXT("/Game/Fable/F1Prototype/SM_F1_Hub.SM_F1_Hub"),W);
        for(int J=0;J<6;++J)
        {
            auto* L=AddPart(*FString::Printf(TEXT("Link%d_%d"),I,J),TEXT("/Engine/BasicShapes/Cylinder.Cylinder"),Chassis);
            L->SetMaterial(0,Metal); Links.Add(L);
        }
    }
    Vehicle=CreateDefaultSubobject<UMiniF1Movement>(TEXT("F1ChaosVehicle"));
    Vehicle->SetUpdatedComponent(Chassis);
    Vehicle->SetRequiresControllerForInputs(false);
    Vehicle->Mass=3; Vehicle->ChassisWidth=25; Vehicle->ChassisHeight=12;
    Vehicle->DragCoefficient=.35f; Vehicle->DownforceCoefficient=0;
    Vehicle->bEnableCenterOfMassOverride=true; Vehicle->CenterOfMassOverride=FVector(-1,0,0);
    // Convex body excludes the wheels and wings; tune effective roll/pitch inertia.
    Vehicle->InertiaTensorScale=FVector(5,5,1);
    Vehicle->bReverseAsBrake=false;
    Vehicle->EngineSetup.MaxTorque=.6f; Vehicle->EngineSetup.MaxRPM=6500; Vehicle->EngineSetup.EngineIdleRPM=500;
    Vehicle->EngineSetup.EngineRevUpMOI=.02f; Vehicle->EngineSetup.EngineRevDownRate=1500;
    Vehicle->EngineSetup.EngineBrakeEffect=.005f;
    auto* Torque=Vehicle->EngineSetup.TorqueCurve.GetRichCurve();
    Torque->AddKey(0,1); Torque->AddKey(4500,1); Torque->AddKey(6500,.65f);
    Vehicle->TransmissionSetup.bUseAutomaticGears=false;
    Vehicle->TransmissionSetup.bUseAutoReverse=false;
    Vehicle->TransmissionSetup.ForwardGearRatios={1}; Vehicle->TransmissionSetup.ReverseGearRatios={1};
    Vehicle->TransmissionSetup.FinalRatio=3; Vehicle->TransmissionSetup.TransmissionEfficiency=.95f;
    Vehicle->DifferentialSetup.DifferentialType=EVehicleDifferential::RearWheelDrive;
    Vehicle->SteeringSetup.SteeringType=ESteeringType::Ackermann;
    auto* Steering=Vehicle->SteeringSetup.SteeringCurve.GetRichCurve(); Steering->Reset();
    Steering->AddKey(0,1); Steering->AddKey(20,1);
    for(int I=0;I<4;++I)
    {
        FChaosWheelSetup Setup;
        Setup.WheelClass=I<2?UMiniF1FrontWheel::StaticClass():UMiniF1RearWheel::StaticClass();
        // Static-mesh chassis: LocateBoneOffset uses AdditionalOffset, no skeleton.
        Setup.BoneName=FName(*FString::Printf(TEXT("Wheel%d"),I));
        Setup.AdditionalOffset=WheelPosition(I); Vehicle->WheelSetups.Add(Setup);
    }
    FrontLidar=CreateDefaultSubobject<URR2DLidarComponent>(TEXT("FrontLidar"));
    RearLidar=CreateDefaultSubobject<URR2DLidarComponent>(TEXT("RearLidar"));
    FrontLidar->SetupAttachment(Chassis); RearLidar->SetupAttachment(Chassis);
    FrontLidar->SetRelativeLocation(FVector(12,0,10)); RearLidar->SetRelativeLocation(FVector(-12,0,10));
    FrontLidar->bShowLidarRays=false; RearLidar->bShowLidarRays=false;
    VehicleSensors=CreateDefaultSubobject<URRTurtlebotVehicleSensors>(TEXT("VehicleSensors"));
    VehicleSensors->SetupAttachment(Chassis); VehicleSensors->CameraOffsetCm=FVector(10,0,12);
    ChaseArm=CreateDefaultSubobject<USpringArmComponent>(TEXT("ChaseArm"));
    ChaseArm->SetupAttachment(Chassis); ChaseArm->TargetArmLength=115;
    ChaseArm->SetRelativeRotation(FRotator(-22,0,0)); ChaseArm->SocketOffset=FVector(0,0,28);
    ChaseArm->bDoCollisionTest=false;
    ChaseCamera=CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
    ChaseCamera->SetupAttachment(ChaseArm,USpringArmComponent::SocketName);
    ChaseCamera->FieldOfView=80;
}
void AMiniF1Car::OnConstruction(const FTransform& T) { Super::OnConstruction(T); AnimateWheels(); }
void AMiniF1Car::BeginPlay()
{
    Super::BeginPlay(); VehicleSensors->SetBody(Chassis);
    EpisodeStart=Chassis->GetComponentTransform();
    Vehicle->SetTargetGear(1,true);
    Socket=FUdpSocketBuilder(TEXT("MiniF1Commands")).AsNonBlocking().BoundToEndpoint(FIPv4Endpoint(FIPv4Address(127,0,0,1),CommandPort));
    if(!Socket) UE_LOG(LogTemp,Error,TEXT("MiniF1: UDP %d unavailable"),CommandPort);
}
void AMiniF1Car::AnimateWheels()
{
    for(int I=0;I<4;++I)
    {
        FVector P=WheelPosition(I); float Steer=0,Spin=0;
        if(Vehicle && Vehicle->HasValidPhysicsState() && Vehicle->Wheels.IsValidIndex(I))
        { auto* W=Vehicle->Wheels[I].Get(); P.Z+=W->GetSuspensionOffset(); Steer=W->GetSteerAngle(); Spin=W->GetRotationAngle(); }
        WheelVisuals[I]->SetRelativeLocation(P);
        WheelVisuals[I]->SetRelativeRotation(FRotator(0,Steer,0).Quaternion()*FRotator(Spin,0,0).Quaternion());
        const float S=I%2==0?-1:1;
        for(int J=0;J<4;++J)
        {
            const bool Upper=J>=2; const float Z=Upper?2:-1;
            LinkTransform(Links[I*6+J],FVector(P.X+(J%2==0?-3.5:3.5),S*3,Z),P+FVector(0,-S*1.8,Upper?1.2:-1));
        }
        const FVector Knuckle=P+FRotator(0,Steer,0).RotateVector(FVector(-1.5,-S*1.8,0));
        LinkTransform(Links[I*6+4],FVector(P.X-3,S*3,0),Knuckle,.16f);
        LinkTransform(Links[I*6+5],FVector(P.X,S*3.5,3.5),P+FVector(0,-S*2,-.5),.35f);
    }
}
TSharedPtr<FJsonObject> AMiniF1Car::Telemetry(bool Sensors) const
{
    auto O=MakeShared<FJsonObject>();
    const FVector P=Chassis->GetComponentLocation(),V=Chassis->GetPhysicsLinearVelocity();
    O->SetStringField(TEXT("schema"),TEXT("f1_sensors_v1"));
    O->SetNumberField(TEXT("vehicle_config_version"),1);
    O->SetNumberField(TEXT("episode_api_version"),1);
    O->SetNumberField(TEXT("episode_id"),EpisodeId);
    O->SetStringField(TEXT("reset_token"),LastResetToken);
    O->SetNumberField(TEXT("max_speed_mps"),MaxSpeedMps);
    O->SetNumberField(TEXT("speed_target_mps"),SpeedTarget);
    O->SetNumberField(TEXT("front_tire_grip_multiplier"),FrontGripMultiplier);
    O->SetNumberField(TEXT("rear_tire_grip_multiplier"),RearGripMultiplier);
    O->SetStringField(TEXT("pose_source"),TEXT("chaos_ground_truth_not_encoder_odometry"));
    O->SetNumberField(TEXT("time_s"),GetWorld()->GetTimeSeconds());
    O->SetNumberField(TEXT("x_m"),(P.X-10000)*.01); O->SetNumberField(TEXT("y_m"),-P.Y*.01); O->SetNumberField(TEXT("z_m"),P.Z*.01);
    O->SetNumberField(TEXT("yaw_rad"),-FMath::DegreesToRadians(Chassis->GetComponentRotation().Yaw));
    O->SetNumberField(TEXT("vx_mps"),V.X*.01); O->SetNumberField(TEXT("vy_mps"),-V.Y*.01);
    O->SetNumberField(TEXT("forward_speed_mps"),Vehicle->GetForwardSpeed()*.01);
    O->SetBoolField(TEXT("physics_valid"),Vehicle->HasValidPhysicsState());
    O->SetNumberField(TEXT("steering_command_rad"),SteeringTarget);
    TArray<TSharedPtr<FJsonValue>> Wheels;
    if(Vehicle->HasValidPhysicsState()) for(int I=0;I<Vehicle->Wheels.Num();++I)
    {
        auto W=MakeShared<FJsonObject>(); auto* Wheel=Vehicle->Wheels[I].Get();
        const auto& Status=Vehicle->GetWheelState(I);
        W->SetNumberField(TEXT("index"),I);
        W->SetNumberField(TEXT("steering_rad"),-FMath::DegreesToRadians(Wheel->GetSteerAngle()));
        W->SetNumberField(TEXT("suspension_offset_m"),Wheel->GetSuspensionOffset()*.01);
        W->SetNumberField(TEXT("drive_torque_nm"),Status.DriveTorque);
        W->SetBoolField(TEXT("in_contact"),Status.bInContact);
        Wheels.Add(MakeShared<FJsonValueObject>(W));
    }
    O->SetArrayField(TEXT("wheels"),Wheels);
    if(Sensors)
    {
        O->SetObjectField(TEXT("front_scan"),FrontLidar->ScanJson(TEXT("front/base_scan")));
        O->SetObjectField(TEXT("rear_scan"),RearLidar->ScanJson(TEXT("rear/base_scan")));
        O->SetObjectField(TEXT("imu"),VehicleSensors->ImuJson()); O->SetObjectField(TEXT("rgb"),VehicleSensors->CameraJson());
    }
    return O;
}
void AMiniF1Car::PollCommands()
{
    uint32 Pending;
    for(int N=0;Socket && N<16 && Socket->HasPendingData(Pending);++N)
    {
        uint8 Buffer[4096]; int32 Read=0; auto Sender=ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        if(!Socket->RecvFrom(Buffer,sizeof(Buffer)-1,Read,*Sender)) break; Buffer[Read]=0;
        TSharedPtr<FJsonObject> M;
        if(!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(FString(UTF8_TO_TCHAR(reinterpret_cast<char*>(Buffer)))),M) || !M.IsValid()) continue;
        // Idempotent localhost reset: retries with the same token do not reset again.
        FString ResetToken;
        if(M->TryGetStringField(TEXT("reset_episode"),ResetToken) && !ResetToken.IsEmpty() && ResetToken.Len()<=80 && ResetToken!=LastResetToken)
        {
            SpeedTarget=SteeringTarget=ThrottleTarget=0; BrakeTarget=1; bDirectInputs=false;
            Chassis->SetWorldTransform(EpisodeStart,false,nullptr,ETeleportType::TeleportPhysics);
            Chassis->SetPhysicsLinearVelocity(FVector::ZeroVector);
            Chassis->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
            Vehicle->ResetVehicle(); Vehicle->SetTargetGear(1,true);
            Vehicle->SetThrottleInput(0); Vehicle->SetBrakeInput(1); Vehicle->SetSteeringInput(0);
            for(int I=0;I<Vehicle->Wheels.Num();++I) Vehicle->SetWheelFrictionMultiplier(I,1.4f*(I<2?FrontGripMultiplier:RearGripMultiplier));
            Chassis->WakeAllRigidBodies(); Vehicle->SetSleeping(false);
            VehicleSensors->SetBody(Chassis);
            LastResetToken=ResetToken; ++EpisodeId; LastCommand=GetWorld()->GetTimeSeconds();
        }
        // Small chunked line upload; points use the same SI bridge coordinates as telemetry.
        bool ClearLine=false; M->TryGetBoolField(TEXT("clear_racing_line"),ClearLine);
        if(ClearLine) BestRacingLine.Empty();
        const TArray<TSharedPtr<FJsonValue>>* LinePoints=nullptr;
        if(M->TryGetArrayField(TEXT("racing_line_points"),LinePoints) && LinePoints->Num()<=20)
        {
            for(const auto& Point:*LinePoints)
            {
                const TArray<TSharedPtr<FJsonValue>>* XYZ=nullptr;
                if(!Point->TryGetArray(XYZ) || XYZ->Num()!=3 || BestRacingLine.Num()>=4096) continue;
                double X,Y,Z;
                if((*XYZ)[0]->TryGetNumber(X) && (*XYZ)[1]->TryGetNumber(Y) && (*XYZ)[2]->TryGetNumber(Z)
                    && FMath::IsFinite(X) && FMath::IsFinite(Y) && FMath::IsFinite(Z) && FMath::Abs(X)<10000 && FMath::Abs(Y)<10000 && FMath::Abs(Z)<1000)
                    BestRacingLine.Add(FVector(10000+100*X,-100*Y,100*Z));
            }
        }
        // Runtime experiment configuration: validate the complete request before mutation.
        // Relative grip 1.0 preserves the original Chaos friction multiplier of 1.4.
        bool ConfigRequested=M->HasField(TEXT("configure_vehicle")),ConfigAccepted=false;
        if(ConfigRequested)
        {
            const TSharedPtr<FJsonObject>* Config=nullptr;
            double Limit,Front,Rear;
            if(M->TryGetObjectField(TEXT("configure_vehicle"),Config) && Config && Config->IsValid()
                && (*Config)->TryGetNumberField(TEXT("max_speed_mps"),Limit)
                && (*Config)->TryGetNumberField(TEXT("front_tire_grip_multiplier"),Front)
                && (*Config)->TryGetNumberField(TEXT("rear_tire_grip_multiplier"),Rear)
                && FMath::IsFinite(Limit) && FMath::IsFinite(Front) && FMath::IsFinite(Rear)
                && Limit>0 && Limit<=138.5824 && Front>=0 && Front<=3 && Rear>=0 && Rear<=3
                && Vehicle->HasValidPhysicsState() && Vehicle->Wheels.Num()==4)
            {
                MaxSpeedMps=Limit; FrontGripMultiplier=Front; RearGripMultiplier=Rear;
                for(int I=0;I<4;++I) Vehicle->SetWheelFrictionMultiplier(I,1.4f*(I<2?FrontGripMultiplier:RearGripMultiplier));
                SpeedTarget=FMath::Clamp(SpeedTarget,-MaxSpeedMps,MaxSpeedMps);
                ConfigAccepted=true;
            }
        }
        double V,S,B;
        if(M->TryGetNumberField(TEXT("speed_mps"),V) && M->TryGetNumberField(TEXT("steering_rad"),S) && FMath::IsFinite(V) && FMath::IsFinite(S))
        { const double Limit=FMath::Clamp(double(MaxSpeedMps),0.,double(F1CommandLimitMps)); SpeedTarget=FMath::Clamp(V,-Limit,Limit); SteeringTarget=FMath::Clamp(S,-double(MaxVirtualSteeringRad()),double(MaxVirtualSteeringRad())); bDirectInputs=false; LastCommand=GetWorld()->GetTimeSeconds(); }
        else if(M->TryGetNumberField(TEXT("throttle"),V) && M->TryGetNumberField(TEXT("steer"),S) && M->TryGetNumberField(TEXT("brake"),B) && FMath::IsFinite(V) && FMath::IsFinite(S) && FMath::IsFinite(B))
        { ThrottleTarget=FMath::Clamp(V,-1.,1.); SteeringTarget=FMath::Clamp(S,-1.,1.)*MaxVirtualSteeringRad(); BrakeTarget=FMath::Clamp(B,0.,1.); bDirectInputs=true; LastCommand=GetWorld()->GetTimeSeconds(); }
        bool Sensors=false,Rgb=false; M->TryGetBoolField(TEXT("sensors"),Sensors); M->TryGetBoolField(TEXT("rgb"),Rgb);
        auto O=Telemetry(Sensors); double Sequence,Chunk;
        if(ConfigRequested) { O->SetBoolField(TEXT("configuration_accepted"),ConfigAccepted); if(!ConfigAccepted) O->SetStringField(TEXT("configuration_error"),TEXT("Require valid physics, max_speed_mps in (0,138.5824], and both tire grip multipliers in [0,3].")); }
        if(Rgb)
        { const bool Captured=VehicleSensors->CaptureRgb(); O->SetBoolField(TEXT("valid"),Captured); O->SetObjectField(TEXT("rgb"),VehicleSensors->CameraJson()); O->SetObjectField(TEXT("imu"),VehicleSensors->ImuJson()); }
        else if(M->TryGetNumberField(TEXT("rgb_sequence"),Sequence) && M->TryGetNumberField(TEXT("rgb_chunk"),Chunk)) O=VehicleSensors->CameraChunk(Sequence,int32(FMath::Clamp(Chunk,-1.,10000.)));
        FString Reply; auto Writer=TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Reply);
        FJsonSerializer::Serialize(O.ToSharedRef(),Writer); FTCHARToUTF8 Bytes(*Reply); int32 Sent;
        Socket->SendTo(reinterpret_cast<const uint8*>(Bytes.Get()),Bytes.Length(),Sent,*Sender);
    }
}
void AMiniF1Car::Tick(float Dt)
{
    Super::Tick(Dt); PollCommands();
    for(int I=1;I<BestRacingLine.Num();++I)
        DrawDebugLine(GetWorld(),BestRacingLine[I-1],BestRacingLine[I],FColor::Green,false,0,0,1.5f);
    if(bKeyboardControl) if(auto* PC=GetWorld()->GetFirstPlayerController())
    {
        if(!bSetView) { PC->SetViewTarget(this); bSetView=true; }
        const float V=(PC->IsInputKeyDown(EKeys::I)?1.f:0.f)-(PC->IsInputKeyDown(EKeys::K)?1.f:0.f);
        const float S=(PC->IsInputKeyDown(EKeys::J)?1.f:0.f)-(PC->IsInputKeyDown(EKeys::L)?1.f:0.f);
        if(V || S || PC->IsInputKeyDown(EKeys::SpaceBar))
        { SpeedTarget=PC->IsInputKeyDown(EKeys::SpaceBar)?0:V*ManualSpeedMps; SteeringTarget=S*MaxVirtualSteeringRad(); bDirectInputs=false; LastCommand=GetWorld()->GetTimeSeconds(); }
    }
    if(GetWorld()->GetTimeSeconds()-LastCommand>.25) { SpeedTarget=0; SteeringTarget=0; bDirectInputs=false; }
    const float Speed=Vehicle->GetForwardSpeed()*.01;
    const float Demand=bDirectInputs?ThrottleTarget:SpeedTarget;
    // UDP/keyboard commands must wake this pawn even without a possessed controller.
    if(FMath::Abs(Demand)>.001f || FMath::Abs(SteeringTarget)>.001f)
    {
        // Chaos's WakeAllEnabledRigidBodies only handles skeletal meshes.
        Chassis->WakeAllRigidBodies();
        Vehicle->SetSleeping(false);
    }
    const float Sign=Demand>=0?1:-1;
    float Throttle=bDirectInputs?FMath::Abs(ThrottleTarget):FMath::Clamp(.04f+.7f*(FMath::Abs(SpeedTarget)-Sign*Speed),0.f,1.f);
    float Brake=bDirectInputs?BrakeTarget:FMath::Clamp((Sign*Speed-FMath::Abs(SpeedTarget))*.4f,0.f,1.f);
    if(!bDirectInputs && FMath::Abs(SpeedTarget)<.001) { Throttle=0; Brake=1; }
    if(Sign*Speed<-.1) { Throttle=0; Brake=1; }
    if(FMath::Abs(Speed)>MaxSpeedMps) { Throttle=0; Brake=FMath::Max(Brake,.3f); }
    Vehicle->SetTargetGear(Sign>0?1:-1,true);
    Vehicle->SetThrottleInput(Throttle); Vehicle->SetBrakeInput(Brake);
    Vehicle->SetSteeringInput(-SteeringTarget/MaxVirtualSteeringRad());
    AnimateWheels();
}
void AMiniF1Car::EndPlay(const EEndPlayReason::Type Reason)
{
    Vehicle->SetThrottleInput(0); Vehicle->SetBrakeInput(1);
    if(Socket) { Socket->Close(); ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket); Socket=nullptr; }
    Super::EndPlay(Reason);
}
