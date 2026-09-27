#include "BoatDynamicsComponent.h"
#include "Components/StaticMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Serialization/JsonSerializer.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "BoatCameraSensor.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"

namespace {
constexpr double G=9.81;
FVector Divide(FVector A,FVector B) {return FVector(A.X/B.X,A.Y/B.Y,A.Z/B.Z);}
FVector Abs(FVector A) {return FVector(FMath::Abs(A.X),FMath::Abs(A.Y),FMath::Abs(A.Z));}
FVector ParallelAxis(double M,FVector D) {return M*FVector(D.Y*D.Y+D.Z*D.Z,D.X*D.X+D.Z*D.Z,D.X*D.X+D.Y*D.Y);}
TArray<TSharedPtr<FJsonValue>> Array(FVector V) {return {MakeShared<FJsonValueNumber>(V.X),MakeShared<FJsonValueNumber>(V.Y),MakeShared<FJsonValueNumber>(V.Z)};}
}
UBoatDynamicsComponent::UBoatDynamicsComponent() {
 bAutoActivate=true;
 PrimaryComponentTick.bCanEverTick=true;
 PrimaryComponentTick.TickGroup=TG_PrePhysics;
}
void UBoatDynamicsComponent::BuildCells() {
 // Outer geometry of SM_BoatHull_13x3ft, in meters. Vertical-column quadrature
 // of the sealed displacement envelope, not arbitrary supporting springs.
 const TArray<FVector2D> P={{1.9812,0},{1.50,.32},{.8,.4572},{-1.50,.4572},{-1.9812,.4},{-1.9812,-.4},{-1.5,-.4572},{.8,-.4572},{1.5,-.32}};
 auto Inside=[&P](double X,double Y,double ScaleX,double ScaleY) {
  for(int I=0;I<P.Num();++I) {const auto A=P[I]*FVector2D(ScaleX,ScaleY);const auto B=P[(I+1)%P.Num()]*FVector2D(ScaleX,ScaleY);
   if((B.X-A.X)*(Y-A.Y)-(B.Y-A.Y)*(X-A.X)<0) return false;}
  return true;
 };
 const double Dx=3.9624/64,Dy=.9144/24;
 Cells.Reset();Area=0;
 for(int I=0;I<64;++I) for(int J=0;J<24;++J) {
  const double X=-1.9812+(I+.5)*Dx,Y=-.4572+(J+.5)*Dy;
  if(!Inside(X,Y,1,1)) continue;
  double Lo=0,Hi=1;
  if(Inside(X,Y,.97,.75)) Hi=0;
  else for(int K=0;K<24;++K) {double T=(Lo+Hi)*.5;if(Inside(X,Y,.97+.03*T,.75+.25*T)) Hi=T;else Lo=T;}
  Cells.Add({FVector(X,Y,0),-.2+.55*Hi,.35,Dx*Dy});Area+=Dx*Dy;
 }
}
void UBoatDynamicsComponent::BeginPlay() {
 Super::BeginPlay();
 Hull=Cast<UStaticMeshComponent>(GetOwner()->GetRootComponent());
 if(!Hull || BaseMassKg<=0 || PayloadMassKg<0) {UE_LOG(LogTemp,Error,TEXT("Boat requires static mesh root and valid mass"));SetComponentTickEnabled(false);return;}
 Mass=BaseMassKg+PayloadMassKg;
 ComM=(BaseMassKg*BaseCenterM+PayloadMassKg*PayloadCenterM)/Mass;
 const auto S=PayloadSizeM;
 Inertia=BaseInertiaKgM2+PayloadMassKg/12*FVector(S.Y*S.Y+S.Z*S.Z,S.X*S.X+S.Z*S.Z,S.X*S.X+S.Y*S.Y)
  +ParallelAxis(BaseMassKg,BaseCenterM-ComM)+ParallelAxis(PayloadMassKg,PayloadCenterM-ComM);
 Hull->SetMobility(EComponentMobility::Movable);
 Hull->SetCollisionProfileName(TEXT("PhysicsActor"));
 Hull->SetMassOverrideInKg(NAME_None,Mass,true);
 Hull->SetLinearDamping(0);Hull->SetAngularDamping(0);
 Hull->SetEnableGravity(false); // Gravity included once in coupled effective-mass force calculation.
 Hull->BodyInstance.bUseCCD=true;
 Hull->BodyInstance.bGyroscopicTorqueEnabled=false; // Euler term is included explicitly below.
 // Decorative parts follow the single root body and do not add collision/mass.
 TArray<UStaticMeshComponent*> Parts;GetOwner()->GetComponents(Parts);
 for(auto* P:Parts) if(P!=Hull) {P->SetSimulatePhysics(false);P->SetCollisionEnabled(ECollisionEnabled::NoCollision);}
 Hull->SetSimulatePhysics(true);Hull->WakeAllRigidBodies();
 StartTransform=Hull->GetComponentTransform();BuildCells();
 if(bEnableCamera) {
  Camera=NewObject<UBoatCameraSensor>(GetOwner(),TEXT("BoatLabCamera"));
  Camera->Width=CameraWidth;Camera->Height=CameraHeight;Camera->HorizontalFovDeg=CameraHorizontalFovDeg;
  Camera->PitchDownDeg=CameraPitchDownDeg;Camera->bDepth=bCameraDepth;
  Camera->SetupAttachment(Hull);
  Camera->SetRelativeLocation(CameraMountCm);
  Camera->RegisterComponent();
  Camera->Init();
 }
 Socket=FUdpSocketBuilder(TEXT("BoatPhysicsCommands")).AsNonBlocking().AsReusable().BoundToAddress(FIPv4Address::InternalLoopback).BoundToPort(CommandPort).WithReceiveBufferSize(65536);
 if(!Socket) UE_LOG(LogTemp,Error,TEXT("Boat UDP bind failed on %d"),CommandPort);
 for(TActorIterator<AActor> It(GetWorld());It;++It) {
  if(!It->ActorHasTag(TEXT("BoatWaterSurface"))) continue;
  if(auto* Water=Cast<UStaticMeshComponent>(It->GetRootComponent())) {
   Water->SetCollisionEnabled(ECollisionEnabled::NoCollision);
   WaterMaterial=Water->CreateDynamicMaterialInstance(0);
  }
 }
 Activate(true);
 SetAsyncPhysicsTickEnabled(true);
}
void UBoatDynamicsComponent::EndPlay(const EEndPlayReason::Type Reason) {
 SetAsyncPhysicsTickEnabled(false);
 if(Socket) {Socket->Close();ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);Socket=nullptr;}
 Super::EndPlay(Reason);
}
double UBoatDynamicsComponent::WaterHeight(const FVector& P,double Time) const {
 return WaveAmplitudeM*FMath::Sin(2*PI*((P.X*FMath::Cos(WaveDirectionRad)+P.Y*FMath::Sin(WaveDirectionRad))/FMath::Max(.1,WaveLengthM)-Time/FMath::Max(.1,WavePeriodS)));
}
double UBoatDynamicsComponent::WaterVerticalSpeed(const FVector& P,double Time) const {
 return -WaveAmplitudeM*2*PI/FMath::Max(.1,WavePeriodS)*FMath::Cos(2*PI*((P.X*FMath::Cos(WaveDirectionRad)+P.Y*FMath::Sin(WaveDirectionRad))/FMath::Max(.1,WaveLengthM)-Time/FMath::Max(.1,WavePeriodS)));
}
double UBoatDynamicsComponent::MotorCurve(double U) const {
 if(FMath::Abs(U)<=Deadband) return 0;
 double A=(FMath::Abs(U)-Deadband)/(1-Deadband);
 if(bSteppedMotor) {int N=U>0?5:3;A=FMath::Clamp(FMath::RoundToDouble(A*N),1.,double(N))/N;}
 // Unmeasured partial/reverse curves: monotonic starting estimates, replace using bollard tests.
 return (U>0?MaxForwardN:-MaxReverseN)*A;
}
void UBoatDynamicsComponent::AsyncPhysicsTickComponent(float Dt,float SimTime) {
 if(!Hull || !Hull->GetBodyInstance() || Dt<=0) return;
 auto Handle=Hull->GetBodyInstance()->GetPhysicsActorHandle();
 if(!Handle) return;
 auto* P=Handle->GetPhysicsThreadAPI();if(!P) return;
 Clock=SimTime;PhysicsDt=Dt;++Steps;
 if(!bInitialized) {
  // Small continuous fluid forces must not be suppressed by rigid-body sleep.
  P->SetSleepType(Chaos::ESleepType::NeverSleep);
  P->SetM(Mass);P->SetInvM(1/Mass);
  P->SetCenterOfMass(ComM*100);P->SetRotationOfMass(FQuat::Identity);
  const FVector I=Inertia*10000;
  P->SetI(Chaos::FVec3f(I));P->SetInvI(Chaos::FVec3f(Divide(FVector(1),I)));
  bInitialized=true;
 }
 if(bReset) {
  const FTransform& Target=bResetToPose?PendingPose:StartTransform;
  P->SetX(Target.GetLocation());P->SetR(Target.GetRotation());P->SetV(FVector::ZeroVector);P->SetW(FVector::ZeroVector);
  bResetToPose=false;
  TL=TR=Left=Right=0;LastCommand=-100;Delay.Reset();bReset=false;
 }
 const FQuat Q=P->R();const FVector Origin=FVector(P->X())*.01;
 const FVector CoM=Origin+Q.RotateVector(ComM),V=FVector(P->V())*.01,W=P->W();
 const FVector VB=Q.UnrotateVector(V),WB=Q.UnrotateVector(W),Rel=Q.UnrotateVector(V-CurrentMps);
 bExpired=Clock-LastCommand>CommandTimeoutS;
 Delay.Add({Clock,bExpired?0:Left,bExpired?0:Right});
 while(Delay.Num()>1 && Delay[1].Time<=Clock-CommandDelayS) Delay.RemoveAt(0,1,EAllowShrinking::No);
 double UL=0,UR=0;
 if(!bExpired && Delay.Num() && Delay[0].Time<=Clock-CommandDelayS) {UL=Delay[0].L;UR=Delay[0].R;}
 Voltage=FMath::Max(0.,BatteryOpenCircuitV-BatteryResistanceOhm*EstimatedMaxCurrentA*(FMath::Abs(UL)+FMath::Abs(UR)));
 const double VoltScale=FMath::Square(Voltage/FMath::Max(1.,NominalVoltage));
 const double Alpha=1-FMath::Exp(-Dt/FMath::Max(.001,MotorTimeConstantS));
 TL+=(MotorCurve(UL)*VoltScale-TL)*Alpha;TR+=(MotorCurve(UR)*VoltScale-TR)*Alpha;
 FVector F(0,0,-Mass*G),T=FVector::ZeroVector;
 auto Apply=[&](FVector Force,FVector Point) {F+=Force;T+=FVector::CrossProduct(Point-CoM,Force);};
 Volume=0;bDeckAwash=false;
 for(const auto& C:Cells) {
  const FVector A=Origin+Q.RotateVector(C.Center+FVector(0,0,C.Bottom));
  const FVector B=Origin+Q.RotateVector(C.Center+FVector(0,0,C.Top));
  const double DA=WaterHeight(A,Clock)-A.Z,DB=WaterHeight(B,Clock)-B.Z;
  double Fraction=0,Mid=0;
  if(DA>=0 && DB>=0) {Fraction=1;Mid=.5;bDeckAwash=true;}
  else if(DA>0) {Fraction=FMath::Clamp(DA/(DA-DB),0.,1.);Mid=Fraction*.5;}
  else if(DB>0) {Fraction=FMath::Clamp(DB/(DB-DA),0.,1.);Mid=1-Fraction*.5;}
  if(Fraction<=0) continue;
  const double DVol=C.Area*(C.Top-C.Bottom)*Fraction;Volume+=DVol;
  const FVector Point=FMath::Lerp(A,B,Mid);
  Apply(FVector(0,0,WaterDensityKgM3*G*DVol),Point);
  const double Vz=(V+FVector::CrossProduct(W,Point-CoM)).Z-WaterVerticalSpeed(Point,Clock);
  // Viscous heave/roll/pitch damping distributed over wetted columns.
  Apply(FVector(0,0,-HeaveDampingNsM*C.Area/Area*Vz*FMath::Min(1.,Fraction*4)),Point);
 }
 const double Wet=FMath::Clamp(Volume/(Mass/WaterDensityKgM3),0.,1.);
 const FVector Drag=-(LinearDrag*Rel+QuadraticDrag*Rel*Abs(Rel))*Wet;
 F+=Q.RotateVector(Drag);
 T+=Q.RotateVector(-(AngularLinearDrag*WB+AngularQuadraticDrag*WB*Abs(WB))*Wet);
 const FVector WR=Q.UnrotateVector(WindMps-V);
 Apply(Q.RotateVector(.5*1.225*WindAreaM2*WR*Abs(WR)),Origin+Q.RotateVector(WindCenterM));
 for(int I=0;I<2;++I) {
  const FVector Point=Origin+Q.RotateVector(FVector(MotorXM,(I==0?-1:1)*MotorSpacingM*.5,MotorZM));
  const double Immersion=FMath::Clamp((WaterHeight(Point,Clock)-Point.Z)/.08,0.,1.);
  // Forward-speed unloading approximation (unmeasured), retaining reverse braking.
  const double Thrust=I==0?TL:TR;
  const double Unload=FMath::Clamp(1-.12*FMath::Max(0.,FMath::Sign(Thrust)*Rel.X),.2,1.);
  Apply(Q.RotateVector(FVector(Thrust*Immersion*Unload,0,0)),Point);
 }
 const FVector MA=AddedMassKg*Wet,JA=AddedInertiaKgM2*Wet;
 const FVector FB=Q.UnrotateVector(F),TB=Q.UnrotateVector(T);
 // Diagonal low-speed added-mass model; includes body-frame Coriolis terms.
 // Current is uniform; spatial gradients and wave radiation memory are not modeled.
 const FVector Acc=Divide(FB+MA*FVector::CrossProduct(WB,Rel)-FVector::CrossProduct(WB,MA*Rel),FVector(Mass)+MA);
 const FVector Ang=Divide(TB-FVector::CrossProduct(WB,(Inertia+JA)*WB)-FVector::CrossProduct(Rel,MA*Rel),Inertia+JA);
 P->AddForce(Q.RotateVector(Acc)*Mass*100);
 P->AddTorque(Q.RotateVector(Inertia*Ang)*10000);
 Position=Origin;Velocity=V;Omega=WB;Attitude=Q.Rotator();
}
TSharedPtr<FJsonObject> UBoatDynamicsComponent::State() const {
 auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("schema"),TEXT("boat_physics_v1"));
 O->SetStringField(TEXT("state_source"),TEXT("simulator_ground_truth_not_navigation_estimate"));
 O->SetStringField(TEXT("calibration"),TEXT("uncalibrated_initial_estimates"));
 O->SetNumberField(TEXT("time_s"),Clock);O->SetNumberField(TEXT("physics_dt_s"),PhysicsDt);O->SetNumberField(TEXT("physics_steps"),double(Steps));
 O->SetNumberField(TEXT("mass_kg"),Mass);O->SetNumberField(TEXT("payload_kg"),PayloadMassKg);
 O->SetArrayField(TEXT("center_of_mass_body_m"),Array(ComM));O->SetArrayField(TEXT("inertia_kgm2"),Array(Inertia));
 O->SetArrayField(TEXT("position_ue_m"),Array(Position));O->SetArrayField(TEXT("velocity_ue_mps"),Array(Velocity));
 O->SetArrayField(TEXT("angular_velocity_body_radps"),Array(Omega));
 O->SetArrayField(TEXT("roll_pitch_yaw_deg"),Array(FVector(Attitude.Roll,Attitude.Pitch,Attitude.Yaw)));
 O->SetNumberField(TEXT("displacement_m3"),Volume);O->SetNumberField(TEXT("left_thrust_n"),TL);O->SetNumberField(TEXT("right_thrust_n"),TR);
 O->SetNumberField(TEXT("battery_voltage"),Voltage);O->SetBoolField(TEXT("command_expired"),bExpired);O->SetBoolField(TEXT("deck_awash"),bDeckAwash);
 O->SetBoolField(TEXT("stepped_motor"),bSteppedMotor);O->SetNumberField(TEXT("wave_amplitude_m"),WaveAmplitudeM);
 O->SetStringField(TEXT("plugin_version"),TEXT("boatlab_v2"));O->SetBoolField(TEXT("camera_available"),Camera!=nullptr);
 return O;
}
void UBoatDynamicsComponent::Poll() {
 if(!Socket) return;uint32 Size;
 // Bounded workload; local simulator interface only, never hardware serial.
 for(int Packet=0;Packet<32 && Socket->HasPendingData(Size);++Packet) {
  TArray<uint8> Data;Data.SetNumUninitialized(FMath::Min(Size,8192u)+1);int32 Read=0;
  auto Sender=ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
  if(!Socket->RecvFrom(Data.GetData(),Data.Num()-1,Read,*Sender)) continue;Data[Read]=0;
  TSharedPtr<FJsonObject> M;auto Reader=TJsonReaderFactory<>::Create(FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Data.GetData()))));
  if(!FJsonSerializer::Deserialize(Reader,M)||!M) continue;
  bool Accepted=true;double L=0,R=0;
  if(M->HasField(TEXT("left"))||M->HasField(TEXT("right"))) {
   Accepted=M->TryGetNumberField(TEXT("left"),L)&&M->TryGetNumberField(TEXT("right"),R)&&FMath::IsFinite(L)&&FMath::IsFinite(R)&&FMath::Abs(L)<=1&&FMath::Abs(R)<=1;
   if(Accepted) {Left=L;Right=R;LastCommand=Clock;}
  }
  bool Reset=false;if(M->TryGetBoolField(TEXT("reset"),Reset)&&Reset) bReset=true;
  const TArray<TSharedPtr<FJsonValue>>* Pose=nullptr;
  if(M->TryGetArrayField(TEXT("reset_pose"),Pose) && Pose->Num()==3) {
   // [x_m, y_m, yaw_deg] in UE axes; keeps the Play-start height so the hull settles naturally
   const double X=(*Pose)[0]->AsNumber(),Y=(*Pose)[1]->AsNumber(),Yaw=(*Pose)[2]->AsNumber();
   if(FMath::IsFinite(X)&&FMath::IsFinite(Y)&&FMath::IsFinite(Yaw)) {
    PendingPose=FTransform(FRotator(0,Yaw,0),FVector(X*100,Y*100,StartTransform.GetLocation().Z));
    bResetToPose=true;bReset=true;
   } else Accepted=false;
  }
  if(bReset) HandleVisual(nullptr);
  const TSharedPtr<FJsonObject>* Visual=nullptr;
  if(M->TryGetObjectField(TEXT("visual"),Visual)) HandleVisual(*Visual);
  const TSharedPtr<FJsonObject>* E=nullptr;
  if(M->TryGetObjectField(TEXT("environment"),E)) {
   // Validate before changing any parameter. All fields mandatory to avoid partial updates.
   double Cx=0,Cy=0,Wx=0,Wy=0,A=0,Period=0,Length=0,Dir=0;
   bool Valid=(*E)->TryGetNumberField(TEXT("current_x"),Cx)&&(*E)->TryGetNumberField(TEXT("current_y"),Cy)&&(*E)->TryGetNumberField(TEXT("wind_x"),Wx)&&(*E)->TryGetNumberField(TEXT("wind_y"),Wy)&&(*E)->TryGetNumberField(TEXT("wave_amplitude"),A)&&(*E)->TryGetNumberField(TEXT("wave_period"),Period)&&(*E)->TryGetNumberField(TEXT("wave_length"),Length)&&(*E)->TryGetNumberField(TEXT("wave_direction"),Dir);
   if(Valid) for(double N:{Cx,Cy,Wx,Wy,A,Period,Length,Dir}) Valid&=FMath::IsFinite(N);
   Valid=Valid&&FMath::Abs(Cx)<=2&&FMath::Abs(Cy)<=2&&FMath::Abs(Wx)<=20&&FMath::Abs(Wy)<=20&&A>=0&&A<=.15&&Period>=1&&Length>=2;
   if(Valid) {CurrentMps=FVector(Cx,Cy,0);WindMps=FVector(Wx,Wy,0);WaveAmplitudeM=A;WavePeriodS=Period;WaveLengthM=Length;WaveDirectionRad=Dir;} else Accepted=false;
  }
  TSharedPtr<FJsonObject> O;
  bool Rgb=false;double Sequence=0,ChunkIndex=0;
  if(M->TryGetBoolField(TEXT("rgb"),Rgb)&&Rgb) {
   O=State();
   const bool Captured=Camera&&Camera->Capture(Clock);
   O->SetBoolField(TEXT("valid"),Captured);
   if(Camera) {O->SetObjectField(TEXT("rgb"),Camera->RgbMeta());O->SetObjectField(TEXT("depth"),Camera->DepthMeta());}
  } else if(Camera&&M->TryGetNumberField(TEXT("rgb_sequence"),Sequence)&&M->TryGetNumberField(TEXT("rgb_chunk"),ChunkIndex)) {
   O=Camera->Chunk(false,Sequence,int32(FMath::Clamp(ChunkIndex,-1.,100000.)));
  } else if(Camera&&M->TryGetNumberField(TEXT("depth_sequence"),Sequence)&&M->TryGetNumberField(TEXT("depth_chunk"),ChunkIndex)) {
   O=Camera->Chunk(true,Sequence,int32(FMath::Clamp(ChunkIndex,-1.,100000.)));
  } else O=State();
  O->SetBoolField(TEXT("accepted"),Accepted);
  FString Reply;auto Writer=TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Reply);FJsonSerializer::Serialize(O.ToSharedRef(),Writer);
  FTCHARToUTF8 Bytes(*Reply);int32 Sent;Socket->SendTo(reinterpret_cast<const uint8*>(Bytes.Get()),Bytes.Length(),Sent,*Sender);
 }
}
void UBoatDynamicsComponent::TickComponent(float Dt,ELevelTick Type,FActorComponentTickFunction* Fn) {
 Super::TickComponent(Dt,Type,Fn);Poll();
 if(WaterMaterial) {
  WaterMaterial->SetScalarParameterValue(TEXT("PhysicsTimeS"),Clock);
  WaterMaterial->SetScalarParameterValue(TEXT("WaveAmplitudeCm"),WaveAmplitudeM*100);
  WaterMaterial->SetScalarParameterValue(TEXT("WavePeriodS"),WavePeriodS);
  WaterMaterial->SetScalarParameterValue(TEXT("WaveLengthM"),WaveLengthM);
  WaterMaterial->SetScalarParameterValue(TEXT("WaveDirectionRad"),WaveDirectionRad);
 }
 if(bKeyboardControl) if(auto* PC=GetWorld()->GetFirstPlayerController()) {
  double L=(PC->IsInputKeyDown(EKeys::I)?1.:0)-(PC->IsInputKeyDown(EKeys::K)?1.:0);
  double R=(PC->IsInputKeyDown(EKeys::O)?1.:0)-(PC->IsInputKeyDown(EKeys::L)?1.:0);
  if(L||R||PC->IsInputKeyDown(EKeys::SpaceBar)) {Left=L;Right=R;LastCommand=Clock;if(PC->IsInputKeyDown(EKeys::SpaceBar)) Left=Right=0;}
 }
}


void UBoatDynamicsComponent::HandleVisual(const TSharedPtr<FJsonObject>& V) {
 // Visual feedback only (no physics): payload landing markers and hiding the recovered case.
 FString Kind=TEXT("clear");
 if(V.IsValid()) V->TryGetStringField(TEXT("kind"),Kind);
 if(Kind==TEXT("clear")) {
  for(AStaticMeshActor* A:Markers) if(IsValid(A)) A->Destroy();
  Markers.Reset();
  for(auto& W:HiddenByTag) if(W.IsValid()) {W->SetActorHiddenInGame(false);W->SetActorEnableCollision(true);}
  HiddenByTag.Reset();
  return;
 }
 if(Kind==TEXT("hide_tag")) {
  FString Tag;if(!V->TryGetStringField(TEXT("tag"),Tag)) return;
  for(TActorIterator<AActor> It(GetWorld());It;++It) if(It->ActorHasTag(FName(*Tag))) {
   It->SetActorHiddenInGame(true);It->SetActorEnableCollision(false);HiddenByTag.Add(*It);
  }
  return;
 }
 if(Kind==TEXT("marker")) {
  double X=0,Y=0,Z=.1,Size=.25,Cr=1,Cg=1,Cb=0;
  V->TryGetNumberField(TEXT("x"),X);V->TryGetNumberField(TEXT("y"),Y);V->TryGetNumberField(TEXT("z"),Z);
  V->TryGetNumberField(TEXT("size"),Size);V->TryGetNumberField(TEXT("r"),Cr);V->TryGetNumberField(TEXT("g"),Cg);V->TryGetNumberField(TEXT("b"),Cb);
  UStaticMesh* Mesh=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Sphere.Sphere"));
  if(!Mesh || Markers.Num()>64) return;
  FActorSpawnParameters Params;Params.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AStaticMeshActor* A=GetWorld()->SpawnActor<AStaticMeshActor>(FVector(X*100,Y*100,Z*100),FRotator::ZeroRotator,Params);
  if(!A) return;
  A->GetStaticMeshComponent()->SetMobility(EComponentMobility::Movable);
  A->GetStaticMeshComponent()->SetStaticMesh(Mesh);
  A->GetStaticMeshComponent()->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  A->SetActorScale3D(FVector(Size));
  if(UMaterialInterface* Base=LoadObject<UMaterialInterface>(nullptr,TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"))) {
   UMaterialInstanceDynamic* Mid=UMaterialInstanceDynamic::Create(Base,A);
   Mid->SetVectorParameterValue(TEXT("Color"),FLinearColor(Cr,Cg,Cb));
   A->GetStaticMeshComponent()->SetMaterial(0,Mid);
  }
  A->Tags.Add(TEXT("boatlab_marker"));
  Markers.Add(A);
 }
}
