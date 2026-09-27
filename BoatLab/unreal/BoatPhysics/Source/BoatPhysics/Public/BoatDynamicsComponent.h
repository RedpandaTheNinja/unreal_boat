#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "BoatDynamicsComponent.generated.h"
class UStaticMeshComponent;
class FSocket;
class UMaterialInstanceDynamic;
class FJsonObject;
class UBoatCameraSensor;
class AStaticMeshActor;

// SI internally; Unreal body convention X forward, Y starboard, Z up.
// Set parameters before PIE. Runtime commands/environment use the UDP interface.
UCLASS(ClassGroup=(Boat), meta=(BlueprintSpawnableComponent))
class BOATPHYSICS_API UBoatDynamicsComponent : public UActorComponent {
 GENERATED_BODY()
public:
 UBoatDynamicsComponent();
 virtual void BeginPlay() override;
 virtual void EndPlay(const EEndPlayReason::Type Reason) override;
 virtual void TickComponent(float Dt,ELevelTick Type,FActorComponentTickFunction* Fn) override;
 virtual void AsyncPhysicsTickComponent(float Dt,float SimTime) override;
 UPROPERTY(EditAnywhere,Category="Boat|Mass",meta=(ClampMin="1")) double BaseMassKg=58.9670081;
 UPROPERTY(EditAnywhere,Category="Boat|Mass",meta=(ClampMin="0")) double PayloadMassKg=113.3980925;
 UPROPERTY(EditAnywhere,Category="Boat|Mass") FVector BaseCenterM=FVector(-.15,0,.05);
 UPROPERTY(EditAnywhere,Category="Boat|Mass") FVector PayloadCenterM=FVector(0,0,.18);
 UPROPERTY(EditAnywhere,Category="Boat|Mass") FVector BaseInertiaKgM2=FVector(5.6,78.7,81.3);
 UPROPERTY(EditAnywhere,Category="Boat|Mass") FVector PayloadSizeM=FVector(1.2,.6,.4);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") FVector AddedMassKg=FVector(12,90,120);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") FVector AddedInertiaKgM2=FVector(10,40,60);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") FVector LinearDrag=FVector(25,100,0);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") FVector QuadraticDrag=FVector(45,220,0);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") FVector AngularLinearDrag=FVector(40,100,55);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") FVector AngularQuadraticDrag=FVector(30,80,100);
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") double HeaveDampingNsM=1700;
 UPROPERTY(EditAnywhere,Category="Boat|Hydro") double WaterDensityKgM3=1000;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double MotorSpacingM=.6096;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double MotorXM=-2.14;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double MotorZM=-.35;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double MaxForwardN=133.44664846;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double MaxReverseN=80.06798908;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double MotorTimeConstantS=.35;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double CommandDelayS=.08;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double CommandTimeoutS=.5;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double Deadband=.06;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") bool bSteppedMotor=false;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double NominalVoltage=12;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double BatteryOpenCircuitV=12;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double BatteryResistanceOhm=0;
 UPROPERTY(EditAnywhere,Category="Boat|Motor") double EstimatedMaxCurrentA=30;
 UPROPERTY(EditAnywhere,Category="Boat|Water") FVector CurrentMps=FVector::ZeroVector;
 UPROPERTY(EditAnywhere,Category="Boat|Water") FVector WindMps=FVector::ZeroVector;
 UPROPERTY(EditAnywhere,Category="Boat|Water") FVector WindAreaM2=FVector(.3,1.2,.1);
 UPROPERTY(EditAnywhere,Category="Boat|Water") FVector WindCenterM=FVector(0,0,.3);
 UPROPERTY(EditAnywhere,Category="Boat|Water") double WaveAmplitudeM=0;
 UPROPERTY(EditAnywhere,Category="Boat|Water") double WavePeriodS=3;
 UPROPERTY(EditAnywhere,Category="Boat|Water") double WaveLengthM=8;
 UPROPERTY(EditAnywhere,Category="Boat|Water") double WaveDirectionRad=0;
 UPROPERTY(EditAnywhere,Category="Boat|Interface") int32 CommandPort=7450;
 UPROPERTY(EditAnywhere,Category="Boat|Interface") bool bKeyboardControl=true;
 // BoatLab v2: forward camera (RGB + ideal depth) served over the same UDP port.
 UPROPERTY(EditAnywhere,Category="Boat|Camera") bool bEnableCamera=true;
 UPROPERTY(EditAnywhere,Category="Boat|Camera") FVector CameraMountCm=FVector(150,0,80);
 UPROPERTY(EditAnywhere,Category="Boat|Camera") float CameraPitchDownDeg=8.f;
 UPROPERTY(EditAnywhere,Category="Boat|Camera") float CameraHorizontalFovDeg=90.f;
 UPROPERTY(EditAnywhere,Category="Boat|Camera") int32 CameraWidth=640;
 UPROPERTY(EditAnywhere,Category="Boat|Camera") int32 CameraHeight=480;
 UPROPERTY(EditAnywhere,Category="Boat|Camera") bool bCameraDepth=true;
private:
 struct FCell { FVector Center; double Bottom,Top,Area; };
 struct FCommand { double Time,L,R; };
 UPROPERTY() UStaticMeshComponent* Hull=nullptr;
 UPROPERTY() UMaterialInstanceDynamic* WaterMaterial=nullptr;
 UPROPERTY() UBoatCameraSensor* Camera=nullptr;
 UPROPERTY() TArray<AStaticMeshActor*> Markers;
 TArray<TWeakObjectPtr<AActor>> HiddenByTag;
 FTransform PendingPose;
 bool bResetToPose=false;
 void HandleVisual(const TSharedPtr<FJsonObject>& V);
 TArray<FCell> Cells;
 TArray<FCommand> Delay;
 FSocket* Socket=nullptr;
 FTransform StartTransform;
 FVector ComM=FVector::ZeroVector,Inertia=FVector::ZeroVector;
 double Mass=0,Area=0,Clock=0,LastCommand=-100,Left=0,Right=0,TL=0,TR=0;
 double Volume=0,Voltage=12,PhysicsDt=0;
 uint64 Steps=0;
 bool bInitialized=false,bReset=false,bExpired=true,bDeckAwash=false;
 FVector Position=FVector::ZeroVector,Velocity=FVector::ZeroVector,Omega=FVector::ZeroVector;
 FRotator Attitude=FRotator::ZeroRotator;
 void BuildCells();
 void Poll();
 TSharedPtr<FJsonObject> State() const;
 double WaterHeight(const FVector& P,double Time) const;
 double WaterVerticalSpeed(const FVector& P,double Time) const;
 double MotorCurve(double Command) const;
};

