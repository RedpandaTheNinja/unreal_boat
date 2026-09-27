// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
// Modified 2026: removed ROS dependencies, kept physical assembly and drive math.
// Licensed under Apache-2.0; see LICENSE and NOTICE.
#include "TurtlebotBurger.h"
#include "Components/StaticMeshComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Modules/ModuleManager.h"
IMPLEMENT_MODULE(FDefaultModuleImpl, RapyutaSimulationPlugins)

static void LockTranslation(UPhysicsConstraintComponent* C)
{
    C->SetLinearXLimit(LCM_Locked, 0);
    C->SetLinearYLimit(LCM_Locked, 0);
    C->SetLinearZLimit(LCM_Locked, 0);
    C->SetDisableCollision(true);
}

ATurtlebotBurgerBase::ATurtlebotBurgerBase()
{
    Base = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base"));
    SetRootComponent(Base);
    BaseMeshComp = Base;
    DefaultRoot = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultRoot"));
    DefaultRoot->SetupAttachment(Base);
    LidarSensor = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LidarSensor"));
    WheelLeft = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelLeft"));
    WheelRight = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelRight"));
    CasterBack = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CasterBack"));
    LidarComponent = CreateDefaultSubobject<URR2DLidarComponent>(TEXT("LidarComp"));
    LidarComponent->SetupAttachment(LidarSensor);
    Base_LidarSensor = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_LidarSensor"));
    Base_CasterBack = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_CasterBack"));
    for (auto* M : {Base, LidarSensor, WheelLeft, WheelRight, CasterBack})
        M->SetSimulatePhysics(true);
    Base_LidarSensor->ComponentName1.ComponentName = TEXT("Base");
    Base_LidarSensor->ComponentName2.ComponentName = TEXT("LidarSensor");
    Base_LidarSensor->SetRelativeLocation(FVector(0,0,17.2));
    LockTranslation(Base_LidarSensor);
    Base_LidarSensor->SetAngularSwing1Limit(ACM_Locked,0);
    Base_LidarSensor->SetAngularSwing2Limit(ACM_Locked,0);
    Base_LidarSensor->SetAngularTwistLimit(ACM_Locked,0);
    Base_CasterBack->ComponentName1.ComponentName = TEXT("Base");
    Base_CasterBack->ComponentName2.ComponentName = TEXT("CasterBack");
    Base_CasterBack->SetRelativeLocation(FVector(-4.9,0,-0.5));
    LockTranslation(Base_CasterBack);
    LidarSensor->SetupAttachment(Base_LidarSensor);
    CasterBack->SetupAttachment(Base_CasterBack);
    Base_LidarSensor->SetupAttachment(Base);
    Base_CasterBack->SetupAttachment(Base);
}

ATurtlebotBurger::ATurtlebotBurger()
{
    Base_WheelLeft = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelLeft"));
    Base_WheelRight = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelRight"));
    for (int32 Side=0; Side<2; ++Side)
    {
        auto* C = Side==0 ? Base_WheelLeft : Base_WheelRight;
        auto* W = Side==0 ? WheelLeft : WheelRight;
        const float Sign = Side==0 ? -1.f : 1.f;
        C->ComponentName1.ComponentName = TEXT("Base");
        C->ComponentName2.ComponentName = Side==0 ? TEXT("WheelLeft") : TEXT("WheelRight");
        C->SetRelativeLocation(FVector(3.2,Sign*8,2.3));
        C->SetRelativeRotation(FRotator(0,Sign*90,0));
        LockTranslation(C);
        C->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
        C->SetAngularDriveParams(MaxForce,MaxForce,MaxForce);
        C->SetAngularVelocityDriveTwistAndSwing(true,false);
        C->SetAngularSwing1Limit(ACM_Locked,0);
        C->SetAngularSwing2Limit(ACM_Locked,0);
        W->SetupAttachment(C);
        W->SetRelativeRotation(FRotator(0,Sign*90,0));
        C->SetupAttachment(Base);
    }
}

void ATurtlebotBurger::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    // Upstream DifferentialDriveComponent::SetWheels overrides position strength.
    for (auto* C : {Base_WheelLeft, Base_WheelRight})
    {
        C->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
        C->SetAngularVelocityDriveTwistAndSwing(true,false);
        C->SetAngularDriveParams(0,MaxForce,MaxForce);
    }
}

void ATurtlebotBurger::SetVelocitySI(float ForwardMps, float YawRadps)
{
    const float V = ForwardMps*100.f;
    const float UEYaw = -YawRadps;
    const float Perimeter = 2.f*PI*WheelRadius;
    Base_WheelLeft->SetAngularVelocityTarget(FVector((V+UEYaw*WheelSeparationHalf)/Perimeter,0,0));
    Base_WheelRight->SetAngularVelocityTarget(FVector(-(V-UEYaw*WheelSeparationHalf)/Perimeter,0,0));
}
