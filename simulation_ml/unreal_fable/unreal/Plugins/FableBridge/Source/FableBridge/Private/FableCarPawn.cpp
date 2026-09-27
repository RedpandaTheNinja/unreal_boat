#include "FableCarPawn.h"
#include "FableConv.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Components/SkeletalMeshComponent.h"

// ------------------------------------------------------------------ wheels

UFableFrontWheel::UFableFrontWheel()
{
	AxleType = EAxleType::Front;
	bAffectedBySteering = true;
	bAffectedByEngine = false;
	bAffectedByBrake = true;
	bAffectedByHandbrake = false;
	MaxSteerAngle = 25.f;
	WheelRadius = 5.4f;              // cm; overwritten by params
	WheelWidth = 2.5f;
	FrictionForceMultiplier = 2.0f;
	SuspensionMaxRaise = 2.f;
	SuspensionMaxDrop = 2.f;
	SpringRate = 60.f;
	SpringPreload = 20.f;
	SuspensionDampingRatio = 0.7f;
	MaxBrakeTorque = 30.f;
}

UFableRearWheel::UFableRearWheel()
{
	AxleType = EAxleType::Rear;
	bAffectedBySteering = false;
	bAffectedByEngine = true;
	bAffectedByBrake = true;
	bAffectedByHandbrake = true;
	WheelRadius = 5.4f;
	WheelWidth = 2.5f;
	FrictionForceMultiplier = 2.0f;
	SuspensionMaxRaise = 2.f;
	SuspensionMaxDrop = 2.f;
	SpringRate = 60.f;
	SpringPreload = 20.f;
	SuspensionDampingRatio = 0.7f;
	MaxBrakeTorque = 30.f;
}

// ------------------------------------------------------------------ pawn

AFableCarPawn::AFableCarPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	if (UChaosWheeledVehicleMovementComponent* M = Chaos())
	{
		M->WheelSetups.SetNum(4);
		M->WheelSetups[0].WheelClass = UFableFrontWheel::StaticClass();
		M->WheelSetups[1].WheelClass = UFableFrontWheel::StaticClass();
		M->WheelSetups[2].WheelClass = UFableRearWheel::StaticClass();
		M->WheelSetups[3].WheelClass = UFableRearWheel::StaticClass();
		M->bMechanicalSimEnabled = true;
		M->EngineSetup.MaxRPM = 8000.f;
		M->EngineSetup.EngineIdleRPM = 200.f;
		M->EngineSetup.MaxTorque = 1.2f;                    // Nm; overwritten by params
		M->EngineSetup.EngineBrakeEffect = 0.02f;
		M->TransmissionSetup.bUseAutomaticGears = true;
		M->TransmissionSetup.bUseAutoReverse = true;
		M->TransmissionSetup.FinalRatio = 1.f;
		M->TransmissionSetup.ForwardGearRatios = { 1.f };
		M->TransmissionSetup.ReverseGearRatios = { 1.f };
		M->TransmissionSetup.GearChangeTime = 0.f;
		M->SteeringSetup.SteeringType = ESteeringType::Ackermann;
		M->SteeringSetup.AngleRatio = 1.f;
		M->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;
	}
	if (USkeletalMeshComponent* SM = GetMesh())
	{
		SM->SetNotifyRigidBodyCollision(true);
		SM->SetCollisionProfileName(TEXT("Vehicle"));
	}
}

UChaosWheeledVehicleMovementComponent* AFableCarPawn::Chaos() const
{
	return Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
}

void AFableCarPawn::BeginPlay()
{
	Super::BeginPlay();
	FableApplyParams(Params);
}

void AFableCarPawn::ConfigureChaos()
{
	UChaosWheeledVehicleMovementComponent* M = Chaos();
	if (!M) return;

	M->Mass = Params.MassKg;
	M->EngineSetup.MaxTorque = Params.MotorTorqueNm;
	M->DifferentialSetup.DifferentialType =
		Params.Drive == TEXT("awd") ? EVehicleDifferential::AllWheelDrive :
		Params.Drive == TEXT("fwd") ? EVehicleDifferential::FrontWheelDrive : EVehicleDifferential::RearWheelDrive;

	const float r_cm = Params.WheelRadiusM * Fable::M2CM;
	const float halfL = Params.WheelbaseM * 0.5f * Fable::M2CM;
	const float halfT = Params.TrackWidthM * 0.5f * Fable::M2CM;
	const FVector Offs[4] = { FVector(halfL, -halfT, 0), FVector(halfL, halfT, 0), FVector(-halfL, -halfT, 0), FVector(-halfL, halfT, 0) };
	for (int32 i = 0; i < 4 && i < M->WheelSetups.Num(); ++i)
	{
		M->WheelSetups[i].BoneName = WheelBoneNames.IsValidIndex(i) ? WheelBoneNames[i] : NAME_None;
		M->WheelSetups[i].AdditionalOffset = M->WheelSetups[i].BoneName.IsNone() ? Offs[i] : FVector::ZeroVector;
	}
	// live wheel instances (exist after the physics state is created)
	for (UChaosVehicleWheel* W : M->Wheels)
	{
		if (!W) continue;
		W->WheelRadius = r_cm;
		W->FrictionForceMultiplier = 2.0f * Params.TireFriction;
		if (W->bAffectedBySteering) W->MaxSteerAngle = FMath::RadiansToDegrees(Params.MaxSteerRad);
		W->MaxBrakeTorque = Params.MaxBrakeMps2 * Params.MassKg * Params.WheelRadiusM / 4.f * 1.5f;   // Nm, with margin
	}
	if (USkeletalMeshComponent* SM = GetMesh())
	{
		SM->SetMassOverrideInKg(NAME_None, Params.MassKg, true);
		SM->SetCenterOfMass(Fable::BodyRosToUe(Params.ComOffsetM) * Fable::M2CM);
	}
	M->RecreatePhysicsState();
}

void AFableCarPawn::FableApplyParams(const FFableVehicleParams& P)
{
	Params = P;
	ConfigureChaos();
	const int32 n = FMath::Max(1, FMath::RoundToInt(P.ActuatorDelayS * 50.f)) ;   // resized properly on first tick with real dt
	DelayLine.Init(FCmd(), n + 1);
	DelayHead = 0;
}

AFableCarPawn::FCmd AFableCarPawn::PushDelay(const FCmd& In)
{
	if (DelayLine.Num() == 0) return In;
	DelayLine[DelayHead] = In;
	DelayHead = (DelayHead + 1) % DelayLine.Num();
	return DelayLine[DelayHead];                 // oldest
}

void AFableCarPawn::FableApplyCommand(const TMap<FString, double>& U)
{
	FCmd C;
	if (const double* v = U.Find(TEXT("throttle"))) C.Throttle = FMath::Clamp(*v, -1.0, 1.0);
	if (const double* v = U.Find(TEXT("steer")))    C.Steer = FMath::Clamp(*v, -1.0, 1.0);
	if (const double* v = U.Find(TEXT("brake")))    C.Brake = FMath::Clamp(*v, 0.0, 1.0);
	Applied = PushDelay(C);
}

void AFableCarPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	UChaosWheeledVehicleMovementComponent* M = Chaos();
	if (!M) return;

	// keep the delay line sized to the actual fixed dt
	const int32 want = FMath::RoundToInt(Params.ActuatorDelayS / FMath::Max(DeltaSeconds, 1e-4f)) + 1;
	if (want != DelayLine.Num()) { DelayLine.Init(FCmd(), want); DelayHead = 0; }

	// --- steering servo: deadband, bias, slew
	double sc = Applied.Steer - Params.SteerBias;
	if (FMath::Abs(sc) < Params.SteerDeadband) sc = 0.0;
	const float target = FMath::Clamp((float)sc, -1.f, 1.f) * Params.MaxSteerRad;
	const float dmax = Params.SteerRateRadS * DeltaSeconds;
	SteerAngle += FMath::Clamp(target - SteerAngle, -dmax, dmax);
	// Chaos steering input is a fraction of MaxSteerAngle; ROS +steer = left = UE negative yaw
	M->SetSteeringInput(-SteerAngle / FMath::Max(Params.MaxSteerRad, 1e-3f));

	// --- governor: reproduce the mock's first-order speed response
	const float v = M->GetForwardSpeed() * Fable::CM2M;      // m/s, signed
	float a;
	if (Applied.Brake > 0.0)
	{
		M->SetThrottleInput(0.f);
		M->SetBrakeInput((float)Applied.Brake);
		return;
	}
	const float v_ref = (float)Applied.Throttle * (Applied.Throttle >= 0 ? Params.MaxSpeedMps : Params.MaxReverseMps);
	a = (v_ref - v) / FMath::Max(Params.AccelTimeConstantS, 1e-3f);
	a = FMath::Clamp(a, -Params.MaxBrakeMps2, Params.MaxAccelMps2);

	// required wheel force -> pedal fractions. With bUseAutoReverse, Chaos treats
	// "throttle" as the forward pedal and "brake" as the backward pedal: moving
	// forward the brake slows you, at rest it reverses, reversing it accelerates
	// backward. So the rule is simply: positive force -> throttle, negative -> brake.
	const float F_max = Params.MotorTorqueNm / FMath::Max(Params.WheelRadiusM, 1e-3f);   // N at the tyre, 1:1 gearing
	const float F = a * Params.MassKg;
	if (F >= 0.f)
	{
		M->SetThrottleInput(FMath::Clamp(F / F_max, 0.f, 1.f));
		M->SetBrakeInput(0.f);
	}
	else
	{
		M->SetThrottleInput(0.f);
		M->SetBrakeInput(v > 0.05f ? FMath::Clamp(-a / Params.MaxBrakeMps2, 0.f, 1.f)
		                           : FMath::Clamp(-F / F_max, 0.f, 1.f));
	}
}

void AFableCarPawn::FableResetTo(double x_m, double y_m, double yaw_rad)
{
	FVector Loc = Fable::RosToUe(x_m, y_m, 0.0);
	// drop from slightly above the ground under the spawn point
	FHitResult Hit;
	if (GetWorld()->LineTraceSingleByChannel(Hit, Loc + FVector(0, 0, 50000), Loc - FVector(0, 0, 50000), ECC_Visibility,
	                                         FCollisionQueryParams(SCENE_QUERY_STAT(FableSpawn), false, this)))
		Loc.Z = Hit.ImpactPoint.Z + Params.WheelRadiusM * Fable::M2CM + 5.f;
	SetActorLocationAndRotation(Loc, Fable::RosYawToRotator(yaw_rad), false, nullptr, ETeleportType::TeleportPhysics);
	if (USkeletalMeshComponent* SM = GetMesh())
	{
		SM->SetPhysicsLinearVelocity(FVector::ZeroVector);
		SM->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	}
	if (UChaosWheeledVehicleMovementComponent* M = Chaos())
	{
		M->SetThrottleInput(0.f); M->SetBrakeInput(0.f); M->SetSteeringInput(0.f);
		M->StopMovementImmediately();
	}
	SteerAngle = 0.f;
	Applied = FCmd();
	for (FCmd& c : DelayLine) c = FCmd();
	PrevVelWorld = FVector::ZeroVector;
	Contacts.Reset();
}

void AFableCarPawn::NotifyHit(UPrimitiveComponent* MyComp, AActor* Other, UPrimitiveComponent* OtherComp, bool bSelfMoved,
                              FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	Super::NotifyHit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
	if (!Other) return;
	// ground contact is not a collision: ignore hits whose normal points mostly up
	if (HitNormal.Z > 0.7f && Other->Tags.Contains(FName(TEXT("ground")))) return;
	if (Other->Tags.Contains(FName(TEXT("ground"))) || Other->Tags.Contains(FName(TEXT("terrain")))) return;
	bHitThisTick = true;
	Contacts.AddUnique(Other->Tags.Num() ? Other->Tags[0].ToString() : Other->GetName());
}

void AFableCarPawn::FableGetState(FFableState& Out, float Dt)
{
	const FTransform X = GetActorTransform();
	Fable::UeToRos(X.GetLocation(), Out.X, Out.Y, Out.Z);
	Fable::RotatorToRos(X.Rotator(), Out.Roll, Out.Pitch, Out.Yaw);

	const FVector Vw = GetVelocity();                                            // cm/s world
	const FVector Vb = X.InverseTransformVectorNoScale(Vw) * Fable::CM2M;        // UE body m/s
	Out.VelBody = Fable::BodyUeToRos(Vb);
	Out.Speed = Vb.Size2D();

	const FVector Aw = (Vw - PrevVelWorld) / FMath::Max(Dt, 1e-4f) * Fable::CM2M;   // m/s^2 world
	PrevVelWorld = Vw;
	const FVector Ab = X.InverseTransformVectorNoScale(Aw + FVector(0, 0, 981.f * Fable::CM2M));
	Out.AccBody = Fable::BodyUeToRos(Ab);

	if (USkeletalMeshComponent* SM = GetMesh())
		Out.OmegaBody = Fable::AngVelUeWorldToRosBody(SM->GetPhysicsAngularVelocityInDegrees(), X);

	Out.UApplied.Add(TEXT("throttle"), Applied.Throttle);
	Out.UApplied.Add(TEXT("steer"), Applied.Steer);
	Out.UApplied.Add(TEXT("brake"), Applied.Brake);
	Out.Extra.Add(TEXT("wheel.steer_angle"), SteerAngle);
	if (UChaosWheeledVehicleMovementComponent* M = Chaos())
	{
		double rpm = 0;
		if (M->Wheels.Num()) rpm = M->Wheels.Last()->GetRotationAngularVelocity() / 6.0;   // deg/s -> rpm
		Out.Extra.Add(TEXT("wheel.rpm"), rpm);
	}
	Out.bCollision = bHitThisTick;
	Out.Contacts = Contacts;
	bHitThisTick = false;
	Contacts.Reset();
	Out.bCapsized = FMath::Abs(Out.Roll) > 1.2 || FMath::Abs(Out.Pitch) > 1.2;
}
