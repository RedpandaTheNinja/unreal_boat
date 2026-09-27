#include "FableBoatPawn.h"
#include "FableConv.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

// force/torque unit helpers: N -> kg*cm/s^2, N*m -> kg*cm^2/s^2
static constexpr float N2UE = 100.f;
static constexpr float NM2UE = 10000.f;

AFableBoatPawn::AFableBoatPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	Hull = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Hull"));
	SetRootComponent(Hull);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (Cube.Succeeded()) Hull->SetStaticMesh(Cube.Object);
	Hull->SetSimulatePhysics(true);
	Hull->SetEnableGravity(true);
	Hull->SetNotifyRigidBodyCollision(true);
	Hull->SetCollisionProfileName(TEXT("PhysicsActor"));
	Hull->SetLinearDamping(0.f);
	Hull->SetAngularDamping(0.f);
}

void AFableBoatPawn::BeginPlay()
{
	Super::BeginPlay();
	FableApplyParams(Params);
}

void AFableBoatPawn::FableApplyParams(const FFableVehicleParams& P)
{
	Params = P;
	if (Params.BuoyancyPointsM.Num() == 0)
	{
		const float hx = Params.LengthM * 0.38f, hy = Params.WidthM * 0.38f, z = -Params.HeightM * 0.3f;
		Params.BuoyancyPointsM = { FVector(hx, hy, z), FVector(hx, -hy, z), FVector(-hx, hy, z), FVector(-hx, -hy, z) };
	}
	if (Hull)
	{
		// engine cube is 100 cm: scale to hull size in metres
		Hull->SetWorldScale3D(FVector(Params.LengthM, Params.WidthM, Params.HeightM));
		Hull->SetMassOverrideInKg(NAME_None, Params.MassKg, true);
		Hull->SetCenterOfMass(Fable::BodyRosToUe(Params.ComOffsetM) * Fable::M2CM);
	}
	DelayLine.Init(FCmd(), 2);
	DelayHead = 0;
}

AFableBoatPawn::FCmd AFableBoatPawn::PushDelay(const FCmd& In)
{
	if (DelayLine.Num() == 0) return In;
	DelayLine[DelayHead] = In;
	DelayHead = (DelayHead + 1) % DelayLine.Num();
	return DelayLine[DelayHead];
}

void AFableBoatPawn::FableApplyCommand(const TMap<FString, double>& U)
{
	FCmd C;
	if (const double* v = U.Find(TEXT("thrust_l"))) C.L = FMath::Clamp(*v, -1.0, 1.0);
	if (const double* v = U.Find(TEXT("thrust_r"))) C.R = FMath::Clamp(*v, -1.0, 1.0);
	Applied = PushDelay(C);
}

float AFableBoatPawn::ThrustFor(double cmd) const
{
	if (FMath::Abs(cmd) < Params.ThrustDeadband) return 0.f;
	return (float)cmd * (cmd > 0 ? Params.MaxThrustFwdN : Params.MaxThrustRevN);
}

void AFableBoatPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	SimT += DeltaSeconds;
	const int32 want = FMath::RoundToInt(Params.ActuatorDelayS / FMath::Max(DeltaSeconds, 1e-4f)) + 1;
	if (want != DelayLine.Num()) { DelayLine.Init(FCmd(), want); DelayHead = 0; }
	ApplyHydro(DeltaSeconds);
}

void AFableBoatPawn::ApplyHydro(float Dt)
{
	if (!Hull || !Hull->IsSimulatingPhysics()) return;
	const FTransform X = Hull->GetComponentTransform();
	const FVector Vw_cm = Hull->GetPhysicsLinearVelocity();
	const FVector Ww_deg = Hull->GetPhysicsAngularVelocityInDegrees();

	// ---- body-frame velocity relative to the water (ROS body: x fwd, y left)
	FVector2D Cur = Env ? Env->CurrentVec() : FVector2D::ZeroVector;                  // ROS world m/s
	const FVector CurUe = Fable::RosToUe(Cur.X, Cur.Y, 0.0);                          // cm... careful: RosToUe scales by 100
	const FVector VrelW = Vw_cm - CurUe;                                              // cm/s world UE
	const FVector Vb = Fable::BodyUeToRos(X.InverseTransformVectorNoScale(VrelW)) * Fable::CM2M;   // ROS body m/s
	const FVector Wb = Fable::AngVelUeWorldToRosBody(Ww_deg, X);                      // ROS body rad/s
	const float u = Vb.X, v = Vb.Y, w = Vb.Z, r = Wb.Z;

	// ---- thrusters (first order), at their hull positions
	const float k = 1.f - FMath::Exp(-Dt / FMath::Max(Params.ThrustTimeConstantS, 1e-3f));
	ThrustL += (ThrustFor(Applied.L) - ThrustL) * k;
	ThrustR += (ThrustFor(Applied.R) - ThrustR) * k;
	const FVector Fwd = X.GetUnitAxis(EAxis::X);
	const FVector PosL = X.TransformPosition(Fable::BodyRosToUe(FVector(Params.ThrusterOffsetXM, +Params.ThrusterOffsetYM, -Params.ThrusterDepthM)) * Fable::M2CM);
	const FVector PosR = X.TransformPosition(Fable::BodyRosToUe(FVector(Params.ThrusterOffsetXM, -Params.ThrusterOffsetYM, -Params.ThrusterDepthM)) * Fable::M2CM);
	Hull->AddForceAtLocation(Fwd * ThrustL * N2UE, PosL);
	Hull->AddForceAtLocation(Fwd * ThrustR * N2UE, PosR);

	// ---- hydrodynamic drag (ROS body) -> UE world force + torque
	const FVector DragBodyRos(-(Params.DragSurgeLin * u + Params.DragSurgeQuad * u * FMath::Abs(u)),
	                          -(Params.DragSwayLin * v + Params.DragSwayQuad * v * FMath::Abs(v)),
	                          -(Params.DragSwayLin * 2.f * w));                          // heave damping
	Hull->AddForce(X.TransformVectorNoScale(Fable::BodyRosToUe(DragBodyRos)) * N2UE);
	const float Nz = -(Params.DragYawLin * r + Params.DragYawQuad * r * FMath::Abs(r));
	// roll/pitch damping so the hull settles; ROS body torque -> UE body torque = flip y and negate (handedness)
	const FVector TorqueBodyRos(-(0.5f * Params.DragYawLin) * Wb.X, -(0.8f * Params.DragYawLin) * Wb.Y, Nz);
	const FVector TorqueBodyUe(-TorqueBodyRos.X, TorqueBodyRos.Y, -TorqueBodyRos.Z);
	Hull->AddTorqueInRadians(X.TransformVectorNoScale(TorqueBodyUe) * NM2UE);

	// ---- wind (relative, quadratic) on the above-water hull
	if (Env)
	{
		const FVector2D Wind = Env->WindVec();
		const FVector WindUe = Fable::RosToUe(Wind.X, Wind.Y, 0.0);                     // cm/s
		const FVector WrelB = Fable::BodyUeToRos(X.InverseTransformVectorNoScale(WindUe - Vw_cm)) * Fable::CM2M;
		const float q = 0.5f * 1.225f * Params.WindCoeff * Params.WindAreaM2;
		const FVector FwRos(q * WrelB.X * FMath::Abs(WrelB.X) * 0.5f, q * WrelB.Y * FMath::Abs(WrelB.Y), 0.f);
		Hull->AddForce(X.TransformVectorNoScale(Fable::BodyRosToUe(FwRos)) * N2UE);
		const float Nw = -0.1f * Params.LengthM * FwRos.Y;
		Hull->AddTorqueInRadians(X.TransformVectorNoScale(FVector(0, 0, -Nw)) * NM2UE);
	}

	// ---- buoyancy: N points, each carries mass*g/N at draft, spring-like in depth
	const int32 N = FMath::Max(Params.BuoyancyPointsM.Num(), 1);
	const float gN = 9.81f * Params.MassKg / N;
	for (const FVector& Pm : Params.BuoyancyPointsM)
	{
		const FVector Pw = X.TransformPosition(Fable::BodyRosToUe(Pm) * Fable::M2CM);
		double px, py, pz;
		Fable::UeToRos(Pw, px, py, pz);
		const float surface = Env ? Env->WaterHeight(px, py, SimT) : 0.f;
		const float depth = surface - (float)pz;                      // m below surface
		if (depth <= 0.f) continue;
		const float frac = FMath::Clamp(depth / FMath::Max(Params.DraftM, 0.02f), 0.f, 2.0f);
		const FVector Vp = Hull->GetPhysicsLinearVelocityAtPoint(Pw) * Fable::CM2M;
		const float damp = 2.0f * FMath::Sqrt(gN / FMath::Max(Params.DraftM, 0.02f) * Params.MassKg / N) * 0.5f;
		const float Fz = gN * frac - damp * Vp.Z;
		Hull->AddForceAtLocation(FVector(0, 0, Fz * N2UE), Pw);
		if (bDrawDebugForces) DrawDebugLine(GetWorld(), Pw, Pw + FVector(0, 0, Fz * 2.f), FColor::Cyan, false, -1.f, 0, 1.f);
	}

	// IMU-style body acceleration for the observation
	const FVector Aw = (Vw_cm - PrevVelWorld) / FMath::Max(Dt, 1e-4f) * Fable::CM2M;
	PrevVelWorld = Vw_cm;
	AccBodyRos = Fable::BodyUeToRos(X.InverseTransformVectorNoScale(Aw + FVector(0, 0, 9.81f)));
}

void AFableBoatPawn::FableResetTo(double x_m, double y_m, double yaw_rad)
{
	const float z = (Env ? Env->WaterLevelM : 0.f) + Params.HeightM * 0.5f - Params.DraftM;
	SetActorLocationAndRotation(Fable::RosToUe(x_m, y_m, z), Fable::RosYawToRotator(yaw_rad), false, nullptr, ETeleportType::TeleportPhysics);
	if (Hull)
	{
		Hull->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Hull->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	}
	ThrustL = ThrustR = 0.f;
	Applied = FCmd();
	for (FCmd& c : DelayLine) c = FCmd();
	PrevVelWorld = FVector::ZeroVector;
	Contacts.Reset();
}

void AFableBoatPawn::NotifyHit(UPrimitiveComponent* MyComp, AActor* Other, UPrimitiveComponent* OtherComp, bool bSelfMoved,
                               FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	Super::NotifyHit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
	if (!Other || Other->Tags.Contains(FName(TEXT("water")))) return;
	bHitThisTick = true;
	Contacts.AddUnique(Other->Tags.Num() ? Other->Tags[0].ToString() : Other->GetName());
}

void AFableBoatPawn::FableGetState(FFableState& Out, float Dt)
{
	const FTransform X = GetActorTransform();
	Fable::UeToRos(X.GetLocation(), Out.X, Out.Y, Out.Z);
	Fable::RotatorToRos(X.Rotator(), Out.Roll, Out.Pitch, Out.Yaw);
	const FVector Vw = Hull ? Hull->GetPhysicsLinearVelocity() : FVector::ZeroVector;
	const FVector Vb = X.InverseTransformVectorNoScale(Vw) * Fable::CM2M;
	Out.VelBody = Fable::BodyUeToRos(Vb);
	Out.Speed = Vb.Size2D();
	Out.AccBody = AccBodyRos;
	if (Hull) Out.OmegaBody = Fable::AngVelUeWorldToRosBody(Hull->GetPhysicsAngularVelocityInDegrees(), X);
	Out.UApplied.Add(TEXT("thrust_l"), Applied.L);
	Out.UApplied.Add(TEXT("thrust_r"), Applied.R);
	Out.Extra.Add(TEXT("thrust.l"), ThrustL);
	Out.Extra.Add(TEXT("thrust.r"), ThrustR);
	Out.bCollision = bHitThisTick;
	Out.Contacts = Contacts;
	bHitThisTick = false;
	Contacts.Reset();
	Out.bCapsized = FMath::Abs(Out.Roll) > 1.3 || FMath::Abs(Out.Pitch) > 1.3;
}
