// FableTypes.h - shared structs: vehicle state, environment, vehicle parameters.
#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "UObject/Interface.h"
#include "FableTypes.generated.h"

// Environmental disturbances, ROS conventions (m/s, rad, direction the flow moves TOWARD).
USTRUCT(BlueprintType)
struct FFableEnv
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WindSpeed = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WindDir = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float GustStd = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float GustPeriod = 8.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float CurrentSpeed = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float CurrentDir = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WaveAmp = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WavePeriod = 3.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WaveDir = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WaterLevelM = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float TimeScale = 1.f;

	// instantaneous (mean + gust), updated by the bridge each tick
	float WindNow = 0.f;

	// wind vector in ROS world frame (m/s)
	FVector2D WindVec() const { return FVector2D(FMath::Cos(WindDir), FMath::Sin(WindDir)) * WindNow; }
	FVector2D CurrentVec() const { return FVector2D(FMath::Cos(CurrentDir), FMath::Sin(CurrentDir)) * CurrentSpeed; }

	// water surface height (m, ROS z) at a ROS world position and sim time
	float WaterHeight(double x_m, double y_m, double t) const
	{
		if (WaveAmp <= 0.f) return WaterLevelM;
		const float k = 2.f * PI / FMath::Max(WavePeriod * 1.2f, 0.5f);      // crude wavelength
		const float w = 2.f * PI / FMath::Max(WavePeriod, 0.1f);
		const float phase = k * (x_m * FMath::Cos(WaveDir) + y_m * FMath::Sin(WaveDir)) - w * t;
		return WaterLevelM + WaveAmp * FMath::Sin(phase);
	}
};

// Snapshot the bridge serialises every tick. Everything already in ROS conventions.
struct FFableState
{
	double X = 0, Y = 0, Z = 0;                 // m, world ENU
	double Roll = 0, Pitch = 0, Yaw = 0;        // rad
	FVector VelBody = FVector::ZeroVector;      // m/s, body (x fwd, y left, z up)
	FVector OmegaBody = FVector::ZeroVector;    // rad/s, body
	FVector AccBody = FVector::ZeroVector;      // m/s^2, body, incl. gravity
	double Speed = 0;                           // ground speed m/s
	TMap<FString, double> UApplied;             // command actually in effect
	TMap<FString, double> Extra;                // wheel.steer_angle, thrust.l ... flattened "a.b"
	bool bCollision = false;
	TArray<FString> Contacts;
	bool bCapsized = false;
};

// Vehicle parameters the pawns need at runtime. Mirrors specs/schema/vehicle_params.schema.json;
// values not listed here are read straight from the JSON when needed.
USTRUCT(BlueprintType)
struct FFableVehicleParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite) FString Name = TEXT("default");
	UPROPERTY(EditAnywhere, BlueprintReadWrite) FString Type = TEXT("car_ackermann");
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MassKg = 3.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float LengthM = 0.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WidthM = 0.27f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float HeightM = 0.15f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) FVector ComOffsetM = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float YawInertia = 0.f;          // 0 = derive from box
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ActuatorDelayS = 0.f;

	// car
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WheelbaseM = 0.32f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float TrackWidthM = 0.24f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WheelRadiusM = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxSteerRad = 0.4f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float SteerRateRadS = 4.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float SteerDeadband = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float SteerBias = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxSpeedMps = 5.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxReverseMps = 1.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float AccelTimeConstantS = 0.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxAccelMps2 = 4.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxBrakeMps2 = 6.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float TireFriction = 1.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MotorTorqueNm = 1.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) FString Drive = TEXT("rwd");

	// boat
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ThrusterOffsetYM = 0.25f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ThrusterOffsetXM = -0.4f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ThrusterDepthM = 0.1f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxThrustFwdN = 20.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float MaxThrustRevN = 10.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ThrustTimeConstantS = 0.25f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ThrustDeadband = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float AddedMassSurge = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float AddedMassSway = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float AddedInertiaYaw = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DragSurgeLin = 2.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DragSurgeQuad = 8.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DragSwayLin = 15.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DragSwayQuad = 60.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DragYawLin = 1.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DragYawQuad = 3.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WindAreaM2 = 0.2f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float WindCoeff = 1.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float DraftM = 0.12f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) TArray<FVector> BuoyancyPointsM;

	// sensors
	UPROPERTY(EditAnywhere, BlueprintReadWrite) bool bLidar = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) int32 LidarRays = 360;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float LidarFovRad = 6.283f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float LidarRangeM = 20.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float LidarNoiseStdM = 0.02f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) FVector LidarMountM = FVector(0.2f, 0.f, 0.15f);
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float LidarRateHz = 10.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) bool bGps = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float GpsNoiseStdM = 1.5f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float GpsRateHz = 5.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float GyroNoiseStd = 0.005f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float AccelNoiseStd = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float SpeedNoiseStd = 0.02f;

	float YawInertiaOrDerived() const
	{
		return YawInertia > 0.f ? YawInertia : MassKg * (LengthM * LengthM + WidthM * WidthM) / 12.f;
	}

	// Fills from the vehicle_params.json object sent by Python. Missing keys keep current values.
	FABLEBRIDGE_API void FromJson(const TSharedPtr<FJsonObject>& Root);
};

// Implemented by AFableCarPawn and AFableBoatPawn; the bridge only talks to this.
UINTERFACE(MinimalAPI, Blueprintable)
class UFableVehicle : public UInterface
{
	GENERATED_BODY()
};

class FABLEBRIDGE_API IFableVehicle
{
	GENERATED_BODY()

public:
	virtual void FableApplyCommand(const TMap<FString, double>& U) = 0;
	virtual void FableApplyParams(const FFableVehicleParams& P) = 0;
	virtual void FableResetTo(double x_m, double y_m, double yaw_rad) = 0;
	virtual void FableSetEnv(const FFableEnv* Env) = 0;
	virtual void FableGetState(FFableState& Out, float Dt) = 0;
	virtual FString FableVehicleType() const = 0;
};
