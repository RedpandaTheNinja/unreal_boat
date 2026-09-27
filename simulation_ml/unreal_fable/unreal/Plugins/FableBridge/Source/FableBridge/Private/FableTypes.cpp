#include "FableTypes.h"

namespace
{
	float Num(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, float Def)
	{
		double v;
		return (O.IsValid() && O->TryGetNumberField(Key, v)) ? (float)v : Def;
	}
	bool Bool(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, bool Def)
	{
		bool v;
		return (O.IsValid() && O->TryGetBoolField(Key, v)) ? v : Def;
	}
	FVector Vec3(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, const FVector& Def)
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr;
		if (O.IsValid() && O->TryGetArrayField(Key, Arr) && Arr->Num() >= 3)
			return FVector((*Arr)[0]->AsNumber(), (*Arr)[1]->AsNumber(), (*Arr)[2]->AsNumber());
		return Def;
	}
	TSharedPtr<FJsonObject> Obj(const TSharedPtr<FJsonObject>& O, const TCHAR* Key)
	{
		const TSharedPtr<FJsonObject>* Sub;
		return (O.IsValid() && O->TryGetObjectField(Key, Sub)) ? *Sub : nullptr;
	}
}

void FFableVehicleParams::FromJson(const TSharedPtr<FJsonObject>& Root)
{
	if (!Root.IsValid()) return;
	Root->TryGetStringField(TEXT("name"), Name);
	Root->TryGetStringField(TEXT("type"), Type);

	if (auto C = Obj(Root, TEXT("common")))
	{
		MassKg = Num(C, TEXT("mass_kg"), MassKg);
		LengthM = Num(C, TEXT("length_m"), LengthM);
		WidthM = Num(C, TEXT("width_m"), WidthM);
		HeightM = Num(C, TEXT("height_m"), HeightM);
		ComOffsetM = Vec3(C, TEXT("com_offset_m"), ComOffsetM);
		YawInertia = Num(C, TEXT("yaw_inertia_kgm2"), YawInertia);
		ActuatorDelayS = Num(C, TEXT("actuator_delay_s"), ActuatorDelayS);
	}
	if (auto K = Obj(Root, TEXT("car")))
	{
		WheelbaseM = Num(K, TEXT("wheelbase_m"), WheelbaseM);
		TrackWidthM = Num(K, TEXT("track_width_m"), TrackWidthM);
		WheelRadiusM = Num(K, TEXT("wheel_radius_m"), WheelRadiusM);
		MaxSteerRad = Num(K, TEXT("max_steer_rad"), MaxSteerRad);
		SteerRateRadS = Num(K, TEXT("steer_rate_rad_s"), SteerRateRadS);
		SteerDeadband = Num(K, TEXT("steer_deadband"), SteerDeadband);
		SteerBias = Num(K, TEXT("steer_bias"), SteerBias);
		MaxSpeedMps = Num(K, TEXT("max_speed_mps"), MaxSpeedMps);
		MaxReverseMps = Num(K, TEXT("max_reverse_mps"), MaxReverseMps);
		AccelTimeConstantS = Num(K, TEXT("accel_time_constant_s"), AccelTimeConstantS);
		MaxAccelMps2 = Num(K, TEXT("max_accel_mps2"), MaxAccelMps2);
		MaxBrakeMps2 = Num(K, TEXT("max_brake_mps2"), MaxBrakeMps2);
		TireFriction = Num(K, TEXT("tire_friction"), TireFriction);
		MotorTorqueNm = Num(K, TEXT("motor_torque_nm"), MotorTorqueNm);
		K->TryGetStringField(TEXT("drive"), Drive);
	}
	if (auto B = Obj(Root, TEXT("boat")))
	{
		ThrusterOffsetYM = Num(B, TEXT("thruster_offset_y_m"), ThrusterOffsetYM);
		ThrusterOffsetXM = Num(B, TEXT("thruster_offset_x_m"), ThrusterOffsetXM);
		ThrusterDepthM = Num(B, TEXT("thruster_depth_m"), ThrusterDepthM);
		MaxThrustFwdN = Num(B, TEXT("max_thrust_fwd_n"), MaxThrustFwdN);
		MaxThrustRevN = Num(B, TEXT("max_thrust_rev_n"), MaxThrustRevN);
		ThrustTimeConstantS = Num(B, TEXT("thrust_time_constant_s"), ThrustTimeConstantS);
		ThrustDeadband = Num(B, TEXT("thrust_deadband"), ThrustDeadband);
		AddedMassSurge = Num(B, TEXT("added_mass_surge_kg"), AddedMassSurge);
		AddedMassSway = Num(B, TEXT("added_mass_sway_kg"), AddedMassSway);
		AddedInertiaYaw = Num(B, TEXT("added_inertia_yaw_kgm2"), AddedInertiaYaw);
		DragSurgeLin = Num(B, TEXT("drag_surge_lin"), DragSurgeLin);
		DragSurgeQuad = Num(B, TEXT("drag_surge_quad"), DragSurgeQuad);
		DragSwayLin = Num(B, TEXT("drag_sway_lin"), DragSwayLin);
		DragSwayQuad = Num(B, TEXT("drag_sway_quad"), DragSwayQuad);
		DragYawLin = Num(B, TEXT("drag_yaw_lin"), DragYawLin);
		DragYawQuad = Num(B, TEXT("drag_yaw_quad"), DragYawQuad);
		WindAreaM2 = Num(B, TEXT("wind_area_m2"), WindAreaM2);
		WindCoeff = Num(B, TEXT("wind_coeff"), WindCoeff);
		DraftM = Num(B, TEXT("draft_m"), DraftM);
		const TArray<TSharedPtr<FJsonValue>>* Pts;
		if (B->TryGetArrayField(TEXT("buoyancy_points"), Pts))
		{
			BuoyancyPointsM.Reset();
			for (const auto& P : *Pts)
			{
				const TArray<TSharedPtr<FJsonValue>>* V;
				if (P->TryGetArray(V) && V->Num() >= 3)
					BuoyancyPointsM.Add(FVector((*V)[0]->AsNumber(), (*V)[1]->AsNumber(), (*V)[2]->AsNumber()));
			}
		}
	}
	if (auto S = Obj(Root, TEXT("sensors")))
	{
		if (auto L = Obj(S, TEXT("lidar")))
		{
			bLidar = Bool(L, TEXT("enabled"), bLidar);
			LidarRays = (int32)Num(L, TEXT("n_rays"), (float)LidarRays);
			LidarFovRad = Num(L, TEXT("fov_rad"), LidarFovRad);
			LidarRangeM = Num(L, TEXT("range_max_m"), LidarRangeM);
			LidarNoiseStdM = Num(L, TEXT("noise_std_m"), LidarNoiseStdM);
			LidarMountM = Vec3(L, TEXT("mount_m"), LidarMountM);
			LidarRateHz = Num(L, TEXT("rate_hz"), LidarRateHz);
		}
		if (auto G = Obj(S, TEXT("gps")))
		{
			bGps = Bool(G, TEXT("enabled"), bGps);
			GpsNoiseStdM = Num(G, TEXT("noise_std_m"), GpsNoiseStdM);
			GpsRateHz = Num(G, TEXT("rate_hz"), GpsRateHz);
		}
		if (auto I = Obj(S, TEXT("imu")))
		{
			GyroNoiseStd = Num(I, TEXT("gyro_noise_std"), GyroNoiseStd);
			AccelNoiseStd = Num(I, TEXT("accel_noise_std"), AccelNoiseStd);
		}
		if (auto O = Obj(S, TEXT("odom")))
		{
			SpeedNoiseStd = Num(O, TEXT("speed_noise_std"), SpeedNoiseStd);
		}
	}
}
