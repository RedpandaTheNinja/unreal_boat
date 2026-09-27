// FableConv.h - the ONLY place Unreal units/frames meet ROS units/frames.
//
//   Unreal: centimetres, degrees, left-handed, X forward, Y RIGHT, Z up, yaw CW-positive
//   ROS   : metres, radians, right-handed, X forward, Y LEFT,  Z up, yaw CCW-positive
//
// Every number crossing the UDP socket goes through these. If you find a
// conversion anywhere else in the plugin, it is a bug.
#pragma once

#include "CoreMinimal.h"

namespace Fable
{
	constexpr float M2CM = 100.f;
	constexpr float CM2M = 0.01f;

	// --- positions (world frame)
	FORCEINLINE FVector RosToUe(float x_m, float y_m, float z_m)
	{
		return FVector(x_m * M2CM, -y_m * M2CM, z_m * M2CM);
	}
	FORCEINLINE void UeToRos(const FVector& P, double& x_m, double& y_m, double& z_m)
	{
		x_m = P.X * CM2M; y_m = -P.Y * CM2M; z_m = P.Z * CM2M;
	}

	// --- velocities / forces expressed in the UE body frame -> ROS body frame (flip Y)
	FORCEINLINE FVector BodyUeToRos(const FVector& V) { return FVector(V.X, -V.Y, V.Z); }
	FORCEINLINE FVector BodyRosToUe(const FVector& V) { return FVector(V.X, -V.Y, V.Z); }

	// --- angles
	FORCEINLINE float YawRosToUe(float yaw_rad) { return -FMath::RadiansToDegrees(yaw_rad); }
	FORCEINLINE float YawUeToRos(float yaw_deg) { return -FMath::DegreesToRadians(yaw_deg); }

	FORCEINLINE FRotator RosYawToRotator(float yaw_rad) { return FRotator(0.f, YawRosToUe(yaw_rad), 0.f); }

	// roll/pitch/yaw (ROS, rad) from a UE rotator
	FORCEINLINE void RotatorToRos(const FRotator& R, double& roll, double& pitch, double& yaw)
	{
		roll = FMath::DegreesToRadians(R.Roll);
		pitch = -FMath::DegreesToRadians(R.Pitch);
		yaw = YawUeToRos(R.Yaw);
	}

	// angular velocity: UE gives world-frame deg/s (left-handed). Convert to ROS body rad/s.
	FORCEINLINE FVector AngVelUeWorldToRosBody(const FVector& WDegPerSec, const FTransform& BodyXform)
	{
		const FVector Wb = BodyXform.InverseTransformVectorNoScale(WDegPerSec);     // UE body, deg/s
		// UE body: X fwd, Y right, Z up, rotations left-handed. Flipping Y and negating
		// the yaw/roll sign gives the ROS right-handed body rates.
		return FVector(-FMath::DegreesToRadians(Wb.X), FMath::DegreesToRadians(Wb.Y), -FMath::DegreesToRadians(Wb.Z));
	}

	FORCEINLINE float WrapPi(float a)
	{
		while (a > PI) a -= 2 * PI;
		while (a < -PI) a += 2 * PI;
		return a;
	}
}
