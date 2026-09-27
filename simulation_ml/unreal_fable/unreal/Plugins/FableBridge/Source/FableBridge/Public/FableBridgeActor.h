// FableBridgeActor.h - drop ONE of these in the level. It:
//   * binds UDP 127.0.0.1:Port and speaks docs/PROTOCOL.md
//   * drives the vehicle pawn (anything implementing IFableVehicle)
//   * sends an observation every tick (enable Use Fixed Frame Rate in project settings)
//   * owns the environment (wind/current/waves), the lidar, GPS, and runtime spawn/despawn
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Containers/Queue.h"
#include "Dom/JsonObject.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "FableTypes.h"
#include "FableBridgeActor.generated.h"

class FSocket;
class FUdpSocketReceiver;
class UFableLidar2D;
class UStaticMesh;

USTRUCT()
struct FFableSpawnKind
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere) TSoftObjectPtr<UStaticMesh> Mesh;
	UPROPERTY(EditAnywhere) FVector DefaultSizeM = FVector(0.5f, 0.5f, 0.5f);   // scale [1,1,1] gives this size
	UPROPERTY(EditAnywhere) bool bFloats = false;                             // sit at water level instead of ground
};

UCLASS()
class FABLEBRIDGE_API AFableBridgeActor : public AActor
{
	GENERATED_BODY()

public:
	AFableBridgeActor();

	// --- config
	UPROPERTY(EditAnywhere, Category = "Fable|Net") int32 Port = 9800;
	UPROPERTY(EditAnywhere, Category = "Fable|Net") bool bLoopbackOnly = true;

	// The pawn to drive. If null, the first actor implementing IFableVehicle is used.
	UPROPERTY(EditAnywhere, Category = "Fable") AActor* Vehicle = nullptr;

	// Default spawn (ROS metres / radians). Overridden by `reset.pose` from Python.
	UPROPERTY(EditAnywhere, Category = "Fable") FVector2D SpawnXYM = FVector2D::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Fable") float SpawnYawRad = 0.f;
	UPROPERTY(EditAnywhere, Category = "Fable") float SpawnJitterM = 0.3f;
	UPROPERTY(EditAnywhere, Category = "Fable") float SpawnJitterYawRad = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Fable|Env") FFableEnv Env;

	// GPS origin: world (0,0) maps to this lat/lon
	UPROPERTY(EditAnywhere, Category = "Fable|Env") double GeoLat = 39.7684;
	UPROPERTY(EditAnywhere, Category = "Fable|Env") double GeoLon = -86.1581;

	// Playable area (ROS metres, centred on origin). Outside -> events.out_of_bounds.
	UPROPERTY(EditAnywhere, Category = "Fable") FVector2D BoundsM = FVector2D(200.f, 200.f);

	// kind -> mesh for runtime `spawn`. Fill in the Blueprint subclass (BP_FableBridge).
	UPROPERTY(EditAnywhere, Category = "Fable|Spawn") TMap<FString, FFableSpawnKind> SpawnKinds;

	UPROPERTY(EditAnywhere, Category = "Fable|Debug") bool bLogMessages = false;

	UPROPERTY() UFableLidar2D* Lidar = nullptr;

	// --- AActor
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;

	// --- Blueprint hooks so a level BP can react (e.g. re-scatter obstacles on reset)
	UFUNCTION(BlueprintImplementableEvent, Category = "Fable") void OnFableReset(int32 Seed);
	UFUNCTION(BlueprintCallable, Category = "Fable") void ReportContact(const FString& Label);

	double SimTime() const { return SimT; }
	const FFableEnv& GetEnv() const { return Env; }

private:
	// net
	FSocket* Socket = nullptr;
	FUdpSocketReceiver* Receiver = nullptr;
	TQueue<TPair<FString, FIPv4Endpoint>, EQueueMode::Mpsc> Inbox;
	// The controller client receives observations; any client (e.g. an MCP world-editor
	// that said hello with role "editor") may send edits and gets acks at its own address.
	bool bHaveClient = false;
	FIPv4Endpoint Client;
	FIPv4Endpoint ReplyTo;
	void OnUdp(const TSharedPtr<class FArrayReader, ESPMode::ThreadSafe>& Data, const FIPv4Endpoint& From);
	void Send(const TSharedPtr<FJsonObject>& Msg, const FIPv4Endpoint& To);
	void SendObs(const TSharedPtr<FJsonObject>& Msg) { if (bHaveClient) Send(Msg, Client); }
	void Ack(const TCHAR* For, bool bOk, const FString& Error = TEXT(""), int64 Tick = -1);

	// protocol
	void Handle(const TSharedPtr<FJsonObject>& M, const FIPv4Endpoint& From);
	void DoReset(const TSharedPtr<FJsonObject>& M);
	void DoSetEnv(const TSharedPtr<FJsonObject>& M);
	void DoSpawn(const TSharedPtr<FJsonObject>& M);
	void DoDespawn(const FString& Label);
	void SendScene();
	void SendObservation(float Dt);

	// sim state
	IFableVehicle* Veh = nullptr;
	AActor* VehActor = nullptr;
	FFableVehicleParams Params;
	TMap<FString, double> LastU;
	int64 Seq = -1;
	int64 TickCount = 0;
	double SimT = 0.0;
	float GustState = 0.f;
	FRandomStream Rng;
	TArray<FString> PendingContacts;
	TMap<FString, TWeakObjectPtr<AActor>> Dynamic;

	// gps
	double LastGpsT = -1e9;
	double GpsLat = 0, GpsLon = 0;

	void FindVehicle();
	void UpdateGust(float Dt);
	float GroundZ(double x_m, double y_m) const;   // UE cm
};
