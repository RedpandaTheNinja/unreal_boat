#include "FableBridgeActor.h"
#include "FableConv.h"
#include "FableLidar2D.h"

#include "Common/UdpSocketBuilder.h"
#include "Common/UdpSocketReceiver.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/WorldSettings.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Kismet/GameplayStatics.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/ArrayReader.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

DEFINE_LOG_CATEGORY_STATIC(LogFable, Log, All);

static constexpr int32 FABLE_PROTOCOL_VERSION = 1;

AFableBridgeActor::AFableBridgeActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;     // observe the state physics just produced
	Lidar = CreateDefaultSubobject<UFableLidar2D>(TEXT("Lidar"));
	Rng.Initialize(42);
}

// ------------------------------------------------------------------ lifecycle

void AFableBridgeActor::BeginPlay()
{
	Super::BeginPlay();
	FindVehicle();

	const FIPv4Address Bind = bLoopbackOnly ? FIPv4Address(127, 0, 0, 1) : FIPv4Address::Any;
	Socket = FUdpSocketBuilder(TEXT("FableBridge"))
		.AsNonBlocking().AsReusable()
		.BoundToAddress(Bind).BoundToPort(Port)
		.WithReceiveBufferSize(1 << 20).WithSendBufferSize(1 << 20);
	if (!Socket)
	{
		UE_LOG(LogFable, Error, TEXT("could not bind UDP %s:%d"), *Bind.ToString(), Port);
		return;
	}
	Receiver = new FUdpSocketReceiver(Socket, FTimespan::FromMilliseconds(5), TEXT("FableBridgeRecv"));
	Receiver->OnDataReceived().BindUObject(this, &AFableBridgeActor::OnUdp);
	Receiver->Start();
	UE_LOG(LogFable, Log, TEXT("listening on %s:%d  (vehicle: %s)"), *Bind.ToString(), Port,
		VehActor ? *VehActor->GetName() : TEXT("NONE - add a FableCarPawn or FableBoatPawn"));

	if (Veh)
	{
		Veh->FableSetEnv(&Env);
		Veh->FableApplyParams(Params);
		if (Lidar)
		{
			Lidar->AttachToComponent(VehActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
			Lidar->SetRelativeLocation(Fable::BodyRosToUe(Params.LidarMountM) * Fable::M2CM);
			Lidar->SetRelativeRotation(FRotator::ZeroRotator);
		}
	}
}

void AFableBridgeActor::EndPlay(const EEndPlayReason::Type Reason)
{
	if (Receiver) { Receiver->Stop(); delete Receiver; Receiver = nullptr; }
	if (Socket) { Socket->Close(); ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket); Socket = nullptr; }
	Super::EndPlay(Reason);
}

void AFableBridgeActor::FindVehicle()
{
	VehActor = Vehicle;
	if (!VehActor)
	{
		for (TActorIterator<AActor> It(GetWorld()); It; ++It)
		{
			if (It->GetClass()->ImplementsInterface(UFableVehicle::StaticClass())) { VehActor = *It; break; }
		}
	}
	Veh = VehActor ? Cast<IFableVehicle>(VehActor) : nullptr;
}

// ------------------------------------------------------------------ network

void AFableBridgeActor::OnUdp(const TSharedPtr<FArrayReader, ESPMode::ThreadSafe>& Data, const FIPv4Endpoint& From)
{
	// receiver thread: decode bytes only, hand the string to the game thread
	FString Str;
	const int32 Len = Data->Num();
	TArray<uint8> Bytes;
	Bytes.SetNumUninitialized(Len + 1);
	FMemory::Memcpy(Bytes.GetData(), Data->GetData(), Len);
	Bytes[Len] = 0;
	Str = FString(UTF8_TO_TCHAR(reinterpret_cast<const ANSICHAR*>(Bytes.GetData())));
	Inbox.Enqueue(TPair<FString, FIPv4Endpoint>(Str, From));
}

void AFableBridgeActor::Send(const TSharedPtr<FJsonObject>& Msg, const FIPv4Endpoint& To)
{
	if (!Socket) return;
	FString Out;
	TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> W = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Out);
	FJsonSerializer::Serialize(Msg.ToSharedRef(), W);
	FTCHARToUTF8 Utf8(*Out);
	int32 Sent = 0;
	Socket->SendTo(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length(), Sent, *To.ToInternetAddr());
}

void AFableBridgeActor::Ack(const TCHAR* For, bool bOk, const FString& Error, int64 Tick)
{
	TSharedPtr<FJsonObject> A = MakeShared<FJsonObject>();
	A->SetStringField(TEXT("type"), TEXT("ack"));
	A->SetStringField(TEXT("for"), For);
	A->SetBoolField(TEXT("ok"), bOk);
	if (!Error.IsEmpty()) A->SetStringField(TEXT("error"), Error);
	if (Tick >= 0) A->SetNumberField(TEXT("tick"), (double)Tick);
	Send(A, ReplyTo);
}

// ------------------------------------------------------------------ tick

void AFableBridgeActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 1. drain inbox (game thread)
	TPair<FString, FIPv4Endpoint> Item;
	while (Inbox.Dequeue(Item))
	{
		TSharedPtr<FJsonObject> M;
		TSharedRef<TJsonReader<>> R = TJsonReaderFactory<>::Create(Item.Key);
		if (FJsonSerializer::Deserialize(R, M) && M.IsValid())
		{
			if (bLogMessages) UE_LOG(LogFable, Log, TEXT("<- %s"), *Item.Key.Left(200));
			Handle(M, Item.Value);
		}
	}

	// 2. advance environment and apply held command (ZOH)
	UpdateGust(DeltaSeconds);
	if (Veh) Veh->FableApplyCommand(LastU);

	SimT += DeltaSeconds;
	++TickCount;

	// 3. observe
	if (bHaveClient) SendObservation(DeltaSeconds);
}

void AFableBridgeActor::UpdateGust(float Dt)
{
	if (Env.GustStd > 0.f)
	{
		const float a = FMath::Exp(-Dt / FMath::Max(Env.GustPeriod, 0.1f));
		GustState = a * GustState + FMath::Sqrt(FMath::Max(1.f - a * a, 0.f)) * Rng.FRandRange(-1.f, 1.f) * 1.7f * Env.GustStd;
	}
	else GustState = 0.f;
	Env.WindNow = FMath::Max(Env.WindSpeed + GustState, 0.f);
}

// ------------------------------------------------------------------ protocol

void AFableBridgeActor::Handle(const TSharedPtr<FJsonObject>& M, const FIPv4Endpoint& From)
{
	const FString Type = M->GetStringField(TEXT("type"));
	ReplyTo = From;

	if (Type == TEXT("hello"))
	{
		const bool bOk = (int32)M->GetNumberField(TEXT("version")) == FABLE_PROTOCOL_VERSION;
		FString Role = TEXT("controller");
		M->TryGetStringField(TEXT("role"), Role);
		if (bOk && Role != TEXT("editor")) { Client = From; bHaveClient = true; }
		Ack(TEXT("hello"), bOk, bOk ? TEXT("") : TEXT("version"));
		return;
	}

	if (Type == TEXT("act"))
	{
		const TSharedPtr<FJsonObject>* U;
		if (M->TryGetObjectField(TEXT("u"), U))
		{
			LastU.Reset();
			for (const auto& KV : (*U)->Values) LastU.Add(KV.Key, KV.Value->AsNumber());
		}
		Seq = (int64)M->GetNumberField(TEXT("seq"));
	}
	else if (Type == TEXT("reset")) DoReset(M);
	else if (Type == TEXT("set_params"))
	{
		const TSharedPtr<FJsonObject>* P;
		if (M->TryGetObjectField(TEXT("params"), P))
		{
			Params.FromJson(*P);
			if (Veh) Veh->FableApplyParams(Params);
			if (Lidar)
			{
				Lidar->NumRays = Params.LidarRays; Lidar->FovRad = Params.LidarFovRad; Lidar->RangeMaxM = Params.LidarRangeM;
				Lidar->NoiseStdM = Params.LidarNoiseStdM; Lidar->RateHz = Params.LidarRateHz;
				Lidar->SetRelativeLocation(Fable::BodyRosToUe(Params.LidarMountM) * Fable::M2CM);
			}
			Ack(TEXT("set_params"), true);
		}
		else Ack(TEXT("set_params"), false, TEXT("missing params"));
	}
	else if (Type == TEXT("set_env")) DoSetEnv(M);
	else if (Type == TEXT("spawn")) DoSpawn(M);
	else if (Type == TEXT("despawn")) DoDespawn(M->GetStringField(TEXT("label")));
	else if (Type == TEXT("query")) SendScene();
	else if (Type == TEXT("bye")) { if (From == Client) bHaveClient = false; }
}

void AFableBridgeActor::DoReset(const TSharedPtr<FJsonObject>& M)
{
	int32 Seed = 0;
	double s;
	if (M->TryGetNumberField(TEXT("seed"), s)) { Seed = (int32)s; Rng.Initialize(Seed); }

	bool bClear = true;
	M->TryGetBoolField(TEXT("clear_dynamic"), bClear);
	if (bClear) DoDespawn(TEXT("dynamic:*"));

	double x = SpawnXYM.X, y = SpawnXYM.Y, yaw = SpawnYawRad;
	const TSharedPtr<FJsonObject>* Pose;
	if (M->TryGetObjectField(TEXT("pose"), Pose))
	{
		(*Pose)->TryGetNumberField(TEXT("x"), x);
		(*Pose)->TryGetNumberField(TEXT("y"), y);
		(*Pose)->TryGetNumberField(TEXT("yaw"), yaw);
	}
	else
	{
		x += Rng.FRandRange(-SpawnJitterM, SpawnJitterM);
		y += Rng.FRandRange(-SpawnJitterM, SpawnJitterM);
		yaw += Rng.FRandRange(-SpawnJitterYawRad, SpawnJitterYawRad);
	}

	LastU.Reset();
	Seq = -1;
	PendingContacts.Reset();
	if (Veh) Veh->FableResetTo(x, y, yaw);
	OnFableReset(Seed);                 // level Blueprint: re-scatter obstacles etc.
	Ack(TEXT("reset"), true, TEXT(""), TickCount);
}

void AFableBridgeActor::DoSetEnv(const TSharedPtr<FJsonObject>& M)
{
	const TSharedPtr<FJsonObject>* O;
	double v;
	if (M->TryGetObjectField(TEXT("wind"), O))
	{
		if ((*O)->TryGetNumberField(TEXT("speed"), v)) Env.WindSpeed = v;
		if ((*O)->TryGetNumberField(TEXT("dir"), v)) Env.WindDir = v;
		if ((*O)->TryGetNumberField(TEXT("gust_std"), v)) Env.GustStd = v;
		if ((*O)->TryGetNumberField(TEXT("gust_period"), v)) Env.GustPeriod = v;
	}
	if (M->TryGetObjectField(TEXT("current"), O))
	{
		if ((*O)->TryGetNumberField(TEXT("speed"), v)) Env.CurrentSpeed = v;
		if ((*O)->TryGetNumberField(TEXT("dir"), v)) Env.CurrentDir = v;
	}
	if (M->TryGetObjectField(TEXT("waves"), O))
	{
		if ((*O)->TryGetNumberField(TEXT("amp"), v)) Env.WaveAmp = v;
		if ((*O)->TryGetNumberField(TEXT("period"), v)) Env.WavePeriod = v;
		if ((*O)->TryGetNumberField(TEXT("dir"), v)) Env.WaveDir = v;
	}
	if (M->TryGetNumberField(TEXT("time_scale"), v))
	{
		Env.TimeScale = v;
		if (AWorldSettings* WS = GetWorldSettings()) WS->SetTimeDilation(v);
	}
	Ack(TEXT("set_env"), true);
}

float AFableBridgeActor::GroundZ(double x_m, double y_m) const
{
	const FVector P = Fable::RosToUe(x_m, y_m, 0.0);
	FHitResult Hit;
	FCollisionQueryParams Q(SCENE_QUERY_STAT(FableGround), false);
	if (GetWorld()->LineTraceSingleByChannel(Hit, P + FVector(0, 0, 100000.f), P - FVector(0, 0, 100000.f), ECC_Visibility, Q))
		return Hit.ImpactPoint.Z;
	return 0.f;
}

void AFableBridgeActor::DoSpawn(const TSharedPtr<FJsonObject>& M)
{
	const FString Kind = M->GetStringField(TEXT("kind"));
	const FString Label = M->GetStringField(TEXT("label"));
	UStaticMesh* Mesh = nullptr;
	FVector SizeM(0.5f, 0.5f, 0.5f);
	bool bFloats = false;

	if (Kind.StartsWith(TEXT("custom:")))
	{
		Mesh = LoadObject<UStaticMesh>(nullptr, *Kind.RightChop(7));
		SizeM = FVector(1.f);
	}
	else if (const FFableSpawnKind* K = SpawnKinds.Find(Kind))
	{
		Mesh = K->Mesh.LoadSynchronous();
		SizeM = K->DefaultSizeM;
		bFloats = K->bFloats;
	}
	if (!Mesh) { Ack(TEXT("spawn"), false, FString::Printf(TEXT("unknown kind '%s' (add it to SpawnKinds on the bridge actor)"), *Kind)); return; }

	double x = M->GetNumberField(TEXT("x")), y = M->GetNumberField(TEXT("y")), z = 0, yaw = 0;
	M->TryGetNumberField(TEXT("z"), z);
	M->TryGetNumberField(TEXT("yaw"), yaw);
	FVector Scale(1.f);
	const TArray<TSharedPtr<FJsonValue>>* S;
	if (M->TryGetArrayField(TEXT("scale"), S) && S->Num() >= 3) Scale = FVector((*S)[0]->AsNumber(), (*S)[1]->AsNumber(), (*S)[2]->AsNumber());
	bool bMovable = false;
	M->TryGetBoolField(TEXT("movable"), bMovable);

	DoDespawn(Label);                      // re-spawn with the same label replaces

	FVector Loc = Fable::RosToUe(x, y, z);
	if (z == 0.0)
	{
		const float halfH = SizeM.Z * Scale.Z * 0.5f * Fable::M2CM;
		Loc.Z = (bFloats ? Env.WaterLevelM * Fable::M2CM : GroundZ(x, y)) + halfH;
	}
	FActorSpawnParameters SP;
	SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AStaticMeshActor* A = GetWorld()->SpawnActor<AStaticMeshActor>(Loc, Fable::RosYawToRotator(yaw), SP);
	if (!A) { Ack(TEXT("spawn"), false, TEXT("SpawnActor failed")); return; }
	A->SetMobility(EComponentMobility::Movable);
	A->GetStaticMeshComponent()->SetStaticMesh(Mesh);
	// mesh bounds -> requested metres
	const FVector Ext = Mesh->GetBounds().BoxExtent * 2.f;      // cm
	A->SetActorScale3D(FVector(SizeM.X * Scale.X * Fable::M2CM / FMath::Max(Ext.X, 1.f),
	                           SizeM.Y * Scale.Y * Fable::M2CM / FMath::Max(Ext.Y, 1.f),
	                           SizeM.Z * Scale.Z * Fable::M2CM / FMath::Max(Ext.Z, 1.f)));
	A->GetStaticMeshComponent()->SetSimulatePhysics(bMovable);
	A->Tags.Add(FName(*Label));
	A->Tags.Add(FName(TEXT("fable_dynamic")));
	A->Tags.Add(FName(*Kind));
#if WITH_EDITOR
	A->SetActorLabel(Label);
#endif
	Dynamic.Add(Label, A);
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("type"), TEXT("ack")); R->SetStringField(TEXT("for"), TEXT("spawn"));
	R->SetBoolField(TEXT("ok"), true); R->SetStringField(TEXT("label"), Label);
	Send(R, ReplyTo);
}

void AFableBridgeActor::DoDespawn(const FString& Label)
{
	int32 Removed = 0;
	if (Label == TEXT("dynamic:*"))
	{
		for (auto& KV : Dynamic) if (KV.Value.IsValid()) { KV.Value->Destroy(); ++Removed; }
		Dynamic.Reset();
	}
	else if (TWeakObjectPtr<AActor>* A = Dynamic.Find(Label))
	{
		if (A->IsValid()) { (*A)->Destroy(); ++Removed; }
		Dynamic.Remove(Label);
	}
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("type"), TEXT("ack")); R->SetStringField(TEXT("for"), TEXT("despawn"));
	R->SetBoolField(TEXT("ok"), true); R->SetNumberField(TEXT("removed"), Removed);
	Send(R, ReplyTo);
}

void AFableBridgeActor::SendScene()
{
	TSharedPtr<FJsonObject> S = MakeShared<FJsonObject>();
	S->SetStringField(TEXT("type"), TEXT("scene"));
	S->SetStringField(TEXT("spec_name"), UGameplayStatics::GetCurrentLevelName(GetWorld()));
	TArray<TSharedPtr<FJsonValue>> Actors, Dyn;
	for (TActorIterator<AActor> It(GetWorld()); It; ++It)
	{
		if (It->Tags.Num() == 0 || *It == this || *It == VehActor) continue;
		TSharedPtr<FJsonObject> A = MakeShared<FJsonObject>();
		double x, y, z;
		Fable::UeToRos(It->GetActorLocation(), x, y, z);
		A->SetStringField(TEXT("label"), It->Tags[0].ToString());
		A->SetNumberField(TEXT("x"), x); A->SetNumberField(TEXT("y"), y); A->SetNumberField(TEXT("z"), z);
		A->SetNumberField(TEXT("yaw"), Fable::YawUeToRos(It->GetActorRotation().Yaw));
		TArray<TSharedPtr<FJsonValue>> Tags;
		for (const FName& T : It->Tags) Tags.Add(MakeShared<FJsonValueString>(T.ToString()));
		A->SetArrayField(TEXT("tags"), Tags);
		Actors.Add(MakeShared<FJsonValueObject>(A));
	}
	for (auto& KV : Dynamic) Dyn.Add(MakeShared<FJsonValueString>(KV.Key));
	S->SetArrayField(TEXT("actors"), Actors);
	S->SetArrayField(TEXT("dynamic"), Dyn);
	Send(S, ReplyTo);
}

void AFableBridgeActor::ReportContact(const FString& Label)
{
	PendingContacts.AddUnique(Label);
}

// ------------------------------------------------------------------ observation

void AFableBridgeActor::SendObservation(float Dt)
{
	if (!Veh) return;
	FFableState St;
	Veh->FableGetState(St, Dt);
	for (const FString& C : PendingContacts) { St.Contacts.AddUnique(C); St.bCollision = true; }
	PendingContacts.Reset();

	TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
	O->SetStringField(TEXT("type"), TEXT("obs"));
	O->SetNumberField(TEXT("tick"), (double)TickCount);
	O->SetNumberField(TEXT("t"), SimT);
	O->SetNumberField(TEXT("dt"), Dt);
	O->SetNumberField(TEXT("seq"), (double)Seq);
	O->SetStringField(TEXT("vehicle"), Veh->FableVehicleType());

	auto Pose = MakeShared<FJsonObject>();
	Pose->SetNumberField(TEXT("x"), St.X); Pose->SetNumberField(TEXT("y"), St.Y); Pose->SetNumberField(TEXT("z"), St.Z);
	Pose->SetNumberField(TEXT("roll"), St.Roll); Pose->SetNumberField(TEXT("pitch"), St.Pitch); Pose->SetNumberField(TEXT("yaw"), St.Yaw);
	O->SetObjectField(TEXT("pose"), Pose);

	auto Vel = MakeShared<FJsonObject>();
	Vel->SetNumberField(TEXT("vx"), St.VelBody.X); Vel->SetNumberField(TEXT("vy"), St.VelBody.Y); Vel->SetNumberField(TEXT("vz"), St.VelBody.Z);
	O->SetObjectField(TEXT("vel"), Vel);
	auto Om = MakeShared<FJsonObject>();
	const float gn = Params.GyroNoiseStd * 1.7f;
	Om->SetNumberField(TEXT("wx"), St.OmegaBody.X + Rng.FRandRange(-gn, gn));
	Om->SetNumberField(TEXT("wy"), St.OmegaBody.Y + Rng.FRandRange(-gn, gn));
	Om->SetNumberField(TEXT("wz"), St.OmegaBody.Z + Rng.FRandRange(-gn, gn));
	O->SetObjectField(TEXT("omega"), Om);
	auto Acc = MakeShared<FJsonObject>();
	const float an = Params.AccelNoiseStd * 1.7f;
	Acc->SetNumberField(TEXT("ax"), St.AccBody.X + Rng.FRandRange(-an, an));
	Acc->SetNumberField(TEXT("ay"), St.AccBody.Y + Rng.FRandRange(-an, an));
	Acc->SetNumberField(TEXT("az"), St.AccBody.Z + Rng.FRandRange(-an, an));
	O->SetObjectField(TEXT("acc"), Acc);

	auto Ua = MakeShared<FJsonObject>();
	for (auto& KV : St.UApplied) Ua->SetNumberField(KV.Key, KV.Value);
	O->SetObjectField(TEXT("u_applied"), Ua);

	// sensors
	auto Sens = MakeShared<FJsonObject>();
	Sens->SetNumberField(TEXT("speed"), FMath::Abs(St.Speed + Rng.FRandRange(-1.f, 1.f) * Params.SpeedNoiseStd * 1.7f));
	if (Lidar && Params.bLidar)
	{
		Lidar->Update(SimT);
		auto L = MakeShared<FJsonObject>();
		L->SetNumberField(TEXT("angle_min"), Lidar->AngleMin());
		L->SetNumberField(TEXT("angle_max"), Lidar->AngleMax());
		L->SetNumberField(TEXT("n"), Lidar->Ranges().Num());
		L->SetNumberField(TEXT("range_max"), Lidar->RangeMaxM);
		TArray<TSharedPtr<FJsonValue>> R;
		R.Reserve(Lidar->Ranges().Num());
		for (float r : Lidar->Ranges()) R.Add(MakeShared<FJsonValueNumber>(FMath::RoundToFloat(r * 1000.f) / 1000.f));
		L->SetArrayField(TEXT("ranges"), R);
		Sens->SetObjectField(TEXT("lidar"), L);
	}
	if (Params.bGps)
	{
		if (SimT - LastGpsT >= 1.0 / FMath::Max(Params.GpsRateHz, 0.1f))
		{
			LastGpsT = SimT;
			const double nx = St.X + Rng.FRandRange(-1.f, 1.f) * Params.GpsNoiseStdM * 1.7f;
			const double ny = St.Y + Rng.FRandRange(-1.f, 1.f) * Params.GpsNoiseStdM * 1.7f;
			GpsLat = GeoLat + ny / 111320.0;
			GpsLon = GeoLon + nx / (111320.0 * FMath::Cos(FMath::DegreesToRadians(GeoLat)));
		}
		auto G = MakeShared<FJsonObject>();
		G->SetNumberField(TEXT("lat"), GpsLat); G->SetNumberField(TEXT("lon"), GpsLon); G->SetNumberField(TEXT("hdop"), 1.0);
		Sens->SetObjectField(TEXT("gps"), G);
	}
	// vehicle extras: keys like "wheel.steer_angle" -> nested objects
	for (auto& KV : St.Extra)
	{
		FString A, B;
		if (KV.Key.Split(TEXT("."), &A, &B))
		{
			TSharedPtr<FJsonObject> Sub = Sens->HasField(A) ? Sens->GetObjectField(A) : MakeShared<FJsonObject>();
			Sub->SetNumberField(B, KV.Value);
			Sens->SetObjectField(A, Sub);
		}
		else Sens->SetNumberField(KV.Key, KV.Value);
	}
	O->SetObjectField(TEXT("sensors"), Sens);

	auto Ev = MakeShared<FJsonObject>();
	Ev->SetBoolField(TEXT("collision"), St.bCollision);
	TArray<TSharedPtr<FJsonValue>> C;
	for (const FString& s : St.Contacts) C.Add(MakeShared<FJsonValueString>(s));
	Ev->SetArrayField(TEXT("contacts"), C);
	Ev->SetBoolField(TEXT("out_of_bounds"), FMath::Abs(St.X) > BoundsM.X * 0.5 || FMath::Abs(St.Y) > BoundsM.Y * 0.5);
	Ev->SetBoolField(TEXT("capsized"), St.bCapsized);
	O->SetObjectField(TEXT("events"), Ev);

	auto E = MakeShared<FJsonObject>();
	auto W = MakeShared<FJsonObject>(); W->SetNumberField(TEXT("speed"), Env.WindNow); W->SetNumberField(TEXT("dir"), Env.WindDir);
	auto Cu = MakeShared<FJsonObject>(); Cu->SetNumberField(TEXT("speed"), Env.CurrentSpeed); Cu->SetNumberField(TEXT("dir"), Env.CurrentDir);
	E->SetObjectField(TEXT("wind"), W); E->SetObjectField(TEXT("current"), Cu);
	O->SetObjectField(TEXT("env"), E);

	SendObs(O);
}
