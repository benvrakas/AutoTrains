#include "ATChatCommand.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATTrafficSubsystem.h"
#include "AutoTrains.h"
#include "Command/CommandSender.h"
#include "EngineUtils.h"
#include "FGMapCompassSettings.h"
#include "FGMapManager.h"
#include "FGMapMarker.h"
#include "FGPlayerController.h"
#include "FGPlayerState.h"
#include "FGRailroadVehicle.h"
#include "FGTrain.h"
#include "GameFramework/Character.h"
#include "Server/FGDSAuthenticationTypes.h"

namespace
{
	// A place a player asked for: a location in world units and how chat names it.
	struct FATPlace
	{
		FVector Location = FVector::ZeroVector;
		FString Label;
	};

	FString Metres(const FVector& Location)
	{
		return FString::Printf(TEXT("(%.0f, %.0f, %.0f)"), Location.X / 100.f, Location.Y / 100.f, Location.Z / 100.f);
	}

	// Compass bearing in degrees of a world-space direction: 0 north (-Y), 90 east (+X), as the game's compass.
	float Bearing(const FVector& Direction)
	{
		return FRotator::ClampAxis(FMath::RadiansToDegrees(FMath::Atan2(Direction.X, -Direction.Y)));
	}

	const TCHAR* CompassName(float BearingDegrees)
	{
		static const TCHAR* Names[] = { TEXT("north"), TEXT("north-east"), TEXT("east"), TEXT("south-east"), TEXT("south"), TEXT("south-west"), TEXT("west"), TEXT("north-west") };
		return Names[FMath::RoundToInt(BearingDegrees / 45.f) % 8];
	}

	// Resolves Arguments[First..] to a place: "x y z" in metres (commas and brackets allowed, as pasted from the log), an
	// actor's name or the number its name ends with, or a train from /at list. Returns false with OutError set otherwise.
	bool FindPlace(const UWorld* World, const AATControlSubsystem* Control, const TArray<FString>& Arguments, int32 First, FATPlace& OutPlace, FString& OutError)
	{
		TArray<FString> Rest;
		for (int32 Index = First; Index < Arguments.Num(); ++Index)
		{
			Rest.Add(Arguments[Index]);
		}
		const FString Joined = FString::Join(Rest, TEXT(" "));
		FString Spaced = Joined.Replace(TEXT("("), TEXT(" ")).Replace(TEXT(")"), TEXT(" ")).Replace(TEXT(","), TEXT(" "));
		TArray<FString> Numbers;
		Spaced.ParseIntoArrayWS(Numbers);
		if (Numbers.Num() == 3 && Numbers[0].IsNumeric() && Numbers[1].IsNumeric() && Numbers[2].IsNumeric())
		{
			OutPlace.Location = FVector(FCString::Atof(*Numbers[0]), FCString::Atof(*Numbers[1]), FCString::Atof(*Numbers[2])) * 100.f;
			OutPlace.Label = Metres(OutPlace.Location);
			return true;
		}

		if (Rest.Num() == 1 && World)
		{
			const FString Suffix = TEXT("_") + Rest[0];
			TArray<AActor*> Matches;
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				const FString Name = It->GetName();
				if (Name.Equals(Rest[0], ESearchCase::IgnoreCase))
				{
					Matches.Reset();
					Matches.Add(*It);
					break;
				}
				if (Rest[0].Len() >= 5 && Name.EndsWith(Suffix, ESearchCase::IgnoreCase))
				{
					Matches.Add(*It);
				}
			}
			if (Matches.Num() == 1)
			{
				OutPlace.Location = Matches[0]->GetActorLocation();
				OutPlace.Label = Matches[0]->GetName();
				return true;
			}
			if (Matches.Num() > 1)
			{
				OutError = FString::Printf(TEXT("%d things end in \"%s\", e.g. %s and %s. Give the full name."), Matches.Num(), *Suffix, *Matches[0]->GetName(), *Matches[1]->GetName());
				return false;
			}
		}

		AFGTrain* Train = Control ? Control->FindTrain(Joined) : nullptr;
		const AFGRailroadVehicle* Vehicle = Train ? Train->GetFirstVehicle() : nullptr;
		if (Vehicle)
		{
			OutPlace.Location = Vehicle->GetActorLocation();
			OutPlace.Label = Train->GetTrainName().ToString();
			return true;
		}
		OutError = FString::Printf(TEXT("Nothing called \"%s\". Give x y z in metres, a name from the Auto Trains log (e.g. Build_RailroadBlockSignal_C_2147163402, or just 2147163402), or a train from /at list."), *Joined);
		return false;
	}

	FString MarkerPrefix(const APlayerController* Player)
	{
		const APlayerState* State = Player ? Player->GetPlayerState<APlayerState>() : nullptr;
		return FString::Printf(TEXT("AT %s"), State ? *State->GetPlayerName() : TEXT("player"));
	}

	// Returns the GUID of the marker named Prefix or "Prefix: ...", or an invalid GUID if there is none; OutIconID gets that
	// marker's icon, else another map marker's, else the game's default stamp icon.
	FGuid FindMarker(AFGMapManager* Maps, const FString& Prefix, int32& OutIconID)
	{
		TArray<FMapMarker> Markers;
		Maps->GetMapMarkers(Markers);
		OutIconID = UFGMapCompassSettings::Get()->mDefaultStampIconID;
		const FMapMarker* Other = Markers.FindByPredicate([](const FMapMarker& Marker) { return Marker.MapMarkerType == ERepresentationType::RT_MapMarker; });
		if (Other)
		{
			OutIconID = Other->IconID;
		}
		for (const FMapMarker& Marker : Markers)
		{
			if (Marker.Name.Equals(Prefix) || Marker.Name.StartsWith(Prefix + TEXT(":")))
			{
				OutIconID = Marker.IconID;
				return Marker.MarkerGUID;
			}
		}
		return FGuid();
	}

	// Puts Player's single /at where marker on Place, seen on every compass; returns the chat line saying so.
	FString MarkPlace(UWorld* World, const APlayerController* Player, const FATPlace& Place)
	{
		AFGMapManager* Maps = World ? AFGMapManager::Get(World) : nullptr;
		if (!Maps)
		{
			UE_LOG(LogAutoTrains, Warning, TEXT("/at where: no map manager, no marker"));
			return TEXT("(No map to mark it on.)");
		}
		const FString Prefix = MarkerPrefix(Player);
		FMapMarker Marker;
		const FGuid Existing = FindMarker(Maps, Prefix, Marker.IconID);
		Marker.Location = Place.Location;
		Marker.Name = (Prefix + TEXT(": ") + Place.Label).Left(60);
		Marker.MapMarkerType = ERepresentationType::RT_MapMarker;
		Marker.Color = FLinearColor(1.f, 0.35f, 0.f);
		Marker.Scale = 1.f;
		Marker.CompassViewDistance = ECompassViewDistance::CVD_Always;
		if (Existing.IsValid())
		{
			Marker.MarkerGUID = Existing;
			Maps->Authority_UpdateMapMarker(Marker);
			UE_LOG(LogAutoTrains, Log, TEXT("/at where: moved marker \"%s\" to %s"), *Marker.Name, *Metres(Place.Location));
			return FString::Printf(TEXT("Moved your marker \"%s\" there; it shows on the compass. /at where clear removes it."), *Marker.Name);
		}
		FMapMarker Added;
		if (!Maps->CanAddNewMapMarker() || !Maps->AddNewMapMarker(Marker, Added))
		{
			UE_LOG(LogAutoTrains, Warning, TEXT("/at where: the game refused marker \"%s\" (%d markers)"), *Marker.Name, Maps->GetNumMapMarkers());
			return TEXT("(The map has no room for another marker.)");
		}
		UE_LOG(LogAutoTrains, Log, TEXT("/at where: added marker \"%s\" at %s, icon %d"), *Marker.Name, *Metres(Place.Location), Marker.IconID);
		return FString::Printf(TEXT("Marked it as \"%s\"; it shows on the compass and map. /at where clear removes it."), *Marker.Name);
	}
}

AATChatCommand::AATChatCommand()
{
	CommandName = TEXT("at");
	Aliases.Add(TEXT("autotrains"));
	MinNumberOfArguments = 1;
	bOnlyUsableByPlayer = true;
	Usage = NSLOCTEXT("AutoTrains", "ChatUsage", "/at traffic on|off|status | reroute on|off | list | autopilot <train|all> on|off | removesignals confirm | settings | set <setting> <value> | where [<place>|clear] | tp <place>|back   (<train> = number from /at list, or part of its name; <place> = x y z in metres, a name from the log, or a train)");
}

bool AATChatCommand::MayChangeWorld(UCommandSender* Sender)
{
	const APlayerController* Player = Sender ? Sender->GetPlayer() : nullptr;
	if (!Player)
	{
		return false;
	}
	const AFGPlayerState* State = Player->GetPlayerState<AFGPlayerState>();
	if (Player->IsLocalController() || (State && State->IsServerAdmin()))
	{
		return true;
	}
	const AFGPlayerController* Controller = Cast<AFGPlayerController>(Player);
	const UWorld* World = Player->GetWorld();
	const EPrivilegeLevel Level = Controller ? Controller->GetDSPrivilegeLevel() : EPrivilegeLevel::None;
	return World && World->GetNetMode() == NM_DedicatedServer && (Level == EPrivilegeLevel::Administrator || Level == EPrivilegeLevel::InitialAdmin);
}

EExecutionStatus AATChatCommand::ExecuteCommand_Implementation(UCommandSender* Sender, const TArray<FString>& Arguments, const FString& Label)
{
	AATControlSubsystem* Control = AATControlSubsystem::Get(this);
	if (!Control)
	{
		Sender->SendChatMessage(TEXT("Auto Trains: control subsystem missing (only the host can use /at)."), FLinearColor::Red);
		UE_LOG(LogAutoTrains, Warning, TEXT("/at: no control subsystem"));
		return EExecutionStatus::UNCOMPLETED;
	}
	UE_LOG(LogAutoTrains, Log, TEXT("/at %s"), *FString::Join(Arguments, TEXT(" ")));

	const FString Verb = Arguments[0].ToLower();
	const bool bReadOnly = Verb == TEXT("list") || Verb == TEXT("settings") || Verb == TEXT("where") || (Verb == TEXT("traffic") && Arguments.Num() == 2 && Arguments[1].Equals(TEXT("status"), ESearchCase::IgnoreCase));
	const bool bAdmin = MayChangeWorld(Sender);
	if (!bReadOnly && !bAdmin)
	{
		const APlayerController* Player = Sender ? Sender->GetPlayer() : nullptr;
		const APlayerState* State = Player ? Player->GetPlayerState<APlayerState>() : nullptr;
		Sender->SendChatMessage(TEXT("Auto Trains: only the host or a server admin can use this /at command."), FLinearColor::Red);
		UE_LOG(LogAutoTrains, Log, TEXT("/at %s refused: %s isn't the host or a server admin"), *Verb, State ? *State->GetPlayerName() : TEXT("the sender"));
		return EExecutionStatus::INSUFFICIENT_PERMISSIONS;
	}
	if (Verb == TEXT("settings"))
	{
		Sender->SendChatMessage(TEXT("Auto Trains settings on this server (* = changed from the default):"));
		for (const FString& Line : UATConfig::DescribeForChat(this))
		{
			Sender->SendChatMessage(Line);
		}
		Sender->SendChatMessage(bAdmin ? TEXT("You're an admin: change one with /at set <setting> <value>.") : TEXT("Only the host or a server admin can change them."));
		const UWorld* World = GetWorld();
		if (World && World->GetNetMode() == NM_DedicatedServer)
		{
			Sender->SendChatMessage(TEXT("Your own Mods menu doesn't change a dedicated server's settings."));
		}
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("set"))
	{
		if (Arguments.Num() < 3)
		{
			Sender->SendChatMessage(TEXT("Usage: /at set <setting> <value>, e.g. /at set DetailedLog on. /at settings lists them."), FLinearColor::Red);
			return EExecutionStatus::BAD_ARGUMENTS;
		}
		FString Message;
		const bool bOk = UATConfig::Write(this, Arguments[1], Arguments[2], Message);
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: /at set %s %s: %s"), *Arguments[1], *Arguments[2], *Message);
		Sender->SendChatMessage(Message, bOk ? FLinearColor::White : FLinearColor::Red);
		return bOk ? EExecutionStatus::COMPLETED : EExecutionStatus::BAD_ARGUMENTS;
	}
	if (Verb == TEXT("where"))
	{
		return RunWhere(Sender, Arguments);
	}
	if (Verb == TEXT("tp"))
	{
		return RunTeleport(Sender, Arguments);
	}
	if (Verb == TEXT("list"))
	{
		for (const FString& Line : Control->ListTrains())
		{
			Sender->SendChatMessage(Line);
		}
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("log"))
	{
		Sender->SendChatMessage(TEXT("/at log is gone: use /at set DetailedLog on (or Mods > Auto Trains) instead."));
		return EExecutionStatus::BAD_ARGUMENTS;
	}
	if (Verb == TEXT("traffic") && Arguments.Num() == 2)
	{
		AATTrafficSubsystem* Traffic = AATTrafficSubsystem::Get(this);
		if (!Traffic)
		{
			Sender->SendChatMessage(TEXT("Auto Trains: traffic subsystem missing."), FLinearColor::Red);
			return EExecutionStatus::UNCOMPLETED;
		}
		const FString Mode = Arguments[1].ToLower();
		if (Mode == TEXT("on") || Mode == TEXT("off"))
		{
			Sender->SendChatMessage(Traffic->SetEnabled(Mode == TEXT("on")));
			return EExecutionStatus::COMPLETED;
		}
		if (Mode == TEXT("status"))
		{
			for (const FString& Line : Traffic->DescribeAll())
			{
				Sender->SendChatMessage(Line);
			}
			return EExecutionStatus::COMPLETED;
		}
	}
	if (Verb == TEXT("reroute") && Arguments.Num() == 2)
	{
		AATTrafficSubsystem* Traffic = AATTrafficSubsystem::Get(this);
		const bool bOn = Arguments[1].Equals(TEXT("on"), ESearchCase::IgnoreCase);
		if (Traffic && (bOn || Arguments[1].Equals(TEXT("off"), ESearchCase::IgnoreCase)))
		{
			Sender->SendChatMessage(Traffic->SetRerouteEnabled(bOn));
			return EExecutionStatus::COMPLETED;
		}
	}
	if (Verb == TEXT("removesignals"))
	{
		if (Arguments.Num() != 2 || !Arguments[1].Equals(TEXT("confirm"), ESearchCase::IgnoreCase))
		{
			Sender->SendChatMessage(TEXT("This dismantles every block and path signal in the world, also the ones that keep trains to one direction on one-way track. Type /at removesignals confirm to do it."));
			return EExecutionStatus::BAD_ARGUMENTS;
		}
		Sender->SendChatMessage(Control->RemoveAllSignals());
		return EExecutionStatus::COMPLETED;
	}
	const bool bAutopilotOn = Arguments.Num() == 3 && Arguments[2].Equals(TEXT("on"), ESearchCase::IgnoreCase);
	const bool bAutopilotArgs = Verb == TEXT("autopilot") && Arguments.Num() == 3 && (bAutopilotOn || Arguments[2].Equals(TEXT("off"), ESearchCase::IgnoreCase));
	if (bAutopilotArgs && Arguments[1].Equals(TEXT("all"), ESearchCase::IgnoreCase))
	{
		Sender->SendChatMessage(Control->SetAutopilotAll(bAutopilotOn));
		return EExecutionStatus::COMPLETED;
	}

	AFGTrain* Train = Arguments.Num() >= 2 ? Control->FindTrain(Arguments[1]) : nullptr;
	if (!Train)
	{
		Sender->SendChatMessage(Arguments.Num() >= 2 ? FString::Printf(TEXT("No train \"%s\". See /at list."), *Arguments[1]) : TEXT("Name a train. See /at list."));
		PrintCommandUsage(Sender);
		return EExecutionStatus::BAD_ARGUMENTS;
	}
	if (bAutopilotArgs)
	{
		Sender->SendChatMessage(Control->SetAutopilot(Train, bAutopilotOn));
		return EExecutionStatus::COMPLETED;
	}
	PrintCommandUsage(Sender);
	return EExecutionStatus::BAD_ARGUMENTS;
}

EExecutionStatus AATChatCommand::RunWhere(UCommandSender* Sender, const TArray<FString>& Arguments)
{
	APlayerController* Player = Sender ? Sender->GetPlayer() : nullptr;
	const APawn* Pawn = Player ? Player->GetPawn() : nullptr;
	UWorld* World = GetWorld();
	if (!Pawn)
	{
		Sender->SendChatMessage(TEXT("Auto Trains: you have no body in the world to measure from."), FLinearColor::Red);
		return EExecutionStatus::UNCOMPLETED;
	}
	if (Arguments.Num() == 2 && Arguments[1].Equals(TEXT("clear"), ESearchCase::IgnoreCase))
	{
		AFGMapManager* Maps = World ? AFGMapManager::Get(World) : nullptr;
		int32 IconID = 0;
		const FString Prefix = MarkerPrefix(Player);
		const FGuid Existing = Maps ? FindMarker(Maps, Prefix, IconID) : FGuid();
		if (Existing.IsValid())
		{
			Maps->Authority_RemoveMapMarkerByID(Existing);
			UE_LOG(LogAutoTrains, Log, TEXT("/at where: removed marker \"%s\""), *Prefix);
		}
		Sender->SendChatMessage(Existing.IsValid() ? TEXT("Removed your marker.") : TEXT("You have no /at where marker."));
		return EExecutionStatus::COMPLETED;
	}

	const FVector Here = Pawn->GetActorLocation();
	const float Facing = FRotator::ClampAxis(Player->GetControlRotation().Yaw + 90.f);
	if (Arguments.Num() == 1)
	{
		Sender->SendChatMessage(FString::Printf(TEXT("You're at %s, facing %s (metres, as in the Auto Trains log; north is -y, east is +x)."), *Metres(Here), CompassName(Facing)));
		return EExecutionStatus::COMPLETED;
	}

	FATPlace Place;
	FString Error;
	if (!FindPlace(World, AATControlSubsystem::Get(this), Arguments, 1, Place, Error))
	{
		Sender->SendChatMessage(Error, FLinearColor::Red);
		return EExecutionStatus::BAD_ARGUMENTS;
	}
	const FVector Delta = Place.Location - Here;
	const float Flat = FVector2D(Delta.X, Delta.Y).Size() / 100.f;
	const float Rise = Delta.Z / 100.f;
	const FString Height = Rise > 2.f ? FString::Printf(TEXT("%.0f m above you"), Rise) : Rise < -2.f ? FString::Printf(TEXT("%.0f m below you"), -Rise) : FString(TEXT("at your height"));
	FString Line;
	if (Flat < 5.f)
	{
		Line = FString::Printf(TEXT("%s %s: you're on it (%.0f m away), %s."), *Place.Label, *Metres(Place.Location), Flat, *Height);
	}
	else
	{
		const float ToPlace = Bearing(Delta);
		const float Turn = FRotator::NormalizeAxis(ToPlace - Facing);
		const FString Way = FMath::Abs(Turn) <= 10.f ? FString(TEXT("straight ahead")) : FMath::Abs(Turn) >= 170.f ? FString(TEXT("behind you"))
			: FString::Printf(TEXT("%.0f degrees to your %s"), FMath::Abs(Turn), Turn > 0.f ? TEXT("right") : TEXT("left"));
		Line = FString::Printf(TEXT("%s %s: %.0f m %s, %s; %s."), *Place.Label, *Metres(Place.Location), Flat, CompassName(ToPlace), *Height, *Way);
	}
	Sender->SendChatMessage(Line);
	Sender->SendChatMessage(MarkPlace(World, Player, Place));
	UE_LOG(LogAutoTrains, Log, TEXT("/at where: %s from %s"), *Line, *Metres(Here));
	return EExecutionStatus::COMPLETED;
}

EExecutionStatus AATChatCommand::RunTeleport(UCommandSender* Sender, const TArray<FString>& Arguments)
{
	APlayerController* Player = Sender ? Sender->GetPlayer() : nullptr;
	ACharacter* Character = Player ? Cast<ACharacter>(Player->GetPawn()) : nullptr;
	if (Arguments.Num() < 2)
	{
		Sender->SendChatMessage(TEXT("Usage: /at tp <x y z | name from the log | train>, or /at tp back."), FLinearColor::Red);
		return EExecutionStatus::BAD_ARGUMENTS;
	}
	if (!Character)
	{
		Sender->SendChatMessage(TEXT("Get out of the vehicle first: /at tp only moves you on foot."), FLinearColor::Red);
		return EExecutionStatus::UNCOMPLETED;
	}

	FATPlace Place;
	if (Arguments.Num() == 2 && Arguments[1].Equals(TEXT("back"), ESearchCase::IgnoreCase))
	{
		const FVector* From = mTeleportedFrom.Find(Player);
		if (!From)
		{
			Sender->SendChatMessage(TEXT("No /at tp to go back from since the session started."), FLinearColor::Red);
			return EExecutionStatus::UNCOMPLETED;
		}
		Place.Location = *From;
		Place.Label = TEXT("where you were");
	}
	else
	{
		FString Error;
		if (!FindPlace(GetWorld(), AATControlSubsystem::Get(this), Arguments, 1, Place, Error))
		{
			Sender->SendChatMessage(Error, FLinearColor::Red);
			return EExecutionStatus::BAD_ARGUMENTS;
		}
		// Lands above the place, so the character isn't put inside what it names.
		Place.Location.Z += 300.f;
	}

	const FVector Start = Character->GetActorLocation();
	if (!Character->TeleportTo(Place.Location, Character->GetActorRotation()))
	{
		Sender->SendChatMessage(TEXT("No room to stand there. Try x y z a few metres to the side."), FLinearColor::Red);
		UE_LOG(LogAutoTrains, Log, TEXT("/at tp: no room at %s"), *Metres(Place.Location));
		return EExecutionStatus::UNCOMPLETED;
	}
	mTeleportedFrom.Add(Player, Start);
	Sender->SendChatMessage(FString::Printf(TEXT("Moved you to %s %s. Mind passing trains. /at tp back returns you."), *Place.Label, *Metres(Character->GetActorLocation())));
	UE_LOG(LogAutoTrains, Log, TEXT("/at tp: %s moved from %s to %s %s"), *MarkerPrefix(Player), *Metres(Start), *Place.Label, *Metres(Character->GetActorLocation()));
	return EExecutionStatus::COMPLETED;
}
