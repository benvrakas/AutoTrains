#include "ATChatCommand.h"
#include "ATConfig.h"
#include "ATControlSubsystem.h"
#include "ATTrafficSubsystem.h"
#include "AutoTrains.h"
#include "Command/CommandSender.h"
#include "FGPlayerController.h"
#include "FGPlayerState.h"
#include "FGTrain.h"
#include "Server/FGDSAuthenticationTypes.h"

AATChatCommand::AATChatCommand()
{
	CommandName = TEXT("at");
	Aliases.Add(TEXT("autotrains"));
	MinNumberOfArguments = 1;
	bOnlyUsableByPlayer = true;
	Usage = NSLOCTEXT("AutoTrains", "ChatUsage", "/at traffic on|off|status | reroute on|off | list | autopilot <train|all> on|off | removesignals confirm | settings | set <setting> <value>   (<train> = number from /at list, or part of its name)");
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
	const bool bReadOnly = Verb == TEXT("list") || Verb == TEXT("settings") || (Verb == TEXT("traffic") && Arguments.Num() == 2 && Arguments[1].Equals(TEXT("status"), ESearchCase::IgnoreCase));
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
