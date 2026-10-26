#include "ATChatCommand.h"
#include "ATControlSubsystem.h"
#include "AutoTrains.h"
#include "Command/CommandSender.h"
#include "FGPlayerController.h"
#include "FGTrain.h"

AATChatCommand::AATChatCommand()
{
	CommandName = TEXT("at");
	Aliases.Add(TEXT("autotrains"));
	MinNumberOfArguments = 1;
	bOnlyUsableByPlayer = true;
	Usage = NSLOCTEXT("AutoTrains", "ChatUsage", "/at list | speed <train> <km/h> | stopat <train> <m> | release <train|all> | signals <train> ignore|obey | route <train> check|via | log on|off   (<train> = number from /at list, or part of its name)");
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
	if (Verb == TEXT("list"))
	{
		for (const FString& Line : Control->ListTrains())
		{
			Sender->SendChatMessage(Line);
		}
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("log") && Arguments.Num() == 2)
	{
		Sender->SendChatMessage(Control->SetLogging(Arguments[1].Equals(TEXT("on"), ESearchCase::IgnoreCase)));
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("release") && Arguments.Num() == 2 && Arguments[1].Equals(TEXT("all"), ESearchCase::IgnoreCase))
	{
		Sender->SendChatMessage(Control->ReleaseAll());
		return EExecutionStatus::COMPLETED;
	}

	AFGTrain* Train = Arguments.Num() >= 2 ? Control->FindTrain(Arguments[1]) : nullptr;
	if (!Train)
	{
		Sender->SendChatMessage(Arguments.Num() >= 2 ? FString::Printf(TEXT("No train \"%s\". See /at list."), *Arguments[1]) : TEXT("Name a train. See /at list."));
		PrintCommandUsage(Sender);
		return EExecutionStatus::BAD_ARGUMENTS;
	}
	if (Verb == TEXT("speed") && Arguments.Num() == 3 && Arguments[2].IsNumeric())
	{
		Sender->SendChatMessage(Control->SetSpeedCap(Train, FCString::Atof(*Arguments[2])));
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("stopat") && Arguments.Num() == 3 && Arguments[2].IsNumeric())
	{
		Sender->SendChatMessage(Control->SetStopAhead(Train, FCString::Atof(*Arguments[2])));
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("release") && Arguments.Num() == 2)
	{
		Sender->SendChatMessage(Control->Release(Train));
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("signals") && Arguments.Num() == 3)
	{
		const bool bIgnore = Arguments[2].Equals(TEXT("ignore"), ESearchCase::IgnoreCase);
		if (bIgnore || Arguments[2].Equals(TEXT("obey"), ESearchCase::IgnoreCase))
		{
			Sender->SendChatMessage(Control->SetIgnoreSignals(Train, bIgnore));
			return EExecutionStatus::COMPLETED;
		}
	}
	if (Verb == TEXT("route") && Arguments.Num() == 3)
	{
		if (Arguments[2].Equals(TEXT("check"), ESearchCase::IgnoreCase))
		{
			Sender->SendChatMessage(Control->CheckRoute(Train));
			return EExecutionStatus::COMPLETED;
		}
		const APawn* Pawn = Sender->GetPlayer() ? Sender->GetPlayer()->GetPawn() : nullptr;
		if (Arguments[2].Equals(TEXT("via"), ESearchCase::IgnoreCase) && Pawn)
		{
			Sender->SendChatMessage(Control->RouteVia(Train, Pawn->GetActorLocation()));
			return EExecutionStatus::COMPLETED;
		}
	}
	PrintCommandUsage(Sender);
	return EExecutionStatus::BAD_ARGUMENTS;
}
