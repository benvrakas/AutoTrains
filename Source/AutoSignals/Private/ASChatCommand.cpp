#include "ASChatCommand.h"
#include "ASTestSignalSubsystem.h"
#include "AutoSignals.h"
#include "Command/CommandSender.h"
#include "FGPlayerController.h"

AASChatCommand::AASChatCommand()
{
	CommandName = TEXT("sigtest");
	MinNumberOfArguments = 1;
	bOnlyUsableByPlayer = true;
	Usage = NSLOCTEXT("AutoSignals", "ChatUsage", "/sigtest place block|path [both]  |  hide  |  show  |  clear  |  info  |  watch on|off");
}

EExecutionStatus AASChatCommand::ExecuteCommand_Implementation(UCommandSender* Sender, const TArray<FString>& Arguments, const FString& Label)
{
	AASTestSignalSubsystem* Signals = AASTestSignalSubsystem::Get(this);
	if (!Signals)
	{
		Sender->SendChatMessage(TEXT("AutoSignals: test subsystem missing (only the host can use /sigtest)."), FLinearColor::Red);
		UE_LOG(LogAutoSignals, Warning, TEXT("/sigtest: no test signal subsystem"));
		return EExecutionStatus::UNCOMPLETED;
	}

	const FString Verb = Arguments[0].ToLower();
	UE_LOG(LogAutoSignals, Log, TEXT("/sigtest %s"), *FString::Join(Arguments, TEXT(" ")));
	if (Verb == TEXT("place") && (Arguments.Num() == 2 || Arguments.Num() == 3))
	{
		const bool bPath = Arguments[1].Equals(TEXT("path"), ESearchCase::IgnoreCase);
		const bool bBlock = Arguments[1].Equals(TEXT("block"), ESearchCase::IgnoreCase);
		const bool bBoth = Arguments.Num() == 3 && Arguments[2].Equals(TEXT("both"), ESearchCase::IgnoreCase);
		AFGPlayerController* Controller = Sender->GetPlayer();
		const APawn* Pawn = Controller ? Controller->GetPawn() : nullptr;
		if ((!bPath && !bBlock) || (Arguments.Num() == 3 && !bBoth) || !Pawn)
		{
			PrintCommandUsage(Sender);
			return EExecutionStatus::BAD_ARGUMENTS;
		}
		const FString Message = Signals->PlaceSignals(Pawn->GetActorLocation(), Controller->GetControlRotation().Vector(), bPath, bBoth);
		Sender->SendChatMessage(Message);
		UE_LOG(LogAutoSignals, Log, TEXT("%s"), *Message);
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("hide") || Verb == TEXT("show"))
	{
		Signals->SetSignalsHidden(Verb == TEXT("hide"));
		Sender->SendChatMessage(Verb == TEXT("hide") ? TEXT("Test signals hidden.") : TEXT("Test signals shown."));
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("clear"))
	{
		Sender->SendChatMessage(FString::Printf(TEXT("Removed %d test signals."), Signals->ClearSignals()));
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("info"))
	{
		for (const FString& Line : Signals->LogState())
		{
			Sender->SendChatMessage(Line);
		}
		return EExecutionStatus::COMPLETED;
	}
	if (Verb == TEXT("watch") && Arguments.Num() == 2)
	{
		const bool bOn = Arguments[1].Equals(TEXT("on"), ESearchCase::IgnoreCase);
		Signals->SetWatching(bOn);
		Sender->SendChatMessage(bOn ? TEXT("Logging train and signal changes every second.") : TEXT("Stopped logging changes."));
		return EExecutionStatus::COMPLETED;
	}
	PrintCommandUsage(Sender);
	return EExecutionStatus::BAD_ARGUMENTS;
}
