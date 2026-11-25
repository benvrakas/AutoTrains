#include "ATRemoteCallObject.h"
#include "ATChatCommand.h"
#include "ATConfig.h"
#include "ATTrafficSubsystem.h"
#include "FGTrain.h"
#include "UI/ATMonitorWindow.h"
#include "AutoTrains.h"
#include "FGChatManager.h"
#include "FGPlayerController.h"
#include "GameFramework/GameStateBase.h"
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"

namespace
{
	// Most settings one message may carry, and the longest key and value; the configuration is well inside these.
	constexpr int32 MaxChanges = 64;
	constexpr int32 MaxTextLength = 64;

	const FLinearColor MessageColor(1.f, 0.6f, 0.2f);

	FChatMessageStruct MakeMessage(const UWorld* World, const FString& Text)
	{
		FChatMessageStruct Message;
		Message.MessageText = FText::FromString(Text);
		Message.MessageType = EFGChatMessageType::CMT_CustomMessage;
		Message.MessageSender = FText::FromString(TEXT("Auto Trains"));
		Message.MessageSenderColor = MessageColor;
		const AGameStateBase* GameState = World ? World->GetGameState() : nullptr;
		Message.ServerTimeStamp = GameState ? GameState->GetServerWorldTimeSeconds() : 0.f;
		return Message;
	}
}

void UATRemoteCallObject::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UATRemoteCallObject, mForceNetField_UATRemoteCallObject);
}

UATRemoteCallObject* UATRemoteCallObject::GetLocal(const UObject* WorldContext)
{
	AFGPlayerController* Controller = WorldContext ? Cast<AFGPlayerController>(UGameplayStatics::GetPlayerController(WorldContext, 0)) : nullptr;
	if (!Controller)
	{
		UE_LOG(LogAutoTrains, Warning, TEXT("Settings: no local player controller to send settings through"));
		return nullptr;
	}
	UATRemoteCallObject* RCO = Controller->GetRemoteCallObjectOfClass<UATRemoteCallObject>();
	if (!RCO)
	{
		UE_LOG(LogAutoTrains, Warning, TEXT("Settings: remote call object not registered on the local player (is Auto Trains on the server?)"));
	}
	return RCO;
}

void UATRemoteCallObject::ShowInChat(const UObject* WorldContext, const FString& Text)
{
	UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
	if (AFGChatManager* ChatManager = World ? AFGChatManager::Get(World) : nullptr)
	{
		ChatManager->AddChatMessageToReceived(MakeMessage(World, Text));
	}
}

FString UATRemoteCallObject::GetPlayerName() const
{
	const AFGPlayerController* Controller = GetOwnerPlayerController();
	const APlayerState* State = Controller ? Controller->GetPlayerState<APlayerState>() : nullptr;
	return State ? State->GetPlayerName() : GetNameSafe(Controller);
}

void UATRemoteCallObject::Client_ShowMessage_Implementation(const FString& Text)
{
	ShowInChat(this, Text);
}

bool UATRemoteCallObject::Server_SetSettings_Validate(const TArray<FATSettingChange>& Changes)
{
	return Changes.Num() <= MaxChanges && !Changes.ContainsByPredicate([](const FATSettingChange& Change)
	{
		return Change.Key.Len() > MaxTextLength || Change.Value.Len() > MaxTextLength;
	});
}

void UATRemoteCallObject::Server_SetSettings_Implementation(const TArray<FATSettingChange>& Changes)
{
	if (!AATChatCommand::MayChangeWorld(GetOwnerPlayerController()))
	{
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: ignored %d Mods menu changes from %s: not a server admin"), Changes.Num(), *GetPlayerName());
		Client_ShowMessage(TEXT("This server keeps its own Auto Trains settings, and only a server admin can change them. Your change still applies in games you host."));
		return;
	}

	TArray<FString> Applied;
	TArray<FString> Refused;
	for (const FATSettingChange& Change : Changes)
	{
		FString Message;
		if (UATConfig::Write(this, Change.Key, Change.Value, Message))
		{
			Applied.Add(FString::Printf(TEXT("%s = %s"), *Change.Key, *Change.Value));
		}
		else
		{
			Refused.Add(Message);
		}
		UE_LOG(LogAutoTrains, Log, TEXT("Settings: Mods menu of %s: %s %s: %s"), *GetPlayerName(), *Change.Key, *Change.Value, *Message);
	}
	if (Refused.Num() > 0)
	{
		Client_ShowMessage(FString::Printf(TEXT("The server didn't take: %s"), *FString::Join(Refused, TEXT(" "))));
	}
	if (Applied.Num() == 0)
	{
		return;
	}
	const FString Text = FString::Printf(TEXT("%s changed the server's settings: %s."), *GetPlayerName(), *FString::Join(Applied, TEXT(", ")));
	if (AFGChatManager* ChatManager = AFGChatManager::Get(GetWorld()))
	{
		ChatManager->BroadcastChatMessage(MakeMessage(GetWorld(), Text));
	}
}

void UATRemoteCallObject::Server_RequestTrain_Implementation(AFGTrain* Train)
{
	if (AATTrafficSubsystem* Traffic = AATTrafficSubsystem::Get(this); Traffic && Train)
	{
		Client_Train(Traffic->MakeTrainView(Train));
	}
}

void UATRemoteCallObject::Server_RequestMonitor_Implementation()
{
	if (AATTrafficSubsystem* Traffic = AATTrafficSubsystem::Get(this))
	{
		Client_Monitor(Traffic->MakeMonitorView());
	}
}

void UATRemoteCallObject::Server_MarkTrain_Implementation(AFGTrain* Train)
{
	AATTrafficSubsystem* Traffic = AATTrafficSubsystem::Get(this);
	if (Traffic && Train)
	{
		const FString Line = Traffic->MarkForPlayer(Train, GetOwnerPlayerController());
		UE_LOG(LogAutoTrains, Log, TEXT("UI: %s marked the way of %s: %s"), *GetPlayerName(), *Train->GetTrainName().ToString(), *Line);
		Client_ShowMessage(Line);
	}
}

void UATRemoteCallObject::Client_Train_Implementation(const FATTrainView& View)
{
	if (View.Train)
	{
		mTrainViews.Add(View.Train, View);
		++mRevision;
	}
}

void UATRemoteCallObject::Client_Monitor_Implementation(const FATMonitorView& View)
{
	mMonitor = View;
	++mRevision;
}

void UATRemoteCallObject::Client_OpenMonitor_Implementation()
{
	UATMonitorWindow::Open(GetOwnerPlayerController());
}

const FATTrainView* UATRemoteCallObject::GetTrainView(const AFGTrain* Train) const
{
	return Train ? mTrainViews.Find(const_cast<AFGTrain*>(Train)) : nullptr;
}
