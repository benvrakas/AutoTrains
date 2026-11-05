#pragma once

#include "CoreMinimal.h"
#include "Command/ChatCommandInstance.h"
#include "ATChatCommand.generated.h"

// /at: traffic switches and status, signal removal, session rename, telemetry and train test commands.
UCLASS()
class AUTOTRAINS_API AATChatCommand : public AChatCommandInstance
{
	GENERATED_BODY()

public:
	AATChatCommand();

	// Returns true if Sender is the host's own player, a player the game flags as server admin, or on a dedicated server a
	// player whose privilege level is Administrator.
	static bool MayChangeWorld(UCommandSender* Sender);

	virtual EExecutionStatus ExecuteCommand_Implementation(UCommandSender* Sender, const TArray<FString>& Arguments, const FString& Label) override;
};
