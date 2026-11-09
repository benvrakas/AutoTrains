#pragma once

#include "CoreMinimal.h"
#include "Command/ChatCommandInstance.h"
#include "ATChatCommand.generated.h"

// /at: traffic switches and status, signal removal, settings, autopilot, and finding places (where, tp).
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

private:
	// /at where [<place>]: tells the sender where they are, and with a place how far and which way it is, and marks it.
	EExecutionStatus RunWhere(UCommandSender* Sender, const TArray<FString>& Arguments);

	// /at tp <place> | back: moves the sender's character to a place, or back to where its last /at tp started.
	EExecutionStatus RunTeleport(UCommandSender* Sender, const TArray<FString>& Arguments);

	// Where each player's last /at tp started, for /at tp back. Not saved.
	TMap<TWeakObjectPtr<APlayerController>, FVector> mTeleportedFrom;
};
