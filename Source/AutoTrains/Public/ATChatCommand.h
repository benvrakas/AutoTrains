#pragma once

#include "CoreMinimal.h"
#include "Command/ChatCommandInstance.h"
#include "ATChatCommand.generated.h"

// /at: test commands that control trains directly and log what they do.
UCLASS()
class AUTOTRAINS_API AATChatCommand : public AChatCommandInstance
{
	GENERATED_BODY()

public:
	AATChatCommand();

	virtual EExecutionStatus ExecuteCommand_Implementation(UCommandSender* Sender, const TArray<FString>& Arguments, const FString& Label) override;
};
