#pragma once

#include "CoreMinimal.h"
#include "Command/ChatCommandInstance.h"
#include "ASChatCommand.generated.h"

// /sigtest: places, hides and removes test signals and logs train and block state.
UCLASS()
class AUTOSIGNALS_API AASChatCommand : public AChatCommandInstance
{
	GENERATED_BODY()

public:
	AASChatCommand();

	virtual EExecutionStatus ExecuteCommand_Implementation(UCommandSender* Sender, const TArray<FString>& Arguments, const FString& Label) override;
};
