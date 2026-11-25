#pragma once

#include "CoreMinimal.h"
#include "FGRemoteCallObject.h"
#include "ATViews.h"
class AFGTrain;
#include "ATRemoteCallObject.generated.h"

// One setting changed in a client's Mods menu: its key and its value as UATConfig::Write takes it.
USTRUCT()
struct FATSettingChange
{
	GENERATED_BODY()

	UPROPERTY()
	FString Key;

	UPROPERTY()
	FString Value;
};

// Carries settings a server admin changed in their Mods menu to the server, which applies and saves them.
UCLASS(NotBlueprintable)
class AUTOTRAINS_API UATRemoteCallObject : public UFGRemoteCallObject
{
	GENERATED_BODY()

public:
	// Applies Changes to the server's configuration if the sending player may change the world (see /at).
	UFUNCTION(Server, Reliable, WithValidation)
	void Server_SetSettings(const TArray<FATSettingChange>& Changes);

	// Shows Text in the local player's chat.
	UFUNCTION(Client, Reliable)
	void Client_ShowMessage(const FString& Text);

	//~ UI: the server answers each request with the matching Client_ call.
	UFUNCTION(Server, Unreliable)
	void Server_RequestTrain(AFGTrain* Train);

	UFUNCTION(Server, Unreliable)
	void Server_RequestMonitor();

	// Puts this player's marker on the train Train waits for (or on Train).
	UFUNCTION(Server, Reliable)
	void Server_MarkTrain(AFGTrain* Train);

	UFUNCTION(Client, Unreliable)
	void Client_Train(const FATTrainView& View);

	UFUNCTION(Client, Unreliable)
	void Client_Monitor(const FATMonitorView& View);

	// "/at monitor": opens the monitor window on this player's screen.
	UFUNCTION(Client, Reliable)
	void Client_OpenMonitor();

	// The latest views on the client, and a number that changes whenever one arrives.
	const FATTrainView* GetTrainView(const AFGTrain* Train) const;
	const FATMonitorView& GetMonitorView() const { return mMonitor; }
	uint32 GetRevision() const { return mRevision; }

	// Returns the local player's instance, or nullptr (logged) if there is none.
	static UATRemoteCallObject* GetLocal(const UObject* WorldContext);

	// Shows Text in the local player's chat as an Auto Trains message.
	static void ShowInChat(const UObject* WorldContext, const FString& Text);

	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

private:
	// Returns the name of the player who owns this object, for messages and the log.
	FString GetPlayerName() const;

	UPROPERTY(Replicated, Meta = (NoAutoJson))
	bool mForceNetField_UATRemoteCallObject = false;

	TMap<TWeakObjectPtr<AFGTrain>, FATTrainView> mTrainViews;
	FATMonitorView mMonitor;
	uint32 mRevision = 0;
};
