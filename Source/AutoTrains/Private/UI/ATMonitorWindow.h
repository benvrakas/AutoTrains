#pragma once

#include "CoreMinimal.h"
#include "UI/ATPanel.h"
#include "ATMonitorWindow.generated.h"

class APlayerController;
class UHorizontalBox;
class UTextBlock;
class UVerticalBox;

// The traffic monitor opened by "/at monitor": tiles (trains, driving, waiting, docked, in trouble, the planner's round
// time against its budget, deadlocks passed through) over the trains in trouble and long waits, each with Show me. A
// popup on the viewport, refreshed every second; Close or Escape closes it.
UCLASS()
class UATMonitorWindow : public UATPanel
{
	GENERATED_BODY()

public:
	// Opens the window for Controller's local player shortly after the chat closes.
	static void Open(APlayerController* Controller);

	void Close();

protected:
	virtual UWidget* WrapRoot(UVerticalBox* Root) override;
	virtual void BuildContent(UVerticalBox* Root) override;
	virtual void OnHidden() override;
	virtual void RequestView(UATRemoteCallObject* RCO) override;
	virtual void Refresh(UATRemoteCallObject* RCO) override;
	virtual FReply NativeOnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent) override;

private:
	static void Create(APlayerController* Controller);
	void AddTile(const FString& Value, const FString& Label, bool bAlert);

	UPROPERTY()
	TObjectPtr<UHorizontalBox> mTiles;

	UPROPERTY()
	TObjectPtr<UVerticalBox> mRows;

	UPROPERTY()
	TObjectPtr<UTextBlock> mNote;

	TWeakObjectPtr<APlayerController> mController;
	bool bInputTaken = false;
	FString mShown;

	static TWeakObjectPtr<UATMonitorWindow> sOpen;
};
