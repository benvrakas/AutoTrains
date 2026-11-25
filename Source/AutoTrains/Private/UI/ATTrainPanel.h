#pragma once

#include "CoreMinimal.h"
#include "UI/ATPanel.h"
#include "ATTrainPanel.generated.h"

class AFGTrain;
class UATButton;
class UTextBlock;
struct FATTrainView;

// What the locomotive menu and the ride HUD show of a train Auto Trains drives: its state, its speed against what the
// plan asks, its next stop with the planned arrival, and what it waits for.
UCLASS(Abstract)
class UATTrainDisplay : public UATPanel
{
	GENERATED_BODY()

protected:
	void BuildTrainContent(UVerticalBox* Root, bool bCompact);
	void ShowTrain(const FATTrainView& View);

	virtual AFGTrain* GetTrain() const { return nullptr; }
	virtual void RequestView(UATRemoteCallObject* RCO) override;

	FBadge mState;

	UPROPERTY()
	TObjectPtr<UTextBlock> mSpeed;

	UPROPERTY()
	TObjectPtr<UTextBlock> mGoal;

	UPROPERTY()
	TObjectPtr<UVerticalBox> mWait;

	UPROPERTY()
	TObjectPtr<UTextBlock> mWaitFor;

	UPROPERTY()
	TObjectPtr<UTextBlock> mReason;

	UPROPERTY()
	TObjectPtr<UATButton> mShowMe;
};

// Auto Trains section at the bottom of the vanilla locomotive menu's Timetable column (Widget_Locomotive_Menu).
UCLASS()
class UATTrainPanel : public UATTrainDisplay
{
	GENERATED_BODY()

protected:
	virtual UWidget* WrapRoot(UVerticalBox* Root) override;
	virtual void BuildContent(UVerticalBox* Root) override;
	virtual void Refresh(UATRemoteCallObject* RCO) override;
	virtual AFGTrain* GetTrain() const override;
};

// The strip at the top right of the screen while the local player rides a train Auto Trains drives. It never takes input.
UCLASS()
class UATRideHud : public UATTrainDisplay
{
	GENERATED_BODY()

public:
	// Starts the client check that shows the strip while the local player rides a train (not on a dedicated server).
	static void Install();

	void SetTrain(AFGTrain* InTrain) { mTrain = InTrain; }

protected:
	virtual UWidget* WrapRoot(UVerticalBox* Root) override;
	virtual void BuildContent(UVerticalBox* Root) override;
	virtual void Refresh(UATRemoteCallObject* RCO) override;
	virtual AFGTrain* GetTrain() const override { return mTrain.Get(); }

private:
	TWeakObjectPtr<AFGTrain> mTrain;

	UPROPERTY()
	TObjectPtr<UUserWidget> mFontWidget;
};
