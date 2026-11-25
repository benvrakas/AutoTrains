#include "UI/ATTrainPanel.h"
#include "ATRemoteCallObject.h"
#include "ATViews.h"
#include "AutoTrains.h"
#include "Blueprint/WidgetTree.h"
#include "Components/Border.h"
#include "Components/HorizontalBox.h"
#include "Components/SizeBox.h"
#include "Components/TextBlock.h"
#include "Components/VerticalBox.h"
#include "Containers/Ticker.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "FGCharacterPlayer.h"
#include "FGDriveablePawn.h"
#include "FGLocomotive.h"
#include "FGTrain.h"
#include "GameFramework/PlayerController.h"

#define LOCTEXT_NAMESPACE "AutoTrains"

namespace
{
	// The HUD strip: its width and its distance from the screen's top right corner, above SmartTrains' card. [slate units]
	constexpr float HudWidth = 380.f;
	const FVector2D HudOffset(-24.f, 60.f);
	constexpr float HudCheckSeconds = 0.5f;
	const TCHAR* MenuLabelClass = TEXT("/Game/FactoryGame/Buildable/Vehicle/Train/Locomotive/UI/BPW_TrainMenuLabel.BPW_TrainMenuLabel_C");

	TWeakObjectPtr<UATRideHud> GHud;

	UObject* GetObjectProperty(const UObject* Owner, FName Name)
	{
		const FObjectPropertyBase* Property = Owner ? FindFProperty<FObjectPropertyBase>(Owner->GetClass(), Name) : nullptr;
		return Property ? Property->GetObjectPropertyValue_InContainer(Owner) : nullptr;
	}

	AFGTrain* GetRiddenTrain(APlayerController* Controller)
	{
		APawn* Pawn = Controller ? Controller->GetPawn() : nullptr;
		const AFGLocomotive* Locomotive = Cast<AFGLocomotive>(Pawn);
		if (!Locomotive)
		{
			if (const AFGCharacterPlayer* Character = Cast<AFGCharacterPlayer>(Pawn))
			{
				Locomotive = Cast<AFGLocomotive>(Character->GetDrivenVehicle());
			}
		}
		return Locomotive ? Locomotive->GetTrain() : nullptr;
	}

	bool TickRideHud(float)
	{
		UWorld* World = GEngine && GEngine->GameViewport ? GEngine->GameViewport->GetWorld() : nullptr;
		APlayerController* Controller = World ? World->GetFirstPlayerController() : nullptr;
		AFGTrain* Train = Controller && Controller->IsLocalController() ? GetRiddenTrain(Controller) : nullptr;
		if (GHud.IsValid() && (!Train || GHud->GetWorld() != World))
		{
			GHud->RemoveFromParent();
			GHud.Reset();
		}
		if (Train && !GHud.IsValid())
		{
			UATRideHud* Hud = CreateWidget<UATRideHud>(Controller, UATRideHud::StaticClass());
			if (!Hud)
			{
				return true;
			}
			Hud->SetTrain(Train);
			Hud->AddToViewport(5);
			// Position first: setting it puts the anchors back to the top left corner.
			Hud->SetPositionInViewport(HudOffset, false);
			Hud->SetAnchorsInViewport(FAnchors(1.f, 0.f));
			Hud->SetAlignmentInViewport(FVector2D(1.f, 0.f));
			GHud = Hud;
		}
		else if (Train)
		{
			GHud->SetTrain(Train);
		}
		return true;
	}
}

//~ UATTrainDisplay

void UATTrainDisplay::BuildTrainContent(UVerticalBox* Root, bool bCompact)
{
	UHorizontalBox* Top = MakeHBox(Root, FMargin(0.f, 2.f));
	mState = MakeBadge(Top);
	mSpeed = MakeText(Top, FText::GetEmpty(), bCompact ? 12 : 13, ATStyle::Text, true, false, FMargin(8.f, 0.f, 0.f, 0.f));
	SetFill(mSpeed);
	SetVAlignCenter(mSpeed);
	mGoal = MakeText(Root, FText::GetEmpty(), 11, ATStyle::Text, false, true, FMargin(0.f, 3.f, 0.f, 0.f));
	mWait = MakeCard(Root, ATStyle::Inset, FMargin(7.f, 5.f), FMargin(0.f, 5.f, 0.f, 0.f));
	UHorizontalBox* WaitLine = MakeHBox(mWait);
	mWaitFor = MakeText(WaitLine, FText::GetEmpty(), 11, ATStyle::Warn, true, true);
	SetFill(mWaitFor);
	SetVAlignCenter(mWaitFor);
	if (!bCompact)
	{
		mShowMe = MakeButton(WaitLine, LOCTEXT("ShowMe", "Show me"), [this]()
		{
			UATRemoteCallObject* RCO = UATRemoteCallObject::GetLocal(this);
			if (RCO && GetTrain())
			{
				RCO->Server_MarkTrain(GetTrain());
			}
		}, FMargin(6.f, 0.f, 0.f, 0.f));
		SetVAlignCenter(mShowMe);
	}
	mReason = MakeText(mWait, FText::GetEmpty(), 9, ATStyle::Muted, false, true, FMargin(0.f, 3.f, 0.f, 0.f));
	if (bCompact)
	{
		mReason->SetVisibility(ESlateVisibility::Collapsed);
	}
}

void UATTrainDisplay::RequestView(UATRemoteCallObject* RCO)
{
	if (AFGTrain* Train = GetTrain())
	{
		RCO->Server_RequestTrain(Train);
	}
}

void UATTrainDisplay::ShowTrain(const FATTrainView& View)
{
	SetBadge(mState, ATStyle::StateText(View.State), ATStyle::StateColor(View.State));
	if (View.State == 0)
	{
		mSpeed->SetText(View.bTraffic ? LOCTEXT("NotDriven", "Self-driving is off: Auto Trains leaves it alone.") : LOCTEXT("TrafficOff", "Traffic control is off (/at traffic on)."));
	}
	else if (View.State == 3)
	{
		mSpeed->SetText(LOCTEXT("DockedNow", "In its station"));
	}
	else
	{
		// Its speed, and the speed the plan asks for: it slows or speeds up to keep its slots.
		const bool bEasing = FMath::Abs(View.TargetSpeed - View.Speed) >= 5.f;
		mSpeed->SetText(FText::FromString(bEasing ? FString::Printf(TEXT("%.0f km/h  →  %.0f km/h"), View.Speed, View.TargetSpeed) : FString::Printf(TEXT("%.0f km/h"), View.Speed)));
	}
	const bool bGoal = !View.Goal.IsEmpty();
	mGoal->SetText(bGoal ? FText::FromString(View.ArrivalSeconds >= 0.f ? FString::Printf(TEXT("To %s · arrives in %s"), *View.Goal, *ATStyle::Duration(View.ArrivalSeconds))
		: FString::Printf(TEXT("To %s"), *View.Goal)) : FText::GetEmpty());
	mGoal->SetVisibility(bGoal && View.State != 0 ? ESlateVisibility::SelfHitTestInvisible : ESlateVisibility::Collapsed);
	const bool bWaits = !View.WaitingFor.IsEmpty() && View.State != 0 && View.State != 3;
	mWait->GetParent()->SetVisibility(bWaits ? ESlateVisibility::SelfHitTestInvisible : ESlateVisibility::Collapsed);
	if (bWaits)
	{
		mWaitFor->SetText(FText::FromString(FString::Printf(TEXT("Waiting for %s · %s"), *View.WaitingFor, *ATStyle::Duration(View.WaitedSeconds))));
		Tint(mWaitFor, View.State == 4 ? ATStyle::Orange : ATStyle::Warn);
		mReason->SetText(FText::FromString(View.Reason));
	}
}

//~ UATTrainPanel

UWidget* UATTrainPanel::WrapRoot(UVerticalBox* Root)
{
	UBorder* Border = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Border->SetBrush(ATStyle::Rounded(ATStyle::Panel));
	Border->SetPadding(FMargin(8.f, 6.f, 8.f, 8.f));
	Border->SetContent(Root);
	USizeBox* Spacing = WidgetTree->ConstructWidget<USizeBox>(USizeBox::StaticClass());
	Spacing->AddChild(Border);
	return Spacing;
}

void UATTrainPanel::BuildContent(UVerticalBox* Root)
{
	MakeTitleBar(Root, FText::GetEmpty(), 20.f);
	BuildTrainContent(Root, false);
}

AFGTrain* UATTrainPanel::GetTrain() const
{
	const UUserWidget* Menu = GetMenu();
	if (AFGTrain* Train = Cast<AFGTrain>(GetObjectProperty(Menu, TEXT("mTrain"))))
	{
		return Train;
	}
	const AFGLocomotive* Locomotive = Cast<AFGLocomotive>(GetObjectProperty(Menu, TEXT("FGLocomotive")));
	if (!Locomotive)
	{
		Locomotive = Cast<AFGLocomotive>(GetObjectProperty(Menu, TEXT("mInteractObject")));
	}
	return Locomotive ? Locomotive->GetTrain() : nullptr;
}

void UATTrainPanel::Refresh(UATRemoteCallObject* RCO)
{
	if (const FATTrainView* View = RCO->GetTrainView(GetTrain()))
	{
		ShowTrain(*View);
	}
}

//~ UATRideHud

void UATRideHud::SetTrain(AFGTrain* InTrain)
{
	mTrain = InTrain;
}

void UATRideHud::Install()
{
	static FTSTicker::FDelegateHandle Handle;
	if (!Handle.IsValid())
	{
		Handle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateStatic(&TickRideHud), HudCheckSeconds);
		UE_LOG(LogAutoTrains, Log, TEXT("UI: ride HUD check started"));
	}
}

UWidget* UATRideHud::WrapRoot(UVerticalBox* Root)
{
	UBorder* Frame = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Frame->SetBrush(ATStyle::Outlined(ATStyle::Panel, ATStyle::Line, 1.f, 5.f));
	Frame->SetPadding(FMargin(10.f, 6.f));
	Frame->SetContent(Root);
	USizeBox* Width = WidgetTree->ConstructWidget<USizeBox>(USizeBox::StaticClass());
	Width->SetWidthOverride(HudWidth);
	Width->AddChild(Frame);
	return Width;
}

void UATRideHud::BuildContent(UVerticalBox* Root)
{
	if (UClass* LabelClass = LoadClass<UUserWidget>(nullptr, MenuLabelClass))
	{
		mFontWidget = WidgetTree->ConstructWidget<UUserWidget>(LabelClass);
		Root->AddChildToVerticalBox(mFontWidget);
		mFontWidget->SetVisibility(ESlateVisibility::Collapsed);
		SetFontSource(mFontWidget);
	}
	UHorizontalBox* Bar = MakeHBox(Root);
	USizeBox* LogoBox = WidgetTree->ConstructWidget<USizeBox>(USizeBox::StaticClass());
	LogoBox->SetWidthOverride(18.f);
	LogoBox->SetHeightOverride(18.f);
	LogoBox->AddChild(WidgetTree->ConstructWidget<UATLogo>(UATLogo::StaticClass()));
	Bar->AddChildToHorizontalBox(LogoBox);
	SetVAlignCenter(LogoBox);
	UVerticalBox* Body = MakeVBox(Bar, FMargin(8.f, 0.f, 0.f, 0.f));
	SetFill(Body);
	BuildTrainContent(Body, true);
	SetVisibility(ESlateVisibility::Collapsed);
}

void UATRideHud::Refresh(UATRemoteCallObject* RCO)
{
	const FATTrainView* View = RCO->GetTrainView(GetTrain());
	if (!View || View->State == 0)
	{
		SetVisibility(ESlateVisibility::Collapsed);
		return;
	}
	SetVisibility(ESlateVisibility::HitTestInvisible);
	ShowTrain(*View);
}

#undef LOCTEXT_NAMESPACE
