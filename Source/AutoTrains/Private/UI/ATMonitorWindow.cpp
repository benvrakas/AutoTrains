#include "UI/ATMonitorWindow.h"
#include "ATRemoteCallObject.h"
#include "ATViews.h"
#include "AutoTrains.h"
#include "Blueprint/WidgetBlueprintLibrary.h"
#include "Blueprint/WidgetTree.h"
#include "Components/Border.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/Overlay.h"
#include "Components/OverlaySlot.h"
#include "Components/ScrollBox.h"
#include "Components/SizeBox.h"
#include "Components/TextBlock.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "FGTrain.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "TimerManager.h"

#define LOCTEXT_NAMESPACE "AutoTrains"

TWeakObjectPtr<UATMonitorWindow> UATMonitorWindow::sOpen;

namespace
{
	constexpr float MonitorWidth = 980.f;
	constexpr float MonitorMaxHeight = 760.f;
	// Wait after the command before taking the input, so the closing chat doesn't take it back. [s]
	constexpr float OpenDelay = 0.2f;
}

void UATMonitorWindow::Open(APlayerController* Controller)
{
	if (!Controller || !Controller->IsLocalController())
	{
		return;
	}
	FTimerHandle Handle;
	TWeakObjectPtr<APlayerController> Weak(Controller);
	Controller->GetWorldTimerManager().SetTimer(Handle, FTimerDelegate::CreateLambda([Weak]()
	{
		if (APlayerController* Still = Weak.Get())
		{
			Create(Still);
		}
	}), OpenDelay, false);
}

void UATMonitorWindow::Create(APlayerController* Controller)
{
	if (sOpen.IsValid() && sOpen->IsInViewport())
	{
		return;
	}
	UATMonitorWindow* Window = CreateWidget<UATMonitorWindow>(Controller, UATMonitorWindow::StaticClass());
	if (!Window)
	{
		UE_LOG(LogAutoTrains, Error, TEXT("UI: monitor window could not be created"));
		return;
	}
	Window->mController = Controller;
	Window->SetIsFocusable(true);
	Window->AddToViewport(100);
	sOpen = Window;
	UWidgetBlueprintLibrary::SetInputMode_UIOnlyEx(Controller, Window, EMouseLockMode::DoNotLock);
	Controller->SetShowMouseCursor(true);
	Window->bInputTaken = true;
	UE_LOG(LogAutoTrains, Log, TEXT("UI: monitor window opened"));
}

void UATMonitorWindow::Close()
{
	RemoveFromParent();
}

void UATMonitorWindow::OnHidden()
{
	if (bInputTaken)
	{
		bInputTaken = false;
		if (APlayerController* Controller = mController.Get())
		{
			UWidgetBlueprintLibrary::SetInputMode_GameOnly(Controller);
			Controller->SetShowMouseCursor(false);
		}
	}
	if (sOpen.Get() == this)
	{
		sOpen.Reset();
	}
}

FReply UATMonitorWindow::NativeOnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent)
{
	if (InKeyEvent.GetKey() == EKeys::Escape)
	{
		Close();
		return FReply::Handled();
	}
	return Super::NativeOnKeyDown(InGeometry, InKeyEvent);
}

UWidget* UATMonitorWindow::WrapRoot(UVerticalBox* Root)
{
	UOverlay* Screen = WidgetTree->ConstructWidget<UOverlay>(UOverlay::StaticClass());
	UBorder* Dim = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Dim->SetBrushColor(FLinearColor(0.f, 0.f, 0.f, 0.5f));
	Dim->SetVisibility(ESlateVisibility::Visible);
	if (UOverlaySlot* DimSlot = Screen->AddChildToOverlay(Dim))
	{
		DimSlot->SetHorizontalAlignment(HAlign_Fill);
		DimSlot->SetVerticalAlignment(VAlign_Fill);
	}
	UBorder* Frame = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Frame->SetBrush(ATStyle::Outlined(ATStyle::Panel, ATStyle::Line, 1.f, 6.f));
	Frame->SetPadding(FMargin(18.f, 14.f));
	USizeBox* Size = WidgetTree->ConstructWidget<USizeBox>(USizeBox::StaticClass());
	Size->SetWidthOverride(MonitorWidth);
	Size->SetMaxDesiredHeight(MonitorMaxHeight);
	Size->AddChild(Root);
	Frame->SetContent(Size);
	if (UOverlaySlot* FrameSlot = Screen->AddChildToOverlay(Frame))
	{
		FrameSlot->SetHorizontalAlignment(HAlign_Center);
		FrameSlot->SetVerticalAlignment(VAlign_Center);
	}
	return Screen;
}

void UATMonitorWindow::BuildContent(UVerticalBox* Root)
{
	UHorizontalBox* Title = MakeTitleBar(Root, LOCTEXT("MonitorHeading", "Traffic monitor"), 30.f);
	UATButton* CloseButton = MakeButton(Title, LOCTEXT("Close", "Close"), [this]() { Close(); });
	SetVAlignCenter(CloseButton);
	mTiles = MakeHBox(Root, FMargin(0.f, 2.f, 0.f, 8.f));
	MakeText(Root, FText::FromString(TEXT("TRAINS IN TROUBLE AND LONG WAITS")), 9, ATStyle::Muted, true, false, FMargin(0.f, 4.f, 0.f, 4.f));
	UScrollBox* Scroll = WidgetTree->ConstructWidget<UScrollBox>(UScrollBox::StaticClass());
	mRows = WidgetTree->ConstructWidget<UVerticalBox>(UVerticalBox::StaticClass());
	Scroll->AddChild(mRows);
	if (UVerticalBoxSlot* ScrollSlot = Root->AddChildToVerticalBox(Scroll))
	{
		ScrollSlot->SetSize(FSlateChildSize(ESlateSizeRule::Fill));
	}
	mNote = MakeText(Root, LOCTEXT("MonitorAsking", "Asking the server..."), 10, ATStyle::Muted, false, true, FMargin(0.f, 6.f, 0.f, 0.f));
}

void UATMonitorWindow::RequestView(UATRemoteCallObject* RCO)
{
	RCO->Server_RequestMonitor();
}

void UATMonitorWindow::AddTile(const FString& Value, const FString& Label, bool bAlert)
{
	UBorder* Box = nullptr;
	UVerticalBox* Inside = MakeCard(mTiles, bAlert ? ATStyle::Alert : ATStyle::Header, FMargin(12.f, 8.f), FMargin(0.f), &Box);
	if (bAlert)
	{
		Box->SetBrush(ATStyle::Outlined(ATStyle::Alert, ATStyle::Orange, 1.f, 4.f));
	}
	SetFill(Box);
	SetSlotPadding(Box, FMargin(mTiles->GetChildrenCount() > 1 ? 6.f : 0.f, 0.f, 0.f, 0.f));
	MakeText(Inside, FText::FromString(Value), 20, bAlert ? ATStyle::Orange : ATStyle::Text, true, false);
	MakeText(Inside, FText::FromString(Label), 9, ATStyle::Muted, false, true);
}

void UATMonitorWindow::Refresh(UATRemoteCallObject* RCO)
{
	const FATMonitorView& View = RCO->GetMonitorView();
	FString Shown = FString::Printf(TEXT("%d/%d/%d/%d/%d/%.0f/%d/%d;"), View.Trains, View.Driving, View.Waiting, View.Docked, View.Troubled, View.CycleMs, View.PassThroughs,
		View.bTraffic ? 1 : 0);
	for (const FATMonitorRow& Row : View.Rows)
	{
		Shown += Row.Name + Row.Kind + FString::Printf(TEXT("%.0f"), Row.Seconds) + Row.Reason + TEXT(";");
	}
	if (Shown == mShown)
	{
		return;
	}
	mShown = Shown;
	mTiles->ClearChildren();
	AddTile(FString::FromInt(View.Trains), TEXT("self-driving trains"), false);
	AddTile(FString::FromInt(View.Driving), TEXT("driving"), false);
	AddTile(FString::FromInt(View.Waiting), TEXT("waiting"), false);
	AddTile(FString::FromInt(View.Docked), TEXT("docked"), false);
	AddTile(FString::FromInt(View.Troubled), TEXT("in trouble"), View.Troubled > 0);
	AddTile(FString::Printf(TEXT("%.0f ms"), View.CycleMs), FString::Printf(TEXT("planner round (budget %.0f ms)"), View.BudgetMs), View.CycleMs > View.BudgetMs * 2.f);
	AddTile(FString::FromInt(View.PassThroughs), TEXT("jams passed through (please report)"), View.PassThroughs > 0);

	mRows->ClearChildren();
	for (const FATMonitorRow& Row : View.Rows)
	{
		const bool bTrouble = Row.Kind != TEXT("Waiting");
		UBorder* Card = nullptr;
		UVerticalBox* Inside = MakeCard(mRows, bTrouble ? ATStyle::Alert : ATStyle::Header, FMargin(10.f, 6.f), FMargin(0.f, 2.f), &Card);
		if (bTrouble)
		{
			Card->SetBrush(ATStyle::Outlined(ATStyle::Alert, ATStyle::Orange, 1.f, 4.f));
		}
		UHorizontalBox* Line = MakeHBox(Inside);
		FBadge Badge = MakeBadge(Line);
		SetBadge(Badge, FText::FromString(Row.Kind), bTrouble ? ATStyle::Orange : ATStyle::Warn);
		UTextBlock* Name = MakeText(Line, FText::FromString(Row.Name), 12, ATStyle::Text, true, true, FMargin(8.f, 0.f, 0.f, 0.f));
		SetFill(Name);
		SetVAlignCenter(Name);
		UTextBlock* Time = MakeText(Line, FText::FromString(ATStyle::Duration(Row.Seconds)), 11, ATStyle::Muted, false, false, FMargin(8.f, 0.f, 8.f, 0.f));
		SetVAlignCenter(Time);
		TWeakObjectPtr<AFGTrain> Train = Row.Train;
		UATButton* ShowMe = MakeButton(Line, LOCTEXT("ShowMe", "Show me"), [this, Train]()
		{
			UATRemoteCallObject* RCO = UATRemoteCallObject::GetLocal(this);
			if (RCO && Train.IsValid())
			{
				RCO->Server_MarkTrain(Train.Get());
			}
		});
		SetVAlignCenter(ShowMe);
		if (!Row.Reason.IsEmpty())
		{
			MakeText(Inside, FText::FromString(Row.Reason), 9, ATStyle::Muted, false, true, FMargin(0.f, 4.f, 0.f, 0.f));
		}
	}
	mNote->SetText(!View.bTraffic ? LOCTEXT("TrafficOff", "Traffic control is off: /at traffic on.")
		: View.Rows.Num() == 0 ? LOCTEXT("AllFine", "No train in trouble and no long waits.")
		: LOCTEXT("ShowMeHint", "Show me puts your compass marker on the train in the way."));
}

#undef LOCTEXT_NAMESPACE
