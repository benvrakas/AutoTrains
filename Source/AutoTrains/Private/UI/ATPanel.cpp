#include "UI/ATPanel.h"
#include "ATRemoteCallObject.h"
#include "AutoTrains.h"
#include "Blueprint/WidgetTree.h"
#include "Brushes/SlateRoundedBoxBrush.h"
#include "Components/Border.h"
#include "Components/Button.h"
#include "Components/ButtonSlot.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/PanelWidget.h"
#include "Components/SizeBox.h"
#include "Components/TextBlock.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"
#include "TimerManager.h"

#define LOCTEXT_NAMESPACE "AutoTrains"

namespace
{
	FLinearColor Srgb(uint8 R, uint8 G, uint8 B, uint8 A = 255)
	{
		return FLinearColor::FromSRGBColor(FColor(R, G, B, A));
	}

	constexpr float RequestInterval = 1.f;
	constexpr float RefreshInterval = 0.2f;
	constexpr float FadeInSeconds = 0.22f;
	constexpr float FadeInRise = 8.f;
}

namespace ATStyle
{
	const FLinearColor Orange = Srgb(250, 149, 73);
	const FLinearColor Panel = Srgb(30, 31, 34, 242);
	const FLinearColor Header = Srgb(38, 39, 43);
	const FLinearColor Card = Srgb(45, 47, 52);
	const FLinearColor Inset = Srgb(28, 29, 32);
	const FLinearColor Line = Srgb(58, 60, 65);
	const FLinearColor Text = Srgb(236, 234, 228);
	const FLinearColor Muted = Srgb(169, 171, 176);
	const FLinearColor Good = Srgb(79, 195, 161);
	const FLinearColor Warn = Srgb(242, 193, 78);
	const FLinearColor Dark = Srgb(26, 26, 26);
	const FLinearColor Alert = Srgb(46, 35, 24);

	FSlateBrush Rounded(const FLinearColor& Color, float Radius)
	{
		return FSlateRoundedBoxBrush(FSlateColor(Color), Radius);
	}

	FSlateBrush Outlined(const FLinearColor& Fill, const FLinearColor& Outline, float Width, float Radius)
	{
		return FSlateRoundedBoxBrush(FSlateColor(Fill), Radius, FSlateColor(Outline), Width);
	}

	FString Duration(double Seconds)
	{
		if (Seconds < 0.0)
		{
			return TEXT("-");
		}
		if (Seconds < 90.0)
		{
			return FString::Printf(TEXT("%.0f s"), Seconds);
		}
		const int32 Minutes = FMath::RoundToInt(Seconds / 60.0);
		return Minutes < 60 ? FString::Printf(TEXT("%d min"), Minutes) : FString::Printf(TEXT("%d h %d min"), Minutes / 60, Minutes % 60);
	}

	FText StateText(uint8 State)
	{
		switch (State)
		{
		case 1: return LOCTEXT("StateDriving", "Driving");
		case 2: return LOCTEXT("StateWaiting", "Waiting");
		case 3: return LOCTEXT("StateDocked", "Docked");
		case 4: return LOCTEXT("StateStuck", "Stuck");
		case 5: return LOCTEXT("StatePassing", "Passing through");
		default: return LOCTEXT("StateOff", "Not driven");
		}
	}

	FLinearColor StateColor(uint8 State)
	{
		switch (State)
		{
		case 1: return Good;
		case 2: return Warn;
		case 3: return Muted;
		case 4: return Orange;
		case 5: return Orange;
		default: return Muted;
		}
	}
}

//~ UATLogo

int32 UATLogo::NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements,
	int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2f Size = AllottedGeometry.GetLocalSize();
	const float S = FMath::Min(Size.X, Size.Y);
	const FVector2f O((Size.X - S) * 0.5f, (Size.Y - S) * 0.5f);
	auto P = [&](float X, float Y) { return FVector2f(O.X + X / 28.f * S, O.Y + Y / 28.f * S); };
	const float Thick = FMath::Max(1.f, S / 14.f);
	const FPaintGeometry Paint = AllottedGeometry.ToPaintGeometry();
	const FLinearColor Tint = InWidgetStyle.GetColorAndOpacityTint();
	TArray<FVector2f> Hex = {P(14, 2), P(25, 8), P(25, 20), P(14, 26), P(3, 20), P(3, 8), P(14, 2)};
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Paint, Hex, ESlateDrawEffect::None, ATStyle::Orange * Tint, true, Thick);
	// Two rails meeting at a switch, and a timing tick over them.
	TArray<FVector2f> Left = {P(8, 21), P(14, 13), P(14, 7)};
	TArray<FVector2f> Right = {P(20, 21), P(14, 13)};
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, Paint, Left, ESlateDrawEffect::None, ATStyle::Text * Tint, true, Thick * 0.9f);
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, Paint, Right, ESlateDrawEffect::None, ATStyle::Text * Tint, true, Thick * 0.9f);
	TArray<FVector2f> Tick = {P(17, 8), P(20, 8)};
	FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, Paint, Tick, ESlateDrawEffect::None, ATStyle::Orange * Tint, true, Thick);
	return LayerId + 2;
}

//~ UATButton

TSharedRef<SWidget> UATButton::RebuildWidget()
{
	BuildTree();
	return Super::RebuildWidget();
}

void UATButton::BuildTree()
{
	if (!WidgetTree)
	{
		WidgetTree = NewObject<UWidgetTree>(this, TEXT("WidgetTree"), RF_Transient);
	}
	if (WidgetTree->RootWidget)
	{
		return;
	}
	mButton = WidgetTree->ConstructWidget<UButton>(UButton::StaticClass());
	mButton->OnClicked.AddUniqueDynamic(this, &UATButton::HandleClicked);
	WidgetTree->RootWidget = mButton;
	mText = WidgetTree->ConstructWidget<UTextBlock>(UTextBlock::StaticClass());
	mText->SetJustification(ETextJustify::Center);
	if (UButtonSlot* TextSlot = Cast<UButtonSlot>(mButton->AddChild(mText)))
	{
		TextSlot->SetHorizontalAlignment(HAlign_Center);
		TextSlot->SetVerticalAlignment(VAlign_Center);
	}
	ApplyLook();
}

void UATButton::ApplyLook()
{
	FButtonStyle Style;
	Style.SetNormal(ATStyle::Rounded(bAccent ? ATStyle::Orange : ATStyle::Card, 3.f));
	Style.SetHovered(bAccent ? ATStyle::Rounded(ATStyle::Orange, 3.f) : ATStyle::Outlined(ATStyle::Card, ATStyle::Orange, 1.f, 3.f));
	Style.SetPressed(ATStyle::Rounded(ATStyle::Line, 3.f));
	Style.SetDisabled(ATStyle::Rounded(ATStyle::Inset, 3.f));
	Style.SetNormalPadding(FMargin(10.f, 6.f));
	Style.SetPressedPadding(FMargin(10.f, 6.f));
	mButton->SetStyle(Style);
	mText->SetColorAndOpacity(FSlateColor(bAccent ? ATStyle::Dark : ATStyle::Text));
}

void UATButton::SetText(const FText& Text)
{
	BuildTree();
	mText->SetText(Text);
}

void UATButton::SetAccent(bool bInAccent)
{
	BuildTree();
	bAccent = bInAccent;
	ApplyLook();
}

void UATButton::SetFont(const FSlateFontInfo& Font)
{
	BuildTree();
	mText->SetFont(Font);
}

void UATButton::HandleClicked()
{
	if (OnClick)
	{
		OnClick();
	}
}

//~ UATPanel

TSharedRef<SWidget> UATPanel::RebuildWidget()
{
	if (!WidgetTree)
	{
		WidgetTree = NewObject<UWidgetTree>(this, TEXT("WidgetTree"), RF_Transient);
	}
	if (!WidgetTree->RootWidget)
	{
		mRoot = WidgetTree->ConstructWidget<UVerticalBox>(UVerticalBox::StaticClass());
		BuildContent(mRoot);
		WidgetTree->RootWidget = WrapRoot(mRoot);
	}
	return Super::RebuildWidget();
}

UWidget* UATPanel::WrapRoot(UVerticalBox* Root)
{
	return Root;
}

UUserWidget* UATPanel::GetMenu() const
{
	for (UObject* Outer = GetOuter(); Outer; Outer = Outer->GetOuter())
	{
		if (UUserWidget* Widget = Cast<UUserWidget>(Outer))
		{
			return Widget;
		}
	}
	return nullptr;
}

void UATPanel::NativeConstruct()
{
	Super::NativeConstruct();
	ApplyFont();
	bActive = OnShown();
	SetVisibility(bActive ? ESlateVisibility::SelfHitTestInvisible : ESlateVisibility::Collapsed);
	if (!bActive)
	{
		return;
	}
	mShownRevision = MAX_uint32;
	mShownFor = 0.f;
	SetRenderOpacity(0.f);
	if (UWorld* World = GetWorld())
	{
		FTimerManager& Timers = World->GetTimerManager();
		Timers.SetTimer(mRequestTimer, FTimerDelegate::CreateWeakLambda(this, [this]()
		{
			if (UATRemoteCallObject* RCO = UATRemoteCallObject::GetLocal(this))
			{
				RequestView(RCO);
			}
		}), RequestInterval, true, 0.f);
		Timers.SetTimer(mRefreshTimer, this, &UATPanel::Tick, RefreshInterval, true, 0.f);
	}
}

void UATPanel::NativeDestruct()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(mRequestTimer);
		World->GetTimerManager().ClearTimer(mRefreshTimer);
	}
	if (bActive)
	{
		bActive = false;
		OnHidden();
	}
	Super::NativeDestruct();
}

void UATPanel::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);
	if (mShownFor < FadeInSeconds)
	{
		mShownFor = FMath::Min(mShownFor + InDeltaTime, FadeInSeconds);
		const float Eased = 1.f - FMath::Square(1.f - mShownFor / FadeInSeconds);
		SetRenderOpacity(Eased);
		SetRenderTranslation(FVector2D(0.f, FadeInRise * (1.f - Eased)));
	}
}

void UATPanel::Tick()
{
	UATRemoteCallObject* RCO = UATRemoteCallObject::GetLocal(this);
	if (RCO && RCO->GetRevision() != mShownRevision)
	{
		mShownRevision = RCO->GetRevision();
		Refresh(RCO);
	}
}

void UATPanel::ApplyFont()
{
	if (!bHasFont)
	{
		UUserWidget* Source = mFontSource.IsValid() ? mFontSource.Get() : GetMenu();
		UTextBlock* Found = nullptr;
		if (Source && Source->WidgetTree)
		{
			Source->WidgetTree->ForEachWidget([&Found](UWidget* Child) { Found = Found ? Found : Cast<UTextBlock>(Child); });
		}
		if (Found)
		{
			mFont = Found->GetFont();
			bHasFont = true;
		}
	}
	for (const FStyled& Each : mStyled)
	{
		if (UTextBlock* Text = Each.Text.Get())
		{
			Text->SetFont(GetFont(Each.Size, Each.bBold));
		}
	}
}

FSlateFontInfo UATPanel::GetFont(int32 Size, bool bBold) const
{
	FSlateFontInfo Font = bHasFont ? mFont : FCoreStyle::GetDefaultFontStyle(bBold ? "Bold" : "Regular", Size);
	if (bHasFont && bBold)
	{
		Font.TypefaceFontName = TEXT("Bold");
	}
	Font.Size = Size;
	return Font;
}

void UATPanel::SetSlotPadding(UWidget* Child, const FMargin& Padding)
{
	UPanelSlot* AnySlot = Child ? Child->Slot : nullptr;
	if (UVerticalBoxSlot* Vertical = Cast<UVerticalBoxSlot>(AnySlot))
	{
		Vertical->SetPadding(Padding);
	}
	else if (UHorizontalBoxSlot* Horizontal = Cast<UHorizontalBoxSlot>(AnySlot))
	{
		Horizontal->SetPadding(Padding);
	}
}

void UATPanel::SetFill(UWidget* Child)
{
	UPanelSlot* AnySlot = Child ? Child->Slot : nullptr;
	if (UHorizontalBoxSlot* Horizontal = Cast<UHorizontalBoxSlot>(AnySlot))
	{
		Horizontal->SetSize(FSlateChildSize(ESlateSizeRule::Fill));
	}
	else if (UVerticalBoxSlot* Vertical = Cast<UVerticalBoxSlot>(AnySlot))
	{
		Vertical->SetSize(FSlateChildSize(ESlateSizeRule::Fill));
	}
}

void UATPanel::SetVAlignCenter(UWidget* Child)
{
	if (UHorizontalBoxSlot* Horizontal = Child ? Cast<UHorizontalBoxSlot>(Child->Slot) : nullptr)
	{
		Horizontal->SetVerticalAlignment(VAlign_Center);
	}
}

UTextBlock* UATPanel::MakeText(UPanelWidget* Parent, const FText& Text, int32 Size, const FLinearColor& Color, bool bBold, bool bWrap, const FMargin& Spacing)
{
	UTextBlock* Block = WidgetTree->ConstructWidget<UTextBlock>(UTextBlock::StaticClass());
	Block->SetText(Text);
	Block->SetAutoWrapText(bWrap);
	Block->SetFont(GetFont(Size, bBold));
	Block->SetColorAndOpacity(FSlateColor(Color));
	mStyled.Add({Block, Size, bBold});
	if (Parent)
	{
		Parent->AddChild(Block);
		SetSlotPadding(Block, Spacing);
	}
	return Block;
}

void UATPanel::Tint(UTextBlock* Text, const FLinearColor& Color)
{
	if (Text)
	{
		Text->SetColorAndOpacity(FSlateColor(Color));
	}
}

UVerticalBox* UATPanel::MakeCard(UPanelWidget* Parent, const FLinearColor& Color, const FMargin& Inner, const FMargin& Spacing, UBorder** OutBorder)
{
	UBorder* Border = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Border->SetBrush(ATStyle::Rounded(Color));
	Border->SetPadding(Inner);
	UVerticalBox* Box = WidgetTree->ConstructWidget<UVerticalBox>(UVerticalBox::StaticClass());
	Border->SetContent(Box);
	if (Parent)
	{
		Parent->AddChild(Border);
		SetSlotPadding(Border, Spacing);
	}
	if (OutBorder)
	{
		*OutBorder = Border;
	}
	return Box;
}

UVerticalBox* UATPanel::MakeVBox(UPanelWidget* Parent, const FMargin& Spacing)
{
	UVerticalBox* Box = WidgetTree->ConstructWidget<UVerticalBox>(UVerticalBox::StaticClass());
	if (Parent)
	{
		Parent->AddChild(Box);
		SetSlotPadding(Box, Spacing);
	}
	return Box;
}

UHorizontalBox* UATPanel::MakeHBox(UPanelWidget* Parent, const FMargin& Spacing)
{
	UHorizontalBox* Box = WidgetTree->ConstructWidget<UHorizontalBox>(UHorizontalBox::StaticClass());
	if (Parent)
	{
		Parent->AddChild(Box);
		SetSlotPadding(Box, Spacing);
	}
	return Box;
}

UATButton* UATPanel::MakeButton(UPanelWidget* Parent, const FText& Text, TFunction<void()> OnClick, const FMargin& Spacing)
{
	UATButton* Button = WidgetTree->ConstructWidget<UATButton>(UATButton::StaticClass());
	Button->SetText(Text);
	Button->SetFont(GetFont(11, true));
	Button->OnClick = MoveTemp(OnClick);
	if (Parent)
	{
		Parent->AddChild(Button);
		SetSlotPadding(Button, Spacing);
	}
	return Button;
}

UHorizontalBox* UATPanel::MakeTitleBar(UPanelWidget* Parent, const FText& Title, float LogoSize)
{
	UBorder* Border = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Border->SetBrush(ATStyle::Rounded(ATStyle::Header, 3.f));
	Border->SetPadding(FMargin(10.f, 7.f));
	UVerticalBox* Stack = WidgetTree->ConstructWidget<UVerticalBox>(UVerticalBox::StaticClass());
	Border->SetContent(Stack);
	UHorizontalBox* Bar = MakeHBox(Stack);
	USizeBox* LogoBox = WidgetTree->ConstructWidget<USizeBox>(USizeBox::StaticClass());
	LogoBox->SetWidthOverride(LogoSize);
	LogoBox->SetHeightOverride(LogoSize);
	LogoBox->AddChild(WidgetTree->ConstructWidget<UATLogo>(UATLogo::StaticClass()));
	Bar->AddChildToHorizontalBox(LogoBox);
	SetVAlignCenter(LogoBox);
	UVerticalBox* Words = MakeVBox(Bar, FMargin(8.f, 0.f, 0.f, 0.f));
	SetFill(Words);
	SetVAlignCenter(Words);
	MakeText(Words, FText::FromString(TEXT("AUTO TRAINS")), 10, ATStyle::Orange, true, false);
	if (!Title.IsEmpty())
	{
		MakeText(Words, Title, 14, ATStyle::Text, true, true);
	}
	UBorder* Rule = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Rule->SetBrush(ATStyle::Rounded(ATStyle::Orange, 1.f));
	Rule->SetPadding(FMargin(0.f, 1.f));
	Stack->AddChildToVerticalBox(Rule);
	SetSlotPadding(Rule, FMargin(0.f, 7.f, 0.f, 0.f));
	if (Parent)
	{
		Parent->AddChild(Border);
		SetSlotPadding(Border, FMargin(0.f, 0.f, 0.f, 6.f));
	}
	return Bar;
}

UATPanel::FBadge UATPanel::MakeBadge(UPanelWidget* Parent)
{
	FBadge Badge;
	Badge.Box = WidgetTree->ConstructWidget<UBorder>(UBorder::StaticClass());
	Badge.Box->SetBrush(ATStyle::Rounded(ATStyle::Good, 9.f));
	Badge.Box->SetPadding(FMargin(8.f, 2.f));
	Badge.Text = MakeText(nullptr, FText::GetEmpty(), 10, ATStyle::Dark, true, false);
	Badge.Box->SetContent(Badge.Text);
	if (Parent)
	{
		Parent->AddChild(Badge.Box);
		SetVAlignCenter(Badge.Box);
	}
	return Badge;
}

void UATPanel::SetBadge(const FBadge& Badge, const FText& Text, const FLinearColor& Color)
{
	if (Badge.Box && Badge.Text)
	{
		Badge.Text->SetText(Text);
		Badge.Box->SetBrush(ATStyle::Rounded(Color, 9.f));
	}
}

#undef LOCTEXT_NAMESPACE
