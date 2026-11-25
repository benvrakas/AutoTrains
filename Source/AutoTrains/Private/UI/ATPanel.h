#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Fonts/SlateFontInfo.h"
#include "Styling/SlateBrush.h"
#include "ATPanel.generated.h"

class UATRemoteCallObject;
class UBorder;
class UButton;
class UHorizontalBox;
class UPanelWidget;
class UTextBlock;
class UVerticalBox;

// Auto Trains' look: dark panels with FICSIT orange, in the game's font (Docs/Architecture.md, "UI").
namespace ATStyle
{
	extern const FLinearColor Orange;
	extern const FLinearColor Panel;
	extern const FLinearColor Header;
	extern const FLinearColor Card;
	extern const FLinearColor Inset;
	extern const FLinearColor Line;
	extern const FLinearColor Text;
	extern const FLinearColor Muted;
	extern const FLinearColor Good;
	extern const FLinearColor Warn;
	extern const FLinearColor Dark;
	extern const FLinearColor Alert;

	FSlateBrush Rounded(const FLinearColor& Color, float Radius = 4.f);
	FSlateBrush Outlined(const FLinearColor& Fill, const FLinearColor& Outline, float Width = 1.f, float Radius = 4.f);
	// "45 s", "3 min"; "-" below 0.
	FString Duration(double Seconds);
	// A train state's badge text and colour (FATTrainView::State).
	FText StateText(uint8 State);
	FLinearColor StateColor(uint8 State);
}

// The Auto Trains mark: a hexagon round two rails meeting under a timing tick.
UCLASS()
class UATLogo : public UUserWidget
{
	GENERATED_BODY()

protected:
	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements,
		int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
};

// A flat rounded button with a text; orange when bAccent.
UCLASS()
class UATButton : public UUserWidget
{
	GENERATED_BODY()

public:
	void SetText(const FText& Text);
	void SetAccent(bool bInAccent);
	void SetFont(const FSlateFontInfo& Font);

	TFunction<void()> OnClick;

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;

private:
	void BuildTree();
	void ApplyLook();

	UFUNCTION()
	void HandleClicked();

	UPROPERTY()
	TObjectPtr<UButton> mButton;

	UPROPERTY()
	TObjectPtr<UTextBlock> mText;

	bool bAccent = false;
};

// Base of Auto Trains' panels: builds its own widget tree, asks the server for its view every second while shown, and
// refreshes when one arrives; fades in when shown.
UCLASS(Abstract)
class UATPanel : public UUserWidget
{
	GENERATED_BODY()

public:
	// For a panel outside a vanilla menu: the widget whose text it copies the game's font from.
	void SetFontSource(UUserWidget* Source) { mFontSource = Source; }

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;

	virtual void BuildContent(UVerticalBox* Root) {}
	virtual UWidget* WrapRoot(UVerticalBox* Root);
	virtual bool OnShown() { return true; }
	virtual void OnHidden() {}
	virtual void RequestView(UATRemoteCallObject* RCO) {}
	virtual void Refresh(UATRemoteCallObject* RCO) {}

	// The vanilla menu the panel sits in (its owning user widget).
	UUserWidget* GetMenu() const;

	UTextBlock* MakeText(UPanelWidget* Parent, const FText& Text, int32 Size, const FLinearColor& Color, bool bBold = false, bool bWrap = true,
		const FMargin& Spacing = FMargin(0.f));
	void Tint(UTextBlock* Text, const FLinearColor& Color);
	UVerticalBox* MakeCard(UPanelWidget* Parent, const FLinearColor& Color, const FMargin& Inner = FMargin(8.f), const FMargin& Spacing = FMargin(0.f, 3.f),
		UBorder** OutBorder = nullptr);
	UVerticalBox* MakeVBox(UPanelWidget* Parent, const FMargin& Spacing = FMargin(0.f));
	UHorizontalBox* MakeHBox(UPanelWidget* Parent, const FMargin& Spacing = FMargin(0.f));
	UATButton* MakeButton(UPanelWidget* Parent, const FText& Text, TFunction<void()> OnClick, const FMargin& Spacing = FMargin(0.f));
	// The logo, "AUTO TRAINS" and Title; returns the bar so a caller can add to its right.
	UHorizontalBox* MakeTitleBar(UPanelWidget* Parent, const FText& Title, float LogoSize = 22.f);

	struct FBadge
	{
		UBorder* Box = nullptr;
		UTextBlock* Text = nullptr;
	};
	FBadge MakeBadge(UPanelWidget* Parent);
	static void SetBadge(const FBadge& Badge, const FText& Text, const FLinearColor& Color);

	FSlateFontInfo GetFont(int32 Size, bool bBold) const;

	static void SetSlotPadding(UWidget* Child, const FMargin& Padding);
	static void SetFill(UWidget* Child);
	static void SetVAlignCenter(UWidget* Child);

	UPROPERTY()
	TObjectPtr<UVerticalBox> mRoot;

private:
	void Tick();
	void ApplyFont();

	struct FStyled
	{
		TWeakObjectPtr<UTextBlock> Text;
		int32 Size = 11;
		bool bBold = false;
	};
	TArray<FStyled> mStyled;
	FSlateFontInfo mFont;
	bool bHasFont = false;
	TWeakObjectPtr<UUserWidget> mFontSource;
	FTimerHandle mRequestTimer;
	FTimerHandle mRefreshTimer;
	uint32 mShownRevision = MAX_uint32;
	float mShownFor = 0.f;
	bool bActive = false;
};
