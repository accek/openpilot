/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/lateral/mads_settings.h"

#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"

MadsSettings::MadsSettings(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(50, 20, 50, 20);
  main_layout->setSpacing(20);

  // Back button
  PanelBackButton *back = new PanelBackButton();
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  ListWidget *list = new ListWidget(this, false);
  // Main cruise
  madsMainCruiseToggle = new ParamControl("MadsMainCruiseAllowed", tr("Toggle with Main Cruise"), "", "");
  list->addItem(madsMainCruiseToggle);

  // Unified Engagement Mode
  madsUnifiedEngagementModeToggle = new ParamControl("MadsUnifiedEngagementMode", tr("Unified Engagement Mode (UEM)"), "", "");
  list->addItem(madsUnifiedEngagementModeToggle);

  // Steering Mode On Brake
  madsSteeringMode = new ButtonParamControl("MadsSteeringMode", tr("Steering Mode on Brake Pedal"), "", "", madsSteeringModeTexts(), 500);
  QObject::connect(madsSteeringMode, &ButtonParamControl::buttonClicked, [=] {
    updateToggles(offroad);
  });
  list->addItem(madsSteeringMode);

  // Pause Lateral On Brake
  std::vector<QString> lateral_on_brake_texts{tr("Remain Active"), tr("Pause Steering")};
  madsPauseLateralOnBrake = new ButtonParamControl(
    "MadsPauseLateralOnBrake",
    tr("Steering Mode After Braking"),
    tr("Choose how Automatic Lane Centering (ALC) behaves after the brake pedal is manually pressed in sunnypilot.\n\n"
       "Remain Active: ALC will remain active even after the brake pedal is pressed.\nPause Steering: ALC will be paused after the brake pedal is manually pressed."),
    "",
    lateral_on_brake_texts,
    500);
  list->addItem(madsPauseLateralOnBrake);

  // Speed-based pause and resume
  madsPauseSpeedWithBlinkerToggle = new ParamControl(
    "MadsPauseSpeedWithBlinkerEnabled",
    tr("Pause Steering at Low Speed with Blinker"),
    tr("Pause lateral steering when blinker is active at low speed."),
    "");
  list->addItem(madsPauseSpeedWithBlinkerToggle);
  madsPauseSpeedWithBlinker = new PauseLateralSpeedWithBlinker();
  madsPauseSpeedWithBlinker->showDescription();
  connect(madsPauseSpeedWithBlinker, &OptionControlSP::updateLabels, madsPauseSpeedWithBlinker, &PauseLateralSpeedWithBlinker::refresh);
  list->addItem(madsPauseSpeedWithBlinker);

  madsPauseSpeedToggle = new ParamControl(
    "MadsPauseSpeedEnabled",
    tr("Pause Steering at Low Speed"),
    tr("Pause lateral steering when speed is below a certain threshold."),
    "");
  list->addItem(madsPauseSpeedToggle);
  madsPauseSpeed = new PauseLateralSpeed();
  madsPauseSpeed->showDescription();
  connect(madsPauseSpeed, &OptionControlSP::updateLabels, madsPauseSpeed, &PauseLateralSpeed::refresh);
  list->addItem(madsPauseSpeed);

  madsResumeSpeedToggle = new ParamControl(
    "MadsResumeSpeedEnabled",
    tr("Resume Steering at High Speed"),
    tr("Resume lateral steering when speed is above a certain threshold."),
    "");
  list->addItem(madsResumeSpeedToggle);
  madsResumeSpeed = new ResumeLateralSpeed();
  madsResumeSpeed->showDescription();
  connect(madsResumeSpeed, &OptionControlSP::updateLabels, madsResumeSpeed, &ResumeLateralSpeed::refresh);
  list->addItem(madsResumeSpeed);

  QObject::connect(uiState(), &UIState::offroadTransition, this, &MadsSettings::updateToggles);

  main_layout->addWidget(new ScrollViewSP(list, this));
}

void MadsSettings::showEvent(QShowEvent *event) {
  updateToggles(offroad);
}

void MadsSettings::updateToggles(bool _offroad) {
  auto mads_steering_mode_param = std::atoi(params.get("MadsSteeringMode").c_str());

  auto steering_mode = static_cast<MadsSteeringMode>(
    std::clamp(mads_steering_mode_param, static_cast<int>(MadsSteeringMode::REMAIN_ACTIVE), static_cast<int>(MadsSteeringMode::DISENGAGE))
  );

  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (isBrandInList(CP.getBrand(), mads_limited_settings_brands)) {
      params.remove("MadsMainCruiseAllowed");
      params.putBool("MadsUnifiedEngagementMode", true);
      params.put("MadsSteeringMode", std::to_string(static_cast<int>(MadsSteeringMode::DISENGAGE)));

      madsMainCruiseToggle->setEnabled(false);
      madsMainCruiseToggle->setDescription(madsDescriptionBuilder(DEFAULT_TO_OFF, MADS_MAIN_CRUISE_BASE_DESC));
      madsMainCruiseToggle->showDescription();

      madsUnifiedEngagementModeToggle->setEnabled(false);
      madsUnifiedEngagementModeToggle->setDescription(madsDescriptionBuilder(DEFAULT_TO_ON, MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC));
      madsUnifiedEngagementModeToggle->showDescription();

      madsSteeringModeValues = convertMadsSteeringModeValues({MadsSteeringMode::DISENGAGE});
      madsSteeringMode->setDescription(madsDescriptionBuilder(STATUS_DISENGAGE_ONLY, madsSteeringModeDescription(steering_mode)));
    } else {
      madsMainCruiseToggle->setEnabled(true);
      madsMainCruiseToggle->setDescription(MADS_MAIN_CRUISE_BASE_DESC);

      madsUnifiedEngagementModeToggle->setEnabled(true);
      madsUnifiedEngagementModeToggle->setDescription(MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC);

      madsSteeringModeValues = convertMadsSteeringModeValues(getMadsSteeringModeValues());
      madsSteeringMode->setDescription(madsSteeringModeDescription(steering_mode));
    }
  } else {
    madsMainCruiseToggle->setEnabled(false);
    madsMainCruiseToggle->setDescription(madsDescriptionBuilder(STATUS_CHECK_COMPATIBILITY, MADS_MAIN_CRUISE_BASE_DESC));
    madsMainCruiseToggle->showDescription();

    madsUnifiedEngagementModeToggle->setEnabled(false);
    madsUnifiedEngagementModeToggle->setDescription(madsDescriptionBuilder(STATUS_CHECK_COMPATIBILITY, MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC));
    madsUnifiedEngagementModeToggle->showDescription();

    madsSteeringModeValues = {};
    madsSteeringMode->setDescription(madsDescriptionBuilder(STATUS_CHECK_COMPATIBILITY, madsSteeringModeDescription(steering_mode)));
  }

  madsSteeringMode->setEnableSelectedButtons(_offroad, madsSteeringModeValues);
  madsSteeringMode->showDescription();

  madsPauseLateralOnBrake->setEnabled(_offroad);
  madsPauseSpeedWithBlinkerToggle->setEnabled(_offroad);
  madsPauseSpeedToggle->setEnabled(_offroad);
  madsResumeSpeedToggle->setEnabled(_offroad);

  offroad = _offroad;
}

PauseLateralSpeed::PauseLateralSpeed() : OptionControlSP(
  "MadsPauseSpeed",
  "",
  tr("Pause lateral actuation when traveling below the desired speed selected and driver steers. Default is 10 MPH or 16 km/h."),
  "../assets/offroad/icon_blank.png",
  {0, 255},
  5) {

  refresh();
}

void PauseLateralSpeed::refresh() {
  QString option = QString:: fromStdString(params.get("MadsPauseSpeed"));
  bool is_metric = params.getBool("IsMetric");

  if (option == "0") {
    setLabel(tr("Default"));
  } else {
    setLabel(option + " " + (is_metric ? tr("km/h") : tr("mph")));
  }
}

PauseLateralSpeedWithBlinker::PauseLateralSpeedWithBlinker() : OptionControlSP(
  "MadsPauseSpeedWithBlinker",
  "",
  tr("Pause lateral actuation when traveling below the desired speed selected and the blinker is active. Default is 20 MPH or 32 km/h."),
  "../assets/offroad/icon_blank.png",
  {0, 255},
  5) {

  refresh();
}

void PauseLateralSpeedWithBlinker::refresh() {
  QString option = QString:: fromStdString(params.get("MadsPauseSpeedWithBlinker"));
  bool is_metric = params.getBool("IsMetric");

  if (option == "0") {
    setLabel(tr("Default"));
  } else {
    setLabel(option + " " + (is_metric ? tr("km/h") : tr("mph")));
  }
}

ResumeLateralSpeed::ResumeLateralSpeed() : OptionControlSP(
  "MadsResumeSpeed",
  "",
  tr("Resume lateral actuation when traveling above the desired speed selected. Default is 40 MPH or 64 km/h."),
  "../assets/offroad/icon_blank.png",
  {0, 255},
  5) {

  refresh();
}

void ResumeLateralSpeed::refresh() {
  QString option = QString:: fromStdString(params.get("MadsResumeSpeed"));
  bool is_metric = params.getBool("IsMetric");

  if (option == "0") {
    setLabel(tr("Default"));
  } else {
    setLabel(option + " " + (is_metric ? tr("km/h") : tr("mph")));
  }
}
