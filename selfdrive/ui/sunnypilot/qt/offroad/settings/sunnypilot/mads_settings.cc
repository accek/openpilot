/**
The MIT License

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Last updated: July 29, 2024
***/

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/sunnypilot/mads_settings.h"

#include <tuple>
#include <vector>

MadsSettings::MadsSettings(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(50, 20, 50, 20);
  main_layout->setSpacing(20);

  // Back button
  PanelBackButton* back = new PanelBackButton();
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  ListWidgetSP *list = new ListWidgetSP(this, false);
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "BelowSpeedPause",
      tr("Pause Lateral Below Speed"),
      tr("Enable this toggle to pause lateral actuation when traveling below the desired speed selected below."),
      "../assets/offroad/icon_blank.png",
    },
    {
      "BelowSpeedPauseWithBlinker",
      tr("Pause Lateral Below Speed with Blinker"),
      tr("Enable this toggle to pause lateral actuation when traveling below the desired speed selected below and the blinker is active."),
      "../assets/offroad/icon_blank.png",
    },
    {
      "AboveSpeedResume",
      tr("Resume Lateral Above Speed"),
      tr("Enable this toggle to resume lateral actuation when traveling above the desired speed selected below if lanes are visible."),
      "../assets/offroad/icon_blank.png",
    },
    {
      "AccMadsCombo",
      tr("Enable ACC+MADS with RES+/SET-"),
      QString("%1<br>"
              "<h4>%2</h4>")
      .arg(tr("Engage both M.A.D.S. and ACC with a single press of RES+ or SET- button."))
      .arg(tr("Note: Once M.A.D.S. is engaged via this mode, it will remain engaged until it is manually disabled via the M.A.D.S. button or car shut off.")),
      "../assets/offroad/icon_blank.png",
    },
    {
      "MadsCruiseMain",
      tr("Toggle M.A.D.S. with Cruise Main"),
      tr("Allows M.A.D.S. engagement/disengagement with \"Cruise Main\" cruise control button from the steering wheel."),
      "../assets/offroad/icon_blank.png",
    }
  };

  // Disengage Lateral on Brake (DLOB)
  std::vector<QString> dlob_settings_texts{tr("Remain Active"), tr("Pause Steering")};
  dlob_settings = new ButtonParamControlSP(
    "DisengageLateralOnBrake",
    tr("Steering Mode After Braking"),
    tr("Choose how Automatic Lane Centering (ALC) behaves after the brake pedal is manually pressed in sunnypilot.\n\n"
       "Remain Active: ALC will remain active even after the brake pedal is pressed.\nPause Steering: ALC will be paused after the brake pedal is manually pressed."),
    "",
    dlob_settings_texts,
    500);
  dlob_settings->showDescription();
  list->addItem(dlob_settings);

  // Pause Lateral Below Speed
  pause_lateral_speed = new PauseLateralSpeed();
  pause_lateral_speed->showDescription();
  connect(pause_lateral_speed, &OptionControlSP::updateLabels, pause_lateral_speed, &PauseLateralSpeed::refresh);

  // Pause Lateral Below Speed w/ Blinker
  pause_lateral_speed_with_blinker = new PauseLateralSpeedWithBlinker();
  pause_lateral_speed_with_blinker->showDescription();
  connect(pause_lateral_speed_with_blinker, &OptionControlSP::updateLabels, pause_lateral_speed_with_blinker, &PauseLateralSpeedWithBlinker::refresh);

  // Resume Lateral Above Speed
  resume_lateral_speed = new ResumeLateralSpeed();
  resume_lateral_speed->showDescription();
  connect(resume_lateral_speed, &OptionControlSP::updateLabels, resume_lateral_speed, &ResumeLateralSpeed::refresh);

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControlSP(param, title, desc, icon, this);

    list->addItem(toggle);
    toggles[param.toStdString()] = toggle;

    if (param == "BelowSpeedPause") {
      list->addItem(pause_lateral_speed);
    } else if (param == "BelowSpeedPauseWithBlinker") {
      list->addItem(pause_lateral_speed_with_blinker);
    } else if (param == "AboveSpeedResume") {
      list->addItem(resume_lateral_speed);
    }

    // trigger offroadTransition when going onroad/offroad
    connect(uiStateSP(), &UIStateSP::offroadTransition, toggle, &ParamControlSP::setEnabled);
  }

  // trigger offroadTransition when going onroad/offroad
  connect(uiStateSP(), &UIStateSP::offroadTransition, dlob_settings, &ButtonParamControlSP::setEnabled);

  connect(toggles["BelowSpeedPause"], &ToggleControlSP::toggleFlipped, [=](bool state) {
    pause_lateral_speed->setEnabled(state);
    pause_lateral_speed->setVisible(state);
  });
  pause_lateral_speed->setEnabled(toggles["BelowSpeedPause"]->isToggled());
  pause_lateral_speed->setVisible(toggles["BelowSpeedPause"]->isToggled());

  connect(toggles["BelowSpeedPauseWithBlinker"], &ToggleControlSP::toggleFlipped, [=](bool state) {
    pause_lateral_speed_with_blinker->setEnabled(state);
    pause_lateral_speed_with_blinker->setVisible(state);
  });
  pause_lateral_speed_with_blinker->setEnabled(toggles["BelowSpeedPauseWithBlinker"]->isToggled());
  pause_lateral_speed_with_blinker->setVisible(toggles["BelowSpeedPauseWithBlinker"]->isToggled());

  connect(toggles["AboveSpeedResume"], &ToggleControlSP::toggleFlipped, [=](bool state) {
    resume_lateral_speed->setEnabled(state);
    resume_lateral_speed->setVisible(state);
  });
  resume_lateral_speed->setEnabled(toggles["AboveSpeedResume"]->isToggled());
  resume_lateral_speed->setVisible(toggles["AboveSpeedResume"]->isToggled());

  main_layout->addWidget(new ScrollViewSP(list, this));
}

void MadsSettings::showEvent(QShowEvent *event) {
  updateToggles();
}

void MadsSettings::updateToggles() {
  if (!isVisible()) {
    return;
  }

  const bool is_offroad = !uiStateSP()->scene.started;
  const bool enable_mads = params.getBool("EnableMads");
  const bool enabled = is_offroad && enable_mads;

  toggles["AccMadsCombo"]->setEnabled(enable_mads);
  toggles["MadsCruiseMain"]->setEnabled(enable_mads);
  dlob_settings->setEnabled(enabled);
}

PauseLateralSpeed::PauseLateralSpeed() : OptionControlSP(
  "PauseLateralSpeed",
  "",
  tr("Pause lateral actuation when traveling below the desired speed selected. Default is 10 MPH or 16 km/h."),
  "../assets/offroad/icon_blank.png",
  {0, 255},
  5) {

  refresh();
}

void PauseLateralSpeed::refresh() {
  QString option = QString:: fromStdString(params.get("PauseLateralSpeed"));
  bool is_metric = params.getBool("IsMetric");

  if (option == "0") {
    setLabel(tr("Default"));
  } else {
    setLabel(option + " " + (is_metric ? tr("km/h") : tr("mph")));
  }
}

PauseLateralSpeedWithBlinker::PauseLateralSpeedWithBlinker() : OptionControlSP(
  "PauseLateralSpeedWithBlinker",
  "",
  tr("Pause lateral actuation when traveling below the desired speed selected and the blinker is active. Default is 20 MPH or 32 km/h."),
  "../assets/offroad/icon_blank.png",
  {0, 255},
  5) {

  refresh();
}

void PauseLateralSpeedWithBlinker::refresh() {
  QString option = QString:: fromStdString(params.get("PauseLateralSpeedWithBlinker"));
  bool is_metric = params.getBool("IsMetric");

  if (option == "0") {
    setLabel(tr("Default"));
  } else {
    setLabel(option + " " + (is_metric ? tr("km/h") : tr("mph")));
  }
}

ResumeLateralSpeed::ResumeLateralSpeed() : OptionControlSP(
  "ResumeLateralSpeed",
  "",
  tr("Resume lateral actuation when traveling above the desired speed selected. Default is 40 MPH or 64 km/h."),
  "../assets/offroad/icon_blank.png",
  {0, 255},
  5) {

  refresh();
}

void ResumeLateralSpeed::refresh() {
  QString option = QString:: fromStdString(params.get("ResumeLateralSpeed"));
  bool is_metric = params.getBool("IsMetric");

  if (option == "0") {
    setLabel(tr("Default"));
  } else {
    setLabel(option + " " + (is_metric ? tr("km/h") : tr("mph")));
  }
}