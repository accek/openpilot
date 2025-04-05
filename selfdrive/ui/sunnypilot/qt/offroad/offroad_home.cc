/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/offroad_home.h"

#include <QStackedWidget>

#include "common/params.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/drive_stats.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/presets.h"

void OffroadHomeSP::addPresetsWidget(){
  auto* presets_widget = new Presets(this);
  center_layout->addWidget(presets_widget);
  connect(presets_widget, &Presets::presetSelected, [=] {
    refresh();
  });

  presets_btn = new QPushButton(tr("Change Preset"));
  connect(presets_btn, &QPushButton::clicked, [=]() {
    params.putBool("PresetSelected", false);
    refresh();
  });
  presets_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 48px;
      font-weight: 500;
      border-radius: 10px;
      border: 2px solid white;
      background-color: black;
      color: white;
      padding: 32px;
    }
    QPushButton:pressed {
      background-color: #aaa;
    }
  )");
  right_column->addWidget(presets_btn);
}

OffroadHomeSP::OffroadHomeSP(QWidget *parent) : OffroadHome(parent) {
  QStackedWidget *left_widget = new QStackedWidget(this);
  left_widget->addWidget(new DriveStats(this));
  left_widget->setStyleSheet("border-radius: 10px;");

  home_layout->insertWidget(0, left_widget);

  addPresetsWidget();
}

int OffroadHomeSP::computeCenterLayoutIndex() {
  if (params.getBool("RequirePresetAtBoot") && !params.getBool("PresetSelected")) {
    return 3;
  }
  return OffroadHome::computeCenterLayoutIndex();
}

void OffroadHomeSP::refresh() {
  OffroadHome::refresh();
  presets_btn->setVisible(params.getBool("RequirePresetAtBoot"));
}
