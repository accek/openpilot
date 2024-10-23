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

#include "selfdrive/ui/sunnypilot/qt/widgets/presets.h"

#include <QButtonGroup>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "common/params.h"

static QWidget * radio_button(QString title, QButtonGroup *group) {
  QPushButton *btn = new QPushButton(title);
  btn->setCheckable(true);
  group->addButton(btn);
  btn->setStyleSheet(R"(
    QPushButton {
      height: 230;
      padding-left: 100px;
      padding-right: 100px;
      text-align: left;
      font-size: 80px;
      font-weight: 400;
      border-radius: 10px;
      background-color: #4F4F4F;
    }
    QPushButton:checked {
      background-color: #465BEA;
    }
  )");

  // checkmark icon
  QPixmap pix(":/img_circled_check.svg");
  btn->setIcon(pix);
  btn->setIconSize(QSize(0, 0));
  btn->setLayoutDirection(Qt::RightToLeft);
  QObject::connect(btn, &QPushButton::toggled, [=](bool checked) {
    btn->setIconSize(checked ? QSize(104, 104) : QSize(0, 0));
  });
  return btn;
}

Presets::Presets(QWidget* parent) : QFrame(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(55, 50, 55, 50);
  main_layout->setSpacing(0);

  main_layout->addStretch();

  // title
  QLabel *title = new QLabel(tr("Choose Operating Mode"), this);
  title->setStyleSheet("font-size: 90px; font-weight: 500;");
  main_layout->addWidget(title, 0, Qt::AlignLeft | Qt::AlignTop);

  main_layout->addSpacing(50);

  // openpilot + custom radio buttons
  QButtonGroup *group = new QButtonGroup(this);
  group->setExclusive(true);

  QWidget *preset_off = radio_button(tr("Off (Dashcam)"), group);
  main_layout->addWidget(preset_off);

  main_layout->addSpacing(30);

  QWidget *preset_1 = radio_button(tr("OpenPilot Lane Assist"), group);
  main_layout->addWidget(preset_1);

  main_layout->addSpacing(30);

  QWidget *preset_2 = radio_button(tr("OpenPilot Lane Assist + ACC"), group);
  main_layout->addWidget(preset_2);

  connect(group, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked), [=](QAbstractButton *btn) {
    btn->setChecked(true);

    auto params = Params();
    if (btn == preset_off) {
      params.put("OpenpilotEnabledToggle", "0");
      params.put("OnroadScreenOff", "-1");
      params.put("OnroadScreenOffBrightness", "0");
      params.put("ExperimentalLongitudinalEnabled", "0");
    } else {
      params.put("OpenpilotEnabledToggle", "1");
      params.put("OnroadScreenOff", "-2");
      params.put("OnroadScreenOffBrightness", "50");
      if (btn == preset_1) {
        params.put("ExperimentalLongitudinalEnabled", "0");
      } else if (btn == preset_2) {
        params.put("ExperimentalLongitudinalEnabled", "1");
      }
    }

    params.put("PresetSelected", "1");
    emit presetSelected();
  });
}