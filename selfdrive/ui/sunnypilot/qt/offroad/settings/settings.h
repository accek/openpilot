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

#pragma once

#include <map>
#include <string>

#include "selfdrive/ui/qt/offroad/settings.h"

class OpLongMaxSpeed : public OptionControlSP {
  Q_OBJECT

public:
  OpLongMaxSpeed();

  void refresh();

signals:
  void ToggleUpdated();

private:
  Params params;
};

class TogglesPanelSP : public TogglesPanel {
  Q_OBJECT

public:
  explicit TogglesPanelSP(SettingsWindow *parent);
  void showEvent(QShowEvent *event) override;

private slots:
  void updateState(const UIStateSP &s);

private:
  ButtonParamControlSP *long_personality_setting;
  ButtonParamControlSP *accel_personality_setting;
  OpLongMaxSpeed *op_long_max_speed;

  ParamWatcher *param_watcher;
  void updateToggles() override;
};

class SettingsWindowSP : public SettingsWindow {
  Q_OBJECT

public:
  explicit SettingsWindowSP(QWidget* parent = nullptr);

protected:
  struct PanelInfo {
    QString name;
    QWidget *widget;
    QString icon;

    PanelInfo(const QString &name, QWidget *widget, const QString &icon) : name(name), widget(widget), icon(icon) {}
  };
};
