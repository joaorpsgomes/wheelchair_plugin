/*
  Copyright 2016 Lucas Walter
*/
#ifndef WHEELCHAIR_PLUGIN_MY_PLUGIN_H
#define WHEELCHAIR_PLUGIN_MY_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <wheelchair_plugin/ui_mainwindow.h>
#include <QWidget>

namespace wheelchair_plugin
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::MainWindow ui_;
  QWidget* widget_;
};
}  // namespace wheelchair_plugin
#endif  // WHEELCHAIR_PLUGIN_MY_PLUGIN_H
