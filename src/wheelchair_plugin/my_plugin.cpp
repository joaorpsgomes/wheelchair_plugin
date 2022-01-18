/*
  Copyright 2016 Lucas Walter
*/

#include "wheelchair_plugin/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

Ui::MainWindow ui_;

namespace wheelchair_plugin
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
  
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  ui_.label_Map_image->setFixedSize(ui_.label_Map_image->sizeHint());

  /////// Publishers ////////
  ros::NodeHandle nh = getNodeHandle();
  goal_pub = getNodeHandle().advertise<geometry_msgs::PoseStamped>("goal", 5);
  //////////////////////////


  /////// Subscribers //////
  sub = nh.subscribe("map", 1000, &MyPlugin::map_callback, this);
  //////////////////////////


  /////// Event handlers ///////
  connect(ui_.Button_Start, SIGNAL(clicked()), this, SLOT(on_Button_Start_clicked()));
  //////////////////////////////

  //ui_.Button_Start->setText("Hello");

}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void MyPlugin::on_Button_Start_clicked()
{
    static int count=0;
    geometry_msgs::PoseStamped goal;
    
    //// header ////
    goal.header.seq=count;
    goal.header.stamp= ros::Time::now();
    goal.header.frame_id="map";
    ////////////////


    ////// position ////sg)
    goal.pose.position.x=0;
    goal.pose.position.y=0;
    goal.pose.position.z=0;
    ////////////////////


    ////// orientation /////
    goal.pose.orientation.x=0;
    goal.pose.orientation.y=0;
    goal.pose.orientation.z=0;
    goal.pose.orientation.w=1;
    ////////////////////////

    
    goal_pub.publish(goal);

    ros::spinOnce();
    count++;
}


void MyPlugin::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  int width=msg->info.width;
  int lenght=msg->info.width*msg->info.height;
  
  QImage image( msg->info.width, msg->info.height,QImage::Format_RGB32);
  for ( int i = 0; i < lenght; ++i )
  {
      
      QRgb grey  = 0xFFA0A0A0;
      QRgb black = 0xFF000000; 
      QRgb white = 0xFFFFFFFF; 
      if(msg->data[i]==-1){
          image.setPixel( i%width, i/width, grey );
      }           
      else if(msg->data[i]==100){
          image.setPixel( i%width, i/width, black );
      }
      else{
          image.setPixel( i%width, i/width, white );
      }


  }
  QImage img2 = image.scaled(681, 441, Qt::KeepAspectRatio);    
  ui_.label_Map_image->setPixmap(QPixmap::fromImage(img2));
}





}  // namespace wheelchair_plugin
PLUGINLIB_EXPORT_CLASS(wheelchair_plugin::MyPlugin, rqt_gui_cpp::Plugin)


