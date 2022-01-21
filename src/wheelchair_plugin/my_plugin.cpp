/*
  Copyright 2016 Lucas Walter
*/

#include <string>
#include "wheelchair_plugin/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QJsonObject>
#include <memory>

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

  /////// Variable Definition ////////
  itemSelected = false;
  ////////////////////////////////////

  /////// Publishers ////////
  ros::NodeHandle nh = getNodeHandle();
  goal_pub = getNodeHandle().advertise<geometry_msgs::PoseStamped>("goal", 5);
  ///////////////////////////


  /////// Subscribers //////
  sub = nh.subscribe("map", 1000, &MyPlugin::map_callback, this);
  //////////////////////////


  /////// Event handlers ///////
  connect(ui_.Button_Start, SIGNAL(clicked()), this, SLOT(on_Button_Start_clicked()));
  connect(ui_.List_points, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(
    on_List_points_itemClicked(QListWidgetItem*)));
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
    //geometry_msgs::PoseStamped goal;
    
    if(count <= 4){ //for debug
      QListWidgetItem* item = new QListWidgetItem;
      item->setText("Hello");
      QJsonObject test;

      test.insert("pos_x",QJsonValue(count));
      test.insert("pos_y",QJsonValue(count));
      test.insert("pos_z",QJsonValue(count));
      test.insert("ori_x",QJsonValue(count));
      test.insert("ori_y",QJsonValue(count));
      test.insert("ori_z",QJsonValue(count));
      test.insert("ori_w",QJsonValue(count));

      QVariant Qtest = QVariant(test);
      item->setData(Qt::UserRole, Qtest);
      ui_.List_points->insertItem(count, item);
    }

    if(itemSelected){
      //// header : Apply time stamp only when button is clicked 
      goal_to_send.header.seq=count;
      goal_to_send.header.stamp= ros::Time::now();
      goal_to_send.header.frame_id="map";
      ////////////////
      
      goal_pub.publish(goal_to_send);
    }

    ros::spinOnce();
    count++;
}

void MyPlugin::on_List_points_itemClicked(QListWidgetItem *item){
  itemSelected = true;
  QJsonObject content = item->data(Qt::UserRole).toJsonObject();
  int x = content.value(QString("pos_x")).toInt();
  int y = content.value(QString("pos_y")).toInt();

  ////// position ////
  goal_to_send.pose.position.x = content.value(QString("pos_x")).toInt();
  goal_to_send.pose.position.y = content.value(QString("pos_y")).toInt();
  goal_to_send.pose.position.z = content.value(QString("pos_z")).toInt();
  ////////////////////


  ////// orientation /////
  goal_to_send.pose.orientation.x = content.value(QString("ori_x")).toInt();
  goal_to_send.pose.orientation.y = content.value(QString("ori_y")).toInt();
  goal_to_send.pose.orientation.z = content.value(QString("ori_z")).toInt();
  goal_to_send.pose.orientation.w = content.value(QString("ori_w")).toInt();
  ////////////////////////

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


