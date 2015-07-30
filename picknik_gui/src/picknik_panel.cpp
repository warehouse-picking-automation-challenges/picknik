/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/Bool.h>

#include "picknik_panel.h"

namespace picknik_gui
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
PickNikPanel::PickNikPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Create a push button
  btn_next_ = new QPushButton(this);
  btn_next_->setText("Next Step");
  connect( btn_next_, SIGNAL( clicked() ), this, SLOT( moveNext() ) );

  // Create a push button
  btn_auto_ = new QPushButton(this);
  btn_auto_->setText("Auto Step");
  connect( btn_auto_, SIGNAL( clicked() ), this, SLOT( moveAuto() ) );

  // Create a push button
  btn_full_auto_ = new QPushButton(this);
  btn_full_auto_->setText("Full Auto");
  connect( btn_full_auto_, SIGNAL( clicked() ), this, SLOT( moveFullAuto() ) );

  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");
  connect( btn_stop_, SIGNAL( clicked() ), this, SLOT( moveStop() ) );

  // Create a push button
  btn_mode_ = new QPushButton(this);
  btn_mode_->setText("Toggle Joint Mode");
  connect( btn_mode_, SIGNAL( clicked() ), this, SLOT( changeJointMode() ) );

  // Create a push button
  btn_reset_ = new QPushButton(this);
  btn_reset_->setText("Reset Robot");
  connect( btn_reset_, SIGNAL( clicked() ), this, SLOT( resetRobot() ) );

  // Create a push button
  btn_bringup_ = new QPushButton(this);
  btn_bringup_->setText("Bringup Robot");
  connect( btn_bringup_, SIGNAL( clicked() ), this, SLOT( bringupRobot() ) );    
  
  // Buttons horizontal
  QHBoxLayout* hlayout = new QHBoxLayout;
  hlayout->addWidget( btn_next_ );
  hlayout->addWidget( btn_auto_ );
  hlayout->addWidget( btn_full_auto_ );
  hlayout->addWidget( btn_stop_ );

  QHBoxLayout* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget( btn_bringup_ );  
  hlayout2->addWidget( btn_reset_ );
  hlayout2->addWidget( btn_mode_ );  

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  //layout->addLayout( topic_layout );
  layout->addLayout( hlayout );
  layout->addLayout( hlayout2 );  
  setLayout( layout );

  next_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/next_command", 1 );
  auto_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/auto_command", 1 );
  full_auto_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/full_auto_command", 1 );
  stop_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/stop_command", 1 );
  mode_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/mode_command", 1 );
  reset_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/reset_command", 1 );
  bringup_publisher_ = nh_.advertise<std_msgs::Bool>( "/picknik_main/bringup_command", 1 );    

  // Make the control widget start disabled, since we don't start with an output topic.
  btn_next_->setEnabled( true );
  btn_auto_->setEnabled( true );
  btn_full_auto_->setEnabled( true );
}

void PickNikPanel::moveNext()
{
  ROS_INFO_STREAM_NAMED("picknik","Move to next step");
  std_msgs::Bool result;
  result.data = true;
  next_publisher_.publish( result );
}

void PickNikPanel::moveAuto()
{
  ROS_INFO_STREAM_NAMED("picknik","Running auto step");
  std_msgs::Bool result;
  result.data = true;
  auto_publisher_.publish( result );
}

void PickNikPanel::moveFullAuto()
{
  ROS_INFO_STREAM_NAMED("picknik","Running auto trajectory");
  std_msgs::Bool result;
  result.data = true;
  full_auto_publisher_.publish( result );
}

void PickNikPanel::moveStop()
{
  ROS_INFO_STREAM_NAMED("picknik","Stopping");
  std_msgs::Bool result;
  result.data = true;
  stop_publisher_.publish( result );
}

void PickNikPanel::changeJointMode()
{
  ROS_INFO_STREAM_NAMED("picknik","Changing joint mode");
  std_msgs::Bool result;
  result.data = true;
  mode_publisher_.publish( result );
}

void PickNikPanel::resetRobot()
{
  ROS_INFO_STREAM_NAMED("picknik","Resetting robot");
  std_msgs::Bool result;
  result.data = true;
  reset_publisher_.publish( result );
}

void PickNikPanel::bringupRobot()
{
  ROS_INFO_STREAM_NAMED("picknik","Bringing robot");
  std_msgs::Bool result;
  result.data = true;
  bringup_publisher_.publish( result );
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PickNikPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void PickNikPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} // end namespace picknik_gui

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(picknik_gui::PickNikPanel,rviz::Panel )
// END_TUTORIAL
