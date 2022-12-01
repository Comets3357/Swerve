// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  

  frontLeft.azimuthCANCoderID = 1;
  frontRight.azimuthCANCoderID = 2;
  backLeft.azimuthCANCoderID = 3;
  backRight.azimuthCANCoderID = 4;

  swerve.RobotInit();
  
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
    double yDrive;
  double xDrive;
  double rDrive;


  yDrive = primary.GetRawAxis(0);
  xDrive = primary.GetRawAxis(3);
  rDrive = primary.GetRawAxis(1);
  if (abs(sqrt(pow(yDrive,2) + pow(xDrive,2))) < 0.1)
  {
    yDrive = 0;
    xDrive = 0;
  }
  if (primary.GetRawAxis(1) > -0.08 && primary.GetRawAxis(1) < 0.1)
  {
    rDrive = 0;
  }
  double maxMetersPerSecond;
  
  
  swerve.Drive(units::meters_per_second_t{yDrive}
              ,units::meters_per_second_t{xDrive},
              units::radians_per_second_t{rDrive}, true);
  frc::SmartDashboard::PutNumber("1", primary.GetRawAxis(1));
  frc::SmartDashboard::PutNumber("2", primary.GetRawAxis(0));
}

void Robot::DisabledInit() {
  swerve.RobotDisabledInit();
}

void Robot::DisabledPeriodic() {
  swerve.RobotDisabled();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
