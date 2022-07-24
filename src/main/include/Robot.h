// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Swerve/Swerve.h"
#include <frc/Joystick.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:

  double maxVelocity = 1;
  double maxTurnVelocity = 1;

  frc::Joystick primary{0};

  SwerveModule::SwerveModuleDefinition frontLeft{};
  SwerveModule::SwerveModuleDefinition frontRight{};
  SwerveModule::SwerveModuleDefinition backLeft{};
  SwerveModule::SwerveModuleDefinition backRight{};

  int i = 0;


  Swerve swerve{frontLeft, frontRight, backLeft, backRight};

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
