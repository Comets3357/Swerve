#pragma once

#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/length.h>
#include "Swerve/SwerveModule.h"

class Swerve
{
public:

// CREATES OBJECTS OF ALL FOUR MODULES ON THE ROBOT
Swerve(SwerveModule::SwerveModuleDefinition frontLeft,
       SwerveModule::SwerveModuleDefinition frontRight,
       SwerveModule::SwerveModuleDefinition backLeft,
       SwerveModule::SwerveModuleDefinition backRight);

// RESPONSIBLE FOR COMMANDING EACH MODULE
void Drive(units::meters_per_second_t xVelocity,
           units::meters_per_second_t yVelocity,
           units::radians_per_second_t rotVelocity,
           bool fieldRelative);

private:

void FieldRelativeDrive(units::meters_per_second_t xVelocity,
                        units::meters_per_second_t yVelocity,
                        units::radians_per_second_t rotVelocity);

void RegularDrive(units::meters_per_second_t xVelocity,
                  units::meters_per_second_t yVelocity,
                  units::radians_per_second_t rotVelocity);

SwerveModule fl;
SwerveModule fr;
SwerveModule bl;
SwerveModule br;
int i = 0;

bool Zeroed;

units::meters_per_second_t moduleMaxSpeed{16_fps}; // arbitrary number that may change

frc::Translation2d frontLeftModule{+0.263525_m, +0.263525_m};
frc::Translation2d frontRightModule{+0.263525_m, -0.263525_m};
frc::Translation2d backLeftModule{-0.263525_m, +0.263525_m};
frc::Translation2d backRightModule{-0.263525_m, -0.263525_m};

frc::SwerveDriveKinematics<4> kinematics{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

AHRS gyro{frc::SPI::Port::kMXP};

};