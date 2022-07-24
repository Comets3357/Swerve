#include "Swerve/Swerve.h"
#include <frc/smartdashboard/SmartDashboard.h>

Swerve::Swerve(SwerveModule::SwerveModuleDefinition frontLeft,
               SwerveModule::SwerveModuleDefinition frontRight,
               SwerveModule::SwerveModuleDefinition backLeft,
               SwerveModule::SwerveModuleDefinition backRight)
: fl(frontLeft, 0, 2, 1),
  fr(frontRight, 90, 4, 3),
  bl(backLeft, 270, 6, 5),
  br(backRight, 180, 8, 7)

  
{
    
}

void Swerve::Drive(units::meters_per_second_t xVelocity, units::meters_per_second_t yVelocity, units::radians_per_second_t rotVelocity, bool fieldRelative)
{
    // DETERMINES IF WE ARE USING FIELD RELATIVE DRIVE
    if (fieldRelative)
    {
        FieldRelativeDrive(xVelocity, yVelocity, rotVelocity);
    }
    else
    {
        RegularDrive(xVelocity, yVelocity, rotVelocity);
    }

}

void Swerve::FieldRelativeDrive(units::meters_per_second_t xVelocity, units::meters_per_second_t yVelocity, units::radians_per_second_t rotVelocity)
{
    frc::SmartDashboard::PutNumber("ROTATION", gyro.GetYaw());
    // CREATES A CHASSIS SPEED OBJECT AND STORES GENERAL VALS FOR THE DRIVE BASE
    // 45 degrees needs to be updated to robot angle (with 0 degrees being facing the opposite alliance wall)
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVelocity, yVelocity, rotVelocity, frc::Rotation2d(units::degree_t{gyro.GetYaw()}));

    // CONVERTS CHASSIS STATE TO SWERVE MODULE STATES AND STORES THEM
    auto moduleStates = kinematics.ToSwerveModuleStates(speeds);

    // DESATURATES THE WHEEL SPEED TO NOT GO OVER THE MAX VELOCITY THE DRIVE BASE CAN GO
    kinematics.DesaturateWheelSpeeds(&moduleStates, moduleMaxSpeed);

    // STORES THE DESATURATED MODULE STATES TO VALUES FOR EACH INDIVIDUAL MODULE
    auto [frontR, frontL, backR, backL] = moduleStates;

    // OPTIMIZES THE MODULES TO MAKE THE MODULE TURN THE LEAST DISTANCE WHEN DRIVING
    // NEED TO INCLUDE THE OPTIMIZATION (NEED A GET DISTANCE FUNCTION FROM SWERVE MODULE CLASS)
    // auto frontROptimized = frc::SwerveModuleState::Optimize(frontR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backROptimized = frc::SwerveModuleState::Optimize(backR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto frontLOptimized = frc::SwerveModuleState::Optimize(frontL, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backLOptimized = frc::SwerveModuleState::Optimize(backL, units::radian_t(azimuthEncoder.GetDistance()));

    // TURNS EACH MODULE TO DESIRED ANGLE
    fl.SetModuleAbsolutePosition(frontL.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FrontLAngle", frontL.angle.Degrees().value());
    fr.SetModuleAbsolutePosition(frontR.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FrontRAngle", frontR.angle.Degrees().value());
    bl.SetModuleAbsolutePosition(backL.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("backLAngle", backL.angle.Degrees().value());
    br.SetModuleAbsolutePosition(backR.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("backRAngle", backR.angle.Degrees().value());

    // DRIVES EACH MODULE AT DESIRED WHEEL SPEED
    fl.DriveModule(frontL.speed.value());
    frc::SmartDashboard::PutNumber("frontLspeed", frontL.speed.value());
    fr.DriveModule(frontR.speed.value());
    frc::SmartDashboard::PutNumber("frontRspeed", frontR.speed.value());
    bl.DriveModule(backL.speed.value());
    frc::SmartDashboard::PutNumber("BackLspeed", backL.speed.value());
    br.DriveModule(backR.speed.value());
    frc::SmartDashboard::PutNumber("backRspeed", backR.speed.value());
}

void Swerve::RegularDrive(units::meters_per_second_t xVelocity, units::meters_per_second_t yVelocity, units::radians_per_second_t rotVelocity)
{
    frc::SmartDashboard::PutNumber("x", (int)xVelocity);
    frc::SmartDashboard::PutNumber("y", (int)yVelocity);
    frc::SmartDashboard::PutNumber("r", (int)rotVelocity);
    i++;
    // CREATES A CHASSIS SPEED OBJECT AND STORES GENERAL VALS FOR THE DRIVE BASE
    frc::ChassisSpeeds speeds{xVelocity, yVelocity, rotVelocity};

    // CONVERTS CHASSIS STATE TO SWERVE MODULE STATES AND STORES THEM
    auto moduleStates = kinematics.ToSwerveModuleStates(speeds);

    // DESATURATES THE WHEEL SPEED TO NOT GO OVER THE MAX VELOCITY THE DRIVE BASE CAN GO
    kinematics.DesaturateWheelSpeeds(&moduleStates, moduleMaxSpeed);

    // STORES THE DESATURATED MODULE STATES TO VALUES FOR EACH INDIVIDUAL MODULE
    auto [frontR, frontL, backR, backL] = moduleStates;

    // OPTIMIZES THE MODULES TO MAKE THE MODULE TURN THE LEAST DISTANCE WHEN DRIVING
    // NEED TO INCLUDE THE OPTIMIZATION (NEED A GET DISTANCE FUNCTION FROM SWERVE MODULE CLASS)
    // auto frontROptimized = frc::SwerveModuleState::Optimize(frontR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backROptimized = frc::SwerveModuleState::Optimize(backR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto frontLOptimized = frc::SwerveModuleState::Optimize(frontL, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backLOptimized = frc::SwerveModuleState::Optimize(backL, units::radian_t(azimuthEncoder.GetDistance()));

    // TURNS EACH MODULE TO DESIRED ANGLE
    fl.SetModuleAbsolutePosition(frontL.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FrontLAngle", frontL.angle.Degrees().value());
    fr.SetModuleAbsolutePosition(frontR.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("FrontRAngle", frontR.angle.Degrees().value());
    bl.SetModuleAbsolutePosition(backL.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("backLAngle", backL.angle.Degrees().value());
    br.SetModuleAbsolutePosition(backR.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("backRAngle", backR.angle.Degrees().value());

    // DRIVES EACH MODULE AT DESIRED WHEEL SPEED
    fl.DriveModule(frontL.speed.value());
    frc::SmartDashboard::PutNumber("frontLspeed", frontL.speed.value());
    fr.DriveModule(frontR.speed.value());
    frc::SmartDashboard::PutNumber("frontRspeed", frontR.speed.value());
    bl.DriveModule(backL.speed.value());
    frc::SmartDashboard::PutNumber("BackLspeed", backL.speed.value());
    br.DriveModule(backR.speed.value());
    frc::SmartDashboard::PutNumber("backRspeed", backR.speed.value());
}
