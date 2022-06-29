#include "Swerve/Swerve.h"

Swerve::Swerve(SwerveModule::SwerveModuleDefinition frontLeft,
               SwerveModule::SwerveModuleDefinition frontRight,
               SwerveModule::SwerveModuleDefinition backLeft,
               SwerveModule::SwerveModuleDefinition backRight)
: fl(frontLeft, 0),
  fr(frontRight, 90),
  bl(backLeft, 270),
  br(backRight, 180)
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
    // CREATES A CHASSIS SPEED OBJECT AND STORES GENERAL VALS FOR THE DRIVE BASE
    // 45 degrees needs to be updated to robot angle (with 0 degrees being facing the opposite alliance wall)
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVelocity, yVelocity, rotVelocity, frc::Rotation2d(45_deg));

    // CONVERTS CHASSIS STATE TO SWERVE MODULE STATES AND STORES THEM
    auto moduleStates = kinematics.ToSwerveModuleStates(speeds);

    // DESATURATES THE WHEEL SPEED TO NOT GO OVER THE MAX VELOCITY THE DRIVE BASE CAN GO
    kinematics.DesaturateWheelSpeeds(&moduleStates, moduleMaxSpeed);

    // STORES THE DESATURATED MODULE STATES TO VALUES FOR EACH INDIVIDUAL MODULE
    auto [frontR, backR, frontL, backL] = moduleStates;

    // OPTIMIZES THE MODULES TO MAKE THE MODULE TURN THE LEAST DISTANCE WHEN DRIVING
    // NEED TO INCLUDE THE OPTIMIZATION (NEED A GET DISTANCE FUNCTION FROM SWERVE MODULE CLASS)
    // auto frontROptimized = frc::SwerveModuleState::Optimize(frontR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backROptimized = frc::SwerveModuleState::Optimize(backR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto frontLOptimized = frc::SwerveModuleState::Optimize(frontL, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backLOptimized = frc::SwerveModuleState::Optimize(backL, units::radian_t(azimuthEncoder.GetDistance()));

    // TURNS EACH MODULE TO DESIRED ANGLE
    fl.turnModule(frontL.angle.Degrees().value());
    fr.turnModule(frontR.angle.Degrees().value());
    bl.turnModule(backL.angle.Degrees().value());
    br.turnModule(backR.angle.Degrees().value());

    // DRIVES EACH MODULE AT DESIRED WHEEL SPEED
    fl.driveModule(frontL.speed.value());
    fr.driveModule(frontR.speed.value());
    bl.driveModule(backL.speed.value());
    br.driveModule(backR.speed.value());
}

void Swerve::RegularDrive(units::meters_per_second_t xVelocity, units::meters_per_second_t yVelocity, units::radians_per_second_t rotVelocity)
{
    // CREATES A CHASSIS SPEED OBJECT AND STORES GENERAL VALS FOR THE DRIVE BASE
    frc::ChassisSpeeds speeds{xVelocity, yVelocity, rotVelocity};

    // CONVERTS CHASSIS STATE TO SWERVE MODULE STATES AND STORES THEM
    auto moduleStates = kinematics.ToSwerveModuleStates(speeds);

    // DESATURATES THE WHEEL SPEED TO NOT GO OVER THE MAX VELOCITY THE DRIVE BASE CAN GO
    kinematics.DesaturateWheelSpeeds(&moduleStates, moduleMaxSpeed);

    // STORES THE DESATURATED MODULE STATES TO VALUES FOR EACH INDIVIDUAL MODULE
    auto [frontR, backR, frontL, backL] = moduleStates;

    // OPTIMIZES THE MODULES TO MAKE THE MODULE TURN THE LEAST DISTANCE WHEN DRIVING
    // NEED TO INCLUDE THE OPTIMIZATION (NEED A GET DISTANCE FUNCTION FROM SWERVE MODULE CLASS)
    // auto frontROptimized = frc::SwerveModuleState::Optimize(frontR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backROptimized = frc::SwerveModuleState::Optimize(backR, units::radian_t(azimuthEncoder.GetDistance()));
    // auto frontLOptimized = frc::SwerveModuleState::Optimize(frontL, units::radian_t(azimuthEncoder.GetDistance()));
    // auto backLOptimized = frc::SwerveModuleState::Optimize(backL, units::radian_t(azimuthEncoder.GetDistance()));

    // TURNS EACH MODULE TO DESIRED ANGLE
    fl.turnModule(frontL.angle.Degrees().value());
    fr.turnModule(frontR.angle.Degrees().value());
    bl.turnModule(backL.angle.Degrees().value());
    br.turnModule(backR.angle.Degrees().value());

    // DRIVES EACH MODULE AT DESIRED WHEEL SPEED
    fl.driveModule(frontL.speed.value());
    fr.driveModule(frontR.speed.value());
    bl.driveModule(backL.speed.value());
    br.driveModule(backR.speed.value());
}