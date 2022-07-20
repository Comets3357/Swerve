#include "Swerve/SwerveModule.h"

SwerveModule::SwerveModule(const SwerveModuleDefinition& definition, int azimuthOffset)
: driveMotor(definition.driveMotorID), 
  azimuthMotor(definition.azimuthMotorID, rev::CANSparkMax::MotorType::kBrushless),
  azimuthPIDController(azimuthMotor.GetPIDController())
{
    driveMotor.ConfigFactoryDefault();

    driveMotor.SetInverted(definition.invertDrive);
    driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    driveMotor.ConfigStatorCurrentLimit(ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration(true, 75, 50, 1.0)); // may need to change this

    driveMotor.Config_kP(0, definition.drivePID.p);
    driveMotor.Config_kI(0, definition.drivePID.i);
    driveMotor.Config_kD(0, definition.drivePID.d);
    driveMotor.Config_kF(0, definition.drivePID.ff);

    azimuthMotor.RestoreFactoryDefaults();

    azimuthMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);   
    azimuthMotor.SetSmartCurrentLimit(45); // need to adjust this val
    
    azimuthPIDController.SetP(definition.turnPID.p);
    azimuthPIDController.SetI(definition.turnPID.i);
    azimuthPIDController.SetD(definition.turnPID.d);
    azimuthPIDController.SetFF(definition.turnPID.ff);

    // need to set up current limiting
    //GEARS 150:7

    switch (azimuthOffset)
    {
        case 0:
            azimuthAbsouteTrueZero = definition.zeroPosition;
        break;
        case 90:
            azimuthAbsouteTrueZero = definition.zeroPosition + 0.75; // arbitrary number, need to prove
        break;
        case 180:
            azimuthAbsouteTrueZero = definition.zeroPosition + 0.5; // arbitrary number, need to prove
        break;
        case 270:
            azimuthAbsouteTrueZero = definition.zeroPosition + 0.25; // arbitrary number, need to prove
        break;
    }

    if (azimuthAbsouteTrueZero >= 1)
    {
        azimuthAbsouteTrueZero -= 1;
    }
    else if (azimuthAbsouteTrueZero < 0)
    {
        azimuthAbsouteTrueZero += 1;
    }
}

void SwerveModule::SetModuleRelativePosition(double degrees)
{
    targetAzimuthAbsolute = DegreesToAbsolute(degrees);
    targetAzimuthRev = targetAzimuthAbsolute * ticksPerRotation;
    azimuthPIDController.SetReference(azimuthRevEncoder.GetPosition() + targetAzimuthRev,rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void SwerveModule::SetModuleAbsolutePosition(double degrees)
{
    double azimuthDegreesToTravel = degrees - GetAzimuthDegrees();
    double azimuthRevToTravel = azimuthDegreesToTravel/360*ticksPerRotation;
    azimuthPIDController.SetReference(azimuthRevEncoder.GetPosition() + azimuthRevToTravel,rev::CANSparkMaxLowLevel::ControlType::kPosition);
}



void SwerveModule::DriveModule(double velocity)
{
    driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, velocity);
}

double SwerveModule::DegreesToAbsolute(double degrees)
{
    double absoluteDesired = (azimuthAbsouteTrueZero + degrees/360);
    if (absoluteDesired > 1)
    {
        absoluteDesired--;
    }
    else if (absoluteDesired < 0)
    {
        absoluteDesired++;
    }
    return absoluteDesired;
}

double SwerveModule::GetAbsolutePosition()
{
    return azimuthAbsoluteEncoder.GetPosition();
}

double SwerveModule::GetAzimuthDegrees()
{
    return std::fmod(360+std::fmod(((GetAbsolutePosition() - azimuthAbsouteTrueZero)*360),360),360);
}



