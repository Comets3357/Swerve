#include "Swerve/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const SwerveModuleDefinition& definition, int azimuthOffset, int driveID, int azimuthID)
: driveMotor(driveID), 
  azimuthMotor(azimuthID, rev::CANSparkMax::MotorType::kBrushless),
  azimuthPIDController(azimuthMotor.GetPIDController()),
  azimuthRevEncoder(azimuthMotor.GetEncoder()),
  azimuthAbsoluteEncoder(azimuthMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192))
{
    azimuthMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    azimuthMotor.SetInverted(true);
    //driveMotor{DriveID};
    //azimuthMotor{swerveDefinition.azimuthMotorID, rev::CANSparkMax::MotorType::kBrushless};

    swerveDefinition = definition;

    driveMotor.ConfigFactoryDefault();

    driveMotor.SetInverted(definition.invertDrive);
    driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    driveMotor.ConfigStatorCurrentLimit(ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration(true, 75, 50, 1.0)); // may need to change this

    //driveMotor.Config_kP(0, definition.drivePID.p);
    //driveMotor.Config_kI(0, definition.drivePID.i);
    //driveMotor.Config_kD(0, definition.drivePID.d);
    //driveMotor.Config_kF(0, definition.drivePID.ff);
    driveMotor.Config_kP(0, 0.07);
    driveMotor.Config_kF(0,1023/20160);

    azimuthMotor.RestoreFactoryDefaults();

    azimuthMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);   
    azimuthMotor.SetSmartCurrentLimit(45); // need to adjust this val
    
    // azimuthPIDController.SetP(definition.turnPID.p);
    // azimuthPIDController.SetI(definition.turnPID.i);
    // azimuthPIDController.SetD(definition.turnPID.d);
    // azimuthPIDController.SetFF(definition.turnPID.ff);
    azimuthPIDController.SetP(1);

    azimuthRevEncoder.SetPosition(0);

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

void SwerveModule::SetModuleRelativePosition(double degreesTarget)
{
    double degrees = degreesTarget;
    if (degrees < 0)
    {
        degrees += 360;
    }
    targetAzimuthAbsolute = DegreesToAbsolute(degrees);
    targetAzimuthRev = targetAzimuthAbsolute * ticksPerRotation;
    
    azimuthPIDController.SetReference(azimuthRevEncoder.GetPosition() + targetAzimuthRev,rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void SwerveModule::SetModuleAbsolutePosition(double degreesTarget)
{
    // if (!zeroed){
    //     azimuthRevEncoder.SetPosition((azimuthAbsoluteEncoder.GetPosition()/encoderMax)-azimuthAbsouteTrueZero);
    //     zeroed = true;
    // }
    double degrees = degreesTarget;
    if (degrees < 0)
    {
        degrees += 360;
    }
    double amountToMove = (degrees) / 360 * ticksPerRotation - std::fmod(ticksPerRotation+std::fmod(azimuthRevEncoder.GetPosition(), ticksPerRotation),ticksPerRotation);
    if (amountToMove > ticksPerRotation / 2)
    {
        amountToMove -= ticksPerRotation;
    }
    else if (amountToMove < ticksPerRotation / -2)
    {
        amountToMove += ticksPerRotation;
    }
    azimuthPIDController.SetReference(amountToMove + azimuthRevEncoder.GetPosition(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
    
}



void SwerveModule::DriveModule(double velocity)
{
    driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, velocity*MAXSPEED);
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

void SwerveModule::DisabledPeriodic()
{
    azimuthMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}



