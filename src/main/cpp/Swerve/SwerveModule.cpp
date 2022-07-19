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

    switch (azimuthOffset)
    {
        case 0:
            azimuthTrueZero = definition.zeroPosition;
        break;
        case 90:
            azimuthTrueZero = definition.zeroPosition - 0.25 * ticksPerRotation; // arbitrary number, need to prove
        break;
        case 180:
            azimuthTrueZero = definition.zeroPosition + 0.5 * ticksPerRotation; // arbitrary number, need to prove
        break;
        case 270:
            azimuthTrueZero = definition.zeroPosition + 0.25 * ticksPerRotation; // arbitrary number, need to prove
        break;
    }

    if (azimuthTrueZero >= 1)
    {
        azimuthTrueZero -= 1;
    }
    else if (azimuthTrueZero < 0)
    {
        azimuthTrueZero += 1;
    }
}

void SwerveModule::turnModule(double degrees)
{
    azimuthPIDController.SetReference(azimuthTrueZero + DegreesToPosition(degrees),rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void SwerveModule::driveModule(double velocity)
{
    driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, velocity);
}

double SwerveModule::DegreesToPosition(double degrees)
{
    return degrees / 360;
}



