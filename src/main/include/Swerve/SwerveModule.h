#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <string>
#include <wpi/numbers>
#include <frc/geometry/Rotation2d.h>


class SwerveModule
{
public:

    struct PIDConfig
    {
        double p;
        double i;
        double d;
        double ff;
    };

    struct SwerveModuleDefinition
    {
        int zeroPosition;

        int driveMotorID;
        int azimuthMotorID;
        int azimuthCANCoderID;

        int ticksPerRotation = 4096;

        bool invertDrive;

        PIDConfig drivePID;
        PIDConfig turnPID;
    };

    SwerveModule(const SwerveModuleDefinition& definition, int azimuthOffset);
    void TurnModule(double radians);
    void SetModulePosition(double radians);
    void DriveModule(double velocity);

    //frc::Rotation2d azimuthRotation = frc::Rotation2d::Rotation2d();



private:

    int ticksPerRotation = 900;

    double RadiansToAbsolutePosition(double radians);
    double RadiansToDegrees(double radians);
    double DegreesToAbsolute(double degrees);
    double GetAbsolutePosition();
    double GetAzimuthDegrees();

    double azimuthAbsouteTrueZero;

    double currentAzimuthAbsolute = 0;
    double targetAzimuthAbsolute = 0;
    double targetAzimuthRev = 0;

    ctre::phoenix::motorcontrol::can::TalonFX driveMotor;
    rev::CANSparkMax azimuthMotor;
    rev::SparkMaxPIDController azimuthPIDController;

    rev::SparkMaxAlternateEncoder azimuthAbsoluteEncoder = azimuthMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 4096);
    rev::SparkMaxRelativeEncoder azimuthRevEncoder = azimuthMotor.GetEncoder();

};
