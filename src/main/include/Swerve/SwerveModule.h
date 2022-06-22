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

        bool invertDrive;

        PIDConfig drivePID;
        PIDConfig turnPID;
    };

    SwerveModule(const SwerveModuleDefinition& definition, int azimuthOffset);
    void turnModule(double degrees);
    void driveModule(double velocity);

private:

    int azimuthTrueZero;

    ctre::phoenix::motorcontrol::can::TalonFX driveMotor;
    rev::CANSparkMax azimuthMotor;
    rev::SparkMaxPIDController azimuthPIDController;
    ctre::phoenix::sensors::WPI_CANCoder azimuthEncoder;

};
