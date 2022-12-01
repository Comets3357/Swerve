#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>



class SwerveModule
{
public:

    void DisabledPeriodic();

    struct PIDConfig
    {
        public:
        double p;
        double i;
        double d;
        double ff;
    };

    struct SwerveModuleDefinition
    {
        public:
        int zeroPosition;

        int driveMotorID;
        int azimuthMotorID;
        int azimuthCANCoderID;


        bool invertDrive;

        PIDConfig drivePID;
        PIDConfig turnPID;
    };

    SwerveModule(const SwerveModuleDefinition& definition, int azimuthOffset, int driveID, int azimuthID);
    void SetModuleRelativePosition(double degrees);
    void SetModuleAbsolutePosition(double degrees);
    void DriveModule(double velocity);

    SwerveModuleDefinition swerveDefinition;



private:

    bool zeroed = false;
    double encoderMax = 1;

    double ticksPerRotation = 21.4;

    double DegreesToAbsolute(double degrees);
    double GetAbsolutePosition();
    double GetAzimuthDegrees();

    double azimuthAbsouteTrueZero;

    double currentAzimuthAbsolute = 0;
    double targetAzimuthAbsolute = 0;
    double targetAzimuthRev = 0;

    double MAXSPEED = 1320*24;

    ctre::phoenix::motorcontrol::can::TalonFX driveMotor;//{swerveDefinition.driveMotorID};
    rev::CANSparkMax azimuthMotor;//{swerveDefinition.azimuthMotorID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController azimuthPIDController;

    rev::SparkMaxAlternateEncoder azimuthAbsoluteEncoder;
    rev::SparkMaxRelativeEncoder azimuthRevEncoder;

};
