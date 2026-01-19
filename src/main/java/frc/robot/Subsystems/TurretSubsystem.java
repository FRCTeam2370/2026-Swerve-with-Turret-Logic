// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  public static TalonFX turretRotationMotor = new TalonFX(TurretConstants.TurretRotationID);
  private static TalonFXConfiguration turretRotConfig = new TalonFXConfiguration();
  private static MotionMagicDutyCycle turretRotMagicCycle = new MotionMagicDutyCycle(0);
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    configTurret();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void aimTurretAtPoint(Pose2d pose){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(turretRotationsToKraken(SwerveSubsystem.turretRotationToPose(pose).getRotations())));
  }

  public static void aimTurretAtDegree(double degrees){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(Rotation2d.fromDegrees(degrees).getRotations()));
  }

  public static void aimtTurretAtRotation(double rot){
    turretRotationMotor.setControl(turretRotMagicCycle.withPosition(turretRotationsToKraken(rot)));
  }

  private static void configTurret(){
    turretRotationMotor.setNeutralMode(NeutralModeValue.Coast);
    turretRotationMotor.setPosition(0);

    turretRotConfig.Slot0.kP = 0.4;
    turretRotConfig.Slot0.kI = 0.0;
    turretRotConfig.Slot0.kD = 0.0;

    turretRotConfig.MotionMagic.MotionMagicAcceleration = 1000;
    turretRotConfig.MotionMagic.MotionMagicCruiseVelocity = 125;

    turretRotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    turretRotationMotor.getConfigurator().apply(turretRotConfig);
  }

  public static double turretRotationsToKraken(double rot){
    return rot * TurretConstants.turretRatio;
  }

}
