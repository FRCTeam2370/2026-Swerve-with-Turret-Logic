// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Commands.FindDriveKS;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.TurretCommands.PointTurretAtPoint;
import frc.robot.Commands.TurretCommands.SetTurretRotation;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);

  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final TurretSubsystem mTurretSubsystem = new TurretSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //Put all NamedCommands here
    NamedCommands.registerCommand("Test", new ResetGyro(mSwerve));

    configureBindings();
  }

  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> driver.getRawAxis(0), ()-> -driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));

    //driver.b().whileTrue(mSwerve.PathfindToPose(() -> new Pose2d(16.3,0.49, Rotation2d.fromDegrees(-50))));
    // try {
    //   driver.b().whileTrue(mSwerve.PathfindThenFollow(PathPlannerPath.fromPathFile("CoralLoadLR")));
    // } catch (FileVersionException | IOException | ParseException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    // driver.x().whileTrue(mSwerve.PathfindToPose(() -> Constants.RedSidePoses.REDFRONTLEFTSCORE));
    // driver.y().whileTrue(mSwerve.PathfindToPose(()-> Constants.RedSidePoses.REDBACKLEFTSCORE));

    driver.a().onTrue(new ResetGyro(mSwerve));

    //driver.leftTrigger().toggleOnTrue(new FindDriveKS(mSwerve));

    driver.b().toggleOnTrue(new PointTurretAtPoint(FieldConstants.HubFieldPose, mTurretSubsystem));
    //driver.b().toggleOnTrue(new PointTurretAtPoint(new Pose2d(2,2, new Rotation2d()), mTurretSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
