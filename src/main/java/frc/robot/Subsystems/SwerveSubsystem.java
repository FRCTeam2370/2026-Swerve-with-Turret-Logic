// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

// import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
//import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.Constants.FieldConstants;

public class SwerveSubsystem extends SubsystemBase {
  public static Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
  private static Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
  public SwerveModule[] mSwerveModules;
  public static SwerveDriveOdometry odometry;

  public static PIDController rotationPIDauto = new PIDController(0.075, 0.0, 0.01);
  public static PIDController rotationPID = new PIDController(0.5, 0, 0);

  public static Field2d field = new Field2d();

  public static SwerveDrivePoseEstimator poseEstimator;

  public static Optional<Alliance> color;

  private static Pose2d limelightPose, limelight2Pose;

  private static Transform2d RobotToTurret = new Transform2d(-0.1, 0, Rotation2d.fromDegrees(0));
  private static FieldObject2d turret = field.getObject("Turret");
  private static FieldObject2d HubFieldPose = field.getObject("HubPose");
  private static Pose2d HubPose = new Pose2d(0,0, new Rotation2d());

  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    color = DriverStation.getAlliance();
    SmartDashboard.putString("Alliace Color", color.toString());

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    configureGyro();

    mSwerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.FLConstants.FLConstants),
      new SwerveModule(1, Constants.FRConstants.FRConstants),
      new SwerveModule(2, Constants.BLConstants.BLConstants),
      new SwerveModule(3, Constants.BRConstants.BRConstants)
    };

    LimelightHelpers.setPipelineIndex("limelight", 0);

    odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kinematics, getRotation2d(), getModulePositions());
    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.kinematics, getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), getRotation2d()));

    resetOdometry(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), readGyro()));

    configurePathPlanner();

    

    //Logging
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Mod 0 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[0].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 1 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[1].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 2 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[2].getCANcoder().getDegrees()).getDegrees());
    // SmartDashboard.putNumber("Mod 3 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[3].getCANcoder().getDegrees()).getDegrees());

    
    // SmartDashboard.putNumber("Wheel MPS", mSwerveModules[0].getWheelMPS());
    // SmartDashboard.putNumber("Wheel Meters", mSwerveModules[0].getModuleMeters());

    // SmartDashboard.putNumber("wheel Velocity", mSwerveModules[0].getWheelVelocity());
    // SmartDashboard.putNumber("drive Voltage", mSwerveModules[0].getWheelVoltage());

    SmartDashboard.putNumber("Gyro Val", getRotation2d().getDegrees());
    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putNumber("pose x", poseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("pose y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("pose rot", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    // SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Calculated Turret Angle", turretRotationToPose(FieldConstants.HubFieldPose).getDegrees());

    updateOdometry();
    //odometry.update(getRotation2d(), getModulePositions());//USE THIS WHEN TESTING AUTOS WITHOUT FIELD LOCALIZATION
    resetOdometry(poseEstimator.getEstimatedPosition());

    field.setRobotPose(poseEstimator.getEstimatedPosition());
    turret.setPose(new Pose2d(turretToField().getTranslation(), turretRotationToPose(HubPose)));
    HubFieldPose.setPose(HubPose);   

    publisher.set(poseEstimator.getEstimatedPosition());
  }

  //This method calculates the position of the turret relative to the field 
  //It uses the position of the turret relative to the robot relative to the field
  public static Pose2d turretToField(){
    double xlocal = RobotToTurret.getX();
    double ylocal = RobotToTurret.getY();

    double xGlobal = xlocal * Math.cos(getRotation2d().getRadians()) + ylocal * Math.sin(getRotation2d().getRadians()) + poseEstimator.getEstimatedPosition().getX();
    double yGlobal = ylocal * Math.cos(getRotation2d().getRadians()) + xlocal * Math.sin(getRotation2d().getRadians()) + poseEstimator.getEstimatedPosition().getY();

    return new Pose2d(xGlobal, yGlobal, poseEstimator.getEstimatedPosition().getRotation());
  }

  //This method calculates the angle from the turret to the target Pose2d
  public static Rotation2d turretRotationToPose(Pose2d pose){
    Pose2d turretpose = turretToField();
    double thetaWorldToTarget = Math.atan2((turretpose.getY() - pose.getY()), (turretpose.getX() - pose.getX()));
    double thetaTurretToTarget = thetaWorldToTarget + Math.PI - getRotation2d().getRadians();// adding pi here is an offset

    return Rotation2d.fromRadians(thetaTurretToTarget);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotationPID.calculate(rotation), readGyro()) :
      new ChassisSpeeds(translation.getX(), translation.getY(), rotationPIDauto.calculate(rotation))
    );    

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    mSwerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);
    mSwerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      mSwerveModules[0].getPosition(),
      mSwerveModules[1].getPosition(),
      mSwerveModules[2].getPosition(),
      mSwerveModules[3].getPosition()
    };
    return positions;
  }

  public static void configureGyro(){
    resetGyro();
  }
  
  public static void resetGyro(){
    gyro.setYaw(Constants.SwerveConstants.gyroOffset);
  }

  public static Rotation2d readGyro(){
    return color.isPresent() && color.get() == Alliance.Blue ? gyro.getRotation2d() : gyro.getRotation2d().rotateBy(Rotation2d.fromDegrees(180));
  }

  public static Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(readGyro().getDegrees() + 90);
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);//pose.getRotation()
    //poseEstimator.resetPose(pose);
  }

  public void updateOdometry(){
    poseEstimator.update(getRotation2d(), getModulePositions());
  }

  public static Pair<Pose2d, Double> getLimelightPose(){  
    boolean doRejectUpdate = false;
    Pose2d returnPose = null;
    double returnTime = 0;

    LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    if(mt2 != null){
      if(Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 720){
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0){
        doRejectUpdate = true;
      }
      if(!doRejectUpdate){
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        returnPose = mt2.pose;
        returnTime = mt2.timestampSeconds;
      }
    }

    return Pair.of(returnPose, returnTime);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();//poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    );
  }

  public void goToPose(Pose2d pose){

  }

  public Command PathfindToPose(Supplier<Pose2d> poseSupplier){
    return new DeferredCommand(()-> AutoBuilder.pathfindToPose(poseSupplier.get(), Constants.SwerveConstants.telePathConstraints), Set.of(this));
  }

  public Command PathfindThenFollow(PathPlannerPath path){
    path.preventFlipping = false;
    return new DeferredCommand(()-> AutoBuilder.pathfindThenFollowPath(path, Constants.SwerveConstants.telePathConstraints), Set.of(this));
  }

  public Command followPath(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false);
  }

  
  public void configurePathPlanner(){
      RobotConfig config = new RobotConfig(36.287, 4.095, new ModuleConfig(0.0508, Constants.SwerveConstants.maxSpeed, 1.200, DCMotor.getKrakenX60(1), 60, 1), Constants.SwerveConstants.kinematics.getModules());
      try{
        config = RobotConfig.fromGUISettings();
        System.out.println("---------------------------Configured from GUI---------------------------");
      }catch(Exception exception){
        exception.printStackTrace();
        System.out.println("Doesn't like to config from gui settings");
      }
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds),//drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond / Constants.SwerveConstants.maxAngularVelocity, false, false),//drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond / 3.1154127, false, true),//driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(0, 0.0, 0.0), // Translation PID constants// 3.8 - p
                      new PIDConstants(0, 0.0, 0.0) // Rotation PID constants//kp 0.00755, ki 0.0001
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                if (color.isPresent()) {
                  return color.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[]{
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    };
    return states;
  }
}
