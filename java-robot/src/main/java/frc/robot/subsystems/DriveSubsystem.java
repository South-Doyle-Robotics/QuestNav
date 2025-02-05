// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.QuestNav;
import frc.robot.Constants.DriveConstants;
import frc.utils.ShuffleUtils;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
// PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private QuestNav m_questNav = new QuestNav();

  private final StructPublisher<Pose2d> m_robotPose = NetworkTableInstance.getDefault().getStructTopic("Robot Pose (QuestNav)", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> m_robotPoseOdometry = NetworkTableInstance.getDefault().getStructTopic("Robot Pose (Odometry)", Pose2d.struct).publish();

  // Slew rate filter variables for tuning lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private Pose2d resetPosition = new Pose2d(new Translation2d(0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
  private double m_angleSetpoint = 0.0;
  private PIDController anglePIDController = new PIDController(1.0/60.0, 0, 0);

  // Odometry class for tracking the robot's pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_questNav.getPose().getRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  // Create a new DriveSubsystem
  public DriveSubsystem() {
    anglePIDController.enableContinuousInput(-180, 180);
    // m_questNav.setInitialPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
    setQuestConnectionStatus(false);
    zeroHeading();
    zeroPosition();

    // PPlanner Config
    RobotConfig ppconfig = null;
    try{
      ppconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
            ),
            ppconfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
  }

  // Update odometry in the periodic block
  @Override
  public void periodic() {
    m_odometry.update(
        m_questNav.getPose().getRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_robotPose.set(invertRotation(getPose()));
    m_robotPoseOdometry.set(invertRotation(getPoseOdometry()));
    setQuestConnectionStatus(m_questNav.connected());

    m_questNav.checkMessages();
    Logger.recordOutput("SwerveOdometry", m_odometry.getPoseMeters());
    Logger.recordOutput("OculusPosituion", m_questNav.getPose());
    Logger.recordOutput("OculusQuaternion", m_questNav.getQuaternion());
  }

  private void setQuestConnectionStatus(boolean connected) {
    GenericEntry state = ShuffleUtils.getEntryByName(Shuffleboard.getTab("QuestNav"), "Connected");
    if (state != null)
      state.setBoolean(connected);

    GenericEntry textState = ShuffleUtils.getEntryByName(Shuffleboard.getTab("QuestNav"), "QuestNav Connected?");
    if (textState != null) {
      textState.setString(connected ? "^ CONNECTED ^" : "!!NOT CONNECTED!!");
    }
  }

  public void questNavTestMessage() {
    m_questNav.testMessages();
  }

  public void questNavTestMessageNot() {
    m_questNav.testMessagesOff();
  }

  // Return the currently-estimated pose of the robot
  public Pose2d getPose() {
    return m_questNav.getPose();
  }

  public Pose2d getPoseOdometry() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d invertRotation(Pose2d pose) {
    Pose2d newPose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(360).minus(pose.getRotation()));
    return newPose;
  }

  // Reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    m_questNav.resetPose();
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
    m_odometry.resetPosition(
        m_questNav.getPose().getRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void setPose(Pose2d pose) {
    m_angleSetpoint -= m_questNav.getPose().getRotation().getDegrees();
    m_angleSetpoint += pose.getRotation().getDegrees();
    anglePIDController.setSetpoint(0.0);
    m_questNav.setPose(pose);

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }


      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }
    if (Math.abs(rot) > 0.05) {
      m_angleSetpoint -= Math.copySign(Math.pow(rot, 2) * 8, rot);
      if (m_angleSetpoint > 360) {
        m_angleSetpoint -= 360;
      } else if (m_angleSetpoint < 0) {
        m_angleSetpoint += 360;
      }
    }
    anglePIDController.setSetpoint(m_angleSetpoint - 180);
    Logger.recordOutput("angleSetpoint", m_angleSetpoint);

    // Convert the commanded speeds to the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = compensateAngle() * DriveConstants.kMaxAngularSpeed;
    double oculusYaw = m_questNav.getPose().getRotation().getDegrees();
    Logger.recordOutput("OculusYaw", oculusYaw);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(360 - oculusYaw))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private double compensateAngle() {
    var ret = -anglePIDController.calculate(m_questNav.getPose().getRotation().getDegrees() - 180);
    Logger.recordOutput("PIDOut", ret);
    return ret;
  }

  // Set the wheels into an X formation to prevent movement
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // Set the swerve module states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  // Zero the encoders on the swerve modules
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // Return the robot heading in degrees, between -180 and 180 degrees
  public double getHeading() {
    return m_questNav.getPose().getRotation().getDegrees();
  }

  // Get the rotation rate of the robot
  public double getTurnRate() {
    return m_questNav.getPose().getRotation().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void cleanupQuestNavMessages() {
    m_questNav.cleanUpQuestNavMessages();
  }
  
  public void zeroHeading() {
    m_angleSetpoint -= m_questNav.getPose().getRotation().getDegrees();
    anglePIDController.setSetpoint(0.0);
    m_questNav.zeroHeading();
  }

  public void zeroPosition() {
    m_questNav.zeroPosition();
  }
}
