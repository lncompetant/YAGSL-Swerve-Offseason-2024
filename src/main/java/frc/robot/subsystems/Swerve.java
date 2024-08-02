// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

//import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Swerve extends SubsystemBase {

  private SwerveDrive swerveDrive; // create a swerve drive
  private double maximumSpeed = Units.feetToMeters(4.5); // max speed is 4.5 feet per second
  private Translation2d centerOfRotation = new Translation2d();

  public Swerve(File directory) {
    try {
      // File swerveJsonDirectory = new
      // File(Filesystem.getDeployDirectory(),"swerve"); Create a "Json directory
      // File" type to pass to the next line (This line is probably not needed
      // anymore)
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed); //
    } catch (IOException e) {
      e.printStackTrace();
    }
  }


  public ChassisSpeeds getTargetSpeeds(double Xinput, double Yinput, double angle, double currentHeading, double maxspeed){
    Xinput=Math.pow(Xinput, 3);
    Yinput=Math.pow(Yinput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(Xinput, Yinput, angle, currentHeading, maxspeed);
  }

  public double getMaxSpeed() {
    return maximumSpeed;
}

  public void driveRobot(Translation2d translation, double rotatation, boolean fieldRelativity, boolean isOpenLoop, Translation2d centerOfRotation){  //theoretically can be used for robot and field oriented drive
    swerveDrive.drive(translation, rotatation, fieldRelativity, isOpenLoop, centerOfRotation);
  }
  

  public void setRotationCenter(Translation2d rotationCenter){
  centerOfRotation=rotationCenter;
}

  public Translation2d getRotationCenter() {
    return centerOfRotation;
}

  public boolean getHeadingCorrection(){
    return swerveDrive.headingCorrection;
  }

  public void setHeadingCorrection(boolean headingCorrection){
    swerveDrive.headingCorrection=headingCorrection;
  }

  public SwerveDriveKinematics getKinematics(){
    return swerveDrive.kinematics;
    }

  public Pose2d getPose() {
    return swerveDrive.getPose();
    }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
}
  public void OneMustImagineSisyphusHappy(){  //Context behind the naming convention: https://www.youtube.com/watch?v=J9elzJqlqfc
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch(){
    return swerveDrive.getPitch();
  }

  public void overrideMaxSpeed(double MaximumSpeed){
    swerveDrive.setMaximumSpeed(MaximumSpeed);
  }

  public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }


  /*public Command driveFieldOriented(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {  //sample code supplied
    swerveDrive.setHeadingCorrection(true); // "Normally you would want heading correction for this kind of control" - YAGSL devs
    return run(() -> { // Returns whatever "Run" is so that VScode doesn't cry
      Translation2d scaledInputs = SwerveMath
          .cubeTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));
      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
          scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  public Command driveRobotOriented(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angleRotation){
    return run(() -> { // Returns whatever "Run" is so that VScode doesn't cry
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
      translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), angleRotation.getAsDouble()*swerveDrive.getMaximumAngularVelocity(),false,false);
    });
  }*/

   
  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
  }
}
