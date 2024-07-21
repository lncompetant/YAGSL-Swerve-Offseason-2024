// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

//import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Swerve extends SubsystemBase {

  private SwerveDrive swerveDrive; // create a swerve drive

  public Swerve(File directory) {
    double maximumSpeed = Units.feetToMeters(4.5); // max speed is 4.5 feet per second
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

  public Command driveFieldOriented(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {  //sample code supplied
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
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
