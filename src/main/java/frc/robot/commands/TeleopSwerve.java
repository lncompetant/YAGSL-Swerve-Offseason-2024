// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveController;

public class TeleopSwerve extends Command {
  /** Creates a new Swerve. */
  Swerve swerve;
  DoubleSupplier velocityX;
  DoubleSupplier velocityY;
  DoubleSupplier omega;  //some sort of variable used for math
  DoubleSupplier throttle;
  BooleanSupplier isFieldRelative;
  boolean isOpenLoop;
  SwerveController controller;
  public TeleopSwerve(Swerve swerve, DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier omega, DoubleSupplier throttle,BooleanSupplier isFieldRelative, boolean isOpenLoop,SwerveController controller) {
    this.swerve=swerve;
    this.velocityX=velocityX;
    this.velocityY=velocityY;
    this.omega=omega;
    this.throttle=throttle;
    this.isFieldRelative=isFieldRelative;
    this.isOpenLoop=isOpenLoop;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double modVelocityX=velocityX.getAsDouble();
    double modVelocityY=velocityY.getAsDouble();
    if(Math.abs(velocityX.getAsDouble())<0.2&&Math.abs(velocityY.getAsDouble())<0.2){
      modVelocityX=0;
      modVelocityY=0;
    }
    double xSpeed = (modVelocityX *swerve.getMaxSpeed()*MathUtil.clamp(throttle.getAsDouble(), 0.1, 1));  //Map the X value of Joystick to a usable double
    double ySpeed = (modVelocityY *swerve.getMaxSpeed()*MathUtil.clamp(throttle.getAsDouble(), 0.1, 1));  //Map the X value of Joystick to a usable double
    double angVelocity = (Math.pow(MathUtil.applyDeadband(omega.getAsDouble(), 0.2), 3) * controller.config.maxAngularVelocity) * 0.5; //Map the X value of right Joystick to a usable double for turning
    swerve.driveRobot(new Translation2d(xSpeed,ySpeed), angVelocity, isFieldRelative.getAsBoolean(), isOpenLoop);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
