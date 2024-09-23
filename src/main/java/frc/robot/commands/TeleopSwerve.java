// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
  /** Creates a new Swerve. */
  Swerve swerve;
  DoubleSupplier velocityX;
  DoubleSupplier velocityY;
  DoubleSupplier omega;  //some sort of variable used for math (currently not in use)
  DoubleSupplier throttle;  //some sort of variable used for math (currently not in use)
  boolean isFieldRelative;
  boolean isOpenLoop;
  DoubleSupplier angularVelocity;


  public TeleopSwerve(Swerve swerve, DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier angularVelocity, BooleanSupplier isFieldRelative, BooleanSupplier isOpenLoop) { 
    this.swerve = swerve;
    this.velocityX=velocityX;
    this.velocityY=velocityY;
    this.angularVelocity=angularVelocity;
    this.isFieldRelative=isFieldRelative.getAsBoolean();
    this.isOpenLoop=isOpenLoop.getAsBoolean();
    this.swerve = swerve;
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
    double modAngVelocity=angularVelocity.getAsDouble();
    
    //dead zone code.  If the raw reading is below 0.2, then floor the value that will be passed to the drive function.  Otherwise, pass to the drive function.
    if(Math.abs(velocityX.getAsDouble())<0.2){
      modVelocityX=0; //dead zone
    }
    if(Math.abs(velocityY.getAsDouble())<0.2){
      modVelocityY=0; //dead zone
    }
    if(Math.abs(angularVelocity.getAsDouble())<0.2){
      modAngVelocity=0;
    }

    //actully drive the robot.
    if(modAngVelocity!=0||modVelocityX!=0||modVelocityY!=0){
    swerve.driveRobot(new Translation2d(modVelocityX,modVelocityY), modAngVelocity, isFieldRelative, isOpenLoop,new Translation2d(0,0));
    }
    else{
      swerve.OneMustImagineSisyphusHappy();
      }
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
