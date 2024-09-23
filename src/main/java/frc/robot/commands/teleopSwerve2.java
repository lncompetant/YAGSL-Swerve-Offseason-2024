// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class teleopSwerve2 extends Command {
  final Swerve swerve2;
  private final DoubleSupplier vX, vY, heading,angle;


  /** Creates a new teleopSwerve2. */
  public teleopSwerve2(Swerve swerve,DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, DoubleSupplier angle) {
    this.swerve2 = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.angle = angle;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // ChassisSpeeds desiredSpeeds = swerve2.getTargetSpeeds(vX, vY, angle, heading,  Units.feetToMeters(4.5));
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(vX.getAsDouble(), vY.getAsDouble(), angle.getAsDouble());
    swerve2.driveChassisSpeed(desiredSpeeds);
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
