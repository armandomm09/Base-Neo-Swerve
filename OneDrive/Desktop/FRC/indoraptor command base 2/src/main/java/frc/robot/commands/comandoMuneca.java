// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.chasisSubsistema;
import frc.robot.subsystems.subsistemaMuneca;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class comandoMuneca extends CommandBase {
 
  private final subsistemaMuneca muneca;
  private final double velocidad;
  
  public comandoMuneca(subsistemaMuneca muneca, double velocidad){   
    this.muneca = muneca;
    this.velocidad = velocidad;
    addRequirements(muneca);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    muneca.darVelocidadMuneca(velocidad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    muneca.darVelocidadMuneca(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
