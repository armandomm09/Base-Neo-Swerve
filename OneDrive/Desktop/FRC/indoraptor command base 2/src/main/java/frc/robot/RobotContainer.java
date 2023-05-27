// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.comandoMuneca;
import frc.robot.subsystems.chasisSubsistema;
import frc.robot.subsystems.subsistemaIntake;
import frc.robot.subsystems.subsistemaMuneca;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final chasisSubsistema chasis = new chasisSubsistema();
  private final subsistemaIntake intake = new subsistemaIntake();
  private final subsistemaMuneca muneca = new subsistemaMuneca();

  private final Joystick control = new Joystick(0);
  public RobotContainer() {
    configureButtonBindings();
  }

 
  private void configureButtonBindings() {
  new JoystickButton(control, )
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
