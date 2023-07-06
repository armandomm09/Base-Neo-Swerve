// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.constantesMuneca;
import frc.robot.Constants.constantesRamsetController;
import frc.robot.Constants.constantesShooter;
import frc.robot.commands.Autos;
import frc.robot.commands.comandoChasis;
import frc.robot.commands.comandoMuneca;
import frc.robot.commands.comandoShooter;
import frc.robot.subsystems.chasisSubsistema;
import frc.robot.subsystems.subsistemaShooter;
import frc.robot.subsystems.subsistemaMuneca;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constantesRamsetController;
import frc.robot.subsystems.chasisSubsistema;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final static chasisSubsistema chasis = new chasisSubsistema();
  private final subsistemaShooter intake = new subsistemaShooter();
  private final subsistemaMuneca muneca = new subsistemaMuneca();

  public static Joystick controlDriver = new Joystick(0);
  private final Joystick controlPlacer = new Joystick(1);

  PathPlannerTrajectory avanceDerecho = PathPlanner.loadPath("PRUEBAA", new PathConstraints(2, 3));

  SendableChooser<Command> chooser = new SendableChooser<>();
 

  
  
  
  public RobotContainer() {
    chasis.setDefaultCommand(new comandoChasis(chasis, true));
    configureButtonBindings();

    
  }

  public static Command cargarTrajectoriaARamsetePP(PathPlannerTrajectory trajectory, Boolean resetearOdometria){
    return new SequentialCommandGroup(
     new InstantCommand(() -> {
         if(resetearOdometria){
             chasis.resetOdometria(trajectory.getInitialPose());
         }
     }
    ),

     new PPRamseteCommand(trajectory, 
     chasis::getPose, 
     new RamseteController(constantesRamsetController.kRamseteB, constantesRamsetController.kRamseteZeta), 
     new SimpleMotorFeedforward(constantesRamsetController.ksVolts, constantesRamsetController.kvVoltsPerMeter, constantesRamsetController.kaVoltsSquaredPerMeter), 
     constantesRamsetController.kinematicasDrive, 
     chasis::getWheelSpeeds, 
     new PIDController(constantesRamsetController.kpDriveVelocity, 0, 0), 
     new PIDController(constantesRamsetController.kpDriveVelocity, 0, 0), 
     chasis::chasisVoltje, 
     true, 
     chasis)
   );
    } 

  private void configureButtonBindings() {
    new JoystickButton(controlPlacer, constantesMuneca.botonApuntarShooter).onFalse(new comandoMuneca(muneca, "apuntar"));
    new JoystickButton(controlPlacer, constantesMuneca.botonGuardarShooter).onFalse(new comandoMuneca(muneca, "guardar"));
    new JoystickButton(controlPlacer,constantesMuneca.botonBajarShooter).onFalse(new comandoMuneca(muneca, "bajar"));
    new JoystickButton(controlPlacer, constantesMuneca.botonPararMotores).onFalse(new comandoMuneca(muneca, "parar"));
   
    new JoystickButton(controlPlacer, constantesShooter.botonDisparoArriba).onFalse(new comandoShooter(intake, "arriba"));
    new JoystickButton(controlPlacer, constantesShooter.botonDisparoAbajo).onFalse(new comandoShooter(intake, "abajo"));

    //new JoystickButton(controlDriver, constantesChasis.botonTank).onFalse(new );
    new JoystickButton(controlDriver, 1).onFalse(new comandoChasis(chasis, true));
    new JoystickButton(controlDriver, 2).onFalse(new comandoChasis(chasis, false));

  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   /*  try {
      return Autos.avanzarDerecho();
     } catch (Exception e) {
      DriverStation.reportWarning("No funciono tu codigo del asco ", true);
      return Commands.waitSeconds(1);
    }
*/
      return Autos.avanzarDerecho();
      
    
  }
}
