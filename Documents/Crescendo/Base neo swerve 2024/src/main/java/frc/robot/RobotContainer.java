/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swerveDriveComando;
import frc.robot.commands.autos.autos;
import frc.robot.commands.limelight.autoAlign;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.swerveSusbsystem;

public class RobotContainer {

    //private final subsistemaSwerve swerveSubsystem = new subsistemaSwerve();
    private swerveSusbsystem swerveSubsystem;
    private LimeLightObject limelight;

    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    
  

    public RobotContainer(){

        swerveSubsystem = swerveSusbsystem.getInstance();
        limelight  = LimeLightObject.getInstance();

        swerveSubsystem.setDefaultCommand(new swerveDriveComando(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                true
                ));

              //limelight.setDefaultCommand(new limelighCommand(swerveSubsystem, limelight, false));

        configureButtonBindings();
    }

    
    private void configureButtonBindings() {

        //APRIL TAG:
       //new JoystickButton(driverJoytick, 5).whileTrue(new autoAlign(swerveSubsystem, limelight, true));

       //REFLECTIVE TAPE:
        //new JoystickButton(driverJoytick, 4).whileTrue(new autoAlign(swerveSubsystem, limelight, false));

    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
       //return autos.autoForward();
       return autos.test_papaya();

        // 5. Add some init and wrap-up, and return everything
    }
}
