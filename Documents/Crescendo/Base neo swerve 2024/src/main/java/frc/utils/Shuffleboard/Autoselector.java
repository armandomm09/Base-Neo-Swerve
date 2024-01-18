/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */
package frc.utils.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shuffleboard;
import frc.robot.commands.autos.autos;

public class Autoselector {

    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public Autoselector(){
        //autoChooser.addOption("Align Home", autos.alignAuto());
        //autoChooser.addOption("Default", autos.autoDefault());

        Shuffleboard.kShuffleboardTab.add(autoChooser);
    }


    public static Command getAutoSelected() {
        return autoChooser.getSelected();
    }
}
