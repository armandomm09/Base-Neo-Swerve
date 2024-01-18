/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */
package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.limelight.autoAlign;
import frc.utils.AutoUtils;

public class autos extends AutoUtils {

    
    private static PathPlannerPath test = PathPlannerPath.fromPathFile("pathTest1");
    /*public static Command autoForward(){
        return Commands.sequence(TrajectoryReader.readTrajectory(goForward, true));
    }


    public static Command alignAuto(){

        return Commands.sequence(
            TrajectoryReader.readTrajectory(holonomic, true),
            new autoAlign(swerve, limelight, true));
    }

    public static Command holonomic(){
        return Commands.sequence(TrajectoryReader.readTrajectory(holonomic, true),
        new autoAlign(swerve, limelight, true));
    }
    public static Command autoDefault(){
        return Commands.sequence(
            TrajectoryReader.readTrajectory(defaultAuto, true)
        );
    }
*/
    public static Command test_papaya() {
        return AutoBuilder.followPath(test);
    }


    
}