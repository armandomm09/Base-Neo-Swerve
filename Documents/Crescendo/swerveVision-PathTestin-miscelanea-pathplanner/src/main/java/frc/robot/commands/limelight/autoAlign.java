/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot.commands.limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.limelightOffsets;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.subsystems.LimeLightObject;
import frc.robot.subsystems.PhotonLL;
import frc.robot.subsystems.swerveSusbsystem;

public class autoAlign extends CommandBase {

   private final swerveSusbsystem swerve;
    private final PhotonLL limelight;
    private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
    private final PIDController drivePID, strafePID, rotationPID;
    private final double driveOffset, strafeOffset, rotationOffset;
    

    /**
     * Driving command
     * 
     * @param swerveSubsystem Instance for the swerve
     * @param limelight Instance for the limelight
     * @param alignToAprilTag It will be aligning to april tag or reflective tape
     */
    public autoAlign(){

        this.swerve = swerveSusbsystem.getInstance();
        this.limelight = PhotonLL.getInstance();

        /**
         * Limiters for acceleration and a better moving of the robot
         * 
         */
        this.xLimiter = new SlewRateLimiter(DriveConstants.kDriveAccelerationLimiter);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kDriveAccelerationLimiter);
        this.giroLimiter = new SlewRateLimiter(DriveConstants.kRotationAccelerationLimiter);

        /**
         * PID Controllers for the align
         */
        this.drivePID = new PIDController(
            limelightConstants.kPdrive, 
            limelightConstants.kIdrive, 
            limelightConstants.kDdrive);

        this.strafePID = new PIDController(
            limelightConstants.kPstrafe, 
            limelightConstants.kIstrafe, 
            limelightConstants.kDstrafe);

        this.rotationPID = new PIDController(
            limelightConstants.kProtation, 
            limelightConstants.kIrotation, 
            limelightConstants.kDrotation);
        /**
         * Boolean for what target to search
         */

        /**
         * Offsets for the limelight
         */
        //this.offsets = limelight.getOffsets(alingToAprilTag);  

        this.driveOffset = 2.1;
        this.strafeOffset = -0.2;
        this.rotationOffset = 10.2;

        addRequirements(swerve);
        addRequirements(limelight);

        
     }

     @Override
     public void initialize() {
        /**
         * Start a camera server so we can visualize the limelight on the Shuffleboard
         */
        CameraServer.startAutomaticCapture();
         
     }
 
     @Override
     public void execute() {

        
        

        
        double velForward = 0;
        double velStrafe = 0;
        double velGiro = 0;
 
        /**
         * If there is a seen target, calculate the PIDs velocities,
         * otherwise, rotate so the robot can search the target
         */
        if(limelight.hasValueTargets()){

            velForward = drivePID.calculate(limelight.getArea(), driveOffset);
            velStrafe = strafePID.calculate(limelight.getXDistance(), strafeOffset);
            velGiro = -rotationPID.calculate(limelight.getYaw(), rotationOffset); 
        } else if(limelight.hasValueTargets() == false){
            velForward = 0;
            velStrafe = 0;
            velGiro = 0.4;  
        } else {
            velForward = 0;
            velStrafe = 0;
            velGiro = 0; 
        }
 
         // 2. Apply deadband
          velForward = Math.abs(velForward) > OIConstants.kDeadband ? velForward : 0.0;
         velStrafe = Math.abs(velStrafe) > OIConstants.kDeadband ? velStrafe : 0.0;
         velGiro = Math.abs(velGiro) > OIConstants.kDeadband ? velGiro : 0.0;
 
          // 3. Make the driving smoother
         velForward = xLimiter.calculate(velForward) * 3;
         velStrafe = yLimiter.calculate(velStrafe) * 3;
         velGiro = giroLimiter.calculate(velGiro) * 5;
 
         // 4. Construct desired chassis speeds
         ChassisSpeeds chassisSpeeds;
         
              //Relative to robot
             chassisSpeeds = new ChassisSpeeds(velForward, velStrafe, velGiro);

 
         // 5. Convert chassis speeds to individual module states
         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
         // 6. Output each module states to wheels
         swerve.setModuleStates(moduleStates);
         
        } 
     
 
     @Override
     public void end(boolean interrupted) {
         swerve.stopModules();
     }
 
     @Override
     public boolean isFinished() {
         return false;
     }
    
    }  
