/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.limelightOffsets;

public final class Constants {

    public static final class ModuleConstants {

        //Diameter of the Wheel in meters
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
       
        //Gear ratio of the drive motor
        public static final double kDriveMotorGearRatio = 1 / 7.13;
        
        //Gear ratio of the turning motor
        public static final double kTurningMotorGearRatio = 1 /  13.71;
        
        // Drive position conversion factor from rotation to meters
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
       
        // Turning position conversion factor from rotation to radias
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
       
        // Drive velocity conversion factor from RPM to M/S
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
       
        // Turning velocity conversion factor from RPM to Rads/Sec
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
       
        //P constant for the turn motor
        public static final double kPTurning = 0.247;
    }

    public static final class DriveConstants {

         /**
         * 
         *             Trackwidth
         *     --------------------------
         *     |                        |
         *     |                        |
         *     |                        | |
         *     |                        | |WHEELBASE
         *     |        midpoint        | | 
         *     |                        |
         *     |                        |
         *     |                        |
         *     |                        |
         *     --------------------------
         */


        

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.25);
        
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(19.25);
       
        public static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        
        /**
        * Create the kinematics of the swerve
        */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxDriveVEL = 6.58;
        public static final double kMaxRotVEL = 3 * 2 * Math.PI;

        public static final double kDriveLimiter = kMaxDriveVEL / 5.5;
        public static final double kRotationLimiter = //
                kMaxRotVEL / 5;
        public static final double kDriveAccelerationLimiter = 5;
        public static final double kRotationAccelerationLimiter = 7.5;
    }

    public static class MODS {
  

        public static final class frontLeftModule {
                  
            public static int driveMotorID = 1;
            public static int turningMotorID = 2;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false;
                       
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
                  
         }
                  
        public static final class frontRightModule {
                  
            public static int driveMotorID = 3;
            public static int turningMotorID = 4;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false;
                      
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
                  
        public static final class rearLeftModule {
                  
            public static int driveMotorID = 5;
            public static int turningMotorID = 6;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static int absoluteEncoderID = 3;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false;
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
                  
        public static final class rearRightModule {
                  
            public static int driveMotorID = 7;
            public static int turningMotorID = 8;
            public static boolean driveMotorInverted = true;
            public static boolean turningMotorInverted = true;
            public static int absoluteEncoderID =  4;
            public static double absoluteEncoderOffsetRad = 0;
            public static boolean absoluteEncoderReversed = false; 
                  
            public static final SwerveModuleConstants constantes = 
            new SwerveModuleConstants(driveMotorID, turningMotorID, driveMotorInverted, 
            turningMotorInverted, absoluteEncoderOffsetRad, absoluteEncoderReversed);
            
                  
        }
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxDriveVEL / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kMaxRotVEL / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        
        
       
    }

    public static final class limelightConstants {


    /**
     * PID constants for the autoalign
     */
     public static final double kPdrive = 0.1;
     public static final double kIdrive = 0;
     public static final double kDdrive = 0;

     public static final double kPstrafe = 0.08;
     public static final double kIstrafe = 0;
     public static final double kDstrafe = 0;

     public static final double kProtation = 0.04;
     public static final double kIrotation = 0;
     public static final double kDrotation = 0;

        public static final class aprilTag{

            public static double driveOffset = 5.4;
            public static double strafeOffset = -1;
            public static double rotationOffset = 17;

            public static final limelightOffsets offsets =  
        new limelightOffsets(driveOffset, strafeOffset, rotationOffset);

        }

        public static final class reflectiveTape{

            public static double driveOffset = 0.08;
            public static double strafeOffset = -5.16;
            public static double rotationOffset = 17;

            public static final limelightOffsets offsets =  
        new limelightOffsets(driveOffset, strafeOffset, rotationOffset);

        }

        
    }

   

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kPlacerControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class Shuffleboard {

        public static final ShuffleboardTab kShuffleboardTab = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Imperator");
    
        }

}
