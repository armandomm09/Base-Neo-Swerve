/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.lib.util;

public class SwerveModuleConstants {
    
    public int driveMotorID;
    public int turnMotorID;
    public boolean driveMotorInverted;
    public boolean turnMotorInverted;
    public double absoluteEncoderOffsetRad;
    public boolean absoluteEncoderReversed;
  

    public SwerveModuleConstants(
        int driveMotorID,
        int turnMotorID,
        boolean driveMotorInverted,
        boolean turnMotorInverted,
        double absoluteEncoderOffsetRad,
        boolean absoluteEncoderReversed
        
    ){
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.driveMotorInverted = driveMotorInverted;
        this.turnMotorInverted = turnMotorInverted;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        

    }
}
