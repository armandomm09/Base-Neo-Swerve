// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.constantesChasis;

public class chasisSubsistema extends SubsystemBase {

  public static Supplier<Pose2d> getPose;
  private final CANSparkMax motorDerAdelante = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax motorDerAtras = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax motorIzqAdelante = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax motorIzqAtras = new CANSparkMax(4, MotorType.kBrushless);
  

  private final MotorControllerGroup motoresIzq  = new MotorControllerGroup(motorIzqAdelante, motorIzqAtras);
  private final MotorControllerGroup motoresDer = new MotorControllerGroup(motorDerAdelante, motorDerAtras);

  private final DifferentialDrive chasis = new DifferentialDrive(motoresIzq, motoresDer);
  
  private final RelativeEncoder encoderIzqCM = motorIzqAdelante.getEncoder();
  private final RelativeEncoder encoderDerCM = motorDerAdelante.getEncoder();

  private final Field2d field = new Field2d();

  private final PIDController pidcontr = new PIDController(0.8, 0 , 0);

  //public final static Gyro navx = new AHRS(SPI.Port.kMXP);
  public final static Gyro navx = new AHRS(SPI.Port.kMXP);

   
   
  private final DifferentialDriveOdometry m_Odometry;

  private static chasisSubsistema instance; 
  



  public chasisSubsistema() {

    //SmartDashboard.putData("Field", field);

    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);

    motoresDer.setInverted(false);
    motoresIzq.setInverted(!motoresDer.getInverted());

    motorDerAtras.follow(motorDerAdelante);
    motorIzqAtras.follow(motorIzqAdelante);


    encoderDerCM.setPositionConversionFactor(constantesChasis.factorDeConversionAMetros);
    encoderIzqCM.setPositionConversionFactor(constantesChasis.factorDeConversionAMetros);
    
    encoderDerCM.setVelocityConversionFactor(constantesChasis.factorDeConversionAMetros/60);
    encoderIzqCM.setVelocityConversionFactor(constantesChasis.factorDeConversionAMetros/60);

    navx.reset();
    navx.calibrate();
    resetEncoders();
    m_Odometry = new DifferentialDriveOdometry(navx.getRotation2d(), encoderIzqCM.getPosition(), encoderDerCM.getPosition());

    m_Odometry.resetPosition(navx.getRotation2d(), encoderIzqCM.getPosition(), encoderDerCM.getPosition(), new Pose2d());
  }

  //ENCOODERS******************************************************************************************
 
  

 
  public double velocidadIzquierda(){
    return encoderIzqCM.getVelocity();
  }

  public double velocidadDerecha(){
    return encoderDerCM.getVelocity();
  }
  
 

  public void resetEncoders(){
    encoderDerCM.setPosition(0);
    encoderIzqCM.setPosition(0);
  }

  public RelativeEncoder getEncoderIzq(){
    return encoderIzqCM;
  }

  public RelativeEncoder getEncoderDer(){
    return encoderDerCM;
  }

 public double promedioDistancia(){
  return (encoderDerCM.getPosition() + encoderIzqCM.getPosition())/2;
 }

  //NAVX******************************************************************************************
  public double getRotacionNavX(){
    return navx.getRotation2d().getDegrees();
  }

  public double getVelocidadGiro(){
    return -navx.getRate();
  }

  public void zeroHeading(){
    navx.calibrate();
    navx.reset();
  }

  public Gyro getGyro(){
    return getGyro();
  }
  
  


  //CHASIS******************************************************************************************
  public void manejar(boolean arcade){
    if(arcade){
      chasis.arcadeDrive(0.7* RobotContainer.controlDriver.getRawAxis(1), (0.7*RobotContainer.controlDriver.getRawAxis(4)));
    }
    else if(arcade == false){
      chasis.tankDrive(0.5* RobotContainer.controlDriver.getRawAxis(1), 0.5*RobotContainer.controlDriver.getRawAxis(5));
    }
  }

  public void setMaxOutput(double maxOutput){
    chasis.setMaxOutput(maxOutput);
  }

  public static chasisSubsistema getInstance(){
    if (instance == null){
      instance = new chasisSubsistema();
    }
    return instance;
  }

  //ODOMETRIA******************************************************************************************
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }

  public void resetOdometria(Pose2d pose){
    resetEncoders();
    navx.reset();
    m_Odometry.resetPosition(navx.getRotation2d(), encoderIzqCM.getPosition(), encoderDerCM.getPosition(), pose);
  }

  

  public void chasisVoltje(double voltajeIzq, double voltajeDer){
    motoresIzq.setVoltage(voltajeIzq);
    motoresDer.setVoltage(voltajeDer);
    chasis.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(encoderIzqCM.getVelocity(), encoderDerCM.getVelocity());
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Distancia Metros Izquierda", encoderIzqCM.getPosition());
    SmartDashboard.putNumber("Distancia Metros Derecha", encoderDerCM.getPosition());
    SmartDashboard.putNumber("promedio distacancia", promedioDistancia());
    SmartDashboard.putNumber("grados giro", getRotacionNavX());
    m_Odometry.update(navx.getRotation2d(), encoderIzqCM.getPosition(), encoderDerCM.getPosition());
    //field.setRobotPose(m_Odometry.getPoseMeters());
    //SmartDashboard.putData("robot", getPose());
  }

  /*public void elegirManejo(boolean modo){
    if(modo == true){
      arcade = true;
    } else {
      arcade = false;
    }
}*/



  
  
  

}