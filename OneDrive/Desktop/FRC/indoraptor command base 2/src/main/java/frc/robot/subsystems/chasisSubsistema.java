// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constantesChasis;

public class chasisSubsistema extends SubsystemBase {

  private final CANSparkMax motorIzqAdelante = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax motorIzqAtras = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax motorDerAdelante = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax motorDerAtras = new CANSparkMax(2, MotorType.kBrushless);

  private final MotorControllerGroup motoresIzq  = new MotorControllerGroup(motorIzqAdelante, motorIzqAtras);
  private final MotorControllerGroup motoresDer = new MotorControllerGroup(motorDerAdelante, motorDerAtras);

  private final DifferentialDrive chasis = new DifferentialDrive(motoresIzq, motoresDer);
  
  private final RelativeEncoder encoderIzqCM = motorIzqAdelante.getEncoder();
  private final RelativeEncoder encoderDerCM = motorDerAdelante.getEncoder();

  public double posicionEncoderDer(){
    return encoderDerCM.getPosition();
  }

  public double posicionEncoderIzq() {
    return encoderIzqCM.getPosition();
  }
  public double distanciaRecorridaMetros() {
    return ((posicionEncoderDer() + posicionEncoderIzq()) / 2) * (Math.PI * 0.1524 / 1.4) / 10; 
  }




  public chasisSubsistema() {

    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);
    motorDerAdelante.setIdleMode(IdleMode.kCoast);

    motoresDer.setInverted(false);
    motoresIzq.setInverted(!motoresDer.getInverted());

    motorDerAtras.follow(motorDerAdelante);
    motorIzqAtras.follow(motorIzqAdelante);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Distancia Metros", distanciaRecorridaMetros());
    // This method will be called once per scheduler run
  }

  public void darVelocidadMotores(double velocidadIzquierda, double velocidadDerecha) {
    motoresIzq.set(velocidadIzquierda);
    motoresDer.set(velocidadDerecha);
  }

}