package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class subsistemaMuneca extends SubsystemBase{

    private final CANSparkMax motorCajaShooterAdelante = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax motorCajaShooterAtras = new CANSparkMax(7, MotorType.kBrushless);
    private final MotorControllerGroup motoresCajaShooter = new MotorControllerGroup(motorCajaShooterAdelante, motorCajaShooterAtras);
    
    private final RelativeEncoder encoderMunecaAdelante = motorCajaShooterAdelante.getEncoder();
    private final RelativeEncoder encoderMunecaAtras = motorCajaShooterAtras.getEncoder();

    public double valorEncoderMunecaAdelante(){
        return encoderMunecaAdelante.getPosition();
    }

    public double valorEncoderMunecaAtras(){
        return encoderMunecaAtras.getPosition();
    }

    public double posicionMuneca(){
        return (valorEncoderMunecaAdelante() + valorEncoderMunecaAtras()) / 2;
    }

  public subsistemaMuneca(){

    }
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Posicion Muñeca", posicionMuneca());
      // This method will be called once per scheduler run
    }

  public void darVelocidadMuneca(double velocidadMotoresMuneca){
    motoresCajaShooter.set(velocidadMotoresMuneca);
  }
}
