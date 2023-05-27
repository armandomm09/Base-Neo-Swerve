package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class subsistemaIntake extends SubsystemBase{

    public final CANSparkMax motorInternoShooterIzq = new CANSparkMax(5, MotorType.kBrushless);
    public final CANSparkMax motorInternoShooterDer = new CANSparkMax(6, MotorType.kBrushless);
    public final WPI_TalonSRX motorExternoShooterDer = new WPI_TalonSRX(9);
    public final WPI_TalonSRX motorExternoShooterIzq = new WPI_TalonSRX(10);
    public final MotorControllerGroup motoresExternosShooter = new MotorControllerGroup(motorExternoShooterDer, motorExternoShooterIzq);
    
    public boolean tiempo_activo = false;
     Timer tiempo = new Timer();

    
    public subsistemaIntake(){       
    }

    public void dispararArriba(int nivelDisparo){
        if(nivelDisparo < 2 && nivelDisparo > 0){
            tiempo.start();
            tirarArriba();
            tiempo.stop();
            tiempo.reset();
            nivelDisparo = 0;
        } else if(nivelDisparo < 3 && nivelDisparo > 1){
            tiempo.start();
            tirarAbajo();
            tiempo.stop();
            tiempo.reset();
            nivelDisparo = 0;
        }
    }
    
    public void tirarArriba(){
        
        if(tiempo.get() > 0 && tiempo.get() < 0.075 && tiempo_activo == true)
      {
        motoresExternosShooter.set(-0.6);
        motorInternoShooterDer.set(0);
      } else if(tiempo.get() > 0.075 && tiempo_activo == true){
        motoresExternosShooter.set(-0.6);
        motorInternoShooterDer.set(-0.25);
      }else if (tiempo_activo == false){
        motoresExternosShooter.set(0);
        motorInternoShooterDer.set(0);
      }
    }

    public void tirarAbajo(){
        if(tiempo.get() > 0 && tiempo.get() < 0.075 && tiempo_activo == true)
      {
        motoresExternosShooter.set(-0.5);
        motorInternoShooterDer.set(0);
      } else if(tiempo.get() > 0.075 && tiempo_activo == true){
        motoresExternosShooter.set(-0.5);
        motorInternoShooterDer.set(-0.25);
      }else if (tiempo_activo == false){
        motoresExternosShooter.set(0);
        motorInternoShooterDer.set(0);
      }
    }
}
