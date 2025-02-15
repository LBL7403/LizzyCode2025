package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SistemaIntake extends Command implements Subsystem {

    private SparkMax motorIntake;
    private final DoubleSolenoid solenoideDoble;
    private final DoubleSolenoid solenoideDoble2;

    public SistemaIntake() {
        // Creamos un doble solenoide que utiliza un PCM (Pneumatics Control Module) de CTRE,
        // el cual está conectado al CAN Bus con ID 4.  
        // - El canal 0 del solenoide se usa para activar el flujo de aire en una dirección (expulsión de aire).  
        // - El canal 1 se usa para invertir el flujo de aire (entrada o dirección opuesta).  
        // Esto permite controlar un actuador neumático en dos posiciones utilizando el módulo neumático en la red CAN.
        motorIntake = new SparkMax(6, MotorType.kBrushless); 
        // Reemplazar 6 por el CanBus ID del motor del intake

        solenoideDoble = new DoubleSolenoid(4, PneumaticsModuleType.CTREPCM, 0, 1);
        solenoideDoble2 = new DoubleSolenoid(4, PneumaticsModuleType.CTREPCM, 0, 1);
        // Para cambiar el CAN Bus ID se debe ajustar el parámetro del módulo o removerlo para utilizar los canales del CAN Bus ID por defecto.
    }

    @Override
    public void initialize() {
        System.out.println("¡Comando inicializado!");
    }

    public void establecerVelocidadMotorIntake(Double velocidad) {
        motorIntake.set(velocidad);
    }
    
    public void cerrarSolenoides() {
        solenoideDoble.set(DoubleSolenoid.Value.kForward);
        solenoideDoble2.set(DoubleSolenoid.Value.kForward);
    }

    public void abrirSolenoides() {
        solenoideDoble.set(DoubleSolenoid.Value.kReverse);
        solenoideDoble2.set(DoubleSolenoid.Value.kReverse);
    }
}
