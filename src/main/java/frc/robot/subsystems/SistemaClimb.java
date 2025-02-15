package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;


public class SistemaClimb extends Command implements Subsystem {

    private SparkMax motorClimb;
    private SparkMaxConfig configuracionMotorSistemaClimb = new SparkMaxConfig();
    private int PosicionTreparJaulaSuperficie;
    private int PosicionTreparJaulaProfunda;
    private SparkClosedLoopController controlMotorTrepar;



    // La creacion del subsistema toma dos posiciones como parametro, la rotacion necesaria del motor para subirse a la jaula ligera y la rotacion  necesaria para subirse a la jaula profunda
    public SistemaClimb(int PosicionJaulaSuperficie, int PosicionJaulaProfunda) {
        motorClimb = new SparkMax(6, MotorType.kBrushless);
        PosicionTreparJaulaSuperficie = PosicionJaulaSuperficie;
        PosicionTreparJaulaProfunda = PosicionJaulaProfunda;
        controlMotorTrepar = motorClimb.getClosedLoopController();


        configuracionMotorSistemaClimb.closedLoop
        .p(Constants.ElevatorConstants.kPMoving)
        .i(Constants.ElevatorConstants.kIMoving)
        .d(Constants.ElevatorConstants.kDMoving)
        .iZone(Constants.ElevatorConstants.kIzMoving)
        .outputRange( -1 * Constants.ElevatorConstants.kMaxAbsOutput, Constants.ElevatorConstants.kMaxAbsOutput)
        .velocityFF(Constants.ElevatorConstants.kFFMoving);
        // Las configuraciones PID solo son validas si la asumcion de que el motor del climb y del elevator son el mismo

        motorClimb.configure(configuracionMotorSistemaClimb, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Asignar el configure para que el motor utilize los pid values ya que no se estan configurando en el rev hardware client
    }



    // Estos son los metodos para rotar a las posiciones correspondientes y, desde la perspectiva mecanica, subirse a las jaulas
    public void cambiarPosicionParaJaulaSuperficial()
    {
        controlMotorTrepar.setReference(PosicionTreparJaulaSuperficie, SparkBase.ControlType.kPosition);
    }

    public void cambiarPosicionParaJaulaProfunda()
    {
        controlMotorTrepar.setReference(PosicionTreparJaulaProfunda, SparkBase.ControlType.kPosition);
    }




    public void ajustarPid(double kP, double kI, double kD, double kIz, double kFF){
        /*pidLiftController.setP(kP);
        pidLiftController.setI(kI);
        pidLiftController.setD(kD);
        pidLiftController.setIZone(kIz);
        pidLiftController.setFF(kFF);*/
        configuracionMotorSistemaClimb.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .iZone(kIz)
        .velocityFF(kFF);
        motorClimb.configure(configuracionMotorSistemaClimb, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // se vuelve a configurar el motorClimb en caso de que el configure no guarde la configuracion actualizada con los nuevos valores PID
      }


      public void reiniciarPID(){

        configuracionMotorSistemaClimb.closedLoop
        .p(Constants.ElevatorConstants.kPMoving)
        .i(Constants.ElevatorConstants.kIMoving)
        .d(Constants.ElevatorConstants.kDMoving)
        .iZone(Constants.ElevatorConstants.kIzMoving)
        .outputRange( -1 * Constants.ElevatorConstants.kMaxAbsOutput, Constants.ElevatorConstants.kMaxAbsOutput)
        .velocityFF(Constants.ElevatorConstants.kFFMoving);
        // Las configuraciones PID solo son validas si la asumcion de que el motor del climb y del elevator son el mismo

        motorClimb.configure(configuracionMotorSistemaClimb, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Se Aplica el configure por la misma razon por la cual se aplica el configure en el metodo "ajustar Pid"

      }



    



}
