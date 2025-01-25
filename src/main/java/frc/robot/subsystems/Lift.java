package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Importing the SubsystemBase class from the WPILib library
// This class provides the base for creating subsystems, which are major parts of the robot

// Importing the Constants class from the robot's code
// This class contains constant values used throughout the robot's code
//import frc.robot.Constants;
import frc.robot.Constants;

public class Lift extends SubsystemBase {

  private static boolean isSolenoidUp = false;

  private SparkMax liftMotor;
  private SparkMax liftMotor1;

  private RelativeEncoder motorEncoder;

  private SparkClosedLoopController pidLiftController;

  private RelativeEncoder liftEncoder;
  
  private DoubleSolenoid m_doubleSolenoid;

  private Solenoid m_finalSolenoid;

  private SparkMaxConfig liftConfig = new SparkMaxConfig();
  private SparkMaxConfig config = new SparkMaxConfig();


  public Lift() {
    liftMotor = new SparkMax(Constants.LiftConstants.MOTOR_ID, MotorType.kBrushless);
    liftMotor1 = new SparkMax(Constants.LiftConstants.MOTOR_ID1, MotorType.kBrushless);
    config.follow(liftMotor);
    config.inverted(true);
        config.signals.primaryEncoderPositionPeriodMs(5);
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0);

    liftMotor.configure(config, null, null);
    
  
    SparkFlex m_motor = new SparkFlex(20, MotorType.kBrushless);

    motorEncoder = liftMotor.getEncoder();

    m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    m_finalSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    liftEncoder = liftMotor.getEncoder();

    SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

    pidLiftController = liftMotor.getClosedLoopController();

    m_controller.setReference(Constants.LiftConstants.setPoint, ControlType.kPosition);

   
    //liftConfig.closedLoop.setClosedLoopRampRate(1); If the robot breaks its probbably this
    
    


    liftConfig.closedLoop
    .p(Constants.LiftConstants.kPMoving)
    .i(Constants.LiftConstants.kIMoving)
    .d(Constants.LiftConstants.kDMoving)
    .iZone(Constants.LiftConstants.kIzMoving)
    .outputRange( -1 * Constants.LiftConstants.kMaxAbsOutput, Constants.LiftConstants.kMaxAbsOutput)
    .velocityFF(Constants.LiftConstants.kFFMoving);

    //liftMotor.setClosedLoopRampRate(1);

    //output range
    }

  public void moveFinalSolenoid(){
    m_finalSolenoid.set(true);
  }

  public void retractFinalSolenoid(){
    m_finalSolenoid.set(false);
  }

  public boolean isSolenoidUp() {
      return isSolenoidUp;
  }

  public void setIsSolenoidUp(boolean isSolenoidUp){
    Lift.isSolenoidUp = isSolenoidUp;
  }

  public void setPosition(double position) {
    pidLiftController.setReference(position, ControlType.kPosition);
  }

  public void moveLift(double speed){

    liftMotor.set(speed);
  }

  public double getLiftEncoderPostion(){

    return liftEncoder.getPosition();
  }

  public void resetLiftEncoder() {
    liftEncoder.setPosition(0);
  }

  public void setLiftEncoder(double num) {
    liftEncoder.setPosition(num);
  }

  public void stopLift() {
    liftMotor.set(0);
  }

  public void lockPosition(double kP, double kI, double kD, double kIz, double kFF) {
    setPID(kP, kI, kD, kIz, kFF);
    pidLiftController.setReference(liftEncoder.getPosition(), ControlType.kPosition);
    /////////////
    /// ControlType.kPosition is very badly programed originally was CANSparkbase.Control
    /// 
    /// 
    /// 
    /// 
    /// 
    /// 
    /// 
    /// 
  }

  public void setPID(double kP, double kI, double kD, double kIz, double kFF){
    /*pidLiftController.setP(kP);
    pidLiftController.setI(kI);
    pidLiftController.setD(kD);
    pidLiftController.setIZone(kIz);
    pidLiftController.setFF(kFF);*/
    liftConfig.closedLoop
    .p(kP)
    .i(kI)
    .d(kD)
    .iZone(kIz)
    .velocityFF(kFF);

  }

  public void resetPID(){
    /*pidLiftController.setP(Constants.LiftConstants.kPMoving);
    pidLiftController.setI(Constants.LiftConstants.kIMoving);
    pidLiftController.setD(Constants.LiftConstants.kDMoving);
    pidLiftController.setIZone(Constants.LiftConstants.kIzMoving);
    pidLiftController.setFF(Constants.LiftConstants.kFFMoving);*/

    liftConfig.closedLoop
    .p(Constants.LiftConstants.kPMoving)
    .i(Constants.LiftConstants.kIMoving)
    .d(Constants.LiftConstants.kDMoving)
    .iZone(Constants.LiftConstants.kIzMoving)
    .outputRange( -1 * Constants.LiftConstants.kMaxAbsOutput, Constants.LiftConstants.kMaxAbsOutput)
    .velocityFF(Constants.LiftConstants.kFFMoving);
  }

  public void closeSolenoid() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void openSolenoid() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  

  @Override
  public void periodic() {

   SmartDashboard.putNumber("LIFT ENCODER", liftEncoder.getPosition());
  }

}
