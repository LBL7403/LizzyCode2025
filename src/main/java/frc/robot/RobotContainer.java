// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Swerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {



    // Aquí creamos un objeto selectorAuto que se encargará de elegir el comando autónomo.
    // Ten en cuenta que, si se añaden varias opciones y no se especifica una predeterminada,
    // se seleccionará una de forma aleatoria.
    private final SendableChooser<Command> selectorAuto = AutoBuilder.buildAutoChooser();

    // Aquí estamos creando un nuevo camino para que el selectorAuto pueda seleccionarlo.
    // Ten en cuenta que el parámetro autoName debe coincidir exactamente con el nombre definido en la GUI.
    Command autoCaminoEjemplo = new PathPlannerAuto("Camino Ejemplo");









    private double MaxSpeed = Constants.Swerve.speedP*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private PhotonCamera camera = new PhotonCamera("photonvision");
    private boolean hasTargets =false;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private static TunerConstants a = new TunerConstants();
    private final Telemetry logger = new Telemetry();
    private final PS5Controller joystick = new PS5Controller(0);
    //private final CommandXboxController joystick = new CommandXboxController(0);
    private final Elevator elev = new Elevator();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public RobotContainer() {
    // Se puede añadir una opción usando el método addOption (esto se aplica a un objeto SendableChooser).
    // Esta opción se agrega como alternativa, pero si no se especifica una opción por defecto,
    // se seleccionará una de forma aleatoria.
    selectorAuto.addOption("Camino Ejemplo", autoCaminoEjemplo);

    // También se puede establecer una opción predeterminada con setDefaultOption,
    // garantizando que se seleccione dicha opción si no se elige otra.
    configureBindings();

    }



    public void aprilTag()
    {
        
         var result = camera.getLatestResult();
         
        if(result.hasTargets())  {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget target = result.getBestTarget();
           

        }
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with positive Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with positive X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        JoystickButton fieldResetButton = new JoystickButton(joystick, PS5Controller.Button.kCircle.value);
        Command fieldResetCommand = new InstantCommand(() -> fieldReset());
        fieldResetButton.onTrue(fieldResetCommand);
        aprilTag();////////////////////////needs to be elswhere

        JoystickButton raiseElevatorButton = new JoystickButton(joystick, PS5Controller.Button.kTriangle.value);
        Command raiseECommand = new InstantCommand(() -> elev.setElevatorEncoder(100));
        raiseElevatorButton.onTrue(raiseECommand);

        JoystickButton moveElevator = new JoystickButton(joystick, PS5Controller.Button.kCross.value);
        Command moveECommand = new InstantCommand(() -> elev.moveElevator());
        moveElevator.whileTrue(moveECommand);
        

        JoystickButton pointToDirection = new JoystickButton(joystick, PS5Controller.Button.kSquare.value);
    pointToDirection.whileTrue(drivetrain.applyRequest(() ->
    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
));

        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        /* 
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.povUp().onTrue(drivetrain.runOnce(()-> drivetrain.seedFieldCentric()));
        
        
    //        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    */
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Selecciona la opción predeterminada (o una al azar si no se especificó una) y retorna el comando autónomo seleccionado.
        return selectorAuto.getSelected();
    }  

    public void fieldReset(){ 
        drivetrain.seedFieldCentric();
    }
}