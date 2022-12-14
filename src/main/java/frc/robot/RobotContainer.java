package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Field3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final CommandXboxController m_driverController = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    @Log
    private final DrivebaseS m_drivebaseS = new DrivebaseS();

    @Log
    private final Field2d m_field = new Field2d();
    @Log
    private final Field3d m_field3d = new Field3d();
    private final FieldObject2d m_target = m_field.getObject("target");
    
    @Log
    SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        m_target.setPose(new Pose2d(0, 0, new Rotation2d()));
        
        
        m_drivebaseS.setDefaultCommand(
            new OperatorControlC(
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX,
                m_drivebaseS
            )
        );

        configureButtonBindings();
        m_autoSelector.setDefaultOption("pathPlanner", new InstantCommand());
    }

    public void configureButtonBindings() {
        new Trigger(RobotController::getUserButton).onTrue(runOnce(()->m_drivebaseS.resetPose(new Pose2d())));
        m_driverController.povCenter().onFalse(
            runOnce(
                ()->m_drivebaseS.setRotationState(
                    Units.degreesToRadians(m_driverController.getHID().getPOV()))
            )
        );
        m_driverController.a().toggleOnTrue(m_drivebaseS.chasePoseC(m_target::getPose));
    }

    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }

    public void periodic() {
        m_drivebaseS.drawRobotOnField(m_field);
        m_field3d.setRobotPose(new Pose3d(m_drivebaseS.getPose()));
    }

    public void onEnabled(){
        m_drivebaseS.resetRelativeRotationEncoders();
    }
}
