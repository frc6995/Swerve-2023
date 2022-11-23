package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.apriltag.AprilTagFieldLayout;
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

    private final CommandXboxController gamepad = new CommandXboxController(InputDevices.GAMEPAD_PORT);
    @Log
    private final DrivebaseS drivebaseS = new DrivebaseS();

    @Log
    private final Field2d field = new Field2d();
    @Log
    private final Field3d field3d = new Field3d();
    private final FieldObject2d target = field.getObject("target");
    
    @Log
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    PathPlannerTrajectory pathPlannerTrajectory;

    public RobotContainer() {
        target.setPose(new Pose2d(4, 4, new Rotation2d()));
        
        
        drivebaseS.setDefaultCommand(
            new OperatorControlC(
                gamepad::getLeftY,
                gamepad::getLeftX,
                gamepad::getRightX,
                drivebaseS
            )
        );

        configureButtonBindings();
        autoSelector.setDefaultOption("pathPlanner", new InstantCommand());
    }

    public void configureButtonBindings() {
        new Trigger(RobotController::getUserButton).onTrue(runOnce(()->drivebaseS.resetPose(new Pose2d())));
        gamepad.povCenter().onFalse(
            runOnce(
                ()->drivebaseS.setRotationState(
                    Units.degreesToRadians(gamepad.getHID().getPOV()))
            )
        );
        gamepad.a().onTrue(drivebaseS.chasePoseC(target::getPose).until(gamepad.a().rising()));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void periodic() {
        drivebaseS.drawRobotOnField(field);
        field3d.setRobotPose(new Pose3d(drivebaseS.getPose()));
    }

    public void onEnabled(){
        drivebaseS.resetRelativeRotationEncoders();
    }
}
