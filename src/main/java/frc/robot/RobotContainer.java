// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.PS5Buttons;
import frc.robot.Subsystems.*;

public class RobotContainer {
    // Vision subsystems
    private final Vision vision;
    private final VisionDataHandler visionDataHandler;
    private final CameraProcessor cameraProcessor;
    
    // Other subsystems
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    
    // Controllers
    private final PS5Controller driverController;
    private final PS5Controller operatorController;
    
    public RobotContainer() {
        // Initialize subsystems
        vision = new Vision();
        visionDataHandler = new VisionDataHandler(vision);
        cameraProcessor = new CameraProcessor(vision);
        
        driveSubsystem = new DriveSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        
        // Initialize controllers
        driverController = new PS5Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
        operatorController = new PS5Controller(ControllerConstants.OPERATOR_CONTROLLER_PORT);
        
        configureButtonBindings();
        configureDefaultCommands();
    }
    
    private void configureButtonBindings() {
        // Driver controls
        
        // Align to coral while holding Cross (X) button
        new JoystickButton(driverController, PS5Buttons.CROSS)
            .whileTrue(new AlignToCoralCommand(vision, driveSubsystem));
            
        // Reset vision stats with PS button
        new JoystickButton(driverController, PS5Buttons.PS)
            .onTrue(Commands.runOnce(() -> {
                vision.resetStats();
                visionDataHandler.resetStats();
            }));
            
        // Operator controls
        
        // Example: Toggle intake with R1
        new JoystickButton(operatorController, PS5Buttons.R1)
            .toggleOnTrue(Commands.startEnd(
                () -> intakeSubsystem.setIntakeSpeed(0.8),
                () -> intakeSubsystem.setIntakeSpeed(0.0)
            ));
    }
    
    private void configureDefaultCommands() {
        // Configure drive command with PS5 controller inputs
        driveSubsystem.setDefaultCommand(
            new DriveCommand(
                driveSubsystem,
                () -> -applyDeadband(driverController.getLeftY()),  // Forward/backward
                () -> -applyDeadband(driverController.getRightX())  // Rotation
            )
        );
    }
    
    private static double applyDeadband(double value) {
        if (Math.abs(value) < ControllerConstants.STICK_DEADBAND) {
            return 0.0;
        }
        return value;
    }
    
    public Command getAutonomousCommand() {
        return Commands.none();
    }
    
    public void disabledInit() {
        cameraProcessor.stop();
    }
    
    // Telemetry methods for controller status
    public void updateControllerStatus() {
        // Log controller connection status
        SmartDashboard.putBoolean("Controller/Driver Connected", driverController.isConnected());
        SmartDashboard.putBoolean("Controller/Operator Connected", operatorController.isConnected());
        
        // Log battery levels if supported
        SmartDashboard.putNumber("Controller/Driver Battery", driverController.getBatteryLevel());
        SmartDashboard.putNumber("Controller/Operator Battery", operatorController.getBatteryLevel());
    }
}