// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Prototipos;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFX intakeRoller = new TalonFX(Constants.IntakeConstants.ROLLER_CAN_ID);

  TalonFX leftIntake = new TalonFX(Constants.IntakeConstants.EXTENSIONLEFT_CAN_ID);
  TalonFX rightIntake = new TalonFX(Constants.IntakeConstants.EXTENSIONRIGHT_CAN_ID);

  public IntakeSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leftIntake.getConfigurator().apply(config);

    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rightIntake.getConfigurator().apply(config);
  }

  public void runIntake(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger.getAsDouble(), 0.1);
    double right = MathUtil.applyDeadband(rightTrigger.getAsDouble(), 0.1);
    double speed = (right - left);
    leftIntake.set(speed);
    rightIntake.setControl(leftIntake.getAppliedControl());
  }

  public void runRollers(Boolean isOn) {
    if (isOn) {
      intakeRoller.setVoltage(Constants.IntakeConstants.ROLLER_INTAKE_VOLTS);
    } else {
      intakeRoller.setVoltage(-Constants.IntakeConstants.ROLLER_INTAKE_VOLTS);
    }
  }

  public void stopIntake() {
    leftIntake.stopMotor();
    rightIntake.stopMotor();
  }

  public void stopRollers() {
    intakeRoller.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
