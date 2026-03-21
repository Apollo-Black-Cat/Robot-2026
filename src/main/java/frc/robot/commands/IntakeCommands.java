// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeCommands {
  private IntakeCommands() {}

  public static Command runIntake(
      Intake intake, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    return Commands.run(
        () -> {
          double left = MathUtil.applyDeadband(leftSupplier.getAsDouble(), 0.1);
          double right = MathUtil.applyDeadband(rightSupplier.getAsDouble(), 0.1);
          intake.extensionVoltage(right - left);
        },
        intake);
  }

  public static Command stopMotors(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopMotors();
        },
        intake);
  }

  public static Command runRollers(Intake intake, Boolean isOn) {
    return Commands.runEnd(
        () -> {
          intake.rollerVoltage(isOn);
          ;
        },
        () -> {
          intake.stopRoller();
        },
        intake);
  }

  public static Command setDistance(Intake intake, double distance) {
    return new RunCommand(
            () -> {
              intake.setDistance(distance);
            },
            intake)
        .until(() -> Math.abs(intake.getExtensionPositionMeters() - distance) <= 0.05)
        .andThen(
            () -> {
              intake.stopMotors();
            });
  }
}
