package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {

    public final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory) {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(), scaledInputs.getY(),
                headingX.getAsDouble(), headingY.getAsDouble(),
                swerveDrive.getOdometryHeading().getRadians(),
                swerveDrive.getMaximumChassisVelocity()
            ));

        });
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                                DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                              angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(), true, false);
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOrientedCommand(ChassisSpeeds velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity));
    }

    public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }
}
