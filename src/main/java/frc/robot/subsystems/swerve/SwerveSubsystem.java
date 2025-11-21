package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.lang.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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

    private void configureAutoBuilder() {
        final boolean enableFeedForward = true;


        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> {
                if (enableFeedForward) {
                    this.swerveDrive.drive(speeds,
                    swerveDrive.kinematics.toSwerveModuleStates(speeds),
                    feedforwards.linearForces());
                }
                else {
                    this.swerveDrive.setChassisSpeeds(speeds);
                }
            },
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
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

    /**
     * Change the position, relative to the robot, by a given translation at a given speed. All units in meters.
     * @param translation The distance on each axis to translate by, in meters
     * @param speed The speed at which to move, in meters. If this is greater than {@link frc.robot.Constants#MAX_SPEED} it will be clamped
     * @return 
     */
    public Command changePosition(Translation2d translation, double speed) {
        final double limitedSpeed = Math.min(speed, Constants.MAX_SPEED);
        return run(() -> drive(translation.times(limitedSpeed/translation.getNorm()), 0.0, false))
        .withTimeout(translation.getNorm()/limitedSpeed);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }
}
