package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter,yLimiter,turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
     Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter=new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter=new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter=new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
     }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //getting real time joystick inputs
        double xSpeed=xSpdFunction.get();
        double ySpeed=ySpdFunction.get();
        double turningSpeed=turningSpdFunction.get();
        //apply deadband
        xSpeed=Math.abs(xSpeed)>OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed=Math.abs(ySpeed)>OIConstants.kDeadband ? xSpeed : 0.0;
        turningSpeed=Math.abs(turningSpeed)>OIConstants.kDeadband ? turningSpeed : 0.0;
        //make the driving smoother (limit accel)
        xSpeed=xLimiter.calculate(xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed=yLimiter.calculate(ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed=turningLimiter.calculate(turningSpeed)*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        //construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            //relative to field
            chassisSpeeds=ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed,swerveSubsystem.getRotation2d());
            
        }else { //relative to robot
            chassisSpeeds=new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);
        }
        //convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates=DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //output each module state to wheel
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
