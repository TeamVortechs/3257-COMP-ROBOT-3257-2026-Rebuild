package frc.robot.subsystems.drive.filtering;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SpeedDriveInputFilter extends DriveInputFilter{

    private double maxSpeed;
    private double maxRotationalSpeed;

    public SpeedDriveInputFilter(double maxSpeed, double maxRotationalSpeed) {
        this.maxSpeed = maxSpeed;
        this.maxRotationalSpeed = maxRotationalSpeed;
    }

    @Override
    protected ChassisSpeeds runFilter(ChassisSpeeds speeds) {

        double x = MathUtil.clamp(speeds.vxMetersPerSecond, -maxSpeed, maxSpeed);
        double y = MathUtil.clamp(speeds.vyMetersPerSecond, -maxSpeed, maxSpeed);

        double theta = MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxRotationalSpeed, maxRotationalSpeed);


        return new ChassisSpeeds(x, y, theta);
    }
    
}
