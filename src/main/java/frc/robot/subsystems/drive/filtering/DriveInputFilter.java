package frc.robot.subsystems.drive.filtering;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveInputFilter {

  private DriveInputFilter nextInputFilter;

  protected abstract ChassisSpeeds runFilter(ChassisSpeeds speeds);

  /**
   * takes the given speeds and returns the filtered version the specific implementation will start
   * the next filter as well if needed
   *
   * @return
   */
  public ChassisSpeeds calculate(ChassisSpeeds speeds) {
    ChassisSpeeds newSpeeds = runFilter(speeds);

    // if no more filters just return new speeds
    if (nextInputFilter == null) return newSpeeds;

    return nextInputFilter.calculate(newSpeeds);
  }

  public DriveInputFilter withNextFilter(DriveInputFilter driveInputFilter) {
    if (nextInputFilter == null) {
      nextInputFilter = driveInputFilter;
    } else {
      nextInputFilter = nextInputFilter.withNextFilter(driveInputFilter);
    }

    return this;
  }
}
