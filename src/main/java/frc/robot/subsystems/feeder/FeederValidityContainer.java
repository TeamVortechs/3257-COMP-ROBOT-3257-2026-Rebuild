package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FeederConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class FeederValidityContainer {

  private BooleanSupplier shooterOnVelocity;
  private BooleanSupplier driveIsValid;
  private BooleanSupplier canScoreMatchTimeline;
  private BooleanSupplier manualOverride;
  private BooleanSupplier isInScoringZone;
  // I was advised not to add this by jason but john said we should. For now I'll add it so we don't
  // have to add it later. We can just supply true in the constructor
  private Timer debounceTimer;

  private final Notifier validityLogger;

  private boolean isValid = false;

  public FeederValidityContainer(
      BooleanSupplier driveIsValid,
      BooleanSupplier shooterOnVelocity,
      BooleanSupplier canScoreMatchTimeline,
      BooleanSupplier manualOverride,
      BooleanSupplier isInScoringZone) {

    this.shooterOnVelocity = shooterOnVelocity;
    this.driveIsValid = driveIsValid;
    this.canScoreMatchTimeline = canScoreMatchTimeline;
    this.manualOverride = manualOverride;
    this.isInScoringZone = isInScoringZone;
    debounceTimer = new Timer();

    validityLogger =
        new Notifier(
            () -> {
              calculateValidityToFeed();
            });

    validityLogger.startPeriodic(1 / FeederConstants.VALIDITY_LOGGING_FREQUENCY_HERTZ);
  }

  private void calculateValidityToFeed() {
    // this stuff should wait on debounce, probably not shooter on speed though
    if (!driveIsValid.getAsBoolean()) {
      debounceTimer.restart();
    }

    // logging part:

    isValid =
        debounceTimer.hasElapsed(FeederConstants.VALIDITY_DEBOUNCE_TIME_SEC)
            && shooterOnVelocity.getAsBoolean()
            && (canScoreMatchTimeline.getAsBoolean()
                || manualOverride.getAsBoolean()
                || !isInScoringZone.getAsBoolean());

    Logger.recordOutput("FeederValidity/IsValidToFeed", isValid);
    Logger.recordOutput("FeederValidity/DebounceTime", debounceTimer.get());
  }

  public boolean isValid() {
    calculateValidityToFeed();
    return isValid;
  }

  public boolean shooterIsRevved() {
    return shooterOnVelocity.getAsBoolean();
  }
}
