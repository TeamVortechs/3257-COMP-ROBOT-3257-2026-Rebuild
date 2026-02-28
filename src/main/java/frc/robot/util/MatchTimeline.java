package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MatchTimeline {
  private MatchPhase currentPhase = MatchPhase.NO_PHASE;

  private Notifier notifer;

  private MatchChangeCallback matchChangeCallback;

  private Timer timer;

  {
    notifer =
        new Notifier(
            () -> {
              advancePhase();
            });

    Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());

    matchChangeCallback = () -> {};

    timer = new Timer();
  }

  public void start() {
    currentPhase = MatchPhase.BEGINNING;
    advancePhase();

    timer.restart();
  }

  private void advancePhase() {
    currentPhase = currentPhase.getNextPhase();
    notifer.startSingle(currentPhase.getTime());
    Logger.recordOutput("MatchTimeline/currentPhase", currentPhase.getDisplayName());
    matchChangeCallback.run();
  }

  public void setMatchChangeCallBack(MatchChangeCallback matchChangeCallback) {
    this.matchChangeCallback = matchChangeCallback;
  }

  public MatchPhase getCurrentPhase() {
    return currentPhase;
  }

  public double getTimeSinceStart() {
    return timer.get();
  }

  private boolean isWinningAuto = false;

  public void setIsWinningAuto(boolean isWinning) {
    this.isWinningAuto = isWinning;
  }

  @AutoLogOutput
  public boolean getIsWinningAuto() {
    return this.isWinningAuto;
  }

  @AutoLogOutput
  public boolean canScore() {
    MatchPhase matchPhase = getCurrentPhase();

    if (matchPhase.getScoreType() == ScoreType.ALL_SCORE) return true;

    if (matchPhase.getScoreType() == ScoreType.WINNING_SCORE && isWinningAuto) return true;

    if (matchPhase.getScoreType() == ScoreType.LOSING_SCORE && !isWinningAuto) return true;

    return false;
  }

  double timeSinceStart;
  double timeUntilNextPhase;
  double matchTimes[] = {20, 30, 55, 80, 105, 130, 160};

  @AutoLogOutput
  public double timeUntilNextPhase() {
    timeSinceStart = getTimeSinceStart();
    timeUntilNextPhase = 0;
    for (double i : matchTimes) {
      if (timeSinceStart > i) {
        continue;
      } else {
        return Math.abs(timeSinceStart - i);
      }
    }
    return 0;
    // if (timeSinceStart < 20) {
    //   timeUntilNextPhase = timeSinceStart - 20;
    // } else if (timeSinceStart < 30) {
    //   timeUntilNextPhase = timeSinceStart - 30;
    // } else if (timeSinceStart < 55) {
    //   timeUntilNextPhase = timeSinceStart - 55;
    // } else if (timeSinceStart < 80) {
    //   timeUntilNextPhase = timeSinceStart - 80;
    // } else if (timeSinceStart < 105) {
    //   timeUntilNextPhase = timeSinceStart - 105;
    // } else if (timeSinceStart < 130) {
    //   timeUntilNextPhase = timeSinceStart - 130;
    // } else {
    //   timeUntilNextPhase = timeSinceStart - 160;
    // }
    // return Math.abs(timeUntilNextPhase);
  }

  interface MatchChangeCallback {
    void run();
  }
}
