package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;

public class MatchTimeline {
  private MatchPhase currentPhase = MatchPhase.NO_PHASE;

  private Notifier notifer;

  private MatchChangeCallback matchChangeCallback;

  {
    notifer =
        new Notifier(
            () -> {
              advancePhase();
            });

    Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());

    matchChangeCallback = () -> {};
  }

  public void start() {
    currentPhase = MatchPhase.BEGINNING;
    advancePhase();
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

  public boolean canScore(boolean isWinning) {

    MatchPhase matchPhase = getCurrentPhase();

    if (matchPhase.getScoreType() == ScoreType.ALL_SCORE) return true;

    if (matchPhase.getScoreType() == ScoreType.WINNING_SCORE && isWinning) return true;

    if (matchPhase.getScoreType() == ScoreType.LOSING_SCORE && !isWinning) return true;

    return false;
  }

  interface MatchChangeCallback {
    void run();
  }
}
