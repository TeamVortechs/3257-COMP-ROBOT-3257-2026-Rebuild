package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;

public class MatchTimeline {
  private MatchPhase currentPhase = MatchPhase.NO_PHASE;

  private Notifier notifer;

  {
    notifer =
        new Notifier(
            () -> {
              advancePhase();
            });

    Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());
  }

  public void start() {
    currentPhase = MatchPhase.BEGINNING;
    advancePhase();
  }

  private void advancePhase() {
    currentPhase = currentPhase.getNextPhase();
    notifer.startSingle(currentPhase.getTime());
    Logger.recordOutput("MatchTimeline/currentPhase", currentPhase.getDisplayName());
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
}
