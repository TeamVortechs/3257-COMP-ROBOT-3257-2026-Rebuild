package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.communication.*;
import org.littletonrobotics.junction.Logger;


public class MatchTimeline {
  private MatchPhase currentPhase = MatchPhase.NO_PHASE;

  private Notifier notifer;
  private Notifier logger;

  private MatchChangeCallback matchChangeCallback;

  private Timer timer;

  private CommandXboxController controller;

  public MatchTimeline(CommandXboxController commandXboxController) {
    this.controller = commandXboxController;
  }

  {
    notifer =
        new Notifier(
            () -> {
              advancePhase();
            });

    logger =
        new Notifier(
            () -> {
              logOutputs();
            });

    logger.startPeriodic(1 / 4.0);

    Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());

    matchChangeCallback = () -> {};

    timer = new Timer();
  }

  private void logOutputs() {
    Logger.recordOutput("MatchTimeline/timeUntilNextPhase", timeUntilNextPhase());
    Logger.recordOutput("MatchTimeline/isWinningAuto", getIsWinningAuto());
    Logger.recordOutput("MatchTimeline/canScore", canScore());
  }

  public void start() {
    currentPhase = MatchPhase.BEGINNING;
    advancePhase();

    timer.restart();
  }

  private void vibrateController() {
    new ControllerVibrateCommand(10, controller).withTimeout(1.0);
  }

  private void advancePhase() {
    currentPhase = currentPhase.getNextPhase();
    notifer.startSingle(currentPhase.getTime());
    Logger.recordOutput("MatchTimeline/currentPhase", currentPhase.getDisplayName());
    matchChangeCallback.run();
    if (currentPhase == MatchPhase.ALMOST_SHIFT_2
        || currentPhase == MatchPhase.ALMOST_SHIFT_3
        || currentPhase == MatchPhase.ALMOST_SHIFT_4
        || currentPhase == MatchPhase.ALMOST_ENDGAME) {
      vibrateController();
    }
  }


  public void setMatchChangeCallBack(MatchChangeCallback matchChangeCallback) {
    this.matchChangeCallback = matchChangeCallback;
  }

  public void setController(CommandXboxController controller) {
    this.controller = controller;
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

  public boolean getIsWinningAuto() {
    return this.isWinningAuto;
  }

  public boolean canScore() {
    MatchPhase matchPhase = getCurrentPhase();

    if (matchPhase.getScoreType() == ScoreType.ALL_SCORE) return true;

    if (matchPhase.getScoreType() == ScoreType.WINNING_SCORE && isWinningAuto) return true;

    if (matchPhase.getScoreType() == ScoreType.LOSING_SCORE && !isWinningAuto) return true;

    return false;
  }

  double timeSinceStart;
  double matchTimes[] = {20, 30, 55, 80, 105, 130, 160};

  public double timeUntilNextPhase() {
    timeSinceStart = getTimeSinceStart();
    for (double i : matchTimes) {
      if (timeSinceStart > i) {
        continue;
      } else {
        return Math.round(Math.abs(timeSinceStart - i));
      }
    }
    return 0;
  }

  interface MatchChangeCallback {
    void run();
  }
}
