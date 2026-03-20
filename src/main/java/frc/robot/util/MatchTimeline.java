package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class MatchTimeline {
  private MatchPhase currentPhase = MatchPhase.NO_PHASE;

  private Notifier notifer;
  private Notifier logger;

  private MatchChangeCallback matchChangeCallback;

  private Timer timer;

  private CommandXboxController controller;
  private CommandXboxController secondController;

  private Optional<Alliance> teamThatWonAuto = Optional.empty();

  public MatchTimeline(
      CommandXboxController commandXboxController, CommandXboxController secondController) {
    this.controller = commandXboxController;
    this.secondController = secondController;
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

    Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());

    matchChangeCallback = () -> {};

    logger.startPeriodic(1 / 4.0);

    timer = new Timer();
  }

  private void logOutputs() {
    Logger.recordOutput("MatchTimeline/timeUntilNextPhase", timeUntilNextPhase());
    Logger.recordOutput("MatchTimeline/isWinningAuto", hasWonAuto());
    Logger.recordOutput("MatchTimeline/canScore", canScore());
    Logger.recordOutput("MatchTimeline/NEWTIMER", timeUntilNextPhaseUPDATED());
    Logger.recordOutput("MatchTimeline/driverstationtimer", DriverStation.getMatchTime());
  }

  public void start() {
    currentPhase = MatchPhase.BEGINNING;
    advancePhase();

    timer.restart();
  }

  private Command vibrateControllerCommand() {
    return new StartEndCommand(
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 1);
              secondController.getHID().setRumble(RumbleType.kBothRumble, 1);
            },
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 0);
              secondController.getHID().setRumble(RumbleType.kBothRumble, 0);
            })
        .withDeadline(new WaitCommand(1));
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
      CommandScheduler.getInstance().schedule(vibrateControllerCommand());
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

  /**
   * returns wether or not your team won auto
   *
   * @return
   */
  public boolean hasWonAuto() {

    return false;

    // if (teamThatWonAuto.isEmpty()) {
    //   updateAutoWinner();
    // }

    // if (DriverStation.getAlliance() == null || DriverStation.getAlliance().isEmpty()) {
    //   return false;
    // }

    // if (teamThatWonAuto.isPresent()) {
    //   return teamThatWonAuto.get() == DriverStation.getAlliance().get();
    // }

    // return false;
  }

  public void updateAutoWinner() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData == null || gameData.length() < 0) {
      return;
    }

    switch (gameData.charAt(0)) {
      case 'R':
        teamThatWonAuto = Optional.of(Alliance.Red);

        break;

      case 'B':
        teamThatWonAuto = Optional.of(Alliance.Blue);
        break;

      default:
        break;
    }
  }

  public boolean canScore() {
    MatchPhase matchPhase = getCurrentPhase();

    if (matchPhase.getScoreType() == ScoreType.ALL_SCORE) return true;

    if (matchPhase.getScoreType() == ScoreType.WINNING_SCORE && hasWonAuto()) return true;

    if (matchPhase.getScoreType() == ScoreType.LOSING_SCORE && !hasWonAuto()) return true;

    return false;
  }

  double timeSinceStart2;
  double matchTimes2[] = {20, 33, 58, 83, 108, 133, 163};

  public double timeUntilNextPhase() {
    timeSinceStart2 = getTimeSinceStart();
    for (double i : matchTimes2) {
      if (timeSinceStart2 > i) {
        continue;
      } else {
        return Math.round(Math.abs(timeSinceStart2 - i));
      }
    }
    return 0;
  }

double matchTimes[] = {20, 33, 58, 83, 108, 133, 163};

public double timeUntilNextPhaseUPDATED() {
  double matchTime = DriverStation.getMatchTime();
  double timeSinceStart = 0;

  if (DriverStation.isAutonomous()) {
    timeSinceStart = 20 - matchTime;
  } else if (DriverStation.isTeleop()) {
    timeSinceStart = 203 - matchTime;
  } else {
    return 0;
  }

  if (timeSinceStart < 0) {
    timeSinceStart = 0;
  }

  for (double i : matchTimes) {
    if (timeSinceStart > i) {
      continue;
    } else {
      return Math.round(Math.abs(i - timeSinceStart));
    }
  }
  return 0;
}

  interface MatchChangeCallback {
    void run();
  }
}
