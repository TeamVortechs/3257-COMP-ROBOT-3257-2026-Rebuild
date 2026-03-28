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
import frc.robot.Constants;
import frc.robot.Constants.MatchTimelineConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class MatchTimeline {
  private MatchPhase currentPhase = MatchPhase.NO_PHASE;

  // private Notifier notifer;
  private Notifier logger;

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
    // notifer =
    //     new Notifier(
    //         () -> {
    //           advancePhase();
    //         });

    logger =
        new Notifier(
            () -> {
              logOutputs();
            });

    Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());

    logger.startPeriodic(MatchTimelineConstants.TIMER_FREQUENCY);

    timer = new Timer();
  }

  private void logOutputs() {
    Logger.recordOutput("MatchTimeline/timeUntilNextPhase", timeUntilNextPhase());
    Logger.recordOutput("MatchTimeline/isWinningAuto", hasWonAuto());
    Logger.recordOutput("MatchTimeline/canScore", canScore());
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
    // notifer.startSingle(currentPhase.getTime());
    Logger.recordOutput("MatchTimeline/currentPhase", currentPhase.getDisplayName());
    // matchChangeCallback.run();
    if (currentPhase == MatchPhase.ALMOST_SHIFT_2
        || currentPhase == MatchPhase.ALMOST_SHIFT_3
        || currentPhase == MatchPhase.ALMOST_SHIFT_4
        || currentPhase == MatchPhase.ALMOST_ENDGAME) {
      CommandScheduler.getInstance().schedule(vibrateControllerCommand());
    }
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

  /**
   * With robot pose, determine distance from goal, and thus determine flight time of ball. Going to
   * be used to score more points. This is a helper class so it's private
   *
   * @return flight time in seconds
   */
  private double getFlightTime() {
    // dummy method, implement actual physics math later
    return 0.0;
  }

  /**
   * determine if robot can score based on current phase and position
   *
   * @return
   */
  public boolean canScore() {
    // new code, figures out if it can score based on 3 seconds + air time (shooting early to score
    // on time)
    MatchPhase current = getCurrentPhase();
    double timeToNext = timeUntilNextPhase();
    double flightTime = getFlightTime();

    boolean isCurrentScorable = isPhaseScorable(current);
    MatchPhase nextPhase = current.getNextPhase();
    boolean isNextScorable = (nextPhase != null) && isPhaseScorable(nextPhase);

    // if it's negative, we've "dipped" into the next phase
    double arrivalTimeRelativeToPhaseEnd = timeToNext - flightTime;

    if (isCurrentScorable) {
      // Must stop shooting if ball arrives AFTER the phase ends + 3s buffer
      return arrivalTimeRelativeToPhaseEnd > -Constants.ShooterConstants.SHOOTING_BUFFER_TIME;
    }

    if (isNextScorable) {
      // can start shooting early if ball arrives within the next phase
      return arrivalTimeRelativeToPhaseEnd < 0;
    }

    return false;
  }

  private boolean isPhaseScorable(MatchPhase phase) {
    // same as the old "canScore()" code
    if (phase.getScoreType() == ScoreType.ALL_SCORE) return true;
    if (phase.getScoreType() == ScoreType.WINNING_SCORE && hasWonAuto()) return true;
    if (phase.getScoreType() == ScoreType.LOSING_SCORE && !hasWonAuto()) return true;
    return false;
  }

  double matchTimes[] = {20, 33, 58, 83, 108, 133, 163};

  public double timeUntilNextPhase() {
    double timeSinceStart = getTimeSinceStart();

    for (double i : matchTimes) {
      if (timeSinceStart > i) {
        continue;
      } else {
        return Math.abs(timeSinceStart - i);
      }
    }
    return 0;
  }

  // interface MatchChangeCallback {
  //   void run();
  // }
}
