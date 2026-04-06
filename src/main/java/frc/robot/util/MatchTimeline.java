// package frc.robot.util;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Constants.MatchTimelineConstants;
// import java.util.Optional;
// import org.littletonrobotics.junction.Logger;

// public class MatchTimeline {
//   private MatchPhase currentPhase = MatchPhase.NO_PHASE;

//   // private Notifier notifer;
//   private Notifier logger;

//   private Timer timer;

//   private CommandXboxController controller;
//   private CommandXboxController secondController;

//   private Optional<Alliance> teamThatWonAuto = Optional.empty();

//   public MatchTimeline(
//       CommandXboxController commandXboxController, CommandXboxController secondController) {
//     this.controller = commandXboxController;
//     this.secondController = secondController;

//     logger =
//         new Notifier(
//             () -> {
//               logOutputs();
//             });

//     Logger.recordOutput("MatchTimeline/currentPhase", MatchPhase.NO_PHASE.getDisplayName());

//     timer = new Timer();
//   }

//   public void logOutputs() {
//     Logger.recordOutput("MatchTimeline/timeUntilNextPhase", timeUntilNextPhase());
//     Logger.recordOutput("MatchTimeline/canScore", canScore());
//     Logger.recordOutput("MatchTimeline/currentPhase", currentPhase.getDisplayName());
//     Logger.recordOutput("MatchTimeline/isWinningAuto", hasWonAuto());
//     // Logger.recordOutput("MatchTimeline/teamThatWonAuto", );
//   }

//   public void start() {
//     currentPhase = MatchPhase.BEGINNING;
//     logger.startPeriodic(MatchTimelineConstants.TIMER_FREQUENCY);

//     advancePhase();

//     timer.restart();
//   }

//   private Command vibrateControllerCommand() {
//     return new StartEndCommand(
//             () -> {
//               controller.getHID().setRumble(RumbleType.kBothRumble, 1);
//               secondController.getHID().setRumble(RumbleType.kBothRumble, 1);
//             },
//             () -> {
//               controller.getHID().setRumble(RumbleType.kBothRumble, 0);
//               secondController.getHID().setRumble(RumbleType.kBothRumble, 0);
//             })
//         .withDeadline(new WaitCommand(1));
//   }

//   // used to ensure controller is only vibrated once per transition
//   private boolean alreadyVibrated = false;

//   public void advancePhase() {
//     double timeSinceStart = getTimeSinceStart();
//     MatchPhase currPhase = MatchPhase.BEGINNING;
//     double timeWindow = 0;
//     while (timeWindow < timeSinceStart) {
//       currPhase = currPhase.getNextPhase();
//       timeWindow += currPhase.getTime();
//     }

//     // crucial step: set the global "currentPhase" to local "currPhase"
//     currentPhase = currPhase;
//     if (currentPhase.shouldVibrate() && !alreadyVibrated) {
//       CommandScheduler.getInstance().schedule(vibrateControllerCommand());
//       alreadyVibrated = true;
//     } else if (!currentPhase.shouldVibrate()) {
//       alreadyVibrated = false;
//     }
//   }

//   public void setController(CommandXboxController controller) {
//     this.controller = controller;
//   }

//   public MatchPhase getCurrentPhase() {
//     return currentPhase;
//   }

//   public double getTimeSinceStart() {
//     return timer.get();
//   }

//   /**
//    * returns wether or not your team won auto
//    *
//    * @return
//    */
//   public boolean hasWonAuto() {
//     if (teamThatWonAuto.isEmpty()) {
//       updateAutoWinner();
//     }

//     if (DriverStation.getAlliance() == null || DriverStation.getAlliance().isEmpty()) {
//       return false;
//     }

//     if (teamThatWonAuto.isPresent()) {
//       return teamThatWonAuto.get() == DriverStation.getAlliance().get();
//     }

//     return false;
//   }

//   public void updateAutoWinner() {
//     String gameData = DriverStation.getGameSpecificMessage();

//     if (gameData == null || gameData.length() <= 0 || gameData.isEmpty()) {
//       return;
//     }

//     switch (gameData.charAt(0)) {
//       case 'R':
//         teamThatWonAuto = Optional.of(Alliance.Red);
//         break;

//       case 'B':
//         teamThatWonAuto = Optional.of(Alliance.Blue);
//         break;

//       default:
//         break;
//     }
//   }

//   /**
//    * With robot pose, determine distance from goal, and thus determine flight time of ball. Going
// to
//    * be used to score more points. This is a helper class so it's private
//    *
//    * @return flight time in seconds
//    */
//   private double getFlightTime() {
//     // dummy method, implement actual physics math later
//     return 3.0;
//   }

//   /**
//    * Determine if robot can score based on current phase. Considers flight time, sensor time, and
//    * tolerance.
//    *
//    * @return
//    */
//   public boolean canScore() {
//     MatchPhase current = getCurrentPhase();
//     double timeToNext = timeUntilNextPhase();
//     double sensorTime = MatchTimelineConstants.SENSOR_TIME;
//     double tolerance = MatchTimelineConstants.SHOOTING_TOLERANCE;

//     // 1. Currently in a scorable phase
//     if (isPhaseScorable(current)) {
//       // Pessimistic: assume max flight, max sensor time, plus tolerance to ensure it lands
// before
//       // phase + buffer ends
//       double maxTotalTime = MatchTimelineConstants.MAX_FLIGHT_LENGTH + sensorTime + tolerance;
//       double arrivalTimeRelativeToPhaseEnd = timeToNext - maxTotalTime;
//       return arrivalTimeRelativeToPhaseEnd > -MatchTimelineConstants.SHOOTING_BUFFER_TIME;
//     }

//     // 2. Entering a scorable phase next
//     MatchPhase nextPhase = current.getNextPhase();
//     if (nextPhase != null && isPhaseScorable(nextPhase)) {
//       // Pessimistic: assume min flight, min sensor time, minus tolerance to ensure it doesn't
// land
//       // before the phase begins
//       double minTotalTime = MatchTimelineConstants.MIN_FLIGHT_LENGTH + sensorTime - tolerance;
//       double arrivalTimeRelativeToPhaseEnd = timeToNext - minTotalTime;
//       return arrivalTimeRelativeToPhaseEnd < 0;
//     }

//     // 3. Just left a scorable phase
//     MatchPhase prevPhase = current.getPrevPhase();
//     if (prevPhase != null && isPhaseScorable(prevPhase)) {
//       // Pessimistic: assume max flight, max sensor time, plus tolerance to ensure it lands
// before
//       // the grace period expires
//       double maxTotalTime = MatchTimelineConstants.MAX_FLIGHT_LENGTH + sensorTime + tolerance;
//       double timeSpentInCurrentPhase = MatchTimelineConstants.SHIFT_LENGTH - timeToNext;
//       return (timeSpentInCurrentPhase + maxTotalTime) <
// MatchTimelineConstants.SHOOTING_BUFFER_TIME;
//     }

//     return false;
//   }

//   private boolean isPhaseScorable(MatchPhase phase) {
//     // same as the old "canScore()" code
//     if (phase.getScoreType() == ScoreType.ALL_SCORE) return true;
//     if (phase.getScoreType() == ScoreType.WINNING_SCORE && hasWonAuto()) return true;
//     if (phase.getScoreType() == ScoreType.LOSING_SCORE && !hasWonAuto()) return true;
//     return false;
//   }

//   double matchTimes[] = {20, 33, 58, 83, 108, 133, 163};

//   public double timeUntilNextPhase() {
//     double timeSinceStart = getTimeSinceStart();
//     // I need to run "advancePhase" every tick, so it was originally run by Logger
//     // But, this would cause it desync with "timeUntilNextPhase", which would make "canScore()"
//     // return the wrong value for .25 secs
//     // So, if I run "advancePhase" here, it runs every tick AND it's in sync!
//     advancePhase();
//     for (double i : matchTimes) {
//       if (timeSinceStart > i) {
//         continue;
//       } else {
//         return Math.abs(timeSinceStart - i);
//       }
//     }
//     return 0;
//   }
// }
