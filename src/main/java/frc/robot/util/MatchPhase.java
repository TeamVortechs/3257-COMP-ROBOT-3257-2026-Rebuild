package frc.robot.util;

import java.util.TreeMap;

public enum MatchPhase {
  BEGINNING(
      ScoreType.ALL_SCORE,
      "beginning(let ben know if u see this in code, it shouldn't be here)",
      0),
  AUTO(ScoreType.ALL_SCORE, "Auto", 20),
  TRANSITION_TO_TRANSITION(ScoreType.ALL_SCORE, "Disabled", 3),
  TRANSITION(ScoreType.ALL_SCORE, "Transition", 10),
  SHIFT_1(ScoreType.LOSING_SCORE, "Shift 1", 18),
  ALMOST_SHIFT_2(ScoreType.LOSING_SCORE, "Transition to Shift 2", 7, true),
  SHIFT_2(ScoreType.WINNING_SCORE, "Shift 2", 18),
  ALMOST_SHIFT_3(ScoreType.WINNING_SCORE, "Transition to Shift 3", 7, true),
  SHIFT_3(ScoreType.LOSING_SCORE, "Shift 3", 18),
  ALMOST_SHIFT_4(ScoreType.LOSING_SCORE, "Transition to Shift 4", 7, true),
  SHIFT_4(ScoreType.WINNING_SCORE, "Shift 4", 18),
  ALMOST_ENDGAME(ScoreType.WINNING_SCORE, "Transition Endgame", 7, true),
  END_GAME(ScoreType.ALL_SCORE, "End Game", 30),
  NO_PHASE(ScoreType.ALL_SCORE, "No Phase", 36000000);

  private ScoreType scoringType;
  private String displayName;
  private double time;
  private boolean vibrateController;

  private static TreeMap<MatchPhase, MatchPhase> NEXT_PHASES;
  private static TreeMap<MatchPhase, MatchPhase> PREV_PHASES;

  static {
    NEXT_PHASES = new TreeMap<>();
    NEXT_PHASES.put(BEGINNING, AUTO);
    NEXT_PHASES.put(AUTO, TRANSITION_TO_TRANSITION);
    NEXT_PHASES.put(TRANSITION_TO_TRANSITION, TRANSITION);
    NEXT_PHASES.put(TRANSITION, SHIFT_1);
    NEXT_PHASES.put(SHIFT_1, ALMOST_SHIFT_2);
    NEXT_PHASES.put(ALMOST_SHIFT_2, SHIFT_2);

    NEXT_PHASES.put(SHIFT_2, ALMOST_SHIFT_3);
    NEXT_PHASES.put(ALMOST_SHIFT_3, SHIFT_3);

    NEXT_PHASES.put(SHIFT_3, ALMOST_SHIFT_4);
    NEXT_PHASES.put(ALMOST_SHIFT_4, SHIFT_4);

    NEXT_PHASES.put(SHIFT_4, ALMOST_ENDGAME);

    NEXT_PHASES.put(ALMOST_ENDGAME, END_GAME);

    NEXT_PHASES.put(END_GAME, NO_PHASE);
    NEXT_PHASES.put(NO_PHASE, NO_PHASE);

    PREV_PHASES = new TreeMap<>();
    PREV_PHASES.put(BEGINNING, NO_PHASE);
    PREV_PHASES.put(AUTO, BEGINNING);
    PREV_PHASES.put(TRANSITION_TO_TRANSITION, AUTO);
    PREV_PHASES.put(TRANSITION, TRANSITION_TO_TRANSITION);
    PREV_PHASES.put(SHIFT_1, TRANSITION);
    PREV_PHASES.put(ALMOST_SHIFT_2, SHIFT_1);
    PREV_PHASES.put(SHIFT_2, ALMOST_SHIFT_2);

    PREV_PHASES.put(ALMOST_SHIFT_3, SHIFT_2);
    PREV_PHASES.put(SHIFT_3, ALMOST_SHIFT_3);

    PREV_PHASES.put(ALMOST_SHIFT_4, SHIFT_3);
    PREV_PHASES.put(SHIFT_4, ALMOST_SHIFT_4);

    PREV_PHASES.put(ALMOST_ENDGAME, SHIFT_4);

    PREV_PHASES.put(END_GAME, ALMOST_ENDGAME);

    PREV_PHASES.put(NO_PHASE, END_GAME);
  }

  private MatchPhase(ScoreType scoringType, String displayName, double time) {
    this.scoringType = scoringType;
    this.displayName = displayName;
    this.time = time;
    this.vibrateController = false;
  }

  private MatchPhase(
      ScoreType scoringType, String displayName, double time, boolean vibrateController) {
    this.scoringType = scoringType;
    this.displayName = displayName;
    this.time = time;
    this.vibrateController = vibrateController;
  }

  public MatchPhase getNextPhase() {
    return NEXT_PHASES.get(this);
  }

  public MatchPhase getPrevPhase() {
    return PREV_PHASES.get(this);
  }

  public ScoreType getScoreType() {
    return scoringType;
  }

  public String getDisplayName() {
    return displayName;
  }

  public double getTime() {
    return time;
  }

  public boolean getVibration() {
    return vibrateController;
  }
}
