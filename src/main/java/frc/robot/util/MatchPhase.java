package frc.robot.util;

import java.util.TreeMap;

public enum MatchPhase {
  BEGINNING(
      ScoreType.ALL_SCORE,
      "beginning(let ben know if u see this in code, it shouldn't be here)",
      0),
  AUTO(ScoreType.ALL_SCORE, "Auto", 20),
  TRANSITION(ScoreType.ALL_SCORE, "Transition", 10),
  SHIFT_1(ScoreType.LOSING_SCORE, "Shift 1", 18),
  ALMOST_SHIFT_2(ScoreType.LOSING_SCORE, "Transition to Shift 2", 7),
  SHIFT_2(ScoreType.WINNING_SCORE, "Shift 2", 18),
  ALMOST_SHIFT_3(ScoreType.WINNING_SCORE, "Transition to Shift 3", 7),
  SHIFT_3(ScoreType.LOSING_SCORE, "Shift 3", 18),
  ALMOST_SHIFT_4(ScoreType.WINNING_SCORE, "Transition to Shift 4", 7),
  SHIFT_4(ScoreType.WINNING_SCORE, "Shift 4", 18),
  ALMOST_ENDGAME(ScoreType.WINNING_SCORE, "Transition Endgame", 7),
  END_GAME(ScoreType.ALL_SCORE, "End Game", 30),
  NO_PHASE(ScoreType.ALL_SCORE, "No Phase", 36000000);

  private ScoreType scoringType;
  private String displayName;
  private double time;

  private static TreeMap<MatchPhase, MatchPhase> NEXT_PHASES;

  static {
    NEXT_PHASES = new TreeMap<>();
    NEXT_PHASES.put(BEGINNING, AUTO);
    NEXT_PHASES.put(AUTO, SHIFT_1);
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
  }

  private MatchPhase(ScoreType scoringType, String displayName, double time) {
    this.scoringType = scoringType;
    this.displayName = displayName;
    this.time = time;
  }

  public MatchPhase getNextPhase() {
    return NEXT_PHASES.get(this);
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
}
