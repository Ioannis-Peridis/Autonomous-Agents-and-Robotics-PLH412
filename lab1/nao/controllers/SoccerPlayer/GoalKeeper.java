//-----------------------------------------------------------------------------
//  File:         GoalKeeper.java (to be used in a Webots java controllers)
//  Date:         April 30, 2008
//  Description:  Goal keeper for "red" or "blue" team
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Changes:      November 4, 2008: Adapted to Webots6
//-----------------------------------------------------------------------------

import com.cyberbotics.webots.controller.*;

public class GoalKeeper extends Player {

  private Motion sideStepLeftMotion, sideStepRightMotion, forwards50Motion, backwardsMotion;
  private int rightStepsCount = 0;

  public GoalKeeper(int playerID, int teamID) {
    super(playerID, teamID);
    forwards50Motion    = new Motion("../../motions/Forwards50.motion");
    backwardsMotion     = new Motion("../../motions/Backwards.motion");
    sideStepLeftMotion  = new Motion("../../motions/SideStepLeft.motion");
    sideStepRightMotion = new Motion("../../motions/SideStepRight.motion");

    // move arms
    Motor leftShoulderRoll = getMotor("LShoulderRoll");
    Motor rightShoulderRoll = getMotor("RShoulderRoll");
    leftShoulderRoll.setPosition(1.32);
    rightShoulderRoll.setPosition(-1.32);
  }

  private void stepRight() {
    playMotion(sideStepRightMotion);
    rightStepsCount++;
  }

  private void stepLeft() {
    playMotion(sideStepLeftMotion);
    rightStepsCount--;
  }

  @Override public void run() {
    step(SIMULATION_STEP);
    step(SIMULATION_STEP);

    while (true) {
      getUpIfNecessary();

      // loop until ball becomes visible
      while (getBallDirection() == NaoCam.UNKNOWN) {
        getUpIfNecessary();
        if (getBallDirection() != NaoCam.UNKNOWN) break;
        headScan();
      }

      double ballDir = getBallDirection();
      double ballDist = getBallDistance();

      if (ballDist < 0.8 && ballDir > -0.15 && ballDir < 0.15) {
        // ball is close and in front: try to kick it
        playMotion(forwards50Motion);

        // move backwards to goal
        for (int i = 0; i < 5; i++)
          playMotion(backwardsMotion);

      }
      else if (ballDist < 2.0) {
        // if the ball is within 2 meters
        // step right/left in the direction of the ball
        if (ballDir > 0.1 && rightStepsCount < 8)
          stepRight();
        else if (ballDir < -0.1 && rightStepsCount > -8)
          stepLeft();
      }
      else {
        // the ball is quite far: step back to the center of the goal
        if (rightStepsCount < 0)
          stepRight();
        else if (rightStepsCount > 0)
          stepLeft();
      }

      runStep();
    }
  }
}
