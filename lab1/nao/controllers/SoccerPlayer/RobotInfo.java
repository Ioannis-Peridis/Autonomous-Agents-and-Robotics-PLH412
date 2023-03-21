//---------------------------------------------------------------------------------------
//  File:         RobotInfo.java (to be used in a Webots java controllers)
//  Date:         February 26, 2009
//  Description:  This class is used to read a binary RobotInfo struct inside a
//                RoboCupGameControlData sent by nao_soccer_supervisor.c
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//---------------------------------------------------------------------------------------

import java.nio.ByteBuffer;

public class RobotInfo {

  // penalties
  public static final short PENALTY_NONE = 0;
  public static final short PENALTY_ILLEGAL_DEFENDER = 4;

  private short penalty;             // the penalty state of the robot
  private short secsTillUnpenalise;  // estimated seconds till unpenalised

  public RobotInfo() {
  }

  public void readBytes(ByteBuffer buffer) {
    penalty = buffer.getShort();
    secsTillUnpenalise = buffer.getShort();
  }

  public short getPenalty() {
    return penalty;
  }

  public short getSecsTillUnpenalised() {
      return secsTillUnpenalise;
  }

  @Override public String toString() {
    StringBuilder result = new StringBuilder();
    String NEW_LINE = System.getProperty("line.separator");
    result.append(this.getClass().getName() + " Object {" + NEW_LINE);
    result.append("   penalty: " + penalty + NEW_LINE);
    result.append("   secsTillUnpenalise: " + secsTillUnpenalise + NEW_LINE);
    result.append("  }");
    return result.toString();
  }
}
