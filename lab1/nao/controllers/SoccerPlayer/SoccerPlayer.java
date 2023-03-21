//---------------------------------------------------------------------------------------
//  File:         SoccerPlayer.java (to be used in a Webots java controllers)
//  Date:         April 30, 2008
//  Description:  This is a bootstrap for the Java controller example of Robotstadium
//                It selects and runs the correct player type: FieldPlayer or GoalKeeper
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Changes:      November 4, 2008: Adapted to Webots 6
//                May 3, 2010: Changed how teamID and playerID are determined
//---------------------------------------------------------------------------------------

public class SoccerPlayer {

  public static void main(String[] args) {

    // get team and player id's from controllerArgs
    int playerID = Integer.parseInt(args[0]);
    int teamID   = Integer.parseInt(args[1]);

    // choose GoalKepper/FieldPlayer role according to playerID
    if (playerID == 0)
      new GoalKeeper(playerID, teamID).run();
    else
      new FieldPlayer(playerID, teamID).run();
  }
}
