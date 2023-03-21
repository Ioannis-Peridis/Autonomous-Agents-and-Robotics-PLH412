//-----------------------------------------------------------------------------
//  File:         Player.java (to be used in a Webots java controllers)
//  Date:         April 30, 2008
//  Description:  Base class for FieldPlayer and GoalKeeper
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Changes:      November 4, 2008: Adapted to Webots 6
//                February 25, 2008: Adapted to NaoV3R.proto (Camera select)
//-----------------------------------------------------------------------------

import com.cyberbotics.webots.controller.*;

public abstract class Player extends Robot {

  public static final int SIMULATION_STEP = 40;  // milliseconds
  public static final int CAMERA_STEP = 160;  // camera refresh rate in milliseconds

  protected RoboCupGameControlData gameControlData = new RoboCupGameControlData();
  protected int teamID;
  protected int playerID;
  private Motion standUpFromFrontMotion;

  // devices
  protected Motor headYaw, headPitch;
  protected PositionSensor headYawPosition, headPitchPosition;
  protected NaoCam camera;
  protected Accelerometer accelerometer;
  protected Gyro gyro;
  protected InertialUnit inertialUnit;
  protected DistanceSensor topLeftUltrasound, topRightUltrasound, bottomLeftUltrasound, bottomRightUltrasound;
  protected LED chestLed, rightEyeLed, leftEyeLed, rightEarLed, leftEarLed, rightFootLed, leftFootLed;
  protected TouchSensor[] fsr;  // force sensitive resistors
  protected Emitter emitter, super_emitter;
  protected Receiver receiver;
  protected GPS gps;  // for debugging only ! This device does not exist on the real robot.
  
  double minHeadYawPosition, maxHeadYawPosition, minHeadPitchPosition, maxHeadPitchPosition;

  public double clamp(double value, double min, double max) {
    if (min > max) {
      assert false;
      return value;
    }
    return value < min ? min : value > max ? max : value;
  }

  public boolean isBlue() {
    return gameControlData.getTeam(RoboCupGameControlData.TEAM_BLUE).getTeamNumber() == teamID;
  }

  public boolean isRed() {
    return gameControlData.getTeam(RoboCupGameControlData.TEAM_RED).getTeamNumber() == teamID;
  }

  public Player(int playerID, int teamID) {
    this.playerID = playerID;
    this.teamID = teamID;

    // initialize accelerometer
    accelerometer = getAccelerometer("accelerometer");
    //accelerometer.enable(SIMULATION_STEP);  // uncomment only if needed !

    // initialize gyro
    gyro = getGyro("gyro");
    //gyro.enable(SIMULATION_STEP);  // uncomment only if needed !
    
    // initialize inertial unit
    inertialUnit = getInertialUnit("inertial unit");
    inertialUnit.enable(SIMULATION_STEP);

    // get "HeadYaw" and "HeadPitch" motors and enable position feedback
    headYaw = getMotor("HeadYaw");
    headYawPosition = getPositionSensor("HeadYawS");
    headYawPosition.enable(SIMULATION_STEP);
    minHeadYawPosition = headYaw.getMinPosition();
    maxHeadYawPosition = headYaw.getMaxPosition();
    headPitch = getMotor("HeadPitch");
    headPitchPosition = getPositionSensor("HeadPitchS");
    headPitchPosition.enable(SIMULATION_STEP);
    minHeadPitchPosition = headPitch.getMinPosition();
    maxHeadPitchPosition = headPitch.getMaxPosition();

    // get all LEDs
    chestLed = getLED("ChestBoard/Led");
    rightEyeLed = getLED("Face/Led/Right");
    leftEyeLed = getLED("Face/Led/Left");
    rightEarLed = getLED("Ears/Led/Right");
    leftEarLed = getLED("Ears/Led/Left");
    rightFootLed = getLED("RFoot/Led");
    leftFootLed = getLED("LFoot/Led");

    // make eyes shine blue
    rightEyeLed.set(0x2222ff);
    leftEyeLed.set(0x2222ff);

    // create camera
    camera = new NaoCam(this, CAMERA_STEP);

    // foot sole touch sensors
    final String[] TOUCH_SENSOR_NAMES = {
      "RFsr", "LFsr"
    };
    fsr = new TouchSensor[TOUCH_SENSOR_NAMES.length];
    for (int i = 0; i < TOUCH_SENSOR_NAMES.length; i++) {
      fsr[i] = getTouchSensor(TOUCH_SENSOR_NAMES[i]);
      //fsr[i].enable(SIMULATION_STEP);  // uncomment only if needed !
    }

    // emitter/receiver devices that can be used for inter-robot communication
    // and for receiving RobotCupGameControleData
    emitter = getEmitter("emitter");
    receiver = getReceiver("receiver");
    receiver.enable(SIMULATION_STEP);

    // for sending 'move' request to Supervisor
    super_emitter = getEmitter("super_emitter");

    // useful to know the position of the robot
    // the real Nao does not have a GPS, this is for testing only
    // this info will be blurred during Robotstadium contest matches
    gps = getGPS("gps");
    //gps.enable(SIMULATION_STEP);  // uncomment only if needed !

    // initialize ultrasound sensors
    //topLeftUltrasound = getDistanceSensor("USSensor3");
    //topRightUltrasound = getDistanceSensor("USSensor1");
    //bottomLeftUltrasound = getDistanceSensor("USSensor4");
    //bottomRightUltrasound = getDistanceSensor("USSensor2");
    //topLeftUltrasound.enable(SIMULATION_STEP);  // uncomment only if needed !
    //topRightUltrasound.enable(SIMULATION_STEP);  // uncomment only if needed !
    //bottomLeftUltrasound.enable(SIMULATION_STEP);  // uncomment only if needed !
    //bottomRightUltrasound.enable(SIMULATION_STEP);  // uncomment only if needed !

    // load motion
    standUpFromFrontMotion = new Motion("../../motions/StandUpFromFront.motion");
  }

  // play until the specified motion is over
  protected void playMotion(Motion motion) {

    if (gameControlData.getState() != RoboCupGameControlData.STATE_PLAYING) {
      runStep();
      return;
    }

    // play to the end
    motion.play();
    do {
      runStep();
    }
    while (! motion.isOver());
  }

  // use inertial unit to detect fall
  protected void getUpIfNecessary() {
    double[] rpy = inertialUnit.getRollPitchYaw();
    if (Math.abs(rpy[0]) > 3.142 || Math.abs(rpy[1]) > 3.142)
      playMotion(standUpFromFrontMotion);
  }

  // move head from left to right and from right to left
  // until the ball is sighted or the scan motion is over
  protected void headScan() {
    final int STEPS = 30;
    final double HEAD_YAW_MAX = 2.0;
    double yawAngle;

    headPitch.setPosition(0.0);  // horizontal head
    camera.selectTop();  // use top camera

    // left to right using TOP camera
    for (int i = 0; i < STEPS; i++) {
      yawAngle = ((double)i / (STEPS - 1) * 2.0 - 1.0) * HEAD_YAW_MAX;
      headYaw.setPosition(clamp(yawAngle, minHeadYawPosition, maxHeadYawPosition));
      step(SIMULATION_STEP);
      camera.processImage();
      if (camera.getBallDirectionAngle() != NaoCam.UNKNOWN)
        return;
    }

    camera.selectBottom();  // use bottom camera

    // right to left using BOTTOM camera
    for (int i = STEPS - 1; i >= 0; i--) {
      yawAngle = ((double)i / (STEPS - 1) * 2.0 - 1.0) * HEAD_YAW_MAX;
      headYaw.setPosition(clamp(yawAngle, minHeadYawPosition, maxHeadYawPosition));
      step(SIMULATION_STEP);
      camera.processImage();
      if (camera.getBallDirectionAngle() != NaoCam.UNKNOWN)
        return;
    }

    // ball was not found: restore head straight position
    headYaw.setPosition(0.0);
  }

  public double getBallDirection() {
    if (camera.getBallDirectionAngle() == NaoCam.UNKNOWN)
      return NaoCam.UNKNOWN;
    else
      return camera.getBallDirectionAngle() - headYawPosition.getValue();
  }

  // compute floor distance between robot (feet) and ball
  // 0.51 -> approx robot camera base height with respect to ground in a standard posture of the robot
  // 0.043 -> ball radius
  public double getBallDistance() {
    if (camera.getBallElevationAngle() == NaoCam.UNKNOWN)
      return NaoCam.UNKNOWN;

    double ballElev = camera.getBallElevationAngle() - headPitchPosition.getValue() - camera.getOffsetAngle();
    return (0.51 - 0.043) / Math.tan(-ballElev);
  }

  // turn head towards ball if ball position is known
  protected void trackBall() {
    final double P = 0.7;
    double ballDirection = camera.getBallDirectionAngle();
    double ballElevation = camera.getBallElevationAngle();

    if (ballDirection == NaoCam.UNKNOWN) return;

    // compute target head pitch
    double pitch = headPitchPosition.getValue() - ballElevation * P;

    if (pitch < -0.4 && camera.getOffsetAngle() > 0.0) { // need to switch to TOP camera ?
      System.out.println("switched to TOP camera");
      camera.selectTop();
      pitch += NaoCam.OFFSET_ANGLE;  // move head down 40 degrees
      headPitch.setPosition(clamp(pitch, minHeadPitchPosition, maxHeadPitchPosition));
      sleepSteps(8);  // allow some time to move head
      camera.processImage();
    }
    else if (pitch > 0.5 && camera.getOffsetAngle() == 0.0) { // need to switch to BOTTOM camera ?
      System.out.println("switched to BOTTOM camera");
      camera.selectBottom();
      pitch -= NaoCam.OFFSET_ANGLE;  // move head up 40 degrees
      headPitch.setPosition(clamp(pitch, minHeadPitchPosition, maxHeadPitchPosition));
      sleepSteps(8);  // allow some time to move head
      camera.processImage();
    }

    headPitch.setPosition(clamp(pitch, minHeadPitchPosition, maxHeadPitchPosition));
    double yawAngle = headYawPosition.getValue() - ballDirection * P;
    headYaw.setPosition(clamp(yawAngle, minHeadYawPosition, maxHeadYawPosition));
  }

  // update torso LED color according to game state
  protected void updateGameControl() {

    // choose goal color according to team's color
    // and display team color on left foot LED
    if (isRed()) {
      camera.setGoalColor(NaoCam.Goal.SKY_BLUE);
      leftFootLed.set(0xff2222);
    }
    else if (isBlue()) {
      camera.setGoalColor(NaoCam.Goal.YELLOW);
      leftFootLed.set(0x2222ff);
    }

    switch (gameControlData.getState()) {
      case RoboCupGameControlData.STATE_INITIAL:
      case RoboCupGameControlData.STATE_FINISHED:
        chestLed.set(0x000000);  // off
        break;
      case RoboCupGameControlData.STATE_READY:
        chestLed.set(0x2222ff);  // blue
        break;
      case RoboCupGameControlData.STATE_SET:
        chestLed.set(0xffff22);  // yellow
        break;
      case RoboCupGameControlData.STATE_PLAYING:
        chestLed.set(0x22ff22);  // green
        break;
    }
  }

  protected void readIncomingMessages() {
    while (receiver.getQueueLength() > 0) {
      byte[] data = receiver.getData();
      if (RoboCupGameControlData.hasValidHeader(data)) {
        gameControlData.update(data);
        //System.out.println(gameControlData);
        updateGameControl();
      }
      // else
      //   System.out.println("readIncomingMessages(): received unexpected message of " + data.length + " bytes");

      receiver.nextPacket();
    }
  }

  // move the robot to a specified position (via a message sent to the Supervisor)
  // [tx ty tz]: the new robot position, alpha: the robot's heading direction
  // For debugging only: this is disabled during the contest rounds
  protected void sendMoveRobotMessage(double tx, double ty, double tz, double alpha) {
    String request = "move robot " + playerID + " " + teamID + " " + tx + " " + ty + " " + tz + " " + alpha + "\0";
    try {
      super_emitter.send(request.getBytes("US-ASCII"));
    }
    catch (java.io.UnsupportedEncodingException e) {
      System.out.println(e);
    }
  }

  // move the ball to a specified position (via a message sent to the Supervisor)
  // [tx ty tz]: the new ball position
  // For debugging only: this is disabled during the contest rounds
  protected void sendMoveBallMessage(double tx, double ty, double tz) {
    String request = "move ball " + tx + " " + ty + " " + tz + "\0";
    try {
      super_emitter.send(request.getBytes("US-ASCII"));
    }
    catch (java.io.UnsupportedEncodingException e) {
      System.out.println(e);
    }
  }
  
  // overidden method of the Robot baseclass
  // we need to read incoming messages at every step
  public int step(int ms) {
    readIncomingMessages();
    return super.step(ms);
  }

  protected void runStep()  {
    trackBall();
    step(SIMULATION_STEP);
    camera.processImage();
  }

  protected void sleepSteps(int steps) {
    for (int i = 0; i < steps; i++)
      step(SIMULATION_STEP);
  }

  public abstract void run();
}
