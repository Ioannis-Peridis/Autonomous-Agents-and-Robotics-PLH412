//-----------------------------------------------------------------------------
//  File:         NaoCam.java (to be used in a Webots java controllers)
//  Date:         April 30, 2008
//  Description:  Camera with simple blob localization capability
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Changes:      November 4, 2008: Adapted to Webots6
//-----------------------------------------------------------------------------

import java.lang.Math;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Robot;

public class NaoCam {

  public static final double UNKNOWN = -999.9;
  public static final double OFFSET_ANGLE = 0.6981;  // 40 degrees between cameras axes
  public enum Goal { SKY_BLUE, YELLOW, UNKNOWN_COLOR };

  boolean topSelected = true;
  Camera topCamera;
  Camera bottomCamera;
  int timeStep;
  private Goal goalColor;
  private double fov;
  private int width, height;
  private int[] image;
  private double blobDirectionAngle = UNKNOWN;
  private double blobElevationAngle = UNKNOWN;
  private double ballDirectionAngle = UNKNOWN;
  private double ballElevationAngle = UNKNOWN;
  private double goalDirectionAngle = UNKNOWN;

  public NaoCam(Robot robot, int timeStep) {
    topCamera = robot.getCamera("CameraTop");
    bottomCamera = robot.getCamera("CameraBottom");
    
    this.timeStep = timeStep;
    goalColor = Goal.UNKNOWN_COLOR;
   
   // start with top camera
   selectTop();
  }
  
  public void setGoalColor(Goal goal) {
    this.goalColor = goal;
  }

  // find a blob whose rgb components match
  private void findColorBlob(int R, int G, int B, int threshold) {

    int x = 0, y = 0;
    int npixels = 0;

    for (int i = 0; i < image.length; i++) {
      int r = Camera.pixelGetRed(image[i]);
      int g = Camera.pixelGetGreen(image[i]);
      int b = Camera.pixelGetBlue(image[i]);

      if (Math.abs(r - R) + Math.abs(g - G) + Math.abs(b - B) < threshold) {
        x += i % width;
        y += i / width;
        npixels++;
      }
    }

    if (npixels > 0) {
      blobDirectionAngle = ((double)x / npixels / width - 0.5) * fov;
      blobElevationAngle = -((double)y / npixels / height - 0.5) * fov;
    }
    else {
      blobDirectionAngle = UNKNOWN;
      blobElevationAngle = UNKNOWN;
    }
  }

  // analyse image and find blobs
  public void processImage() {

    if (topSelected)
      image = topCamera.getImage();
    else
      image = bottomCamera.getImage();
    
    // find orange ball
    findColorBlob(240, 140, 50, 60);
    ballDirectionAngle = blobDirectionAngle;
    ballElevationAngle = blobElevationAngle;
    //System.out.println("camera: ball: dir: " + ballDirectionAngle + " elev: " + ballElevationAngle);

    // find goal
    switch (goalColor) {
    case SKY_BLUE:
      findColorBlob(30, 200, 200, 60);
      break;
    case YELLOW:
      findColorBlob(140, 140, 15, 60);
      break;
    default:
      goalDirectionAngle = UNKNOWN;
    }

    goalDirectionAngle = blobDirectionAngle;
    //System.out.println("camera: goal: dir: " + goalDirectionAngle);
  }

  // all direction and elevation angles are indicated in radians
  // with respect to the camera focal (or normal) line
  // a direction angle will not exceed +/- half the field of view
  // a positive direction is towards the right of the camera image
  // a positive elevation is towards the top of the camera image
  public double getBallDirectionAngle() {
    return ballDirectionAngle;
  }

  public double getBallElevationAngle() {
    return ballElevationAngle;
  }

  public double getGoalDirectionAngle() {
    return goalDirectionAngle;
  }
  
  public void selectTop() {
    bottomCamera.disable();
    topCamera.enable(timeStep);
    fov = topCamera.getFov();
    width = topCamera.getWidth();
    height = topCamera.getHeight();
    topSelected = true;
  }

  public void selectBottom() {
    topCamera.disable();
    bottomCamera.enable(timeStep);
    fov = bottomCamera.getFov();
    width = bottomCamera.getWidth();
    height = bottomCamera.getHeight();
    topSelected = false;
  }
  
  public double getOffsetAngle() {
    if (topSelected)
      return 0.0;
    else
      return OFFSET_ANGLE;
  }
}
