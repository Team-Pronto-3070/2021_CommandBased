package frc.robot;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

public class Vision {


    static ArrayList<PixelPoint> targetInfo = new ArrayList<>();
    // Creates a new PhotonCamera.
    static PhotonCamera camera;

    public Vision(){
        if(camera == null){
            camera = new PhotonCamera("MyCamera");
        }
    }

    public static void takeSnapshot(){

        // Get the latest pipeline result.
        PhotonPipelineResult result = camera.getLatestResult();


        if(result.hasTargets()){

            for (PhotonTrackedTarget target: result.getTargets()){
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();

                PixelPoint info = new PixelPoint(pitch, yaw, skew, area);

                targetInfo.add(info);

            }
        }
    }

    public String selectPath(){
        return choosePath(targetInfo);
    }

    public void updateProfile(String profile){
        setProfiles(targetInfo, profile);
    }

    /**
   * This is the pathing section!! I know it's excessibe; I'll admit, I just wanted to see how enums worked.
   */

  PixelPoint[] ARED_POINTS;
  PixelPoint[] ABLUE_POINTS;
  PixelPoint[] BRED_POINTS;
  PixelPoint[] BBLUE_POINTS;

    private void setProfiles(ArrayList<PixelPoint> points, String path){
        switch(path){
            case "aRed":
                ARED_POINTS = (PixelPoint[]) points.toArray();
                break;
            case "aBlue":
                ABLUE_POINTS = (PixelPoint[]) points.toArray();
                break;
            case "bRed":
                BRED_POINTS = (PixelPoint[]) points.toArray();
                break;
            case "bBlue":
                BBLUE_POINTS = (PixelPoint[]) points.toArray();
                break;
            case "noSet":
                break;
        }
    }
  //Formatted x, y, area, pointTolerance (optional) (add as many points as you want though they 
  //need to be in the same order as the camera point list)

  

  //Create profile objects for the points lists (can compare objects to get match value)
  public final PixelProfile ARED_PROFILE = new PixelProfile(ARED_POINTS, "aRed");
  public final PixelProfile ABLUE_PROFILE = new PixelProfile(ABLUE_POINTS, "aBlue");
  public final PixelProfile BRED_PROFILE = new PixelProfile(BRED_POINTS, "bRed");
  public final PixelProfile BBLUE_PROFILE = new PixelProfile(BBLUE_POINTS, "bBlue");
  
  //List of preset profiles to compare too
  PixelProfile[] profiles = {ARED_PROFILE, ABLUE_PROFILE,BRED_PROFILE,BBLUE_PROFILE};

  /**
   * This method takes a network table and a list of points to compare to. 
   * It will match the closest matching profile and return the chosen path in the network table. 
   * @param table
   * @param points
   */
    private String choosePath(ArrayList<PixelPoint> points){
        PixelProfile visibleProfile = new PixelProfile((PixelPoint[]) points.toArray(), "none");

        String chosenPath = visibleProfile.match(profiles);
    
        if(chosenPath.equals("none")){
            System.out.println("No path chosen in java-multiCameraServer/Main.java: Main.choosePath()");
        }

        return chosenPath;
    }
}
