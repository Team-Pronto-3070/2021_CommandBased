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
            camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        }
    }

    public static void takeSnapshot(){

        // Get the latest pipeline result.
        PhotonPipelineResult result = camera.getLatestResult();

        System.out.println("Testing for targets");
        
        targetInfo.clear();
        if(result.hasTargets()){
            System.out.println("-----------Found "+ result.getTargets().size());
            for (PhotonTrackedTarget target: result.getTargets()){
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                
                targetInfo.add(new PixelPoint(pitch, yaw, area));
            }
        System.out.println("------------Captured "+targetInfo.size());
        }
    }

    public String selectPath(){
        if(ARED_PROFILE != null && ABLUE_PROFILE != null && BRED_PROFILE != null && BBLUE_PROFILE != null){
            System.out.println("selecting path");
            Vision.takeSnapshot();
            return choosePath(targetInfo);
        }else{
            System.out.println("One or more paths not set. Returning none.");
            return "none";
        }
    }

    public void updateProfile(String profile){
        System.out.println("Updating Profiles");
        Vision.takeSnapshot();
        setProfiles(targetInfo, profile);
    }

    

    /**
   * This is the pathing section!! I know it's excessibe; I'll admit, I just wanted to see how enums worked.
   */

    ArrayList<PixelPoint> ARED_POINTS = new ArrayList<>();
    ArrayList<PixelPoint> ABLUE_POINTS = new ArrayList<>();
    ArrayList<PixelPoint> BRED_POINTS = new ArrayList<>();
    ArrayList<PixelPoint> BBLUE_POINTS = new ArrayList<>();

    //Create profile objects for the points lists (can compare objects to get match value)
    public PixelProfile ARED_PROFILE;
    public PixelProfile ABLUE_PROFILE;
    public PixelProfile BRED_PROFILE;
    public PixelProfile BBLUE_PROFILE;
 
    private void setProfiles(ArrayList<PixelPoint> points, String path){

        if(points.size() == 0){
            System.out.println("No points to set.");
            return;
        }

        switch(path){
            case "aRed":
                ARED_POINTS.addAll(points);
                System.out.println("Setting ARED path");
                break;
            case "aBlue":
                ABLUE_POINTS.addAll(points);
                System.out.println("Setting ABLUE path");
                break;
            case "bRed":
                BRED_POINTS.addAll(points);
                System.out.println("Setting BRED path");
                break;
            case "bBlue":
                BBLUE_POINTS.addAll(points);
                System.out.println("Setting BBLUE path");
                break;
            case "noSet":
                break;
        }
        ARED_PROFILE = new PixelProfile(ARED_POINTS, "aRed");
        ABLUE_PROFILE = new PixelProfile(ABLUE_POINTS, "aBlue");
        BRED_PROFILE = new PixelProfile(BRED_POINTS, "bRed");
        BBLUE_PROFILE = new PixelProfile(BBLUE_POINTS, "bBlue");
        getPathInfo(path);
    }
  //Formatted x, y, area, pointTolerance (optional) (add as many points as you want though they 
  //need to be in the same order as the camera point list)

  


  
  //List of preset profiles to compare too

  /**
   * This method takes a network table and a list of points to compare to. 
   * It will match the closest matching profile and return the chosen path in the network table. 
   * @param table
   * @param points
   */
    private String choosePath(ArrayList<PixelPoint> points){
        PixelProfile visibleProfile = new PixelProfile(points, "none - DEFAULT");
        PixelProfile[] profiles = {ARED_PROFILE, ABLUE_PROFILE,BRED_PROFILE,BBLUE_PROFILE};

        String chosenPath = visibleProfile.match(profiles);
    
        if(chosenPath.equals("none - DEFAULT")){
            System.out.println("No path chosen in java-multiCameraServer/Main.java: Main.choosePath()");
        }

        return chosenPath;
    }



    public String getPathInfo(String profile){
        String output = "";

        if(profile.equals("aRed")){
            for(PixelPoint point: ARED_POINTS){
                System.out.println("---");
                System.out.println("Area:" + point.area); 
                System.out.println("Yaw" + point.yaw); 
                System.out.println("Pitch" + point.pitch); 
            }
        }
        else if(profile.equals("aBlue")){
            for(PixelPoint point: ABLUE_POINTS){
                System.out.println("---");
                System.out.println("Area:" + point.area); 
                System.out.println("Yaw" + point.yaw); 
                System.out.println("Pitch" + point.pitch); 
            }
    
        }
        else if(profile.equals("bRed")){
            for(PixelPoint point: BRED_POINTS){
                System.out.println("---");
                System.out.println("Area:" + point.area); 
                System.out.println("Yaw" + point.yaw); 
                System.out.println("Pitch" + point.pitch); 
            }
        }
        else if(profile.equals("bBlue")){
            for(PixelPoint point: BBLUE_POINTS){
                System.out.println("---");
                System.out.println("Area:" + point.area); 
                System.out.println("Yaw" + point.yaw); 
                System.out.println("Pitch" + point.pitch); 
               
            }
        }
        else{
            output = "no path selected";
        }

        return output;

    }
}
