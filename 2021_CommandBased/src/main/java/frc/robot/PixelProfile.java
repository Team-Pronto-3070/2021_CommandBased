package frc.robot;

import java.util.ArrayList;
import java.util.Collections;

/**
 * This class is a collection of PixelPoint objects that adds comparation functionality;
 */

public class PixelProfile {
    
    //Enum name for this profile
    String self;    

    //List of points this class contains
    public ArrayList<PixelPoint> points = new ArrayList<>();;

    public PixelProfile(ArrayList<PixelPoint> points, String self){
        this.points.addAll(points);
        this.self = self;
    }

    /**
     * This method takes in a list of PixelProfiles and returns the name of the closest match
     * @param otherProfiles
     * @return
     */
    public String match(PixelProfile[] otherProfiles){
        
        //Contains the name of the closest path
        String closestPath = self;

        //Contains the certainty of the match
        double closestPathCertainty = 0;

        //Contains the lowest difference between self and another profile
        double lowestDifference = compareTo(otherProfiles[0])[0];
        closestPath = otherProfiles[0].self;

        //Contains the raw difference and certainty of a profile comparation
        double[] difference;

        System.out.println("Matching possible paths to current profile:");
        System.out.println("Summary (difference, certainty):");

        //For every profile in otherProfiles, compare to self and set closest match
        for(int i = 1; i < otherProfiles.length; i++){
            difference = compareTo(otherProfiles[i]);
            if(difference[0] < lowestDifference){
                lowestDifference = difference[0];
                closestPath = otherProfiles[i].self;
                closestPathCertainty = difference[1];
            }

            System.out.println(otherProfiles[i].self+": "+difference[0]+", "+difference[1]);  
        }

        //Print best match
        System.out.println("\nBest Match: "+closestPath+" difference: "+lowestDifference+" certainty: "+closestPathCertainty * 100+"%");

        return closestPath;
    }

    /**
     * This method does the heavy lifting inside of the match() method.
     * it takes in a single profile and returns the difference between them and the certainty of its result.
     * @param profile
     * @return
     */
    private double[] compareTo(PixelProfile profile){
        
        //Contains the two values to return
        double differenceSum = 0;
        double certaintyScore = 0;

        //Holds the result of the PixelPoint.getDifference() method
        double[] differenceProfile; 

        //Holds the length of the other profile
        int otherProfileLength = profile.points.size();

        //Holds how many points have been compared
        int numComparisions = 0;

        //Holds an unsorted/sorted list of point differences
        ArrayList<Double> pointListDif = new ArrayList<Double>();

        //For each point in this profile, compare to every point in that profile and get differences
        for(PixelPoint thisPoint : points){
            for(PixelPoint thatPoint : profile.points){
                differenceProfile = thisPoint.getDifference(thatPoint);
                pointListDif.add(differenceProfile[0]/(differenceProfile[1] + 1));
                certaintyScore += differenceProfile[1];
                numComparisions++;
            }
        }

        //Sort the point differences by size
        Collections.sort(pointListDif);

        //Get the length of the profile with the least points
        int shortestLength = points.size() < otherProfileLength ? points.size() : otherProfileLength;


        //Only collect the values for points that are most likely to corrospond
        for(int i = 0; i < shortestLength; i++){
            differenceSum += pointListDif.get(i);
        }

        //divides the certainty score by the number of points tested (*3 for the amount of data in each point)
        certaintyScore /= numComparisions * 3;

        //divides the calculated difference by the amount of used points compared (offsets the "less points = less difference" problem)
        differenceSum /= shortestLength;

        return new double[]{differenceSum, certaintyScore};
    }
}
