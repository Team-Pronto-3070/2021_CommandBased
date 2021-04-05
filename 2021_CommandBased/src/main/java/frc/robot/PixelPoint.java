package frc.robot;

/**
 * This class represents a point in the middle of a box with an x, y, and area value
 */

public class PixelPoint {
    
    //If a difference is less than the tolerance the value will be discarded (no test for match)
    private double pointTolerance;

    //Data fields
    final double pitch;
    final double yaw;
    final double area;

    //data fields in list form
    final double[] values;

    public PixelPoint(double pitch, double yaw, double area){
        this.pitch = pitch;
        this.yaw = yaw;
        this.area = area;

        this.pointTolerance = Double.MAX_VALUE;

        values = new double[3];
        values[0] = pitch;
        values[1] = yaw;
        values[2] = area;
    }

    public PixelPoint(double pitch, double yaw, double area, double pointTolerance){
        this(pitch, yaw, area);
        this.pointTolerance = pointTolerance; 
    }

    /**
     * This method returns the difference and the number of valid data points between two PixelPoints
     * @param point
     * @return
     */
    public double[] getDifference(PixelPoint point){
        
        //Sum of the difference
        double sum = 0;

        //Number of valid data points
        double validPoints = 3;

        //For each value in this.values, compare with the equivilant values in point.values and discard values outside tolerance
        for(int i = 0; i< values.length; i++){
            double difference = Math.abs(Math.abs(values[i]) - Math.abs(point.values[i]));
            if(difference < pointTolerance){
                sum += difference;
            }else{
                validPoints--;
                System.out.println("PixelPoint value difference out of tolerance: "+pointTolerance);
            }     
        }

        //Print number of valid points found
        if(validPoints == 0){
            System.out.println("No valid differences.");
        }else{
            System.out.println(validPoints+" valid differences found.");
        }
        
        return new double[]{sum, validPoints/3};
    }

    public String toString(){
        return "START POINT\nPITCH "+pitch+"\nYAW "+yaw+"\nAREA "+area+"\nEND POINT\n";
    }
}
