package bbb.utils;

public class bbbDoubleUtils{
    public static double normalize(double d){
        if(d > 1.0){
            return 1.0;
        }
        if(d < -1.0){
            return -1.0;
        } else {
            return d;
        }
    }

    public static double clamp(double d, double min, double max){
        if(d > max){
            return max;
        }
        else if(d < min){
            return min;
        } else {
            return d;
        }
    }
}