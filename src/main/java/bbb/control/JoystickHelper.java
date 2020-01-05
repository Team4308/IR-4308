package bbb.control;

import bbb.math.bbbVector2;
import frc.robot.Constants;

public class JoystickHelper{
    public static bbbVector2 DeadbandRadial(bbbVector2 stickInput){
        double d2 = stickInput.x*stickInput.x + stickInput.y*stickInput.y;

        if(d2 < Constants.Input.kInputDeadband*Constants.Input.kInputDeadband || d2 < 0){
            return new bbbVector2();
        } else {
            double d = Math.sqrt(d2);

            double nx = stickInput.x/d;
            double ny = stickInput.y/d;

            d = (d-Constants.Input.kInputDeadband)/(1-Constants.Input.kInputDeadband);

            d = Math.min(d, 1);

            return new bbbVector2(nx*d, ny*d);
        }
    }

    public static bbbVector2 DeadbandNormal(bbbVector2 stickInput){
        bbbVector2 newStickInput = new bbbVector2(stickInput.x, stickInput.y);
        if(Math.abs(newStickInput.x) < Constants.Input.kInputDeadband){
            newStickInput.x = 0.0;
        }
        if(Math.abs(newStickInput.y) < Constants.Input.kInputDeadband){
            newStickInput.y = 0.0;
        }
        return newStickInput;
    }

    public static boolean isStickCentered(bbbVector2 stickInput){
        if(Math.abs(stickInput.x) < Constants.Input.kInputDeadband && Math.abs(stickInput.y) < Constants.Input.kInputDeadband){
            return true;
        } else {
            return false;
        }
    }

    public static bbbVector2 scaleStick(bbbVector2 stickInput, double scale){
        bbbVector2 newStickInput = new bbbVector2(Math.signum(stickInput.x) * Math.abs(Math.pow(stickInput.x, scale)), Math.signum(stickInput.y) * Math.abs(Math.pow(stickInput.y, scale)));
        return newStickInput;
    }
}