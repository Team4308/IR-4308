package bbb.control;

import bbb.math.bbbVector2;
import bbb.utils.bbbDoubleUtils;
import frc.robot.Constants;

public class JoystickHelper {
    public static bbbVector2 SlopedScaledAxialDeadzone(bbbVector2 stickInput) {
        double deadzoneX = Constants.Config.Input.kInputDeadband * Math.abs(stickInput.y);
        double deadzoneY = Constants.Config.Input.kInputDeadband * Math.abs(stickInput.x);

        bbbVector2 result = new bbbVector2(0.0, 0.0);
        bbbVector2 sign = new bbbVector2(Math.signum(stickInput.x), Math.signum(stickInput.y));

        if (Math.abs(stickInput.x) > deadzoneX) {
            result.x = sign.x * bbbDoubleUtils.mapRangeNew(Math.abs(stickInput.x), deadzoneX, 1, 0, 1);
        }
        if (Math.abs(stickInput.y) > deadzoneY) {
            result.y = sign.y * bbbDoubleUtils.mapRangeNew(Math.abs(stickInput.y), deadzoneY, 1, 0, 1);
        }
        return result;
    }

    public static bbbVector2 ScaledRadialDeadzone(bbbVector2 stickInput) {
        double inputMagnitude = stickInput.magnitude();
        double deadzone = Constants.Config.Input.kInputDeadband;
        if (inputMagnitude < Constants.Config.Input.kInputDeadband) {
            return new bbbVector2(0.0, 0.0);
        } else {
            double legalRange = 1.0 - deadzone;
            double normalizedMag = Math.min(1.0, (inputMagnitude - deadzone) / legalRange);
            double scale = normalizedMag / inputMagnitude;
            return new bbbVector2(stickInput.normalizeNew().x * scale, stickInput.normalizeNew().y * scale);
        }
    }

    public static bbbVector2 HybridDeadzone(bbbVector2 stickInput) {
        double inputMagnitude = stickInput.magnitude();
        if (inputMagnitude < Constants.Config.Input.kInputDeadband) {
            return new bbbVector2(0.0, 0.0);
        } else {
            bbbVector2 partialOutput = ScaledRadialDeadzone(stickInput);
            bbbVector2 finalOutput = SlopedScaledAxialDeadzone(partialOutput);
            return finalOutput;
        }
    }

    public static bbbVector2 AxialDeadzone(bbbVector2 stickInput) {
        bbbVector2 newStickInput = new bbbVector2(stickInput.x, stickInput.y);
        if (Math.abs(newStickInput.x) < Constants.Config.Input.kInputDeadband) {
            newStickInput.x = 0.0;
        }
        if (Math.abs(newStickInput.y) < Constants.Config.Input.kInputDeadband) {
            newStickInput.y = 0.0;
        }
        return newStickInput;
    }

    // USE THIS ONE FOR SPLIT STICK TANK DRIVE
    public static bbbVector2 ScaledAxialDeadzone(bbbVector2 stickInput) {
        bbbVector2 result = new bbbVector2(0.0, 0.0);
        bbbVector2 sign = new bbbVector2(Math.signum(stickInput.x), Math.signum(stickInput.y));

        if (Math.abs(stickInput.x) > Constants.Config.Input.kInputDeadband) {
            result.x = sign.x * bbbDoubleUtils.mapRangeNew(Math.abs(stickInput.x),
                    Constants.Config.Input.kInputDeadband, 1, 0, 1);
        }
        if (Math.abs(stickInput.y) > Constants.Config.Input.kInputDeadband) {
            result.y = sign.y * bbbDoubleUtils.mapRangeNew(Math.abs(stickInput.y),
                    Constants.Config.Input.kInputDeadband, 1, 0, 1);
        }

        return result;
    }

    public static boolean isStickCentered(bbbVector2 stickInput) {
        if (Math.abs(stickInput.x) < Constants.Config.Input.kInputDeadband
                && Math.abs(stickInput.y) < Constants.Config.Input.kInputDeadband) {
            return true;
        } else {
            return false;
        }
    }

    public static bbbVector2 scaleStick(bbbVector2 stickInput, double scale) {
        bbbVector2 newStickInput = new bbbVector2(Math.signum(stickInput.x) * Math.abs(Math.pow(stickInput.x, scale)),
                Math.signum(stickInput.y) * Math.abs(Math.pow(stickInput.y, scale)));
        return newStickInput;
    }

    public static bbbVector2 alternateScaleStick(bbbVector2 stickInput, double scale) {
        double mag = stickInput.magnitude();
        if (mag == 0) {
            return new bbbVector2();
        } else {
            bbbVector2 norm = new bbbVector2(stickInput.x / mag, stickInput.y / mag);
            return new bbbVector2(norm.x * Math.pow(mag, scale), norm.y * Math.pow(mag, scale));
        }
    }

    public static bbbVector2 clampStick(bbbVector2 stickInput) {
        return new bbbVector2(bbbDoubleUtils.clamp(stickInput.x, -1, 1), bbbDoubleUtils.clamp(stickInput.y, -1, 1));
    }
}