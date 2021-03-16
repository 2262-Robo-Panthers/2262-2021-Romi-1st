package frc.robot;

public class Util
{
    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
    public static float clamp(float val, float clamp) {
        return Math.max(-clamp, Math.min(clamp, val));
    }
}
//Expiremental
class PIDCoefficients
{
    double p,i,d;
    PIDCoefficients(double P, double I, double D)
    {
        p = P;
        i = I;
        d = D;
    }
}