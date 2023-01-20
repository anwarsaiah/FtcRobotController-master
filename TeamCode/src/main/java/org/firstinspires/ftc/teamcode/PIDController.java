package org.firstinspires.ftc.teamcode;

public class PIDController {
    public double p, i, d, input, output, rest, prev;
    public PIDController(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
        this.prev = input;
    }
    public void calculate(){
        this.output = (rest-input)*p+d*(input-prev);
        prev = input;
    }
}
