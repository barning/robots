package SimplestRobotExample;
import processing.core.PApplet;
import shiffman.box2d.*;
import de.hfkbremen.robots.challenge.*;

/**
 * SimplestRobotExample
 *
 * Created by felixkroll on 11.12.14.
 */
public class example1 extends PApplet {

    Environment mEnvironment;
    MyRobot mRobot;

    public void setup() {
        size(1024, 768);
        mEnvironment = new Environment(this, Environment.MAP_SIMPLE);
        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);
    }

    public void draw() {
        background(255);
        mEnvironment.update();
        mEnvironment.draw(g);
    }

    class MyRobot extends Robot {

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
        }

        public void update() {
        /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        steer(steer() + 0.1f);
                        break;
                    case 'd':
                        steer(steer() - 0.1f);
                        break;
                    case 'w':
                        speed(50);
                        break;
                    case 's':
                        speed(0);
                        break;
                }
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{example1.class.getName()});
    }
}
