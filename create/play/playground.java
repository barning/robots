package play;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

/**
 *
 *
 * Created by felixkroll on 11.12.14.
 */
public class playground extends PApplet{

    Environment mEnvironment;
    MyRobot mRobot;
    float mScale = 5;

    public void setup() {
        size(1024, 768);
        mEnvironment = new Environment(this, Environment.MAP_BAELLEBAD);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);
    }

    public void draw() {
        background(255);
        mEnvironment.update();
        mEnvironment.draw(g, mScale, mRobot.position());
    }

    class MyRobot extends Robot {

        private final float STANDING = 0.5f;
        //Roboter speichert Situationen und spielt diese ab
        private final HashMap<Integer, float[]> actions = new HashMap<>();
        //Zweiter Parameter ist ein Array der den Stati der Sensoren teilt und die steeringAngles sowie den speed,
        // ob Rückwärts oder Vorwärts

        private final Sensor mSensor_front;
        private final Sensor mSensor_front_left;
        private final Sensor mSensor_front_right;
        private final Sensor mSensor_back_left;
        private final Sensor mSensor_back_right;

        private Vec2 old_position;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensor_front = addSensor(0, 40.0f);
            mSensor_front_left = addSensor(- PI/4 , 30.0f);
            mSensor_front_right = addSensor(PI/4 , 30.0f);
            mSensor_back_left = addSensor(- 3*PI/4, 20.0f);
            mSensor_back_right = addSensor(3*PI/4, 20.0f);
        }

        public void update() {
            Vec2 position = this.position();

            /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        steer(steer() + 0.05f);
                        break;
                    case 'd':
                        steer(steer() - 0.05f);
                        break;
                    case 'w':
                        speed(maxForwardSpeed);
                        break;
                    case 's':
                        speed(0);
                        steer(0);
                        break;
                    case 'x':
                        speed(maxBackwardSpeed);
                        break;
                    case '-':
                        mScale -= 0.5f;
                        mScale = max(1, mScale);
                        break;
                    case '+':
                        mScale += 0.5f;
                        mScale = min(5, mScale);
                        break;
                }
            }

            old_position = position;
        }

        public void draw(PGraphics g) {
        /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);
            if (!mSensor_front.triggered()) {
                rect(1.5f, -2, 2, 2);
            }
            if (!mSensor_front_left.triggered()) {
                ellipse(-1.5f, -2, 2, 2);
            }
        }

        private boolean isStanding(final Vec2 old_position, final Vec2 current_position) {
            if (old_position != null && current_position != null) {

                Vec2 velocity = current_position.sub(old_position);
                return velocity.length() <= STANDING;
            }
            return false;
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{playground.class.getName()});
    }
}
