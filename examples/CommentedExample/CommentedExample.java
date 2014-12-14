package CommentedExample;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

/**
 * Created by niklasbarning on 14.12.14.
 */
public class CommentedExample extends PApplet {
    Environment mEnvironment;
    MyRobot mRobot;

    public void setup() {
        /**
         * specify window size.
         */
        size(1024, 768);

        /**
         * create an instance of the environment with one of the default maps.
         * custom maps can also be create by creating a clas that implements
         * 'EnvironmentMap'.
         */
        mEnvironment = new Environment(this, Environment.MAP_BAELLEBAD);

        /**
         * create an instance of a custom robot.
         */
        mRobot = new MyRobot(mEnvironment);
        /**
         * add the robot to the environment.
         */
        mEnvironment.add(mRobot);
    }

    public void draw() {
        background(255);

        /**
         * call this method every frame to advane the physical simulation. this
         * method also updates each robot.
         */
        mEnvironment.update();
        /**
         * returns the duration of the last frame in seconds. this value might
         * be fixed to a constant value for coherent simulations.
         */
        float mDeltaTime = mEnvironment.delta_time();
        /**
         * simple way to draw all entities like obstacle, robots, and sensors.
         * there are multiple ways to draw the environment like:
         *
         * 1. draw(PGraphics, float, Vec2) draw with zoom factor and origin
         *
         * 2. beginDraw(PGraphics) + draw something additional +
         * endDraw(PGraphics)
         *
         * 3. draw(PGraphics): simply draw the environment
         */
        mEnvironment.draw(g);
    }

    class MyRobot extends Robot {

        final Sensor mSensor;

        float mAngle;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            /**
             * add a sensor to the robot. specify the sensors position relative
             * to the robot by supplying an angle ( in radians ) from the robots
             * forward direction and a radius from the robot s center.
             */
            mSensor = addSensor(PI / 4, 20.0f);

            /**
             * returns a list of the robot s sensors
             */
            ArrayList<Sensor> mSensors = sensors();
        }

        /**
         * override this method to update robot. this method is automatically
         * called by all variants of Environment.update()
         *
         * @param pDeltaTime
         */
        public void update(float pDeltaTime) {
            /**
             * returns robot s position.
             */
            Vec2 mPosition = position();

            /**
             * set angle of sensor in radians
             */
            mAngle += pDeltaTime;
            mSensor.angle(sin(mAngle * 2) * 0.5f);

    /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        /**
                         * get the current steering angle ( in radians )
                         */
                        final float mSteeringAngle = steer();
                        /**
                         * set the desired steering angle
                         */
                        steer(maxSteeringAngle + 0.05f);
                        break;
                    case 'd':
                        steer(steer() - 0.05f);
                        break;
                    case 'w':
                        /**
                         * set the desired robot speed
                         */
                        speed(10);
                        break;
                    case 's':
                        speed(0);
                        steer(0);
                        break;
                    case 'x':
                        speed(-10);
                        break;
                }
            }
        }

        /**
         * override to draw in robot s local coordinate space i.e. anything
         * drawn in here move and rotates with the robot. this method is
         * automatically called by all Environment.draw() variants.
         *
         * @param g
         */
        public void draw(PGraphics g) {
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);
            /**
             * query if the sensor is triggered
             */
            if (!mSensor.triggered()) {
                rect(1.5f, -2, 2, 2);
                /**
                 * returns the world postion of the detected obstacle.
                 */
                Vec2 mObstaclePosition = mSensor.obstacle();
                /**
                 * returns the distance to the current obstacle.
                 */
                float mObstacleDistance = mSensor.obstacleDistance();
            }
        }
    }
    public static void main(String[] args) {
        PApplet.main(new String[]{CommentedExample.class.getName()});
    }

}
