/*
 * Author: Chris Campbell - www.iforce2d.net
 *
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package de.hfkbremen.robots.challenge;

import de.hfkbremen.robots.challenge.FUD.RobotBodyFUD;
import java.util.ArrayList;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import processing.core.PApplet;
import processing.core.PGraphics;

import static de.hfkbremen.robots.challenge.Constants.DEGTORAD;
import static de.hfkbremen.robots.challenge.Constants.RADTODEG;

public class Robot {

    /* 
     * FEATURE REQUEST
     *
     * change sensor angle, radius 
     *
     */
    private final Body mBody;
    private final ArrayList<Tire> mTires = new ArrayList<Tire>();
    private final RevoluteJoint flJoint;
    private final RevoluteJoint frJoint;
    private float mSteeringAngle;
    private final boolean DISCRET_STEERING = false;
    private final ArrayList<Sensor> mSensors = new ArrayList<Sensor>();
    private final World mWorld;
    private final Environment mEnvironment;

    final static float SCALE_ROBOT = 2;
    final static int MAX_NUM_OF_SENSORS = 5;
    public final float maxForwardSpeed = 50;
    public final float maxBackwardSpeed = -20;
    public final float maxSensorRange = 50.f;
    public final float minSensorRange = 5.f;
    public final float maxSteeringAngleDeg = 35.f;
    public final float maxSteeringAngle = maxSteeringAngleDeg * DEGTORAD;

    /**
     * create an instance of the a robot in a specific environment.
     *
     * @param pEnvironment
     */
    public Robot(Environment pEnvironment) {

        mEnvironment = pEnvironment;
        mWorld = pEnvironment.world();

        //create car body
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyType.DYNAMIC;
        mBody = mWorld.createBody(bodyDef);
        mBody.setAngularDamping(3);

        Vec2[] vertices = new Vec2[8];
        vertices[0] = new Vec2(1.5f, 0f);
        vertices[1] = new Vec2(3f, 2.5f);
        vertices[2] = new Vec2(2.8f, 5.5f);
        vertices[3] = new Vec2(1f, 10f);
        vertices[4] = new Vec2(-1f, 10f);
        vertices[5] = new Vec2(-2.8f, 5.5f);
        vertices[6] = new Vec2(-3f, 2.5f);
        vertices[7] = new Vec2(-1.5f, 0f);

        /* modify shape */
        for (Vec2 vertice : vertices) {
            vertice.addLocal(0, -5);
            vertice.mulLocal(SCALE_ROBOT);
        }

        PolygonShape polygonShape = new PolygonShape();
        polygonShape.set(vertices, 8);
        Fixture fixture = mBody.createFixture(polygonShape, 0.1f);
        fixture.setUserData(new RobotBodyFUD());

        //prepare common joint parameters
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = mBody;
        jointDef.enableLimit = true;
        jointDef.lowerAngle = 0;
        jointDef.upperAngle = 0;
        jointDef.localAnchorB.setZero();//center of tire

        float backTireMaxDriveForce = 150;
        float frontTireMaxDriveForce = 250;
        float backTireMaxLateralImpulse = 8.5f * 4;
        float frontTireMaxLateralImpulse = 7.5f * 4;

        //back left tire
        Tire tire = new Tire(mWorld);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire.body();
        jointDef.localAnchorA.set(-3, 0.75f - 5);
        jointDef.localAnchorA.mulLocal(SCALE_ROBOT);
        mWorld.createJoint(jointDef);
        mTires.add(tire);

        //back right tire
        tire = new Tire(mWorld);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire.body();
        jointDef.localAnchorA.set(3, 0.75f - 5);
        jointDef.localAnchorA.mulLocal(SCALE_ROBOT);
        mWorld.createJoint(jointDef);
        mTires.add(tire);

        //front left tire
        tire = new Tire(mWorld);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire.body();
        jointDef.localAnchorA.set(-3, 8.5f - 5);
        jointDef.localAnchorA.mulLocal(SCALE_ROBOT);
        flJoint = (RevoluteJoint) mWorld.createJoint(jointDef);
        mTires.add(tire);

        //front right tire
        tire = new Tire(mWorld);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire.body();
        jointDef.localAnchorA.set(3, 8.5f - 5);
        jointDef.localAnchorA.mulLocal(SCALE_ROBOT);
        frJoint = (RevoluteJoint) mWorld.createJoint(jointDef);
        mTires.add(tire);

        mBody.setUserData(new BodyUserData(this, BodyUserData.ShapeType.ROBOT));
    }

    public ArrayList<Sensor> sensors() {
        return mSensors;
    }

    /**
     * DO NOT USE. return an instance of the box2d body
     *
     * @return
     */
    public Body body() {
        return mBody;
    }

    /**
     * DO NOT USE. checks if the tested body is one of the robot s bodies.
     *
     * @param pBody body to test against
     * @return
     */
    public boolean isBody(Body pBody) {
        if (pBody == body()) {
            return true;
        }
        for (Tire mTire : mTires) {
            if (pBody == mTire.body()) {
                return true;
            }
        }
        return false;
    }

    /**
     * DO NOT CHANGE. current position of the robot in world coordinates.
     *
     * @return
     */
    public Vec2 position() {
        return body().getPosition();
    }

    /**
     * adds a sensor to the robot and returns the instance of the new sensor.
     * number and radius of sensors are limitied.
     *
     * @param pAngle angle in relation to the forward direction of the robot, in
     * radians
     * @param pRadius distance from the robots center
     * @return
     */
    public Sensor addSensor(final float pAngle, final float pRadius) {
        float sensorRange = pRadius;
        if (sensorRange > maxSensorRange) {
            System.out.println("Sensor range exceeds maximum allowed range. Clipped range to " + maxSensorRange + ".");
            sensorRange = maxSensorRange;
        } else if (sensorRange < minSensorRange) {
            System.out.println("Sensor range is too small. Clipped range to " + minSensorRange + ".");
            sensorRange = minSensorRange;
        }
        Sensor mSensor = new Sensor(mWorld, this, pAngle, pRadius);
        mSensors.add(mSensor);
        if (mSensors.size() > MAX_NUM_OF_SENSORS) {
            System.out.println("Too many sensors! You tried to add " + mSensors.size()
                               + " sensors, but the limit is " + MAX_NUM_OF_SENSORS + "!");
            System.exit(0);
        }

        return mSensor;
    }

    /**
     * this method is called automatically by the environment each frame.
     * implement the robots *thinking* in here.
     *
     * @param pDelta
     */
    public void update(final float pDelta) {
    }

    /**
     * set the steering angle of the robot. the desired angle is not necessarily
     * achieved immediately.
     *
     * @param pAngle steering angle in relation to the robots forward direction,
     * in radians
     */
    public void steer(float pAngle) {
        pAngle = PApplet.min(maxSteeringAngle, PApplet.max(-maxSteeringAngle, pAngle));
        mSteeringAngle = pAngle;
    }

    /**
     * set the steering angle of the robot. the desired angle is not necessarily
     * achieved immediately.
     *
     * @param pAngleInDegrees steering angle in relation to the robots forward
     * direction, in degrees
     */
    public void steerDeg(float pAngleInDegrees) {
        steer(pAngleInDegrees * DEGTORAD);
    }

    /**
     * returns the current, desired steering angle. the return value of this
     * method does not necessarily match the actual steering angle. see
     * 'steer(float pAngle)'
     *
     * @return steering angle in relation to the robots forward direction, in
     * radians
     */
    public float steer() {
        return mSteeringAngle;
    }

    /**
     * returns the current, desired steering angle. the return value of this
     * method does not necessarily match the actual steering angle. see
     * 'steer(float pAngle)'
     *
     * @return steering angle in relation to the robots forward direction, in
     * degrees
     */
    public float steerDeg() {
        return mSteeringAngle * RADTODEG;
    }

    /**
     * DO NOT CALL THIS METHOD.
     */
    final void _update() {
        for (Tire m_tire : mTires) {
            m_tire.updateFriction();
        }
        for (Tire m_tire : mTires) {
            m_tire.updateDrive(mMotorState);
        }

        update_steering();

        /* update sensors */
        for (Sensor mSensor : mSensors) {
            mSensor.update();
        }

        /* call user-defined update */
        update(mEnvironment.delta_time());
    }

    private void update_steering() {
        float turnSpeedPerSec = 160 * DEGTORAD;//from lock to lock in 0.5 sec
        float turnPerTimeStep = turnSpeedPerSec / 60.0f;
        float desiredAngle = 0;

        if (DISCRET_STEERING) {
            final float mLockAngle = 35 * DEGTORAD;
            switch (mSteeringState) {
                case ROBOT_STEERING_LEFT:
                    desiredAngle = mLockAngle;
                    break;
                case ROBOT_STEERING_RIGHT:
                    desiredAngle = -mLockAngle;
                    break;
                case ROBOT_STEERING_CENTER:
                    desiredAngle = 0;
                    break;
            }
        } else {
            desiredAngle = mSteeringAngle;
            if (mSteeringAngle > 0) {
                mSteeringState = ROBOT_STEERING_LEFT;
            } else if (mSteeringAngle < 0) {
                mSteeringState = ROBOT_STEERING_RIGHT;
            } else {
                mSteeringState = ROBOT_STEERING_CENTER;
            }
        }

        float angleNow = flJoint.getJointAngle();
        float angleToTurn = desiredAngle - angleNow;
        angleToTurn = MathUtils.clamp(angleToTurn, -turnPerTimeStep, turnPerTimeStep);
        float newAngle = angleNow + angleToTurn;
        flJoint.setLimits(newAngle, newAngle);
        frJoint.setLimits(newAngle, newAngle);
    }

    /**
     * return the speed of the robot. the return value of this method does not
     * necessarily match the actual speed. see 'speed(float pSpeed)'
     *
     * @return
     */
    public float speed() {
        return mTires.get(0).speed();
    }

    /**
     * set the speed of the robot. the desired speedis not necessarily achieved
     * immediately.
     *
     * @param pSpeed desired robots speed
     */
    public void speed(float pSpeed) {
        for (Tire m_tire : mTires) {
            m_tire.speed(pSpeed);
        }
    }

    /**
     * override this methods to draw into the robots local space.
     *
     * @param g use this PGraphics to send drawing commands to like e.g.
     * 'g.line(0, 5, 10, 20);'
     */
    public void draw(PGraphics g) {
    }

    public static final int ROBOT_MOTOR_FORWARD = 0;
    public static final int ROBOT_MOTOR_STOP = 1;
    public static final int ROBOT_MOTOR_BACKWARD = 2;

    private int mMotorState = ROBOT_MOTOR_STOP;

    private void motor(int pDirection) {
        mMotorState = pDirection;
    }

    public static final int ROBOT_STEERING_LEFT = 0;
    public static final int ROBOT_STEERING_CENTER = 1;
    public static final int ROBOT_STEERING_RIGHT = 2;

    private int mSteeringState = ROBOT_MOTOR_STOP;

    private void steering(int pDirection) {
        mSteeringState = pDirection;
    }
}
