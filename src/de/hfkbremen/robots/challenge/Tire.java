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

import de.hfkbremen.robots.challenge.BodyUserData.ShapeType;
import de.hfkbremen.robots.challenge.FUD.GroundAreaFUD;
import de.hfkbremen.robots.challenge.FUD.RobotTireFUD;
import java.util.ArrayList;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import processing.core.PApplet;

import static de.hfkbremen.robots.challenge.Robot.*;

public class Tire {

    private final Body m_body;
    private float m_maxForwardSpeed;
    private float m_maxBackwardSpeed;
    private float m_maxDriveForce;
    private float m_maxLateralImpulse;
    private final ArrayList<GroundAreaFUD> m_groundAreas = new ArrayList<GroundAreaFUD>();
    private float m_currentTraction;

    private final boolean DISCRET_DRIVING = false;

    private float mSpeed = 0;
    private final float mTractionModifier = 0.1f;
    private final float mTurningTorque = 15;

    public Tire(World world) {
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyType.DYNAMIC;
        m_body = world.createBody(bodyDef);

        PolygonShape mPolygonShape = new PolygonShape();
        mPolygonShape.setAsBox(0.5f * Robot.SCALE_ROBOT, 1.25f * Robot.SCALE_ROBOT);
        Fixture mFixture = m_body.createFixture(mPolygonShape, 1);//shape, density
        mFixture.setUserData(new RobotTireFUD());

        m_body.setUserData(new BodyUserData(this, ShapeType.TIRE));

        m_currentTraction = 1;
    }

    Body body() {
        return m_body;
    }

    public void setCharacteristics(float maxForwardSpeed, float maxBackwardSpeed, float maxDriveForce, float maxLateralImpulse) {
        m_maxForwardSpeed = maxForwardSpeed;
        m_maxBackwardSpeed = maxBackwardSpeed;
        m_maxDriveForce = maxDriveForce;
        m_maxLateralImpulse = maxLateralImpulse;
    }

    public void addGroundArea(GroundAreaFUD ga) {
        m_groundAreas.add(ga);
        updateTraction();
    }

    public void removeGroundArea(GroundAreaFUD ga) {
        m_groundAreas.remove(ga);
        updateTraction();
    }

    public void updateTraction() {
        if (m_groundAreas.isEmpty()) {
            m_currentTraction = 1;
        } else {
            //find area with highest traction
            m_currentTraction = 0;
            for (GroundAreaFUD ga : m_groundAreas) {
                if (ga.frictionModifier > m_currentTraction) {
                    m_currentTraction = ga.frictionModifier;
                }
            }
        }
    }

    public Vec2 getLateralVelocity() {
        Vec2 currentRightNormal = m_body.getWorldVector(new Vec2(1, 0));
        return currentRightNormal.mul(Vec2.dot(currentRightNormal, m_body.getLinearVelocity()));
    }

    public Vec2 getForwardVelocity() {
        Vec2 currentForwardNormal = m_body.getWorldVector(new Vec2(0, 1));
        return currentForwardNormal.mul(Vec2.dot(currentForwardNormal, m_body.getLinearVelocity()));
    }

    public void updateFriction() {
        //lateral linear velocity
        Vec2 mImpulse = getLateralVelocity().mul(m_body.getMass() * -1);
        if (mImpulse.length() > m_maxLateralImpulse) {
            mImpulse.mulLocal(m_maxLateralImpulse / mImpulse.length());
        }
        m_body.applyLinearImpulse(mImpulse.mul(m_currentTraction), m_body.getWorldCenter(), false);

        //angular velocity
        m_body.applyAngularImpulse(m_currentTraction * mTractionModifier * m_body.getInertia() * -m_body.getAngularVelocity());

        //forward linear velocity
        Vec2 currentForwardNormal = getForwardVelocity();
        float currentForwardSpeed = currentForwardNormal.normalize();
        float dragForceMagnitude = -2 * currentForwardSpeed;
        m_body.applyForce(currentForwardNormal.mul(m_currentTraction * dragForceMagnitude), m_body.getWorldCenter());
    }

    public void speed(float pSpeed) {
        pSpeed = PApplet.min(m_maxForwardSpeed, PApplet.max(m_maxBackwardSpeed, pSpeed));
        mSpeed = pSpeed;
    }

    public float speed() {
        return mSpeed;
    }

    public void updateDrive(int pMotorState) {

        //find desired speed
        float desiredSpeed = 0;
        if (DISCRET_DRIVING) {
            switch (pMotorState) {
                case ROBOT_MOTOR_FORWARD:
                    desiredSpeed = m_maxForwardSpeed;
                    break;
                case ROBOT_MOTOR_BACKWARD:
                    desiredSpeed = m_maxBackwardSpeed;
                    break;
                default:
                    return;//do nothing
            }
        } else {
            desiredSpeed = mSpeed;
        }

        //find current speed in forward direction
        Vec2 currentForwardNormal = m_body.getWorldVector(new Vec2(0, 1));
        float currentSpeed = Vec2.dot(getForwardVelocity(), currentForwardNormal);

        //apply necessary force
        float force = 0;
        if (desiredSpeed > currentSpeed) {
            force = m_maxDriveForce;
        } else if (desiredSpeed < currentSpeed) {
            force = -m_maxDriveForce;
        } else {
            return;
        }
        m_body.applyForce(currentForwardNormal.mul(m_currentTraction * force), m_body.getWorldCenter());
    }

    public void updateTurn(int pSteeringState) {
        /* TODO not sure about this one ... */
        float desiredTorque = 0;
        switch (pSteeringState) {
            case ROBOT_STEERING_LEFT:
                desiredTorque = mTurningTorque;
                break;
            case ROBOT_STEERING_RIGHT:
                desiredTorque = -mTurningTorque;
                break;
            default: ;//nothing
            }
        m_body.applyTorque(desiredTorque);
    }
}
