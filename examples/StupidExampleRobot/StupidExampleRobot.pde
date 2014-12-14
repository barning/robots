import shiffman.box2d.*;
import de.hfkbremen.robots.challenge.*;

Environment mEnvironment;
MyRobot mRobot;
float mScale = 5;

void setup() {
  size(1024, 768);
  mEnvironment = new Environment(this, Environment.MAP_BAELLEBAD);

  mRobot = new MyRobot(mEnvironment);
  mEnvironment.add(mRobot);
}

void draw() {
  background(255);
  mEnvironment.update();
  mEnvironment.draw(g, mScale, mRobot.position());
}

class MyRobot extends Robot {

  final Sensor frontSensor;
  final Sensor backSensor;
  boolean obstacleFront = false;

  MyRobot(Environment pEnvironment) {
    super(pEnvironment);
    frontSensor = addSensor(0, 50.0f);
    backSensor  = addSensor(PI, 50.0f);
  }

  void update() {
    /* steer robot and controll its motor */
    if (frontSensor.triggered()) 
    {
      obstacleFront = true;
    } else if (backSensor.triggered())
    {
      obstacleFront = false;
    }
    if (obstacleFront)
      speed(maxBackwardSpeed);
    else
      speed(maxForwardSpeed);
  }
}


