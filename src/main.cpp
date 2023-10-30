/*
* Simple FOC Tuning: ESP32, AS5600, DRV8313, BLDC 2804.
*
* This example helps you tune the PID gains of the motor. It uses the
* Commander library to allow you to change the target value and the PID
* parameters via the serial monitor. The following commands are available:
* - Send ? to see the node list;
* - Send L1 to turn the logs on and L0 to turn it off;
* - Send T<value> to set the target value (float);
* - Send [p/i/d]<value> to set the velocity gains (float);
* - Send [P/I/D]<value> to set the angle gains (float);
* - Send C<type> to set the control type to angle (`A`), velocity (`V`) or torque (`T`);
*/

#include <Arduino.h>
#include <SimpleFOC.h>

#define LOG_PID 1
#define LOG_MOTOR 1

#define MOTOR_POLE_PAIRS 7
#define MOTOR_IN_1 17
#define MOTOR_IN_2 16
#define MOTOR_IN_3 4
#define MOTOR_EN 18


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_IN_1, MOTOR_IN_2, MOTOR_IN_3, MOTOR_EN);

// Used to set log interval
static unsigned long last_time = 0;
// Target value for the motor
float target = 0;
// Whether to show logs
bool show_logs = false;

// User communication functions
Commander command = Commander(Serial);
void doLogs(char *cmd)
{
    show_logs = !show_logs;
    digitalWrite(LED_BUILTIN, HIGH ? show_logs : LOW);
};
void doTarget(char *cmd)
{
    command.scalar(&target, cmd);
    Serial.print("Target: ");
    Serial.println(target);
}
void doControlType(char *cmd)
{
    target = 0;
    Serial.print("Control type: ");
    if (cmd[0] == 'A')
    {
        motor.controller = MotionControlType::angle;
        Serial.print("Angle");
    }
    else if (cmd[0] == 'V')
    {
        motor.controller = MotionControlType::velocity;
        Serial.print("Velocity");
    }
    else if (cmd[0] == 'T')
    {
        motor.controller = MotionControlType::torque;
        Serial.print("Torque");
    }
    else
    {
        Serial.print("Unknown");
    }
    Serial.println();
}
void doVelocityP(char *cmd)
{
    motor.PID_velocity.P = atof(cmd);
    Serial.print("Velocity P gain: ");
    Serial.println(motor.PID_velocity.P);
};
void doVelocityI(char *cmd)
{
    motor.PID_velocity.I = atof(cmd);
    Serial.print("Velocity I gain: ");
    Serial.println(motor.PID_velocity.I);
};
void doVelocityD(char *cmd)
{
    motor.PID_velocity.D = atof(cmd);
    Serial.print("Velocity D gain: ");
    Serial.println(motor.PID_velocity.D);
};
void doAngleP(char *cmd)
{
    motor.P_angle.P = atof(cmd);
    Serial.print("Angle P gain: ");
    Serial.println(motor.P_angle.P);
};
void doAngleI(char *cmd)
{
    motor.P_angle.I = atof(cmd);
    Serial.print("Angle I gain: ");
    Serial.println(motor.P_angle.I);
};
void doAngleD(char *cmd)
{
    motor.P_angle.D = atof(cmd);
    Serial.print("Angle D gain: ");
    Serial.println(motor.P_angle.D);
};

void setup()
{

    pinMode(LED_BUILTIN, OUTPUT);

    // initialise magnetic sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 24;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);

    // maximal voltage to be set to the motor
    motor.voltage_limit = 6;
    // maximal velocity of the position control
    motor.velocity_limit = 20;

    // choose FOC modulation (optional)
    // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // set motion control loop to be used
    motor.controller = MotionControlType::angle;

    // velocity controller parameters
    motor.PID_velocity.P = 0.15f;
    motor.PID_velocity.I = 2.0f;
    motor.PID_velocity.D = 0.001;
    motor.LPF_velocity.Tf = 0.01f;

    // angle controller parameters
    motor.P_angle.P = 20.0f;
    motor.P_angle.I = 0.0f;
    motor.P_angle.D = 0.0f;
    motor.LPF_angle.Tf = 0.01f;

    // Manually set based on previous sensor/motor alignment
    motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 2.63f;

    // use monitoring with serial
    Serial.begin(115200);
    // comment out if not needed
    motor.useMonitoring(Serial);

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // add new commands
    command.add('T', doTarget, "Target");
    command.add('C', doControlType, "Control type");
    command.add('L', doLogs, "Logs on/off");
    command.add('p', doVelocityP, "Velocity P gain");
    command.add('i', doVelocityI, "Velocity I gain");
    command.add('d', doVelocityD, "Velocity D gain");
    command.add('P', doAngleP, "Angle P gain");
    command.add('I', doAngleI, "Angle I gain");
    command.add('D', doAngleD, "Angle D gain");

    Serial.println(F("Commander listening"));
    Serial.println(F(" - Send ? to see the node list..."));
    Serial.println(F(" - Send L1 to turn the logs on and L0 to turn it off"));
    Serial.println(F(" - Send T<value> to set the target value (float)"));
    Serial.println(F(" - Send [p/i/d]<value> to set the velocity gains (float)"));
    Serial.println(F(" - Send [P/I/D]<value> to set the angle gains (float)"));
    Serial.println(F(" - Send C<type> to set the control type to angle (`A`), velocity (`V`) or torque (`T`)"));

    _delay(1000);
}

void loop()
{
    motor.loopFOC();
    motor.move(target);

    // user communication
    command.run();
    if (show_logs && (millis() - last_time > 100.0))
    {
        last_time = millis();
#if LOG_PID
        Serial.print("Vel. P: ");
        Serial.print(motor.PID_velocity.P);
        Serial.print("\tVel. I: ");
        Serial.print(motor.PID_velocity.I);
        Serial.print("\tVel. D: ");
        Serial.print(motor.PID_velocity.D);
        Serial.print("\tAng. P: ");
        Serial.print(motor.P_angle.P);
        Serial.print("\tAng. I: ");
        Serial.print(motor.P_angle.I);
        Serial.print("\tAng. D: ");
        Serial.print(motor.P_angle.D);
#endif
#if LOG_MOTOR
        Serial.print("\tType: ");
        Serial.print(motor.controller);
        Serial.print("\tTarget: ");
        Serial.print(target);
        Serial.print("\tAngle: ");
        Serial.print(sensor.getAngle());
        Serial.print("\tVelocity: ");
        Serial.print(sensor.getVelocity());
#endif
#if LOG_PID || LOG_MOTOR
        Serial.println();
#endif
    }
}
