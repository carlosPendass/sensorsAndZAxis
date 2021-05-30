//This code contains all the necessary thing to get Sensors Data and move the elevator.
//Don't forget to change PID library PID.compute function to dont update with sampletime constant.
// it may be necessary to change some code in PIDs library cause we still have functions that depends on Sampletime variable.
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <WireSlave.h>
#include <config.h>

/////////////////////////////////////////////////////////////////////////////////////
//Global Variables for zAxis
double absolutPostition = 0; // It's an absolut position that we get from zAxisCalibration Function.
double position0, OutputPWM, Setpoint; //Those are used for the PID function, we can say that postion0 is equal to encoder.getCount()
int SampleTime = 1; //Really necessary cause it has a strech relation with PID constants
/////////////////////////////////////////////////////////////////////////////////////
//Globar Variables for ChessBoard
byte dirSensor[6][6][6]; // sensor addresses order by [pcb bar, square bar, sensor]
bool muxValues[6][6][6][6]; //sensor values store by [Mux, pcb bar, square bar, sensor]
bool orderedSensorValues[100][5]; //array ordered according to the format requiered by void sendSensorValues(). it is ordered by [square of the board, sensor id of the square]
bool previousSensorValues[100][5];

bool muxSquaresValues[6][6][6];
bool orderedSquaresValues[100];
bool orderedPreviousSquareValue[100];
/////////////////////////////////////////////////////////////////////////////////////
//Global Variables for i2C
char code[2];
int band = 0;

///////////////////////////////////////////////////////////////////////
// Function Declaration zAxis
void elevatorDirUp();
void elevatorDirDown();
void zAxisCalibration();
void pidPWM(double);
void elevatorGoMaxUp();
void elevatorGoMaxDown();
////////////////////////////////////////////////////////////////////////
//Function Declaration for Sensors
void sensorsDir();
int detectMove(int);
void readRawChessBoard();
void detectChessBoard();
void printSerial(int, int, int, int, int);
void sendSensorValues(bool[][AMOUNTOFSENSORS], int, int);
void sendI2CBools(bool[], int);
void sendI2CString(String);
/////////////////////////////////////////////////////////////////////////////
//Function declaration for I2C
void requestEvent();
void receiveEvent(int howMany);
//////////////////////////////////////////////////////////////////////////
ESP32Encoder encoder;
PID myPID(&position0, &OutputPWM, &Setpoint, PROPORTIONAL_CONSTANT, INTEGRAL_CONSTANT, DERIVATIVE_CONSTANT, DIRECT); //Position is the current position,OutputPwm is the PIDs output, Setpoint is the target position

void setup()
{
    Serial.begin(115200);
    ///////////////////////////////////////////////////////////////////////////////
    // zAxis Declaring in/outs and doing the Setup of Encoder, PWM, and PID.
    pinMode(control1, OUTPUT);
    pinMode(control2, OUTPUT);
    pinMode(PWM, OUTPUT);

    ESP32Encoder::useInternalWeakPullResistors = UP; // this line tells the ESP32 to use the internal PullUp resistor on the pins we are using
    encoder.attachFullQuad(encoderA, encoderB); // AttachFullQuad allow us to use two interruption per encoder and the full lenght resolution
    encoder.clearCount(); // We make sure to set zero the encoder position.

    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION); // Setup PWM                                  // PWM setup,
    ledcAttachPin(PWM, PWM_CHANNEL);

    myPID.SetSampleTime(SampleTime);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(PID_OUTPUTLIMITS * -1, PID_OUTPUTLIMITS);
    zAxisCalibration();

    pinMode(mux8_0, OUTPUT);
    pinMode(mux8_1, OUTPUT);
    pinMode(mux8_2, OUTPUT);

    pinMode(mux16_0, OUTPUT);
    pinMode(mux16_1, OUTPUT);
    pinMode(mux16_2, OUTPUT);
    pinMode(mux16_3, OUTPUT);

    pinMode(mux16Out_0, INPUT);
    pinMode(mux16Out_1, INPUT);
    pinMode(mux16Out_2, INPUT);
    pinMode(mux16Out_3, INPUT);
    pinMode(mux16Out_4, INPUT);

    sensorsDir();

    ////////////////////////////////////////////////////////////////////////////////
    ////Wire declaration for I2C COMS
    bool res = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
    if (!res) {
        Serial.println("I2C slave init failed");
        while (1)
            delay(100);
    }

    WireSlave.onRequest(requestEvent);
    WireSlave.onReceive(receiveEvent);

    Serial.printf("Slave joined I2C bus with addr #%d\n", I2C_SLAVE_ADDR);
}

void loop()
{
    detectMove(FULL);
    Serial.println("change");
    printSerial(0, FULL, 2, 3, 4);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//zAxis functions
void pidPWM(double setPosition)
{

    Setpoint = setPosition;

    bool limitFlag = LOW;
    unsigned long currentTime = 0;
    unsigned long previousTime = 0;
    const long timeforLimit = TIME_PID;

    unsigned long currentTimePIDTol = 0;
    unsigned long previousTimePIDTol = 0;
    const long timeforLimitPIDTol = TIME_PIDTol;

    if (Setpoint < absolutPostition && Setpoint > 0) // this condition forbids the PID to work if we are out of range.
    {
        while (limitFlag != HIGH) // This flag is HIGH when a couple of conditions are true, when we go out we stop computing data and PID is off.
        {

            currentTime = micros();

            if (currentTime - previousTime >= timeforLimit) // With this condition we overstep the computing time set by the library and choose our own rithm of computing data.
            {
                previousTime = currentTime;
                position0 = encoder.getCount();
                myPID.Compute();

                if (OutputPWM < 0) // With these two conditions we set he direction the DC motor is going depending on the pwm signus.
                {
                    elevatorDirDown();
                } else if (OutputPWM >= 0) {
                    elevatorDirUp();
                }

                ledcWrite(PWM_CHANNEL, abs(OutputPWM));
            }

            currentTimePIDTol = millis();

            if (currentTimePIDTol - previousTimePIDTol >= timeforLimitPIDTol) // We ask for the error every TIME_PIDTol and if its on our range of PID_TOl we get out of the computing
            {
                previousTimePIDTol = currentTimePIDTol;
                if ((Setpoint - PID_TOL) <= encoder.getCount() && (Setpoint + PID_TOL) >= encoder.getCount()) {
                    limitFlag = HIGH;
                }
            }
        }

        ledcWrite(PWM_CHANNEL, 0);

    } else {
        Serial.println("POS ERROR");
    }
}

void zAxisCalibration()
{
    bool limitFlag = LOW;
    unsigned long currentTime = 0;
    unsigned long previousTime = 0;
    const long timeforLimit = TIME_LIMITS;

    elevatorDirDown();

    while (limitFlag != HIGH) // Stays on the While until the previousPos is equal to the currentPost for a periode of time
    {
        currentTime = millis();
        ledcWrite(PWM_CHANNEL, PWM_LIMITS);

        if (currentTime - previousTime >= timeforLimit) {
            previousTime = currentTime;

            if (encoder.getCount() == absolutPostition) {
                limitFlag = HIGH;
            }

            absolutPostition = encoder.getCount();
            ;
        }
    }

    ledcWrite(PWM_CHANNEL, 0);
    elevatorDirUp();
    limitFlag = LOW;
    encoder.clearCount();
    absolutPostition = 0;

    while (limitFlag != HIGH) // Now that we know we are on one side, we go to the other side witch counter on Zero and we the same dynamic of the previous while
    {
        currentTime = millis();
        ledcWrite(PWM_CHANNEL, PWM_LIMITS);

        if (currentTime - previousTime >= timeforLimit) {
            previousTime = currentTime;

            if (encoder.getCount() == absolutPostition) {
                limitFlag = HIGH;
            }
            absolutPostition = encoder.getCount();
        }
    }

    ledcWrite(PWM_CHANNEL, 0);
    limitFlag = LOW;
}

void elevatorDirUp()
{
    digitalWrite(control1, LOW);
    digitalWrite(control2, HIGH);
}

void elevatorDirDown()
{
    digitalWrite(control1, HIGH);
    digitalWrite(control2, LOW);
}

void elevatorGoMaxUp()
{
    pidPWM((absolutPostition - MEC_TOL));
}

void elevatorGoMaxDown()
{
    pidPWM(absolutPostition - absolutPostition + MEC_TOL);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//ChessBoard functions

int detectMove(int Selection)
{
    int OriginalPiecesonBoard = 0;
    int totalPiecesonBoard = 0;
    int amountOfChanges = 0;
    int previousAmountOfChanges = 0;
    unsigned long currentTime = 0;
    unsigned long previousTime = 0;
    unsigned long timeofRejection = CONSTOFREJECTION;
    bool changeFlag = LOW;
    detectChessBoard();
    if (Selection == REJECTION) {
        for (int i = 0; i < 100; i++) {
            orderedPreviousSquareValue[i] = orderedSquaresValues[i];
            if (orderedSquaresValues[i] == LOW) {
                OriginalPiecesonBoard++;
            }
        }

        while (changeFlag != HIGH) {
            detectChessBoard();
            for (int i = 0; i < 100; i++) {

                if (orderedSquaresValues[i] != orderedPreviousSquareValue[i]) {
                    amountOfChanges++;
                }
            }

            currentTime = millis();
            if (currentTime - previousTime >= timeofRejection) {
                previousTime = currentTime;
                if (amountOfChanges != 0 && amountOfChanges == previousAmountOfChanges && totalPiecesonBoard == OriginalPiecesonBoard) {
                    changeFlag = HIGH;
                }
                previousAmountOfChanges = amountOfChanges;
            }
            totalPiecesonBoard = 0;

            for (int i = 0; i < 100; i++) {
                orderedPreviousSquareValue[i] = orderedSquaresValues[i];
                if (orderedSquaresValues[i] == LOW) {
                    totalPiecesonBoard++;
                }
            }
        }
        return -1;
    } else if (Selection == FULL) {
        for (int i = 0; i < 100; i++) {
            for (int j = 0; j < 5; j++) {
                previousSensorValues[i][j] = orderedSensorValues[i][j];
            }
        }

        while (changeFlag != HIGH) {
            detectChessBoard();
            for (int i = 0; i < 100; i++) {
                for (int j = 0; j < 5; j++) {

                    if (orderedSensorValues[i][j] != previousSensorValues[i][j]) {
                        amountOfChanges++;
                        changeFlag = HIGH;
                    }
                }
            }

            for (int i = 0; i < 100; i++) {
                for (int j = 0; j < 5; j++) {
                    previousSensorValues[i][j] = orderedSensorValues[i][j];
                }
            }
        }
        return -1;
    } else if (Selection == SIMPLE) {
        for (int i = 0; i < 100; i++) {
            orderedPreviousSquareValue[i] = orderedSquaresValues[i];
        }

        while (changeFlag != HIGH) {
            detectChessBoard();
            for (int i = 0; i < 100; i++) {
                if (orderedSquaresValues[i] != orderedPreviousSquareValue[i]) {
                    amountOfChanges++;
                    changeFlag = HIGH;
                }
            }

            for (int i = 0; i < 100; i++) {
                for (int j = 0; j < 5; j++) {
                    orderedPreviousSquareValue[i] = orderedSquaresValues[i];
                }
            }
        }
    }

    return -1;
}

void readRawChessBoard()
{
    for (int k = 0; k < 4; k++) //it represents the sensor pcb bar connected according to the 16 bits mux.
    {
        for (int i = 0; i < 5; i++) 
        {
            for (int j = 0; j < 5; j++) 
            {
                digitalWrite(mux8_0, bitRead(dirSensor[k][i][j], 0));
                digitalWrite(mux8_1, bitRead(dirSensor[k][i][j], 1));
                digitalWrite(mux8_2, bitRead(dirSensor[k][i][j], 2));

                digitalWrite(mux16_0, bitRead(dirSensor[k][i][j], 4));
                digitalWrite(mux16_1, bitRead(dirSensor[k][i][j], 5));
                digitalWrite(mux16_2, bitRead(dirSensor[k][i][j], 6));
                digitalWrite(mux16_3, bitRead(dirSensor[k][i][j], 7));

                delayMicroseconds(timeBsensors); // This delay is to give time to switch the gates, if this is commented noisy data may be read.

                muxValues[0][k][i][j] = (digitalRead(mux16Out_0));
                muxValues[1][k][i][j] = (digitalRead(mux16Out_1));
                muxValues[2][k][i][j] = (digitalRead(mux16Out_2));
                muxValues[3][k][i][j] = (digitalRead(mux16Out_3));
                muxValues[4][k][i][j] = (digitalRead(mux16Out_4));

                if (j == 4) {
                    for (int z = 0; z < 5; z++)
                    {
                        if (muxValues[z][k][i][0] == 0 || muxValues[z][k][i][1] == 0 || muxValues[z][k][i][2] == 0 || muxValues[z][k][i][3] == 0 || muxValues[z][k][i][4] == 0) {
                            muxSquaresValues[z][k][i] = 0;
                        } else {
                            muxSquaresValues[z][k][i] = 1;
                        }
                    }
                }
            }
        }
    }
}

void detectChessBoard()
{
    readRawChessBoard();
    //Mux 0 and 4
    //first row
    //1 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i][j] = muxValues[4][3][i][j];
        }
        orderedSquaresValues[i] = muxSquaresValues[4][3][i];
    }
    //2 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 5][j] = muxValues[0][0][i][j];
        }
        orderedSquaresValues[k + 5] = muxSquaresValues[0][0][i];
    }
    //second row
    //3 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 10][j] = muxValues[4][2][i][j];
        }
        orderedSquaresValues[i + 10] = muxSquaresValues[4][2][i];
    }
    //4 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 15][j] = muxValues[0][1][i][j];
        }
        orderedSquaresValues[k + 15] = muxSquaresValues[0][1][i];
    }
    //third row
    //5 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 20][j] = muxValues[4][1][i][j];
        }
        orderedSquaresValues[i + 20] = muxSquaresValues[4][1][i];
    }
    //6 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 25][j] = muxValues[0][2][i][j];
        }
        orderedSquaresValues[k + 25] = muxSquaresValues[0][2][i];
    }
    //fourth row
    //7 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 30][j] = muxValues[4][0][i][j];
        }
        orderedSquaresValues[i + 30] = muxSquaresValues[4][0][i];
    }
    //8 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 35][j] = muxValues[0][3][i][j];
        }
        orderedSquaresValues[k + 35] = muxSquaresValues[0][3][i];
    }
    //Mux 1 y 3
    //fifth row
    //9 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 40][j] = muxValues[3][3][i][j];
        }
        orderedSquaresValues[i + 40] = muxSquaresValues[3][3][i];
    }
    //10 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 45][j] = muxValues[1][0][i][j];
        }
        orderedSquaresValues[k + 45] = muxSquaresValues[1][0][i];
    }
    //sixth row
    //11 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 50][j] = muxValues[3][2][i][j];
        }
        orderedSquaresValues[i + 50] = muxSquaresValues[3][2][i];
    }
    //12 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 55][j] = muxValues[1][1][i][j];
        }
        orderedSquaresValues[k + 55] = muxSquaresValues[1][1][i];
    }
    //seventh row
    //13 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 60][j] = muxValues[3][1][i][j];
        }
        orderedSquaresValues[i + 60] = muxSquaresValues[3][1][i];
    }
    //14 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 65][j] = muxValues[1][2][i][j];
        }
        orderedSquaresValues[k + 65] = muxSquaresValues[1][2][i];
    }
    //eigth row
    //15 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 70][j] = muxValues[3][0][i][j];
        }
        orderedSquaresValues[i + 70] = muxSquaresValues[3][0][i];
    }
    //16 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 75][j] = muxValues[1][3][i][j];
        }
        orderedSquaresValues[k + 75] = muxSquaresValues[1][3][i];
    }
    //Mux 2
    //nineth row
    //17 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 80][j] = muxValues[2][0][i][j];
        }
        orderedSquaresValues[i + 80] = muxSquaresValues[2][0][i];
    }
    //18 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 85][j] = muxValues[2][3][i][j];
        }
        orderedSquaresValues[k + 85] = muxSquaresValues[2][3][i];
    }
    //tenth row
    //19 pcb bar
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[i + 90][j] = muxValues[2][1][i][j];
        }
        orderedSquaresValues[i + 90] = muxSquaresValues[2][1][i];
    }
    //20 pcb bar
    for (int i = 4, k = 0; i >= 0; i--, k++) {
        for (int j = 0; j <= 4; j++) {
            orderedSensorValues[k + 95][j] = muxValues[2][2][i][j];
        }
        orderedSquaresValues[k + 95] = muxSquaresValues[2][2][i];
    }
}

void printSerial(int time, int Selection, int p, int k, int j)
{
    detectChessBoard();
    if (Selection == SIMPLE) {
        for (int i = 0; i < 100; i++) {
            Serial.print(orderedSquaresValues[i]);

            Serial.print(" ");
            if (i == 9 || i == 19 || i == 29 || i == 39 || i == 49 || i == 59 || i == 69 || i == 79 || i == 89 || i == 99) {
                Serial.println();
            } else {
                Serial.print(" ");
            }
        }

        Serial.println();

        if (time != 0) {
            delay(time);
        }
    } else if (Selection == FULL) {

        for (int i = 0; i < 100; i++) {
            for (int j = 0; j < 5; j++) {
                Serial.print(orderedSensorValues[i][j]);
            }
            if (i == 9 || i == 19 || i == 29 || i == 39 || i == 49 || i == 59 || i == 69 || i == 79 || i == 89 || i == 99) {
                Serial.println();
            } else {
                Serial.print(" ");
            }
        }

        Serial.println();

        if (time != 0) {
            delay(time);
        }

    } else if (Selection == GROUPOFSENSORS) {

        for (int i = 0; i < 5; i++) 
        {
            digitalWrite(mux8_0, bitRead(dirSensor[k][i][j], 0));
            digitalWrite(mux8_1, bitRead(dirSensor[k][i][j], 1));
            digitalWrite(mux8_2, bitRead(dirSensor[k][i][j], 2));

            digitalWrite(mux16_0, bitRead(dirSensor[k][i][j], 4));
            digitalWrite(mux16_1, bitRead(dirSensor[k][i][j], 5));
            digitalWrite(mux16_2, bitRead(dirSensor[k][i][j], 6));
            digitalWrite(mux16_3, bitRead(dirSensor[k][i][j], 7));

            delayMicroseconds(timeBsensors); 

            switch (p) {
            case 0:
                Serial.print(digitalRead(mux16Out_0));
                Serial.print("  ");
                break;
            case 1:
                Serial.print(digitalRead(mux16Out_1));
                Serial.print("  ");
                break;
            case 2:
                Serial.print(digitalRead(mux16Out_2));
                Serial.print("  ");
                break;
            case 3:
                Serial.print(digitalRead(mux16Out_3));
                Serial.print("  ");
                break;
            case 4:
                Serial.print(digitalRead(mux16Out_4));
                Serial.print("  ");
                break;
            default:
                break;
            }
        }
        Serial.println();

        if (time != 0) {
            delay(time);
        }
    }
}

void sensorsDir()
{

    dirSensor[0][4][4] = 0x03;
    dirSensor[0][4][3] = 0x06;
    dirSensor[0][4][2] = 0x04;
    dirSensor[0][4][1] = 0x07;
    dirSensor[0][4][0] = 0x05;
    dirSensor[0][3][4] = 0x16;
    dirSensor[0][3][3] = 0x01;
    dirSensor[0][3][2] = 0x02;
    dirSensor[0][3][1] = 0x00;
    dirSensor[0][3][0] = 0x14;
    dirSensor[0][2][4] = 0x10;
    dirSensor[0][2][3] = 0x12;
    dirSensor[0][2][2] = 0x15;
    dirSensor[0][2][1] = 0x11;
    dirSensor[0][2][0] = 0x13;
    dirSensor[0][1][4] = 0x25;
    dirSensor[0][1][3] = 0x24;
    dirSensor[0][1][2] = 0x17;
    dirSensor[0][1][1] = 0x26;
    dirSensor[0][1][0] = 0x27;
    dirSensor[0][0][4] = 0x30;
    dirSensor[0][0][3] = 0x22;
    dirSensor[0][0][2] = 0x21;
    dirSensor[0][0][1] = 0x20;
    dirSensor[0][0][0] = 0x23;

    dirSensor[1][4][4] = 0x43;
    dirSensor[1][4][3] = 0x46;
    dirSensor[1][4][2] = 0x44;
    dirSensor[1][4][1] = 0x47;
    dirSensor[1][4][0] = 0x45;
    dirSensor[1][3][4] = 0x56;
    dirSensor[1][3][3] = 0x41;
    dirSensor[1][3][2] = 0x42;
    dirSensor[1][3][1] = 0x40;
    dirSensor[1][3][0] = 0x54;
    dirSensor[1][2][4] = 0x50;
    dirSensor[1][2][3] = 0x52;
    dirSensor[1][2][2] = 0x55;
    dirSensor[1][2][1] = 0x51;
    dirSensor[1][2][0] = 0x53;
    dirSensor[1][1][4] = 0x65;
    dirSensor[1][1][3] = 0x64;
    dirSensor[1][1][2] = 0x57;
    dirSensor[1][1][1] = 0x66;
    dirSensor[1][1][0] = 0x67;
    dirSensor[1][0][4] = 0x70;
    dirSensor[1][0][3] = 0x62;
    dirSensor[1][0][2] = 0x61;
    dirSensor[1][0][1] = 0x60;
    dirSensor[1][0][0] = 0x63;

    dirSensor[2][4][4] = 0xF3;
    dirSensor[2][4][3] = 0xF6;
    dirSensor[2][4][2] = 0xF4;
    dirSensor[2][4][1] = 0xF7;
    dirSensor[2][4][0] = 0xF5;
    dirSensor[2][3][4] = 0xE6;
    dirSensor[2][3][3] = 0xF1;
    dirSensor[2][3][2] = 0xF2;
    dirSensor[2][3][1] = 0xF0;
    dirSensor[2][3][0] = 0xE4;
    dirSensor[2][2][4] = 0xE0;
    dirSensor[2][2][3] = 0xE2;
    dirSensor[2][2][2] = 0xE5;
    dirSensor[2][2][1] = 0xE1;
    dirSensor[2][2][0] = 0xE3;
    dirSensor[2][1][4] = 0xD5;
    dirSensor[2][1][3] = 0xD4;
    dirSensor[2][1][2] = 0xE7;
    dirSensor[2][1][1] = 0xD6;
    dirSensor[2][1][0] = 0xD7;
    dirSensor[2][0][4] = 0xC0;
    dirSensor[2][0][3] = 0xD2;
    dirSensor[2][0][2] = 0xD1;
    dirSensor[2][0][1] = 0xD0;
    dirSensor[2][0][0] = 0xD3;

    dirSensor[3][4][4] = 0xB3;
    dirSensor[3][4][3] = 0xB6;
    dirSensor[3][4][2] = 0xB4;
    dirSensor[3][4][1] = 0xB7;
    dirSensor[3][4][0] = 0xB5;
    dirSensor[3][3][4] = 0xA6;
    dirSensor[3][3][3] = 0xB1;
    dirSensor[3][3][2] = 0xB2;
    dirSensor[3][3][1] = 0xB0;
    dirSensor[3][3][0] = 0xA4;
    dirSensor[3][2][4] = 0xA0;
    dirSensor[3][2][3] = 0xA2;
    dirSensor[3][2][2] = 0xA5;
    dirSensor[3][2][1] = 0xA1;
    dirSensor[3][2][0] = 0xA3;
    dirSensor[3][1][4] = 0x95;
    dirSensor[3][1][3] = 0x94;
    dirSensor[3][1][2] = 0xA7;
    dirSensor[3][1][1] = 0x96;
    dirSensor[3][1][0] = 0x97;
    dirSensor[3][0][4] = 0x80;
    dirSensor[3][0][3] = 0x92;
    dirSensor[3][0][2] = 0x91;
    dirSensor[3][0][1] = 0x90;
    dirSensor[3][0][0] = 0x93;
}

void sendSensorValues(bool values[][AMOUNTOFSENSORS], int amountOfSquares, int amountOfSensors)
{
    // now transmit the packed data

    int amountOfValues = amountOfSquares * amountOfSensors;
    bool arrayValues[amountOfValues];

    // it sends the initial character
    sendI2CString(".");

    // we transform the array of squares and sensors into a one dimension array.
    for (int square = 0; square < amountOfSquares; square++) {
        for (int sensor = 0; sensor < amountOfSensors; sensor++) {
            arrayValues[square * amountOfSensors + sensor] = values[square][sensor];
        }
    }

    // sending the boolean data in packages of 32 bytes.
    int offset = 0;
    while (true) {
        if (amountOfValues > MessageSize) {
            sendI2CBools(arrayValues + offset, MessageSize);
            offset += MessageSize;
            amountOfValues -= MessageSize;
        } else {
            sendI2CBools(arrayValues + offset, amountOfValues);
            break;
        }
    }

    // sending the end character
    sendI2CString("\n");
}

void sendI2CBools(bool values[], int amount)
{
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    for (int element = 0; element < amount; element++) {
        if (values[element]) {
            Wire.write("1");
        } else {
            Wire.write("0");
        }
    }
    Wire.endTransmission(); // stop transmitting
}

void sendI2CString(String value)
{
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    for (int element = 0; element < value.length(); element++) {
        Wire.write(value.charAt(element));
    }
    Wire.endTransmission(); // stop transmitting
}

void requestEvent()
{
    static byte y = 0;

    WireSlave.print("1234567890");
}

void receiveEvent(int howMany)
{
    for (int i = 0; i < howMany; i++) {
        char c = WireSlave.read();
        Serial.print(c); 
        code[i] = c;
    }
    band = 0;
    Serial.println("");
}