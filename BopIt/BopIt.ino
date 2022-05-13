#include <I2Cdev.h>
#include <LiquidCrystal.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

/*
 * Define action button/potentiometer pins
 */

#define BOP_IT_PIN 41
#define PULL_IT_PIN 43
#define TWIST_IT_PIN A8

/*
 * Define gyroscope information
 */

MPU6050 gyro;

// This flag will be true if there is an error while initializing the gyro
bool error = false;
// This stores the input from the gyroscope prior to processing
uint8_t gyroBuffer[64];

// These values store intermediate processed values from the gyroscope
Quaternion quaternion;
VectorFloat gravity;
float currentAngles[3];
float currentRotation;

/*
 * Define LCD pins and objects
 */

#define LCD_RS 22
#define LCD_RW 24
#define LCD_EN 26
#define LCD_D4 28
#define LCD_D5 30
#define LCD_D6 32
#define LCD_D7 34

LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Since we include a flip mechanic, we need to be able to display text upside down
// These are just the normal renderings upside down
byte flipB[8] = {
  B01111,
  B10001,
  B10001,
  B01111,
  B10001,
  B10001,
  B01111,
  B00000
};
byte flipL[8] = {
  B11111,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00000
};
byte flipN[8] = {
  B10001,
  B10001,
  B11001,
  B10101,
  B10011,
  B10001,
  B10001,
  B00000
};
byte flipP[8] = {
  B00001,
  B00001,
  B00001,
  B01111,
  B10001,
  B10001,
  B01111,
  B00000
};
byte flipR[8] = {
  B10001,
  B01001,
  B00101,
  B01111,
  B10001,
  B10001,
  B01111,
  B00000
};
byte flipT[8] = {
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B11111,
  B00000
};
byte flipU[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B00000
};
byte flipW[8] = {
  B01010,
  B10101,
  B10101,
  B10101,
  B10001,
  B10001,
  B10001,
  B00000
};

#define FLIP_B byte(0)
#define FLIP_L byte(1)
#define FLIP_N byte(2)
#define FLIP_P byte(3)
#define FLIP_R byte(4)
#define FLIP_T byte(5)
#define FLIP_U byte(6)
#define FLIP_W byte(7)

/*
 * Define instruction information
 */

#define BOP_IT 1
#define PULL_IT 2
#define TWIST_IT 3
#define TURN_IT 4

void setup() {
  // Initialize action ports
  pinMode(BOP_IT_PIN, INPUT);
  pinMode(PULL_IT_PIN, INPUT);
  
  // Initialize upside down characters
  lcd.createChar((int) FLIP_B, flipB);
  lcd.createChar((int) FLIP_L, flipL);
  lcd.createChar((int) FLIP_N, flipN);
  lcd.createChar((int) FLIP_P, flipP);
  lcd.createChar((int) FLIP_R, flipR);
  lcd.createChar((int) FLIP_T, flipT);
  lcd.createChar((int) FLIP_U, flipU);
  lcd.createChar((int) FLIP_W, flipW);
  lcd.begin(16, 2);
  lcd.clear();

  // Generate pseudo-random seed using two analog reads on unconnected pins (as recommended by Arduino docs)
  unsigned long randSeed = analogRead(0) * analogRead(1);
  randomSeed(randSeed);

  // Begin serial for debugging
  Serial.begin(9600);

  // Initialize gyroscope
  Wire.begin();
  Wire.setClock(400000);
  gyro.initialize();
  error = gyro.dmpInitialize() != 0; // Check that the DMP initialized correctly

  if (!error) {
    gyro.setDMPEnabled(true);
  } else {
    lcd.print("Gyroscope error!");
  }
}

// These values track the current status of the game as a whole
bool gameRunning = false;
unsigned int gameScore = 0;
int currentInstruction;

// These values track the current action being performed (and the time it has left)
bool needNewAction = true;
unsigned long actionStartTime = 0;
unsigned long actionTimeLimit = 5000;

// These values contain the statuses of the buttons and potentiometer, used
// for the press-and-release logic preventing multiple actions from being
// triggered by a single press
int bopItButtonCurrent = LOW;
int bopItButtonPrevious = LOW;
bool bopItStatus = false;
int pullItButtonCurrent = LOW;
int pullItButtonPrevious = LOW;
bool pullItStatus = false;
bool twistItStatus = false;

// This value tracks the rotation of the device at the start of an action
// which is used to calculate when a 180-degree turn has been performed
float startingRotation = 0.0;

// This value track the current orientation of the screen to determine
// if text needs to be drawn upside down or not
bool screenFlipped = false;

void loop() {
  // If our gyroscope errored, we can't run
  if (error)
    return;
  
  // Update the gyroscope value
  updateGyro();
  
  // If the game is running, do the game logic.  If not, show the start screen
  if (gameRunning) {
    unsigned long millisNow = millis();
    // If we've timed out, end the game
    bool shouldEndGame = false;
    bool score = false;

    // If we need a new action, generate it
    if (needNewAction) {
      needNewAction = false;
      currentInstruction = giveNextInstruction(currentInstruction, screenFlipped);
      actionStartTime = millisNow;
      
      // Set the current rotation for the action
      startingRotation = currentRotation;
    }
    
    // If we're waiting on an action, check if an action has been performed
    if (millisNow < actionStartTime + actionTimeLimit) {
      int action = checkAction();
      if (action > 0) {
        // If we performed the correct action, notify the game that we need a new one
        if (action == currentInstruction) {
          needNewAction = true;
          score = true;

          // If this was a turn action, we need to rotate the display
          if (action == TURN_IT)
            screenFlipped = !screenFlipped;
        } else {
          shouldEndGame = true;
        }
      }
    } else {
      shouldEndGame = true;
    }

    // If we scored, add one to the score
    // If the game is over, go to the end screen
    if (!shouldEndGame && score)
      gameScore++;
    else if (shouldEndGame)
      endGame();
  } else {
    initializeGame();
  }
}

void initializeGame() {
  // Print the start message to the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  PRESS BOP-IT  ");
  lcd.setCursor(0, 1);
  lcd.print("    TO START    ");

  // Wait for the Bop-It button to be pressed
  while (!checkButtonPressed(BOP_IT_PIN, bopItButtonPrevious, bopItButtonCurrent, bopItStatus));

  // Initialize the game
  gameRunning = true;
  gameScore = 0;
  actionStartTime = 0;
  needNewAction = true;
  screenFlipped = false;
}

void endGame() {
  gameRunning = false;

  // Print the player's score, then wait 5 seconds until resetting
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   GAME OVER!   ");
  lcd.setCursor(0, 1);
  lcd.print("SCORE: ");
  lcd.print(gameScore);

  delay(5000);
}

int giveNextInstruction(int previousInstruction, bool flipped) {
  // Generate a new instruction
  int instruction = previousInstruction;
  while (instruction == previousInstruction)
    instruction = random(1, 5);

  // Print the instruction to the LCD
  printInstruction(instruction, flipped);

  return instruction;
}

void printInstruction(int instruction, bool flipped) {
  // Clear the LCD and reset the cursor (if we're drawing flipped, the cursor needs to be on the opposite row)
  lcd.clear();
  if (flipped)
    lcd.setCursor(0, 1);
  else
    lcd.setCursor(0, 0);
  
  // If the device is flipped, we need to render using our upside-down letters
  switch (instruction) {
    case BOP_IT:
      if (flipped) {
        lcd.print("*    ");
        lcd.write(FLIP_T);
        lcd.write('I');
        lcd.write(' ');
        lcd.write(FLIP_P);
        lcd.write('O');
        lcd.write(FLIP_B);
        lcd.print("    *");
      } else {
        lcd.print("*    BOP IT    *");
      }
      break;
    case PULL_IT:
      if (flipped) {
        lcd.print("*    ");
        lcd.write(FLIP_T);
        lcd.write('I');
        lcd.write(' ');
        lcd.write(FLIP_L);
        lcd.write(FLIP_L);
        lcd.write(FLIP_U);
        lcd.write(FLIP_P);
        lcd.print("   *");
      } else {
        lcd.print("*   PULL IT    *");
      }
      break;
    case TWIST_IT:
      if (flipped) {
        lcd.print("*   ");
        lcd.write(FLIP_T);
        lcd.write('I');
        lcd.write(' ');
        lcd.write(FLIP_T);
        lcd.write('S');
        lcd.write('I');
        lcd.write(FLIP_W);
        lcd.write(FLIP_T);
        lcd.print("   *");
      } else {
        lcd.print("*   TWIST IT   *");
      }
      break;
    case TURN_IT:
      if (flipped) {
        lcd.print("*    ");
        lcd.write(FLIP_T);
        lcd.write('I');
        lcd.write(' ');
        lcd.write(FLIP_N);
        lcd.write(FLIP_R);
        lcd.write(FLIP_U);
        lcd.write(FLIP_T);
        lcd.print("   *");
      } else {
        lcd.print("*   TURN IT    *");
      }
      break;
  }
}

int checkAction() {
  // Check if the "Bop It" button is pressed (if it is, return if the action was correct)
  if (checkButtonPressed(BOP_IT_PIN, bopItButtonPrevious, bopItButtonCurrent, bopItStatus))
    return BOP_IT;

  // Check if the "Pull It" button is pressed
  if (checkButtonPressed(PULL_IT_PIN, pullItButtonPrevious, pullItButtonCurrent, pullItStatus))
    return PULL_IT;

  // Check if the "Twist It" potentiometer has been turned
  if (checkPotentiometerChanged(TWIST_IT_PIN, twistItStatus))
    return TWIST_IT;

  // Check if the device has been rotated (for "Turn It")
  if (checkDeviceTurned())
    return TURN_IT;

  return 0;
}

unsigned long debounceTime = 0;
const unsigned long debounceDelay = 50;

bool checkButtonPressed(int pin, int& previous, int& current, bool& currentStatus) {
  // If the button is pressed, check if it was previously
  // If it was, we need to wait for it to be released before it can be pressed again
  if (checkButtonDebounced(pin, previous, current)) {
    if (currentStatus == false) {
      currentStatus = true;
      return true;
    }
  } else {
    currentStatus = false;
  }

  return false;
}

bool checkButtonDebounced(int pin, int& previous, int& current) {
  // Get the current value of the pin
  int reading = digitalRead(pin);

  // If the reading is different, wait for the debounce
  if (reading != previous)
    debounceTime = millis();

  if (millis() > debounceTime + debounceDelay)
    current = reading;

  // Set the previous reading and return true if the value is high
  previous = reading;
  return current == HIGH;
}

bool checkPotentiometerChanged(int pin, bool& currentStatus) {
  // Get the value of the potentiometer
  int initialRead = analogRead(pin);

  // If the read value is different than the current status, then the potentiometer has changed
  if (currentStatus && initialRead <= 512) {
    currentStatus = false;
    return true;
  } else if (!currentStatus && initialRead > 512) {
    currentStatus = true;
    return true;
  }

  return false;
}

const float angleMargin = M_PI / 8;

bool checkDeviceTurned() {
  // Calculate smallest rotational distance between the current
  // and starting rotation
  float rotDiff = abs(currentRotation - startingRotation);
  rotDiff = min(2 * M_PI - rotDiff, rotDiff);

  // Check if we've rotated 180 degrees (pi radians)
  // This target value is the margin we allow for the rotation (in our case, we allow
  // for an 16th of a full rotation as the margin)
  float target = M_PI - angleMargin;
  return rotDiff > target;
}

void updateGyro() {
  // Check for an updated packet
  if (gyro.dmpGetCurrentFIFOPacket(gyroBuffer)) {
    // Pull out the quaternion rotation and then Euler angles
    gyro.dmpGetQuaternion(&quaternion, gyroBuffer);
    gyro.dmpGetGravity(&gravity, &quaternion);
    gyro.dmpGetYawPitchRoll(currentAngles, &quaternion, &gravity);

    // Pull out the Z component (this is the one we want to rotate around)
    currentRotation = currentAngles[0];
  }
}
