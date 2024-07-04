// -*- mode: c++ -*-

/*
Copyright (c) 2024 Carri King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <Servo.h>

#include "LittleFS.h"

//#define STEPPER_BYJ48
#define STEPPER_A9888

/// ----------------------------------------------
/// File system
/// ----------------------------------------------
class FileSystem {
public:
  bool removeFile(const String& name) {
    String filename = sm_path + "/" + name;
    return LittleFS.remove(filename);
  }

  bool appendFile(const String& name, const String& line) {
    String filename = sm_path + "/" + name;
    File f = LittleFS.open(filename, "a");
    if(f) {
      const uint8_t * data = (const uint8_t *)line.c_str();
      size_t len = line.length();
      f.write(data, len);
      f.close();
      return true;
    }
    return false;
  }

  int readFileChunk(const String& name, uint32_t pos, uint8_t *buf, size_t size) {
    memset(buf, 0, size);
    String filename = sm_path + "/" + name;
    File file = LittleFS.open(filename, "r");
    if(file) {
      file.seek(pos);
      int len = file.read(buf, size-1);
      file.close();
      return len;
    } else {
      return 0;
    }
  }

  uint32_t getFileSize(const String& name) {
    String filename = sm_path + "/" + name;
    auto file = LittleFS.open(filename, "r");
    uint32_t filesize = 0;
    if(file) {
      filesize = file.size();
      file.close();
    }
    return filesize;
  }

  bool openDir() {
    m_dir = LittleFS.openDir(sm_path);
    return true;
  }

  void closeDir() {
    // dunno..
    m_dir.rewind();
  }

  bool getNextFileInfo(String& name, unsigned long& size) {
    if(m_dir.next()) {
      name = m_dir.fileName();
      size = m_dir.fileSize();
      return true;
    }
    return false;
  }

  bool getFSInfo(unsigned long& totalSpace, unsigned long& usedSpace) {
    // this crashes the ESP8266 :(
    FSInfo64 fsInfo;
    LittleFS.info64(fsInfo);
    totalSpace = fsInfo.totalBytes;
    usedSpace = fsInfo.usedBytes;
    return true;
  }

private:
  FileSystem() { 
    LittleFS.begin();
  }

  File m_file;
  Dir m_dir;
  static String sm_path;

public:
  static FileSystem& Instance() {
    static FileSystem inst;
    return inst;
  }
};

/* static */ String FileSystem::sm_path("/gcode");

/// ----------------------------------------------
/// Test code
/// ----------------------------------------------

const char *g_gcode = R"_X_(
G21 (All units in mm)
G00 Z5.000000
G00 X115.660710 Y38.553570
G01 Z-0.125000 F100.0(Penetrate)
G01 X102.202420 Y40.868460 Z-0.125000 F400.000000
G01 X92.665754 Y31.094190 Z-0.125000
G01 X90.708511 Y44.609130 Z-0.125000
G01 X78.465638 Y50.658630 Z-0.125000
G01 X90.714286 Y56.696420 Z-0.125000
G01 X92.684442 Y70.209480 Z-0.125000
G01 X102.211770 Y60.426110 Z-0.125000
G01 X115.672260 Y62.728140 Z-0.125000
G01 X109.311830 Y50.643890 Z-0.125000
G01 X115.660710 Y38.553570 Z-0.125000
G00 Z5.000000
)_X_";

static const int NUM_STEPS = 8;
static const int NUM_STEPPER_PINS = 4;

/// half step sequence.
static const int STEP_SEQ[NUM_STEPS][NUM_STEPPER_PINS] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// Low level interface to the BYJ48 mini stepper using the preferred half stepping sequence.
class BYJ48Stepper {
protected:  
  BYJ48Stepper()
  : m_seqPos(0),
    m_idle(false)
    {}
    
  void idleReal(const int *pins) {
    if(!m_idle) {
       for(int n = 0; n<NUM_STEPPER_PINS; n++) {
         digitalWrite(pins[n], 0);      
      }
      m_idle = true;
    }
  }
  
  void incStepReal(const int *pins, int stepsRemaining, int stepsTotal) {
    int n = m_seqPos;
    n++;
    if(n == NUM_STEPS)
      n = 0;
    setSeqPos(pins, n);
  }

  void decStepReal(const int *pins, int stepsRemaining, int stepsTotal) {
    int n = m_seqPos;
    if(n > 0)
      --n;
      else
        n = (NUM_STEPS-1);
    setSeqPos(pins, n);
  }

  void setSeqPos(const int *pins, int seqPos) {
    m_seqPos = seqPos;
    m_idle = false;
    const int *seq = STEP_SEQ[m_seqPos];
    for(int n = 0; n<NUM_STEPPER_PINS; n++) {
      digitalWrite(pins[n], seq[n]);      
    }
  }

  int m_seqPos;
  bool m_idle;
};

// If true the motor will alter its delay times to ease in out and reduce sway
bool g_smoothing = false;

// Low level interface to the A9888 stepper driver, this implements the stepping sequence itself.
class A9888Stepper {
protected:
  void idleReal(const int *pins) {
    // can't idle this, without power the motor will freewheel and the plotter will fall
  }
  void incStepReal(const int *pins, int stepsRemaining, int stepsTotal) {
    digitalWrite(pins[0], HIGH);
    //delayMicroseconds(50);
    singleStep(pins, stepsRemaining, stepsTotal);
  }

  void decStepReal(const int *pins, int stepsRemaining, int stepsTotal) {
    digitalWrite(pins[0], LOW);
    //delayMicroseconds(50);
    singleStep(pins, stepsRemaining, stepsTotal);
  }

  void singleStep(const int *pins, long stepsRemaining, long stepsTotal) {
    long delayTime = 500;
    if(g_smoothing) {    
      long extraDelayTime = 300;
      long accelStepCount = 300;
      long leadInOutSteps = min(accelStepCount, stepsTotal/2);
    
  	  if(stepsTotal > 0) {
	  	  int stepsMade = stepsTotal - stepsRemaining;
		    if(stepsMade < leadInOutSteps) {
			    delayTime += extraDelayTime - (((stepsMade * extraDelayTime) / accelStepCount));
		    } else if(stepsRemaining <= leadInOutSteps) {
			    delayTime += extraDelayTime - (((stepsRemaining * extraDelayTime) / accelStepCount));
		    }
	    }  
    }
    
    digitalWrite(pins[1], HIGH);
    delayMicroseconds(delayTime * 10);
    digitalWrite(pins[1], LOW);
    delayMicroseconds(delayTime * 10);
  }
};

/// Controller for a single stepper motor.
template<typename StepperType> class StepperControllerCore : StepperType {
public:
  using StepperType::decStepReal;
  using StepperType::incStepReal;
  using StepperType::idleReal;
  
  StepperControllerCore(int p1, int p2, int p3, int p4, bool swapped)
  : StepperType(),
    m_swapped(swapped),
    m_absPos(0)
  {
    m_pins[0] = p1;
    m_pins[1] = p2;
    m_pins[2] = p3;
    m_pins[3] = p4;
  }

  void setup() {
    for(int n = 0; n<NUM_STEPPER_PINS; n++) {
      if(m_pins[n] >= 0)
        pinMode(m_pins[n], OUTPUT);
    }
  }

  void idle() {
    idleReal(m_pins);
  }
  
  void incStep(int stepsRemaining, int stepsTotal) {
    if(m_swapped)
      decStepReal(m_pins, stepsRemaining, stepsTotal);
    else
      incStepReal(m_pins, stepsRemaining, stepsTotal);
    ++m_absPos;
  }

  void decStep(int stepsRemaining, int stepsTotal) {
    if(m_swapped)
      incStepReal(m_pins, stepsRemaining, stepsTotal);
    else
      decStepReal(m_pins, stepsRemaining, stepsTotal);
    --m_absPos;
  }

  void setAbsPos(long t) {
    m_absPos = t;
  }

  long getAbsPos() const {
    return m_absPos;
  }

  bool stepToHome() {
    if(m_absPos == 0)
      return true;
    else if(m_absPos > 0)
      decStep(0, 0);
    else if(m_absPos < 0)
      incStep(0, 0);
    return false;
  }

private:
  int m_pins[NUM_STEPPER_PINS];
  bool m_swapped;
  long m_absPos;
};

#if defined(STEPPER_BYJ48)
typedef StepperControllerCore<BYJ48Stepper> StepperController;
#elif defined(STEPPER_A9888)
typedef StepperControllerCore<A9888Stepper> StepperController;
#else
#error("Unknown stepper type")
#endif

/// Controller for the plotter with both steppers and servo
// This is a state machine, each time update() is called the motor is stepped or the next action
// in the gcode is processed.
class WallPlotter {
public:
  WallPlotter(int lp1, int lp2, int lp3, int lp4, bool invLeft, int rp1, int rp2, int rp3, int rp4, bool invRight)
  : m_left(lp1, lp2, lp3, lp4, invLeft),
    m_right(rp1, rp2, rp3, rp4, invRight),
    m_action(STOP),
    m_nextAction(STOP),
    m_testSeqStep(0),    
    m_stepsTotal(0),
    m_stepsRemaining(0),
    m_fracStep(0.0f),
    m_fracPos(0.0),
    m_servoPos(0),
    m_moveToZ(0),
    m_fileSize(0),
    m_readPos(0),
    m_lastX(0.0),
    m_lastY(0.0),
    m_baseLineLen(410),
    m_zeroLen(300)
  {

  }

  void setup() {
    m_left.setup();
    m_right.setup();
#if defined(STEPPER_BYJ48)    
    m_servo.attach(0);
#elif defined(STEPPER_A9888)
    m_servo.attach(13); // D7
#else
#error("Unknown config")
#endif
    m_servo.write(m_servoPos);
  }

  long getAbsPosLeft() const {
    return m_left.getAbsPos();
  }

  long getAbsPosRight() const {
    return m_right.getAbsPos();
  }

  long getAbsPosZ() const {
    return m_servoPos;
  }

  long getLastPosX() const {
    return m_lastX;
  }

  long getLastPosY() const {
    return m_lastY;
  }

  float getProgramProgress() const {
    if(m_fileSize > 0)
      return (float)m_readPos / (float)m_fileSize;
    return 0.0;
  }

  void resetAbsPositions() {
    m_left.setAbsPos(0);
    m_right.setAbsPos(0);
    m_lastX = 0.0;
    m_lastY = 0.0;
  }

  enum Action {STOP, UP, DOWN, LEFT_IN, LEFT_OUT, RIGHT_IN, RIGHT_OUT, MOVE_IN_L, MOVE_IN_R, MOVE_IN_Z, MOVE_IN_CW, MOVE_IN_CCW, HOME, TEST, FILE};
  
  bool setActionName(const char *name) {
    int a = 0;
    const char *action;
    while(action = sm_actionNames[a]) {
      if(strcmp(name, action) == 0) {
        setAction((Action)a);
        return true;
      }
      ++a;
    }
    return false;
  }

  const char *getActionName() const {
    return sm_actionNames[getAction()];
  }

  void setAction(Action action) {
    m_action = action;
  }

  Action getAction() const {
    return m_action;
  }

  void update() {
    switch(m_action) {
      case STOP:
        m_left.idle();
        m_right.idle();
        break;
      case UP:
        m_left.decStep(0, 0);
        m_right.decStep(0, 0);
        break;
      case DOWN:
        m_left.incStep(0, 0);
        m_right.incStep(0, 0);
        break;
      case LEFT_IN:
        m_left.incStep(0, 0);
        m_right.idle();
        break;
      case LEFT_OUT:
        m_left.decStep(0, 0);
        m_right.idle();
        break;
      case RIGHT_IN:
        m_left.idle();
        m_right.incStep(0, 0);
        break;
      case RIGHT_OUT:
        m_left.idle();
        m_right.decStep(0, 0);
        break;
      case MOVE_IN_L:
      case MOVE_IN_R:
        updateMove();
        break;
      case MOVE_IN_Z:
        updateMoveZ();
        break;
      case MOVE_IN_CW:
      case MOVE_IN_CCW:
        updateCircleMove();
        break;
      case HOME: {
        bool sl = m_left.stepToHome();
        bool sr = m_right.stepToHome();
        if(sl && sr)
          m_action = STOP;
        break;
      }
      case TEST:
        updateTest();
        break;
      case FILE:
        updateFile();
        break;
    }
  }

  bool runProgram(const String& name) {
    m_fileSize = FileSystem::Instance().getFileSize(name);
    if(m_fileSize == 0) {
      return false;
    }

    m_name = name;
    m_action = FILE;
    m_nextAction = FILE;
    m_readPos = 0;
    m_endOfFile = false;
    readNextChunk();
    return true;
  }

  void updateFile() {
    if(parseGCode()) {
      m_nextAction = FILE;
    } else {
      m_action = STOP;
      m_nextAction = STOP;
    }
  }

  void updateTest() {
    if(m_nextAction == STOP) {
      m_name = "";
      m_codePtr = g_gcode;
    }
    
    if(parseGCode()) {
      m_nextAction = TEST;
    } else {
      m_nextAction = STOP;
    }
  }

  void readNextChunk() {
    if(m_endOfFile)
      return;
    int len = FileSystem::Instance().readFileChunk(m_name, m_readPos, (uint8_t *)m_readBuffer, sizeof(m_readBuffer));
    m_readPos += len;
    if(m_readPos >= m_fileSize)
      m_endOfFile = true;
    m_codePtr = m_readBuffer;
  }

  void incParserBuffer() {
    ++m_codePtr;
    if(*m_codePtr == 0 /*&& m_action == FILE*/)
      readNextChunk();
  }

	// if a token is longer than 64 chars then the behaviour is undefined.
  static const int TOKEN_LEN=64;
  enum ParseState {PS_EOD, PS_EOL, PS_NEXT};
  ParseState parseGCodeToken(char *token) {
    *token = 0;
    int tokenLen = 0;
    while(*m_codePtr) {
      switch(*m_codePtr) {
      case '(':
        while(1) {
	        if(*m_codePtr == ')' || *m_codePtr == 0)
	          break;
	        else
            incParserBuffer();
        }
      break;
      case ' ':
        *token = 0;
        incParserBuffer();
        return PS_NEXT;
      case '\n':
        *token = 0;      
        incParserBuffer();
        return PS_EOL;
        break;
      case 0:
        break;
      default:
        if(tokenLen < TOKEN_LEN) {
          *token++ = *m_codePtr;
          ++tokenLen;
        }
        incParserBuffer();
        break;
      }
    }
    return PS_EOD;
  }

  enum {R_X, R_Y, R_Z, R_I, R_J};

  bool parseGCode() {
    int validRegs = 0;
    float regs[5];
    int code = -1;
    ParseState ps;
    char token[TOKEN_LEN];
    bool startOfLine = true;
    int lineCount = 0;
    memset(token, 0, TOKEN_LEN);

    while((ps = parseGCodeToken(token)) != PS_EOD) {
      if(token[0] == 0)
        continue;
      if(code < 0) {
        if(startOfLine && toupper(token[0]) == 'G' && token[1] != 0)
        	code = atoi(token+1);
      } else {
        switch(toupper(token[0])) {
          case 'X':
	          regs[R_X] = atof(token+1);
	          validRegs |= (1<<R_X);
	          break;
          case 'Y':
	          regs[R_Y] = atof(token+1);
	          validRegs |= (1<<R_Y);	
	          break;
          case 'Z':
	          regs[R_Z] = atof(token+1);
	          validRegs |= (1<<R_Z);		
	          break;
          case 'I':
	          regs[R_I] = atof(token+1);
	          validRegs |= (1<<R_I);		
	          break;
          case 'J':
	          regs[R_J] = atof(token+1);
	          validRegs |= (1<<R_J);		
	          break;
          default:
	          break;
        }
        startOfLine = false;
      }

      if(ps == PS_EOL) {
        ++lineCount;
        startOfLine = true;
        if(code >= 0) {
          execGCode(code, validRegs, regs);
          if(code == 1 || code == 0)
            delay(10);          
          else
            delay(10);
	        return true;
        }
      }
    }           
    return false;
  }

  void execGCode(int code, int validRegs, float *regs) {
		// these constants are the servo positions for up and down and will vary
		// by servo and arrangement
    const int penDown = 80;
    const int penUp = 0;    
    
    #define V(a) (validRegs & (1<<(a)))

		// note that circular interpolation of radius only is not supported currently.
		
    if(code == 0) {
      // rapid positioning
      if(V(R_X) && V(R_Y)) {
        moveToOrthoAbs(regs[R_X], regs[R_Y]);
      } else if(V(R_Z)) {
        float z = regs[R_Z];
        if(z > 0) {
          moveToZ(penUp);
        } else {
          moveToZ(penDown);
        }
      }
    } else if(code == 1) {
      // linear interpolation
       if(V(R_X) && V(R_Y)) {
        moveToOrthoAbs(regs[R_X], regs[R_Y]);
      } else if(V(R_Z)) {
        float z = regs[R_Z];
        if(z > 0) {
          moveToZ(penUp);
        } else {
          moveToZ(penDown);
        }
      }
    } else if(code == 2) {
      // CW interpolation
       if(V(R_X) && V(R_Y) && V(R_I) && V(R_J)) {
        moveToCWAbs(regs[R_X], regs[R_Y], regs[R_I], regs[R_J]);
      }
    } else if(code == 3) {
      // CCW interpolation
      if(V(R_X) && V(R_Y) && V(R_I) && V(R_J)) {
        moveToCCWAbs(regs[R_X], regs[R_Y], regs[R_I], regs[R_J]);
      }
    } else if(code == 21) {
      // program in mm
    }
  }
  #undef V
  
  void moveStepL() {
    if(m_moveToDirL)
      m_left.incStep(m_stepsRemaining, m_stepsTotal);
    else
      m_left.decStep(m_stepsRemaining, m_stepsTotal);
  }

  void moveStepR() {
    if(m_moveToDirR)
      m_right.incStep(m_stepsRemaining, m_stepsTotal);
    else
      m_right.decStep(m_stepsRemaining, m_stepsTotal);
  }

  void updateMove() {
    if(m_stepsRemaining == 0) {
      m_stepsTotal = 0;
      m_action = m_nextAction;
      return;
    }

    bool stepMinor = false;
    m_fracPos += m_fracStep;
    if(m_fracPos >= 1.0) {
      stepMinor = true;
      m_fracPos -= 1.0;
    }

    if(m_action == MOVE_IN_L) {
      moveStepL();
      if(stepMinor)
        moveStepR();
    } else if(m_action == MOVE_IN_R) {
      moveStepR();
      if(stepMinor)
        moveStepL();
    }

    --m_stepsRemaining;
  }
  
  void updateCircleMove() {
     bool stop = false;
    if(m_action == MOVE_IN_CCW) {
      m_circleAngle += m_circleDelta;
      if(m_circleAngle >= m_circleEndAngle)
        stop = true;
    } else if(m_action == MOVE_IN_CW) {
      m_circleAngle -= m_circleDelta;
      if(m_circleAngle <= m_circleEndAngle)
        stop = true;
    }

    if(stop) {
      m_nextAction = FILE;
      moveToOrthoAbs(m_circleEndX, m_circleEndY);
      return;
    }

    float x = m_circleCentreX + (cos(m_circleAngle) * m_circleRadius);
    float y = m_circleCentreY + (sin(m_circleAngle) * m_circleRadius);

    m_nextAction = m_action;
    moveToOrthoAbs(x, y);
  }

  void moveToCWAbs(float x, float y, float i, float j) {
    moveToCircle(x, y, i, j, MOVE_IN_CW);
  }

  void moveToCCWAbs(float x, float y, float i, float j) {
    moveToCircle(x, y, i, j, MOVE_IN_CCW);
  }

  void moveToCircle(float x, float y, float i, float j, Action action) {
    m_action = action;
    m_nextAction = action;
    m_circleCentreX = m_lastX + i;
    m_circleCentreY = m_lastY + j;
    m_circleEndX = x;
    m_circleEndY = y;
  
    float sx = m_lastX - m_circleCentreX;
    float sy = m_lastY - m_circleCentreY;
  
    m_circleRadius = sqrt((sx * sx) + (sy * sy));
    m_circleAngle = atan2(sy, sx);
  
    float tx = x - m_circleCentreX;
    float ty = y - m_circleCentreY;
    m_circleEndAngle = atan2(ty, tx);

    // step the angle so the pen moves in small segments around the arc
    m_circleDelta = atan2(3.0, m_circleRadius);
    const float minStep = (M_PI * 2 * 1.0) / 180.0;
    const float maxStep = (M_PI * 2 * 90.0) / 180.0;
    
    if(m_circleDelta < minStep)
      m_circleDelta = minStep;
    else if(m_circleDelta > maxStep)
      m_circleDelta = maxStep;

    if(action == MOVE_IN_CW) {
      if(m_circleAngle < m_circleEndAngle) {
        m_circleAngle += (M_PI*2);
      }
    } else if(action == MOVE_IN_CCW) {
      if(m_circleAngle > m_circleEndAngle) {
        m_circleAngle -= (M_PI*2);
      }
    }
  }

  void orthoToBilinearCoords(float x, float y, long& absPosL, long& absPosR) const {
#if defined(STEPPER_BYJ48)
    const float step_per_mm = 2100.0/40.0;
#elif defined(STEPPER_A9888)
    //const float step_per_mm = 131.0/40.0;
    // big spool
    //const float step_per_mm = 353.0/70.0;
    //small spool
    const float step_per_mm = 695.0 / (150.0 * 0.5);
#else
#error("unknown config")
#endif
        
    // distance between the hanging points
    float baseline = m_baseLineLen;
    // length of line when the abs pos is zero
    float zeroLen = m_zeroLen;

    // A4 landscape is 297 x 210
    const float pageSizeX = 297;
    const float pageSizeY = 210;

    // make the middle of the page exactly between the two hanging points
    x -= (pageSizeX/2);
    x += (baseline/2);
    
    y -= (pageSizeY/2);
    y *= -1.0;
    y += (baseline * (3.0/4.0)) ;

    float y2 = y * y;
    float lx2 = x * x;
    float rx2 = (baseline - x) * (baseline - x);
    float lenL = sqrt(lx2 + y2);
    float lenR = sqrt(rx2 + y2);

    absPosL = -(lenL - zeroLen) * step_per_mm;
    
    // ensure the position is not a fractional step 
    //absPosL = (absPosL / 4) * 4;

    absPosR = -(lenR - zeroLen) * step_per_mm;
    
    // ensure the position is not a fractional step
    //absPosR = (absPosR / 4) * 4;
  }

  void moveToOrthoAbs(float x, float y) {
    m_lastX = x;
    m_lastY = y;
    long absPosL;
    long absPosR;
    orthoToBilinearCoords(x, y, absPosL, absPosR);
    moveToAbs(absPosL, absPosR);
  }

  void moveToAbs(long absL, long absR) {
    long deltaL = absL - m_left.getAbsPos();
    long deltaR = absR - m_right.getAbsPos();
    moveTo(deltaL, deltaR);
  }

  void moveTo(long deltaL, long deltaR) {
    if(deltaL == 0 && deltaR == 0) {
      m_action = m_nextAction;
      return;
    }
    m_moveToDirL = (deltaL >= 0);
    m_moveToDirR = (deltaR >= 0);
    deltaL = abs(deltaL);
    deltaR = abs(deltaR);
    if(deltaL > deltaR) {
      m_action = MOVE_IN_L;
      m_fracStep = (float)deltaR / (float)deltaL;
      m_fracPos = 0.0;
      m_stepsTotal = deltaL;
      m_stepsRemaining = m_stepsTotal;
    } else {
      m_action = MOVE_IN_R;
      m_fracStep = (float)deltaL / (float)deltaR;
      m_fracPos = 0.0;
      m_stepsTotal = deltaR;
      m_stepsRemaining = m_stepsTotal;
    }
  }

  void moveToZ(long absZ) {
    if(absZ == m_servoPos) {
      m_action = m_nextAction;
      return;
    }
    m_moveToZ = absZ;
    m_action = MOVE_IN_Z;
  }

  void updateMoveZ() {
    if(m_moveToZ > m_servoPos) {
      ++m_servoPos;
      m_servo.write(m_servoPos);
    } else if(m_moveToZ < m_servoPos) {
      --m_servoPos;
      m_servo.write(m_servoPos);
    } else {
      m_action = m_nextAction;
    }
  }

  float getBaseLineLen() const {
    return m_baseLineLen;
  }
  
  float getZeroLen() const {
    return m_zeroLen;
  }

  void setBaseLineLen(float b) {
    m_baseLineLen = b;
  }

  void setZeroLen(float b) {
    m_zeroLen = b;
  }
  
private:
  Action m_action;
  Action m_nextAction;
  int m_testSeqStep;
  StepperController m_left;
  StepperController m_right;
  bool m_moveToDirL;
  bool m_moveToDirR;
  long m_stepsTotal;
  long m_stepsRemaining;
  float m_fracStep;
  float m_fracPos;
  const char *m_codePtr;
  float m_lastX;
  float m_lastY;
  float m_circleRadius;
  float m_circleEndX;
  float m_circleEndY;
  float m_circleCentreX;
  float m_circleCentreY;
  float m_circleDelta;
  float m_circleAngle;
  float m_circleEndAngle;
  char m_readBuffer[1025];
  String m_name;
  size_t m_readPos;
  bool m_endOfFile;
  size_t m_fileSize;
  Servo m_servo;
  int m_servoPos;
  int m_moveToZ;

  float m_baseLineLen;
  float m_zeroLen;

  static const char *sm_actionNames[];

public:
  static WallPlotter& Instance() {
    // dont use pins 1 or 0 on the esp8266
#if defined(STEPPER_BYJ48)
    static WallPlotter g_inst(3, 5, 4, 2, true, 14, 12, 13, 15, false);
#elif defined(STEPPER_A9888)
    // direction pin, step pin
    static WallPlotter g_inst(4, 5, -1, -1, false, 12, 14, -1, -1, true);
#else    
    #error("Unknown stepper type")
#endif
        
    return g_inst;
  }
};

/*static*/ const char *WallPlotter::sm_actionNames[] = {"stop",
   "up",
  "down",
  "left_in", "left_out",
  "right_in", "right_out",
  "move_in_l", "move_in_r",
  "move_in_z",
  "move_in_cw", "move_in_ccw",
  "home", "test", "file", NULL};

/// Class to call a function at a fixed frequency
template<typename EVENT> class TimeEvent {
public:
  TimeEvent(unsigned long rate) 
  : m_lastTime(millis()), m_tickRate(rate)
  {    
  }

  void tick(unsigned long currentTime) {
    unsigned long elapsed = currentTime - m_lastTime;
    if(elapsed > m_tickRate) {
      EVENT::OnEvent();
      m_lastTime = currentTime;
    }
  }

  void setTickRate(unsigned long rate) {
    m_tickRate = rate;
  }

  unsigned long getTickRate() const {
    return m_tickRate;
  }
  
  unsigned long m_lastTime;
  unsigned long m_tickRate;
};

/// Handles the frequent event to step the wall plotter.
struct OnStepperTimeEvent {
  static void OnEvent() {
    WallPlotter::Instance().update();
  }
};

TimeEvent<OnStepperTimeEvent> g_stepperTimeEvent(7);

/// ----------------------------------------------
/// Wifi interface
/// ----------------------------------------------

IPAddress g_apIP(192, 168, 23, 1);

struct WifiSettings {
  WifiSettings() 
    : m_version(0), m_apMode(true)
  {
    memset(m_ssid, 0, sizeof(m_ssid));
    memset(m_password, 0, sizeof(m_password));

    EEPROM.begin(sizeof(WifiSettings));
    EEPROM.get(0, *this);
    if(m_version != 1) {
      
      m_version = 1;
      m_apMode = false;
			// add your details here to put the in the eeprom, upload the program, then clear
			// the values. That's kind of ungly but it works.
      strcpy(m_ssid, "XXX");
      strcpy(m_password, "XXXXXXXXXXXXXXXXXXXX");
      EEPROM.put(0, *this);
      EEPROM.commit();
      
      memset(m_ssid, 0, sizeof(m_ssid));
      memset(m_password, 0, sizeof(m_password));
      m_apMode = true;
    }
  }
    
  unsigned char m_version;
  char m_ssid[64];
  char m_password[64];
  bool m_apMode;
};

WifiSettings g_wifiSettings;

// Define a web server at port 80 for HTTP
ESP8266WebServer g_server(80);

// this is the web page the plotter serves, the javascript in here calls back to the plotter's api
static const char *g_bodyText = R"_X_(
<html>
  <head>
    <!-- <meta http-equiv='refresh' content='10'/> -->
    <title>Plotter</title>
    <style>
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; font-size: 1.0em; Color: #000000; }
      h1 { Color: #AA0000; }
    </style>
  </head>
  <script>
    function CallAPI(path, callback) {
      var request = new XMLHttpRequest();
      if(callback) {
        request.onload = function() {
            if(request.status === 200) {
                callback(request.response, request);
              } else {
                callback("error", request);
              }
        }
      }
      request.open('GET', path, true);
      request.send(null);
    }

    function CallAPIWithValue(path, id, callback) {
      const value = document.getElementById(id).value;
      if(value == "" || value == 0) {
        alert("Enter a valid value for " + id);
        return;
      }
      CallAPI(path + "?value=" + value, callback);
    }

    function runProgram(prog) {
        CallAPI("/run_program?name=" + prog, function(response) { });
    }
    
    let toRead = 0;
    let readPos = 0;
    let readData = "";
    function readProgram(filename, dataLen) {
        let status = document.getElementById("status");
        status.innerHTML = "Reading " + filename;
        toRead = dataLen;
        readPos = 0;
        readData = "";
        function getChunk() {
            if(toRead == 0) {
                status.innerHTML = "";
                if(readData.length == dataLen) {
                    document.getElementById("filename").value = filename;
                    document.getElementById("filedata").value = readData;
                }
                return;
            }
            CallAPI("/get_file_chunk?name=" + encodeURIComponent(filename) + "&pos=" + readPos, function(response, request) {
                if(request.status != 200) {
                    alert("Failed to read file chunk : " + request.status);
                    toRead = 0;
                } else {
                    readData += response;
                    toRead -= response.length;
                    readPos += response.length;
                    status.innerHTML = "Read " + readData.length + "/" + dataLen;
                }
                getChunk();             
            });
        }
        getChunk();
    }
    function toFriendlyTime(t) {
      function padNum(num, size) {
        var s = String(num);
        while (s.length < (size || 2)) {s = "0" + s;}
        return s;
      }

      let secs = Math.floor(t / 1000.0);
      let ms = t - (secs * 1000);
      return padNum(Math.floor(secs / 60.0)) + ":" + padNum(secs % 60) + ":" + padNum(ms, 4);
    }
    
    function init() {
        setInterval(function() {
            CallAPI("/status", function(response, req) {
              let v = JSON.parse(response);
              document.getElementById("uptime").value = toFriendlyTime(v.status.uptime);
              document.getElementById("action").value = v.status.action;
              document.getElementById("left_pos").value = v.status.absleft;
              document.getElementById("right_pos").value = v.status.absright;
              document.getElementById("z_pos").value = v.status.absz;
              document.getElementById("lastx").value = v.status.lastx;
              document.getElementById("lasty").value = v.status.lasty;
              document.getElementById("baselinelen").value = v.status.baselinelen;
              document.getElementById("zerolen").value = v.status.zerolen;
              document.getElementById("tickrate").value = v.status.tickrate;
              document.getElementById("smoothing").value = v.status.smoothing;
              const progress = (v.status.progress * 100.0).toFixed(1);
              document.getElementById("progress").value = progress;
              document.getElementById("progtext").innerHTML = "<br><b>Progress:  " + progress + "%</b>";
            });
        }, 3000);

        setTimeout(function() {
            refreshFileList();
        }, 1000);
    }
    function toFriendlySize(size) {
      if(size > 1024 * 1024) {
        size /= (1024 * 1024);
        return size.toFixed(2) + " MB";
      }
      if(size > 1024) {
        size /= 1024;
        return size.toFixed(2) + " KB";
      }
      return size + " bytes";
    }

    function refreshFileList() {
        document.getElementById("files").innerHTML = "waiting...";
        CallAPI("/get_dir", function(response) {
            //response =  '{"files":[{"name":"foo", "size":"22"}, {"name":"bah", "size":"42"}]}';
            let v = JSON.parse(response);
            if(v.files.length) {
                let html = "<table style=\"width: 50%\">";
                html += "<tr>";
                html += "<td><b>Name</b></td>";
                html += "<td><b>Size</b></td>";
                html += "<td><b>Run</b></td>";
                html += "<td><b>Read</b></td>";                         
                html += "<td><b>Delete</b></td>";                         
                html += "</tr>";                
                for(let i = 0; i<v.files.length; i+=1) {
                    html += "<tr>";
                    html += "<td>" + v.files[i].name + "</td>";
                    html += "<td width=\"15%\">" + toFriendlySize(v.files[i].size) + "</td>";
                    html += "<td width=\"10%\"><button onclick=runProgram('" + v.files[i].name + "')>Run</button></td>";
                    html += "<td width=\"10%\"><button onclick='readProgram(\"" + v.files[i].name + "\", " + v.files[i].size + ")'>Read</button></td>";  
                    html += "<td width=\"10%\"><button onclick='CallAPI(\"/delete_file?name=" + v.files[i].name + "\")'>Delete</button></td>";                  		    		                                     
                    html += "</tr>";
                }
                html += "</table>";
                let percentUsed = (v.flash.used / v.flash.total) * 100.0
                html += "<br>Storage: " + toFriendlySize(v.flash.used) + " / " + toFriendlySize(v.flash.total) + " - " + percentUsed.toFixed(1) + "%";
                document.getElementById("files").innerHTML = html;
            } else {
                document.getElementById("files").innerHTML = "Flash empty";             
            }
        });
    }

    let sendpos = 0;
    let sendsize = 0;
    let chunksize = 512;
    let sending = false;
    
    function sendFile() {
        if(sending) {
            sendsize = 0;
            return;
        }
        let btn = document.getElementById("sendfile");
        let filename = document.getElementById("filename").value;
        let data = document.getElementById("filedata").value;

        if(filename == "") {
            alert("Enter a name for the program.");
            return;
        }

        if(data == "") {
            alert("Enter some program data.");
            return;
        }
        
        let datalen = data.length;
        sending = true;
        sendpos = 0;
        sendsize = datalen;
        
        function sendChunk() {
            if(sendsize == 0) {
                btn.innerHTML = "Send Program";
                sending = false;
                return;
            }
            btn.innerHTML = "Sent " + sendpos + "/" + datalen + " bytes - Abort";
            let len = Math.min(sendsize, chunksize);
            line = data.substr(sendpos, len);
            sendsize -= len;
            sendpos += len;
            CallAPI("/append_file?name=" + encodeURIComponent(filename) + "&line=" + encodeURIComponent(line),
                    function(response, request) {
                        if(response != "OK") {
                            alert("Error sending " + request.status + " " + request.response);
                            sendsize = 0;
                        }
                        sendChunk();
                    });
        }

        sendChunk();
    }
    
  </script>
  <body onload="init()">
    <h1> &#x1F58D; Plotter &#x1F58D;</h1>
    <hr>
    <b>&#x2699; Status</b><br>
    Uptime <input type="text" id="uptime" readonly=1 style="width: 10%"><br>
    Action <input type="text" id="action" readonly=1 style="width: 10%"><br>
    Motor Position <input type="text" id="left_pos" readonly=1 style="width: 5%"> <input type="text" id="right_pos" readonly=1 style="width: 5%"> <input type="text" id="z_pos" readonly=1 style="width: 5%"><br>
    Page Position <input type="text" id="lastx" readonly=1 style="width: 5%"> <input type="text" id="lasty" readonly=1 style="width: 5%"><br>
    Baseline / Zero length (mm) <input type="text" id="baselinelen" readonly=1 style="width: 5%"> <input type="text" id="zerolen" readonly=1 style="width: 5%"><br>
    Tick Rate<input type="text" id="tickrate" readonly=1 style="width: 5%"><br>
    Smoothing<input type="text" id="smoothing" readonly=1 style="width: 5%"><br>
    <div id="progtext"><br><b>Progress</b></div>
    <progress id="progress" value="0" max="100" style="width: 20%"></progress><br>
    <hr>
    <b>&#x1F3AE; Controls</b><br>
    <button onclick="CallAPI('/action?name=stop')" style="width: 25%">STOP</button><br>
    <button onclick="CallAPI('/action?name=home')" style="width: 25%">HOME</button><br>
    <button onclick="CallAPI('/set_home')" style="width: 25%">SET HOME</button><br>
    <!--<button onclick="CallAPI('/action?name=test')" style="width: 25%">TEST DRAWING</button><br>        -->
    <button onclick="CallAPI('/action?name=up')" style="width: 12%">UP</button> <button onclick="CallAPI('/action?name=down')" style="width: 12%">DOWN</button><br>
    <button onclick="CallAPI('/action?name=left_in')" style="width: 12%">LEFT IN</button> <button onclick="CallAPI('/action?name=left_out')" style="width: 12%">LEFT OUT</button><br>
    <button onclick="CallAPI('/action?name=right_in')" style="width: 12%">RIGHT IN</button> <button onclick="CallAPI('/action?name=right_out')" style="width: 12%">RIGHT OUT</button><br>
    Baseline length (mm) <input type="text" id="baselinelen_in"> <button onclick="CallAPIWithValue('/set_baseline', 'baselinelen_in')" style="width: 5%">SET</button><br>    
    Zero length (mm) <input type="text" id="zerolen_in"> <button onclick="CallAPIWithValue('/set_zerolen', 'zerolen_in')" style="width: 5%">SET</button><br>    
    Tick Rate <input type="text" id="tickrate_in"> <button onclick="CallAPIWithValue('/set_tickrate', 'tickrate_in')" style="width: 5%">SET</button><br>    
    Smoothing <input type="text" id="smoothing_in"> <button onclick="CallAPIWithValue('/set_smoothing', 'smoothing_in')" style="width: 5%">SET</button>    
    <hr>
    <b>&#x1F4BE; Flash Memory</b><br>
    <p id="files"></p>
    <p id="status"></p>    
    <button onclick="refreshFileList()" style="width: 25%">Refresh</button><br>    
    <hr>
    <b>&#x1F527; Load Program</b><br>
    <button id="sendfile" onclick="sendFile()" style="width: 25%">Send Program</button><br>
    Name <input type="text" id="filename" style="width: 10%" placeholder="Name for program"><br>
    <textarea id="filedata" rows="20" cols="60"></textarea>
    
  </body>
</html>


)_X_";

void HandleRoot()
{
  // Build an HTML page to display on the web-server root address
  g_server.send ( 200, "text/html", g_bodyText );
}

void HandleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += g_server.uri();
  message += "\nMethod: ";
  message += ( g_server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += g_server.args();
  message += "\n";

  for ( uint8_t i = 0; i < g_server.args(); i++ ) {
    message += " " + g_server.argName ( i ) + ": " + g_server.arg ( i ) + "\n";
  }

  g_server.send ( 404, "text/plain", message );
}

void SetupWifi()
{
  bool useApMode = g_wifiSettings.m_apMode;
  IPAddress myIP;
  if(!useApMode) {
    WiFi.begin(g_wifiSettings.m_ssid, g_wifiSettings.m_password);
    int count = 0;
    while (WiFi.status() != WL_CONNECTED && count < 30) {
      delay(1000);
      ++count;
    }
    if(WiFi.status() != WL_CONNECTED) {
      useApMode = true;
    } else {
      myIP = WiFi.localIP();
      Serial.begin(115200);
      Serial.println(myIP);
      Serial.flush();
      Serial.end();
    }
  }

  if(useApMode) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(g_apIP, g_apIP, IPAddress(255, 255, 255, 0));
		// update these as needed.
    const char *ssid = "PLOTTER";
    const char *password = "XXXXXX";
	  WiFi.softAP(ssid, password);
    myIP = WiFi.softAPIP();
  } 

  g_server.on("/", HandleRoot);

  g_server.on("/action", []() {
    bool success = false;
    if(g_server.args() == 1) 
      success = WallPlotter::Instance().setActionName(g_server.arg(0).c_str());

    if(success)
      g_server.send(200, "text/plain", "OK");
    else
      g_server.send(400, "text/plain", "ERROR");
    
  });

  g_server.on("/action_stop", []() {
    WallPlotter::Instance().setAction(WallPlotter::STOP);
    g_server.send(200, "text/plain", "STOP");
  });

  g_server.on("/status", []() {
    auto& inst = WallPlotter::Instance();
    String ret = "{\"status\":{";
    ret += String("\"uptime\":\"") + millis() + "\", ";
    ret += String("\"action\":\"") + inst.getActionName() + "\", ";
    ret += String("\"absleft\":\"") + inst.getAbsPosLeft() + "\", ";
    ret += String("\"absright\":\"") + inst.getAbsPosRight() + "\", ";
    ret += String("\"absz\":\"") + inst.getAbsPosZ() + "\", ";
    ret += String("\"lastx\":\"") + inst.getLastPosX() + "\", ";
    ret += String("\"lasty\":\"") + inst.getLastPosY() + "\", ";
    ret += String("\"progress\":\"") + inst.getProgramProgress() + "\", ";
    ret += String("\"baselinelen\":\"") + inst.getBaseLineLen() + "\", ";
    ret += String("\"zerolen\":\"") + inst.getZeroLen() + "\", ";
    ret += String("\"smoothing\":\"") + g_smoothing + "\", ";
    ret += String("\"tickrate\":\"") + g_stepperTimeEvent.getTickRate() + "\"";
    ret += "}}";
    g_server.send(200, "application/json", ret);
  });

  g_server.on("/uptime", []() {
    char buf[32];
    sprintf(buf, "%lu", millis());
    g_server.send(200, "text/plain", buf);
  });

  g_server.on("/get_action", []() {
    g_server.send(200, "text/plain", WallPlotter::Instance().getActionName());
  });

  g_server.on("/set_home", []() {
    WallPlotter::Instance().resetAbsPositions();
    g_server.send(200, "text/plain", "OK");
  });

  g_server.on("/get_left_pos", []() {
    char buf[32];
    sprintf(buf, "%li", WallPlotter::Instance().getAbsPosLeft());
    g_server.send(200, "text/plain", buf);
  });

  g_server.on("/get_right_pos", []() {
    char buf[32];
    sprintf(buf, "%li", WallPlotter::Instance().getAbsPosRight());
    g_server.send(200, "text/plain", buf);
  });
  
  g_server.on("/append_file", []() {
    if(g_server.args() != 2) {
      g_server.send(400, "text/plain", String("Too few args ") + g_server.args());
      return;
    }
    if(!g_server.hasArg("name")) {
      g_server.send(400, "text/plain", "Missing 'name'");
      return;
    }
    if(!g_server.hasArg("line")) {
      g_server.send(400, "text/plain", "Missing 'line'");
      return;
    }
    if(FileSystem::Instance().appendFile(g_server.arg("name"), g_server.arg("line")))
      g_server.send(200, "text/plain", "OK");
    else
      g_server.send(500, "text/plain", "FS error");
  });

  g_server.on("/get_file_chunk", []() {
    if(g_server.args() != 2 || !g_server.hasArg("name") || !g_server.hasArg("pos")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }
    char buf[513];
    if(FileSystem::Instance().readFileChunk(g_server.arg("name"), atol(g_server.arg("pos").c_str()), (uint8_t *)buf, sizeof(buf)) > 0){
      g_server.send(200, "text/plain", buf);        
    } else {
      g_server.send(500, "text/plain", "FS error");
    }
  });

  g_server.on("/get_dir", []() {
    unsigned long diskTotal = 3 * 1024 * 1024, diskUsed = 0;
    String ret;
    auto& fs = FileSystem::Instance();
    fs.openDir();
    String name;
    unsigned long size;
    int count = 0;
    ret += "{\"files\":[";
    while(fs.getNextFileInfo(name, size)) {
      if(count > 0)
        ret += ",";
      ret += "{\"name\": \"" + name + "\", \"size\": \"" + size + "\"}";
      ++count;
      diskUsed += size;
    }
    ret += "],\n";
    fs.closeDir();
    
    ret += "\"flash\":{\"total\":\"";
    ret += diskTotal;
    ret += "\", \"used\":\"";
    ret += diskUsed;
    ret += "\"}}";
    
    g_server.send(200, "application/json", ret);
  });
  
   g_server.on("/run_program", []() {
    if(g_server.args() != 1 || !g_server.hasArg("name")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }
    if(WallPlotter::Instance().runProgram(g_server.arg("name")))
      g_server.send(200, "text/plain", "OK");
    else
      g_server.send(500, "text/plain", "Error");
   });

   g_server.on("/delete_file", []() {
    if(g_server.args() != 1 || !g_server.hasArg("name")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }
    if(FileSystem::Instance().removeFile(g_server.arg("name")))
      g_server.send(200, "text/plain", "OK");
    else
      g_server.send(500, "text/plain", "Error");
   });

   g_server.on("/set_baseline", []() {
    if(g_server.args() != 1 || !g_server.hasArg("value")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }

    float baseline = atof(g_server.arg("value").c_str());
    if(baseline > 0)
      WallPlotter::Instance().setBaseLineLen(baseline);
    g_server.send(200, "text/plain", "OK");
   });

  g_server.on("/set_zerolen", []() {
    if(g_server.args() != 1 || !g_server.hasArg("value")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }

    float zerolen = atof(g_server.arg("value").c_str());
    if(zerolen > 0)
      WallPlotter::Instance().setZeroLen(zerolen);
    g_server.send(200, "text/plain", "OK");
   });

  g_server.on("/set_tickrate", []() {
    if(g_server.args() != 1 || !g_server.hasArg("value")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }

    int tickrate = atoi(g_server.arg("value").c_str());
    if(tickrate > 0)
      g_stepperTimeEvent.setTickRate(tickrate);
    g_server.send(200, "text/plain", "OK");
   });

  g_server.on("/set_smoothing", []() {
    if(g_server.args() != 1 || !g_server.hasArg("value")) {
      g_server.send(400, "text/plain", "Invalid params");
      return;
    }

    int smoothing = atoi(g_server.arg("value").c_str());
    g_smoothing = smoothing > 0;
    g_server.send(200, "text/plain", "OK");
   });

  g_server.onNotFound(HandleNotFound);
	
	g_server.begin();
}

/// ----------------------------------------------
/// Arduino interface implementations
/// ----------------------------------------------

void setup() {
  SetupWifi();
  WallPlotter::Instance().setup();
}

void loop() {
  g_server.handleClient();
  unsigned long now = millis();
  g_stepperTimeEvent.tick(now);
}
