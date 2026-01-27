
/* These functions are based on the Arduino test program at
*  https://github.com/adafruit/Adafruit-SSD1351-library/blob/master/examples/test/test.ino
*
*  You can use these high-level routines to implement your
*  test program.
*/

// TODO Configure SPI port and use these libraries to implement
// an OLED test program. See SPI example program.

#include "oled_test.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "i2c_if.h"

//*****************************************************************************
//  function delays 3*ulCount cycles
static void delay(unsigned long ulCount){
    int i;
    do{
        ulCount--;
        for (i=0; i<65535; i++);
    }while(ulCount);
}

#define BMA222_ADDR   0x18
#define REG_ACC_X     0x03
#define REG_ACC_Y     0x05

#define DEADZONE      4
#define MAX_STEP_PIXELS   7
#define FULL_TILT_COUNTS  64


static int ball_x, ball_y;
static int old_x, old_y; 

// helper function
static int clamp_int(int v, int lo, int hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void clampBall(void){
    ball_x = clamp_int(ball_x, BALL_RADIUS, (OLED_WIDTH  - 1) - BALL_RADIUS);
    ball_y = clamp_int(ball_y, BALL_RADIUS, (OLED_HEIGHT - 1) - BALL_RADIUS);
}


static void eraseBall(int x, int y){
    fillCircle(x, y, BALL_RADIUS, BLACK);
}
static void drawBall(int x, int y){
    fillCircle(x, y, BALL_RADIUS, BALL_COLOR);
}

// Reading accelerometer data
static signed char bma222_read_s8(unsigned char reg){
    unsigned char raw = 0;
    I2C_IF_Write(BMA222_ADDR, &reg, 1, 0);
    I2C_IF_Read(BMA222_ADDR, &raw, 1);
    return (signed char)raw;
}

// Convert tilt to movement speed
static int accel_to_step(int a){
    int sign = 1;
    if (a < 0) { sign = -1; a = -a; }

    if (a <= DEADZONE) return 0;

    a -= DEADZONE;

    if (a > FULL_TILT_COUNTS) a = FULL_TILT_COUNTS;

    int step = 1 + (a * (MAX_STEP_PIXELS - 1)) / FULL_TILT_COUNTS;

    return sign * step;
}

// Move left or right using X axis
void ballRoll(void){
    int ax = (int)bma222_read_s8(REG_ACC_X);
    int dx = accel_to_step(ax);
    ball_x += dx;
}

// Move up or down using Y axis
void ballPitch(void){
    int ay = (int)bma222_read_s8(REG_ACC_Y);
    int dy = accel_to_step(ay);


// ball_y += dy OR ball_y -= dy;
    ball_y += dy;
}

void checkoffInit(void){
    ball_x = OLED_WIDTH / 2;
    ball_y = OLED_HEIGHT / 2;

    old_x = ball_x;
    old_y = ball_y;
    
    // Draw initial ball
    fillScreen(BLACK);
    drawBall(ball_x, ball_y);
}

void checkoffLoop(void){
    // Save old position
    old_x = ball_x;
    old_y = ball_y;

    // Update position from accelerometer
    ballRoll();
    ballPitch();
    clampBall();

    // Only redraw if position changed
    if (ball_x != old_x || ball_y != old_y){
        eraseBall(old_x, old_y);
        drawBall(ball_x, ball_y);
    }

    // Max speed
    delay(5);
}



















