/*
 * test.h
 *
 *  Created on: Jan 27, 2024
 *      Author: rtsang
 */

#ifndef OLED_OLED_TEST_H_
#define OLED_OLED_TEST_H_

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

// OLED size
#define OLED_WIDTH 128 // width
#define OLED_HEIGHT 128 // height

// Ball properties
#define BALL_RADIUS  10
#define BALL_COLOR   WHITE




void ballRoll(void);
void ballPitch(void);
void checkoffInit(void);
void checkoffLoop(void);



#endif /* OLED_OLED_TEST_H_ */
