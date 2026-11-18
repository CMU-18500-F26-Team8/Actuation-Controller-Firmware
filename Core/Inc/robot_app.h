/**
 * @file    robot_app.h
 * @brief   Top level: Pi-controlled driving (ROBOT_MODE 0) or the bench demo
 *          (ROBOT_MODE 1). main.c calls these two functions only.
 */
#ifndef ROBOT_APP_H
#define ROBOT_APP_H

void robot_app_init(void);

/** Call continuously from the main while(1) loop. Non-blocking. */
void robot_app_poll(void);

#endif /* ROBOT_APP_H */
