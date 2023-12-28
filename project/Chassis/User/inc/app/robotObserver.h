#ifndef _ROBOT_OBSERVER_H
#define _ROBOT_OBSERVER_H

#include "Observer.h"

extern StateObserver_t robot_observer;

void robotObserverInit(void);
void robotObserverUpdate(float theta, float x, float x_dot, float pitch);

#endif // !_ROBOT_OBSERVER_H
