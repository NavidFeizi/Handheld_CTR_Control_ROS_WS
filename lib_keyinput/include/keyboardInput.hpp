#ifndef KeyboardManipulator_HPP
#define KeyboardManipulator_HPP

#include "keyboardInput.hpp"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <vector>
#include <thread>
#include <chrono>

#include <iostream>

class KeyboardInput
{
public:
    KeyboardInput(std::vector<double> &target, std::vector<double> &coefficients, int *counter);
    ~KeyboardInput();
    void startInputThread();
    void reset();
    bool exit();

private:
    Display *display;
    int *counter;
    std::vector<double> &target;
    std::vector<double> &coefficients;
    bool shiftQPressed;
    std::thread inputThread;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastPressTime;

    void inputLoop();
};

#endif // KEYBOARD_INPUT_HPP