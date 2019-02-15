#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H
#include<mutex>
#include<thread>
#include <termios.h>

class KeyboardInput {
private:    
    // LV, LH, RV, RH, S1, S2, LS, RS, SA, SB, SC, SD, SE, SF, SG, SH 
    std::mutex mutex_;
    double LV_, LH_, RV_, RH_, S1_, S2_, LS_, RS_, SA_, SB_, SC_, SD_, SE_, SF_, SG_, SH_;
    int ch;
    struct termios oldt;

    /**
     * @brief restore terminal settings
     */
    void restoreTerminalSettings();

    /**
     * @brief enable reading keyboard without pressing enter
     */
    void disableWaitingForEnter();


    

public:
    /**
     * @brief KeyboardInput constructor for the plugin class
     */
    KeyboardInput();

    /**
     * @brief KeyboardInput destructor for the plugin class
     */
    ~KeyboardInput();

    /**
     * @brief getInput() reads the input from keyboard
     */
    void getInput();
    /** 
     * @brief public functions for reading radio values
     */
    double LV();
    double LH();
    double RV();
    double RH();
    double S1();
    double S2();
    double LS();
    double RS();
    double SA();
    double SB();
    double SC();
    double SD();
    double SE();
    double SF();
    double SG();
    double SH();
};
#endif

