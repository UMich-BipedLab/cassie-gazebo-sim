#include "KeyboardInput.hpp"
#include <iostream>

KeyboardInput::KeyboardInput():
LV_ {0},
LH_ {0},
RV_ {0}, 
RH_ {0}, 
S1_ {0}, 
S2_ {0}, 
LS_ {0}, 
RS_ {0}, 
SA_ {0}, 
SB_ {0}, 
SC_ {0}, 
SD_ {0}, 
SE_ {0}, 
SF_ {0}, 
SG_ {0}, 
SH_ {0}
{   
    std::cout<<"created keyboard class \n";
}

KeyboardInput::~KeyboardInput()
{
    restoreTerminalSettings();
}

void KeyboardInput::getInput() 
{
    std::lock_guard<std::mutex> lock(mutex_);
    // std::cout << "ready to get char\n";
    disableWaitingForEnter();
    while(true){
        ch = getchar();
        // std::cout<<"the char is: " << ch << "\n";
        if(ch==27){     // if "esc" is pressed, exit while and restore terminal setting
            restoreTerminalSettings();
            break;
        }
        switch(ch){
            // W for LV_ ++
            case 119:
                // std::cout<<"You pressed w \n";
                LV_<100? LV_+=1:LV_=100;
                break;
            // S for LV_ -- 
            case 115:
                // std::cout<<"You pressed s \n";
                LV_>-100? LV_-=1:LV_=-100;
                break;
            // D for LH_ ++
            case 100:
                // std::cout<<"You pressed a \n";
                LH_<100? LH_+=1:LH_=100;
                break; 
            // A for LH_ --
            case 97:   
                // std::cout<<"You pressed d \n";
                LH_>-100? LH_-=1:LH_=-100;
                break;
            // I for RV_ ++
            case 105:   
                // std::cout<<"You pressed i \n";
                RV_<100? RV_+=1:RV_=100;
                break;
            // K for RV_ --
            case 107:   
                // std::cout<<"You pressed k \n";
                RV_>-100? RV_-=1:RV_=-100;
                break;
            // L for RH_ ++
            case 108:   
                // std::cout<<"You pressed l \n";
                RH_<100? RH_+=1:RH_=100;
                break;
            // J for RH_ --
            case 106:   
                // std::cout<<"You pressed l \n";
                RH_>-100? RH_-=1:RH_=-100;
                break;
            // 6 for S1_ ++
            case 54:   
                // std::cout<<"You pressed 6 \n";
                S1_<100? S1_+=1:S1_=100;
                break;
            // 5 for S1_ --
            case 53:   
                // std::cout<<"You pressed 5 \n";
                S1_>-100? S1_-=1:S1_=-100;
                break;
            // 8 for S2_ ++
            case 56:   
                // std::cout<<"You pressed 8 \n";
                S2_<100? S2_+=1:S2_=100;
                break;
            // 7 for S2_ --
            case 55:   
                // std::cout<<"You pressed 7 \n";
                S2_>-100? S2_-=1:S2_=-100;
                break;
            // X for LS_ ++
            case 120:   
                // std::cout<<"You pressed x \n";
                LS_<100? LS_+=1:LS_=100;
                break;
            // Z for LS_ --
            case 122:   
                // std::cout<<"You pressed z \n";
                LS_>-100? LS_-=1:LS_=-100;
                break;
            // , for RS_ ++
            case 44:   
                // std::cout<<"You pressed , \n";
                RS_<100? RS_+=1:RS_=100;
                break;
            // M for RS_ --
            case 109:   
                // std::cout<<"You pressed m \n";
                RS_>-100? RS_-=1:RS_=-100;
                break;
            // 2 for SA_ ++
            case 50:   
                // std::cout<<"You pressed m \n";
                SA_<100? SA_+=100:SA_=100;
                break;
            // 1 for SA_ --
            case 49:   
                // std::cout<<"You pressed m \n";
                SA_>-100? SA_-=100:SA_=-100;
                break;
            // 4 for SB_ ++
            case 52:   
                // std::cout<<"You pressed m \n";
                SB_<100? SB_+=100:SB_=100;
                break;
            // 3 for SB_ --
            case 51:   
                // std::cout<<"You pressed m \n";
                SB_>-100? SB_-=100:SB_=-100;
                break;
            // 0 for SC_ ++
            case 48:   
                // std::cout<<"You pressed m \n";
                SC_<100? SC_+=100:SC_=100;
                break;
            // 9 for SC_ --
            case 57:   
                // std::cout<<"You pressed m \n";
                SC_>-100? SC_-=100:SC_=-100;
                break;
            // = for SD_ ++
            case 61:   
                // std::cout<<"You pressed m \n";
                SD_<100? SD_+=100:SD_=100;
                break;
            // - for SD_ --
            case 45:   
                // std::cout<<"You pressed m \n";
                SD_>-100? SD_-=100:SD_=-100;
                break;
            // C for SE_ ++
            case 99:   
                // std::cout<<"You pressed m \n";
                SE_<100? SE_+=100:SE_=100;
                break;
            // V for SE_ --
            case 118:   
                // std::cout<<"You pressed m \n";
                SE_>-100? SE_-=100:SE_=-100;
                break;
            // R for SF_ ++
            case 114:   
                // std::cout<<"You pressed m \n";
                (SF_+200<100)? SF_+=200:SF_=100;
                break;
            // E for SF_ --
            case 101:   
                // std::cout<<"You pressed m \n";
                (SF_-200>-100)? SF_-=200:SF_=-100;
                break;
            // N for SG_ ++
            case 110:   
                // std::cout<<"You pressed m \n";
                SG_<100? SG_+=100:SG_=100;
                break;
            // B for SG_ --
            case 98:   
                // std::cout<<"You pressed m \n";
                SG_>-100? SG_-=100:SG_=-100;
                break;
            // U for SH_ ++
            case 117:   
                // std::cout<<"You pressed m \n";
                (SH_+200<100)? SH_+=200:SH_=100;
                break;
            // Y for SH_ --
            case 121:   
                // std::cout<<"You pressed m \n";
                (SH_-200>-100)? SH_-=200:SH_=-100;
                break;
            // default:
                // std::cout<<"what r u doing dude?\n";
                // break;
        }
    }
}

void KeyboardInput::restoreTerminalSettings(void)
{
    struct termios offt;
    tcgetattr(0, &offt);  /* Access current terminal settings */
    offt.c_lflag |= ICANON;     /* Change terminal settings back */
    offt.c_lflag |= ECHO;
    tcsetattr(0, TCSANOW, &offt);
}

void KeyboardInput::disableWaitingForEnter(void)
{
    struct termios newt;
    tcgetattr(0, &newt);  /* Save terminal settings */
    newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
    tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

double KeyboardInput::LV(void)
{
    return LV_;
}
double KeyboardInput::LH(void)
{
    return LH_;
}
double KeyboardInput::RV(void)
{
    return RV_;
}
double KeyboardInput::RH(void)
{
    return RH_;
}
double KeyboardInput::S1(void)
{
    return S1_;
}
double KeyboardInput::S2(void)
{
    return S2_;
}
double KeyboardInput::LS(void)
{
    return LS_;
}
double KeyboardInput::RS(void)
{
    return RS_;
}
double KeyboardInput::SA(void)
{
    return SA_;
}
double KeyboardInput::SB(void)
{
    return SB_;
}
double KeyboardInput::SC(void)
{
    return SC_;
}
double KeyboardInput::SD(void)
{
    return SD_;
}
double KeyboardInput::SE(void)
{
    return SE_;
}
double KeyboardInput::SF(void)
{
    return SF_;
}
double KeyboardInput::SG(void)
{
    return SG_;
}
double KeyboardInput::SH(void)
{
    return SH_;
}