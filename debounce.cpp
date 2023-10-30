#include "debounce.h"
#include "myDelay.h"

extern uint8_t globalIndex;
static uint8_t maxButtons;
extern Timer myTimer;

void startButon(_sButton *myButton, uint8_t numButton){
    maxButtons = numButton;
    for(globalIndex=0; globalIndex<maxButtons;globalIndex++){
        myButton[globalIndex].mask = 0;
        myButton[globalIndex].mask |= (1<<globalIndex) ;
        myButton[globalIndex].currentState = BUTTON_UP;
        myButton[globalIndex].stateInput = NO_EVENT;
        myButton[globalIndex].flagDetected = NOFLAG;
        myButton[globalIndex].timePressed = 0;
        myButton[globalIndex].timeDiff = 0;
    }
}
void buttonTask(_delay_t *timedebounce, _sButton *myButton, uint8_t statePulsadores){   
    if(delayRead(timedebounce)){
        //statePulsadores = pulsadores.read();
        for(globalIndex=0; globalIndex<maxButtons;globalIndex++){
            myButton[globalIndex].stateInput = (_eEventInput)((statePulsadores & myButton[globalIndex].mask)>>globalIndex);
            switch (myButton[globalIndex].currentState){
                case BUTTON_UP:
                    myButton[globalIndex].flagDetected = NOFLAG;
                    if(myButton[globalIndex].stateInput==PRESSED)
                        myButton[globalIndex].currentState=BUTTON_FALLING;
                break;
                case BUTTON_FALLING:
                    if(myButton[globalIndex].stateInput==PRESSED){
                            myButton[globalIndex].currentState=BUTTON_DOWN;
                            myButton[globalIndex].flagDetected = FALLINGFLAG;
                             myButton[globalIndex].timePressed = myTimer.read_ms();
                    }else{
                        myButton[globalIndex].currentState=BUTTON_UP;
                    }
                break;
                case BUTTON_DOWN:
                    if(myButton[globalIndex].stateInput==NOT_PRESSED)
                        myButton[globalIndex].currentState=BUTTON_RISING;
                break;
                case BUTTON_RISING:
                    if(myButton[globalIndex].stateInput==NOT_PRESSED){
                            myButton[globalIndex].currentState=BUTTON_UP;
                             myButton[globalIndex].flagDetected = RISINGFLAG;
                            myButton[globalIndex].timeDiff = myTimer.read_ms()-myButton[globalIndex].timePressed;
                    }else{
                        myButton[globalIndex].currentState=BUTTON_DOWN;
                    }
                break;
                default:
                    myButton[globalIndex].currentState=BUTTON_UP;
                break;
            }         
        }
    }
    
}