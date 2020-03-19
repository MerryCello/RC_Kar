#include "msp.h"
#include "stdbool.h"
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <string.h>

#define CAR
//#define REMOTE

#define CLOCK_SPEED 3000000
#define BAUD_RATE 9600

#define DELAY 300


#define RT_PORT     P3
#define TRANSMITTER BIT2
#define RECEIVER    BIT3
// Signals to control car
#define SIGNAL_STOP     0x10
#define SIGNAL_FORWARD  0x11
#define SIGNAL_BACKWARD 0x12
#define SIGNAL_LEFT     0x13
#define SIGNAL_RIGHT    0x14


// macro defines for the remote
#ifdef REMOTE
    #define BTN_PORT P4
    #define FORWARD  BIT1
    #define BACKWARD BIT2
    #define LEFT     BIT3
    #define RIGHT    BIT4

    void setup(DIO_PORT_Even_Interruptable_Type*, uint16_t, uint16_t, DIO_PORT_Odd_Interruptable_Type*, uint16_t, uint16_t);
    void send(char);
    char dataReceived(void);
#endif // REMOTE


// macro defines for the Car
#ifdef CAR
        // on port 4
    #define MOTOR_PORT P4
    #define MOTOR_1_FORWARD  BIT2
    #define MOTOR_1_BACKWARD BIT3
    #define MOTOR_1_STOP     ~(BIT2 | BIT3)
    #define MOTOR_2_FORWARD  BIT4
    #define MOTOR_2_BACKWARD BIT5
    #define MOTOR_2_STOP     ~(BIT4 | BIT5)


    char RXData[1000] = "";
    int buf_ind = 0, i = 0;

    void setup(DIO_PORT_Even_Interruptable_Type*, uint16_t, uint16_t, DIO_PORT_Odd_Interruptable_Type*, uint16_t, uint16_t);
    void initHMslave(void);
    void handleClientBLE(void);
    void sendData(char *data, char *ack, char *err, const int timeout);
    void delay_ms(uint16_t ms);
    void send(char);
    char dataReceived(void);

/********************
 * Car main.c code
 ********************/
void main(void)
{
    setup(MOTOR_PORT, MOTOR_1_FORWARD | MOTOR_1_BACKWARD, MOTOR_2_FORWARD | MOTOR_2_BACKWARD,
          RT_PORT, RECEIVER, TRANSMITTER);

    char data;
    while(true) {
//        handleClientBLE();
        data = dataReceived();

        if (data == SIGNAL_STOP) {
            MOTOR_PORT->OUT &= MOTOR_1_STOP; // set motor_1 to off
            MOTOR_PORT->OUT &= MOTOR_2_STOP; // set motor_2 to off
        } else {
            MOTOR_PORT->OUT &= (MOTOR_1_STOP | MOTOR_2_STOP);
            if (data == SIGNAL_FORWARD) {
                MOTOR_PORT->OUT |= (MOTOR_1_FORWARD | MOTOR_2_FORWARD);
            }
            if (data == SIGNAL_BACKWARD) {
                MOTOR_PORT->OUT |= (MOTOR_1_BACKWARD | MOTOR_2_BACKWARD);
            }
            if (data == SIGNAL_LEFT) {
                MOTOR_PORT->OUT |= (MOTOR_2_FORWARD);
            }
            if (data == SIGNAL_RIGHT) {
                MOTOR_PORT->OUT |= (MOTOR_1_FORWARD);
            }
        }
    }
}

/********************
 * SETUP
 ********************/
void setup(
    DIO_PORT_Even_Interruptable_Type* motorsPort, // I/O Port
    uint16_t motorPin_1,
    uint16_t motorPin_2,
    DIO_PORT_Odd_Interruptable_Type* rtPort, // Receiver/Transmitter Port
    uint16_t receiverPin,
    uint16_t transmitterPin) {

//    stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

//    Setup output
    motorsPort->DIR |= motorPin_1;
    motorsPort->DIR |= motorPin_2;
    motorsPort->OUT &= ~(motorPin_1 | motorPin_2); // set both motors to off


//    Setup UART
    EUSCI_A2->CTLW0  = 0x00c1; // = 0000 0000 1100 0001 bit settings
    EUSCI_A2->BRW    = CLOCK_SPEED / BAUD_RATE;
    EUSCI_A2->MCTLW &= ~BIT0;

//    Setup P3.2 (receiver) and P3.3 (Transmitter)
    rtPort->DIR  &= ~(receiverPin); // set receiver on P3 as input
    rtPort->DIR  |= transmitterPin; // set transmitter on P3 as output
    rtPort->REN  |= receiverPin;    // turn on receiver pull resister
    rtPort->OUT  |= receiverPin;    // configure resistors as pull-up
    rtPort->SEL0 |= transmitterPin|receiverPin;    // set for GPIO
    rtPort->SEL1 &= ~(transmitterPin|receiverPin); // set for GPIO

    EUSCI_A2->CTLW0 &= ~(BIT0);

//    initHMslave();
}

void initHMslave(void) {
    sendData("AT\r\n", "OK", "ERROR", 1000);
    sendData("AT+RENEW\r\n", "OK", "ERROR", 1000);
    sendData("AT+RESET\r\n", "OK", "ERROR", 1000);
    delay_ms(3000);
}

void handleClientBLE(void) {
    char receivedData[1000] = "";

    if(buf_ind>0) {
        delay_ms(1000);
        strncpy(receivedData, RXData, buf_ind);
        memset(RXData, 0, sizeof(RXData));
        buf_ind = 0;
    }

    if(strstr(receivedData, "D") != NULL || strstr(receivedData, "d") != NULL) {
        // do something
    }
    else if(strstr(receivedData, "S") != NULL || strstr(receivedData, "s") != NULL) {
        // do something else
    }
}

/**
 * SEND
 * Transmits data to board2
 */
void send(char data) {
    while ((EUSCI_A2->IFG & BIT1) == 0);  // Busy.  Wait for previous output.
    EUSCI_A2->TXBUF = data;                // Start transmission when IFG = 1.
}

/**
 * DATA_RECEIVED
 * Reads the data received from board2 if there is any
 * Return data read
 */
char dataReceived(void) {
    if ((EUSCI_A2->IFG & BIT0) == 1) {
       return ( (char) (EUSCI_A2->RXBUF) );
    } else {
        return 0xEE;
    }
}

/***************************
 * SEND_DATA
 * does a thing, not sure what
 * oh, it sets up the HM10 for use
 * like command line for the HM10 :)
 ***************************/
void sendData(char *data, char *ack, char *err, const int timeout){
//    long int time = 0;
//    memset(RXData, 0, sizeof(RXData));
//    buf_ind = 0;
//
//    while (*data != 0x00)
//        UART_transmitData(EUSCI_A2_BASE, *data++);
//
//    while(timeout > time){
//        time++;
//        delay_ms(1);
//        if(strstr(RXData, ack) != NULL) break;
//        else if(strstr(RXData, err) != NULL) while(1);
//    }
//    delay_ms(100);
}

/**
 * DELAY_MS
 */
void delay_ms(uint16_t ms){
  uint16_t delay;
  volatile uint32_t i;

  for (delay = ms; delay >0 ; delay--)
    for (i=1200; i >0;i--);
}
#endif // CAR

#ifdef REMOTE
/********************
 * Remote main.c code
 ********************/
void main(void)
{
    setup(BTN_PORT, FORWARD | BACKWARD, LEFT | RIGHT,
          RT_PORT, RECEIVER, TRANSMITTER);

    int i;
    while(true) {
//      Delay for switch debouncing
        for (i = 0; i < DELAY; i++) {}

//  1)  Read BTNs
        bool f_Pressed = (BTN_PORT->IN & FORWARD) == FORWARD;
        bool b_Pressed = (BTN_PORT->IN & BACKWARD) == BACKWARD;
        bool l_Pressed = (BTN_PORT->IN & LEFT) == LEFT;
        bool r_Pressed = (BTN_PORT->IN & RIGHT) == RIGHT;

        if (f_Pressed || b_Pressed || l_Pressed || r_Pressed) {
            if (f_Pressed) {
                send(SIGNAL_FORWARD);
            } else if (b_Pressed) {
                send(SIGNAL_BACKWARD);
            } else if (l_Pressed) {
                send(SIGNAL_LEFT);
            } else if (r_Pressed) {
                send(SIGNAL_RIGHT);
            }
        } else {
            send(SIGNAL_STOP);
        }
    }
}

/********************
 * SETUP
 ********************/
void setup(
    DIO_PORT_Even_Interruptable_Type* btnsPort, // I/O Port
    uint16_t btnPin_1,
    uint16_t btnPin_2,
    DIO_PORT_Odd_Interruptable_Type* rtPort, // Receiver/Transmitter Port
    uint16_t receiverPin,
    uint16_t transmitterPin) {

//    stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

//    Setup inputs
    btnsPort->DIR &= ~(btnPin_1 | btnPin_2);


//    Setup UART
    EUSCI_A2->CTLW0  = 0x00c1; // = 0000 0000 1100 0001 bit settings
    EUSCI_A2->BRW    = CLOCK_SPEED / BAUD_RATE;
    EUSCI_A2->MCTLW &= ~BIT0;

//    Setup P3.2 (receiver) and P3.3 (Transmitter)
    rtPort->DIR  &= ~(receiverPin); // set receiver on P3 as input
    rtPort->DIR  |= transmitterPin; // set transmitter on P3 as output
    rtPort->REN  |= receiverPin;    // turn on receiver pull resister
    rtPort->OUT  |= receiverPin;    // configure resistors as pull-up
    rtPort->SEL0 |= transmitterPin|receiverPin;    // set for GPIO
    rtPort->SEL1 &= ~(transmitterPin|receiverPin); // set for GPIO

    EUSCI_A2->CTLW0 &= ~(BIT0);

//    initHMslave();
}

/**************************
 * SEND
 * Transmits data to board2
 **************************/
void send(char data) {
    while ((EUSCI_A2->IFG & BIT1) == 0);  // Busy.  Wait for previous output.
    EUSCI_A2->TXBUF = data;               // Start transmission when IFG = 1.
}

/*****************************************************
 * DATA_RECEIVED
 * Reads the data received from board2 if there is any
 * Return data read
 *****************************************************/
char dataReceived(void) {
    if ((EUSCI_A2->IFG & BIT0) == 1) {
       return ( (char) (EUSCI_A2->RXBUF) );
    } else {
        return 0xEE;
    }
}
#endif // REMOTE




