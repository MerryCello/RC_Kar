#include "msp.h"
#include "stdbool.h"

// Signals to control car
#define SIGNAL_MOTOR_1_STOP     0x10
#define SIGNAL_MOTOR_1_FORWARD  0x11
#define SIGNAL_MOTOR_1_BACKWARD 0x12
#define SIGNAL_MOTOR_1_STOP     0x20
#define SIGNAL_MOTOR_2_FORWARD  0x21
#define SIGNAL_MOTOR_2_BACKWARD 0x22

// macro defines for the Car
    // on port 3
#define RECEIVER BIT3
    // on port 4
#define MOTOR_1_FORWARD  BIT2
#define MOTOR_1_BACKWARD BIT3
#define MOTOR_1_STOP     ~(BIT2 | BIT3)
#define MOTOR_2_FORWARD  BIT4
#define MOTOR_2_BACKWARD BIT5
#define MOTOR_2_STOP     ~(BIT4 | BIT5)

// macro defines for the remote
    // on port 3
#define TRANSMITTER BIT2


void setup(DIO_PORT_Odd_Interruptable_Type*, uint16_t, uint16_t, DIO_PORT_Odd_Interruptable_Type*, uint16_t, uint16_t);
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
    setup(P4, MOTOR_1_FORWARD | MOTOR_1_BACKWARD, MOTOR_2_FORWARD | MOTOR_2_BACKWARD,
          P3, RECEIVER, TRANSMITTER);

    while(true) {
//        handleClientBLE();

        switch (dataReceived()) {
            case SIGNAL_MOTOR_1_FORWARD:
                break;
        }
    }
}

/********************
 * SETUP
 ********************/
void setup(
    DIO_PORT_Odd_Interruptable_Type* motorsPort, // I/O Port
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

/********************
 * Remote main.c code
 ********************/
//void main(void)
//{
//    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
//}
