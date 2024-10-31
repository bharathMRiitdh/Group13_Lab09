#include <stdint.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define I2C0_MCS_RUN       0x00000001  // I2C Master Enable
#define I2C0_MCS_STOP      0x00000004  // I2C Master Stop
#define I2C0_MCS_START     0x00000002  // I2C Master Start
#define I2C0_MCS_ERROR     0x00000002  // I2C Master Error
#define MCP4725_ADDR       0x60         // Default I2C address of MCP4725 DAC
#define PI                 3.14159265
#define NUM_POINTS         100           // Number of points in one sine wave cycle
#define DELAY_MS           1              // Delay in milliseconds for frequency control

// Function Prototypes
void I2C0_Init(void);
void I2C0_Write(uint8_t device_addr, uint16_t data);
void delayMs(int n);
void generateSineWave(int amplitude, int offset);

uint16_t sine_wave[NUM_POINTS];  // Array to store sine wave data points

int main(void) {
    I2C0_Init();                       // Initialize I2C0 for DAC communication
    generateSineWave(2047, 2048);    // Generate sine wave data (12-bit DAC max 4095)

    while (1) {
        // Send the sine wave data points to the DAC continuously
        int i;
        for ( i = 0; i < NUM_POINTS; i++) {
            I2C0_Write(MCP4725_ADDR, sine_wave[i]);  // Write the sine value to DAC
            delayMs(DELAY_MS);   // Control the frequency of the sine wave by adjusting the delay
        }
    }
}

// I2C0 initialization function
void I2C0_Init(void) {
    SYSCTL_RCGCI2C_R |= 0x01;       // Enable clock for I2C0
    SYSCTL_RCGCGPIO_R |= 0x02;      // Enable clock for Port B

    GPIO_PORTB_AFSEL_R |= 0x0C;     // Enable alternate functions for PB2 and PB3 (SCL, SDA)
    GPIO_PORTB_ODR_R |= 0x08;       // Set SDA (PB3) as open drain
    GPIO_PORTB_DEN_R |= 0x0C;       // Enable digital functionality on PB2 and PB3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) | 0x00003300; // Configure PB2 and PB3 as I2C

    I2C0_MCR_R = 0x10;              // Initialize I2C0 as master
    I2C0_MTPR_R = 9;                // Set TPR value for 100 Kbps I2C clock speed
}

// Function to write data to MCP4725 DAC over I2C
void I2C0_Write(uint8_t device_addr, uint16_t data) {
    uint8_t upper_data_byte = (data >> 4) & 0xFF;    // Upper 8 bits (12-bit data format)
    uint8_t lower_data_byte = (data << 4) & 0xF0;    // Lower 4 bits with padding

    I2C0_MSA_R = (device_addr << 1);                 // Set slave address with write operation
    I2C0_MDR_R = 0x40;                               // Command to set DAC register
    I2C0_MCS_R = I2C0_MCS_START | I2C0_MCS_RUN;      // Send start condition and run

    while (I2C0_MCS_R & I2C0_MCS_ERROR);             // Wait until transmission is done

    I2C0_MDR_R = upper_data_byte;                    // Send upper data byte
    I2C0_MCS_R = I2C0_MCS_RUN;                       // Continue transmission

    while (I2C0_MCS_R & I2C0_MCS_ERROR);             // Wait until transmission is done

    I2C0_MDR_R = lower_data_byte;                    // Send lower data byte
    I2C0_MCS_R = I2C0_MCS_STOP | I2C0_MCS_RUN;       // Stop transmission

    while (I2C0_MCS_R & I2C0_MCS_ERROR);             // Wait until transmission is done
}

// Function to generate sine wave values
void generateSineWave(int amplitude, int offset) {
    int i;
    for ( i = 0; i < NUM_POINTS; i++) {
        // Generate sine wave value in the range of 0 to 4095 for 12-bit DAC
        sine_wave[i] = (uint16_t)(amplitude * sin(2 * PI * i / NUM_POINTS) + offset);
    }
}

// Simple delay function
void delayMs(int n) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < 3180; j++) {
            // Do nothing
        }
    }
}
