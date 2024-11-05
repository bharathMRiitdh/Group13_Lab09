#include <stdint.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define I2C0_MCS_RUN       0x00000001  // Master operation in progress
#define I2C0_MCS_STOP      0x00000004  // Master generate STOP condition
#define I2C0_MCS_START     0x00000002  // Master generate START condition
#define I2C0_MCS_ERROR     0x00000002  // Error status
#define MCP4725_ADDR       0x60        // Default I2C address of the MCP4725 DAC
#define PI                 3.14159265  // Approximate value of Pi
#define NUM_POINTS         100         // Number of points to represent one sine wave cycle
#define DELAY_MS           1           // Delay in milliseconds for setting the frequency

// Function Prototypes
void I2C0_Init(void);                      // Initialize I2C0 for communication
void I2C0_Write(uint8_t device_addr, uint16_t data); // Write data to DAC via I2C
void delayMs(int n);                       // Delay function for frequency control
void generateSineWave(int amplitude, int offset); // Generate sine wave data

uint16_t sine_wave[NUM_POINTS];  // Array to hold sine wave values

int main(void) {
    I2C0_Init();                          // Set up I2C0 for DAC communication
    generateSineWave(2047, 2048);         // Create sine wave data (for 12-bit DAC with max 4095)

    while (1) {
        // Continuously send sine wave data points to the DAC
        int i;
        for (i = 0; i < NUM_POINTS; i++) {
            I2C0_Write(MCP4725_ADDR, sine_wave[i]); // Send sine wave value to the DAC
            delayMs(DELAY_MS);                      // Adjust delay to control output frequency
        }
    }
}

// I2C0 initialization function
void I2C0_Init(void) {
    SYSCTL_RCGCI2C_R |= 0x01;            // Enable the clock for I2C0
    SYSCTL_RCGCGPIO_R |= 0x02;           // Enable the clock for Port B

    GPIO_PORTB_AFSEL_R |= 0x0C;          // Set PB2 and PB3 for alternate functions (SCL, SDA)
    GPIO_PORTB_ODR_R |= 0x08;            // Configure PB3 (SDA) as open drain
    GPIO_PORTB_DEN_R |= 0x0C;            // Enable digital functions on PB2 and PB3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) | 0x00003300; // Configure I2C signals

    I2C0_MCR_R = 0x10;                   // Set I2C0 to master mode
    I2C0_MTPR_R = 9;                     // Configure for 100 Kbps clock speed
}

// Function to transmit data to the MCP4725 DAC using I2C
void I2C0_Write(uint8_t device_addr, uint16_t data) {
    uint8_t upper_data_byte = (data >> 4) & 0xFF;  // Extract upper 8 bits of the 12-bit data
    uint8_t lower_data_byte = (data << 4) & 0xF0;  // Extract lower 4 bits, padded with zeros

    I2C0_MSA_R = (device_addr << 1);               // Set the slave address with the write flag
    I2C0_MDR_R = 0x40;                             // DAC command to update register
    I2C0_MCS_R = I2C0_MCS_START | I2C0_MCS_RUN;    // Start condition and run the operation

    while (I2C0_MCS_R & I2C0_MCS_ERROR);           // Wait for the transmission to complete

    I2C0_MDR_R = upper_data_byte;                  // Transmit upper byte
    I2C0_MCS_R = I2C0_MCS_RUN;                     // Continue transmission

    while (I2C0_MCS_R & I2C0_MCS_ERROR);           // Wait for the transmission to complete

    I2C0_MDR_R = lower_data_byte;                  // Transmit lower byte
    I2C0_MCS_R = I2C0_MCS_STOP | I2C0_MCS_RUN;     // Send stop condition and run

    while (I2C0_MCS_R & I2C0_MCS_ERROR);           // Wait for the transmission to complete
}

// Function to generate sine wave values and store them in an array
void generateSineWave(int amplitude, int offset) {
    int i;
    for (i = 0; i < NUM_POINTS; i++) {
        // Compute sine value and scale it to fit within 0 to 4095 for a 12-bit DAC
        sine_wave[i] = (uint16_t)(amplitude * sin(2 * PI * i / NUM_POINTS) + offset);
    }
}

// Function to implement a simple delay
void delayMs(int n) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < 3180; j++) {
            // Wait loop to generate a delay
        }
    }
}
