/******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Master using HAL APIs. The I2C master reads the
* data from I2C slave and displays it in the UART terminal
* everytime user presses the button.
*
* Related Document: See README.md
*
*
*******************************************************************************
* $ Copyright 2021-2023 Cypress Semiconductor $
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/***************************************
*            Constants
****************************************/

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x42)

/* I2C bus frequency */
#define I2C_FREQ                (100000)

/* I2C slave interrupt priority */
#define I2C_SLAVE_IRQ_PRIORITY  (7u)

/* TEMPERATURE REGISTER OFFSET */
#define TEMPERATURE_REG    0X07

/* GPIO interrupt priority */
#define GPIO_INTERRUPT_PRIORITY (3u)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/*******************************************************************************
* Global Variables
********************************************************************************/
uint8_t tx_buffer={TEMPERATURE_REG};

volatile bool button_pressed = false;

struct {
          float temp;
          float humidity;
          float light;
          float pot;
} rx_buffer;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CYW943907 MCU.
*   1. Initialises the user button and enables Interrupt for the button press.
*   2. Initialises the I2C Master.
*   3. I2C Master reads the data from slave everytime user presses the button.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    cyhal_i2c_t mI2C;
    cyhal_i2c_cfg_t mI2C_cfg;

    cyhal_gpio_callback_data_t cb_data =
    {
        .callback     = gpio_interrupt_handler,
        .callback_arg = (void*)NULL
    };

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                                  CY_RETARGET_IO_BAUDRATE);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_SW1, CYHAL_GPIO_DIR_INPUT,
                       CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

    /* gpio init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_SW1,&cb_data);
    cyhal_gpio_enable_event(CYBSP_SW1,CYHAL_GPIO_IRQ_FALL,3,true);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("**************************\r\n");
    printf("CYW43907 MCU I2C Master\r\n");
    printf("**************************\r\n\n");

    /* Configure I2C Master */
    printf(">> Configuring I2C Master..... ");

    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;

    /*I2C init*/
    result = cyhal_i2c_init( &mI2C, PIN_I2C1_SDATA, PIN_I2C1_CLK, NULL);

    /* I2C init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*I2C Config*/
    result = cyhal_i2c_configure( &mI2C, &mI2C_cfg);

    /* I2C config failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
       CY_ASSERT(0);
    }

    printf("Done\r\n\n");

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        if(button_pressed)
        {
            /*I2C master write*/
            result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR,&tx_buffer,
                                                   sizeof(tx_buffer), 0, true);

            /*I2C write failed . Stop program execution */
            if (result != CY_RSLT_SUCCESS)
            {
               CY_ASSERT(0);
            }

            /*I2C master read*/
            result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR, (uint8_t*)&rx_buffer,
                                                sizeof(rx_buffer) , 0, true);

            /*I2C read failed . Stop program execution */
            if (result != CY_RSLT_SUCCESS)
            {
               CY_ASSERT(0);
            }

            printf("Temperature:%.1f Humidity:%.1f Light:%.1f Potentiometer:%.1f\r\n",rx_buffer.temp,
                                   rx_buffer.humidity ,rx_buffer.light,rx_buffer.pot);

            button_pressed = false;

        }
    }
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler updates the button_pressed.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t event (unused)
*
* Return:
*  void
*
*******************************************************************************/

static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);
    button_pressed = true;
}

/* [] END OF FILE */
