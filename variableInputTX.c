/*! ---------------------------------------------------------------------------- 
 *  @file    simple_tx.c
 *  @brief   Simple TX example code with J-Link RTT input
 *
 * @attention
 *
 * Copyright 2015 - 2020 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2021 (c) Callender-Consulting, LLC  (port to Zephyr)
 *
 * All rights reserved.
 *
 * @author Decawave
 */  

 #include <deca_device_api.h>
 #include <deca_regs.h>
 #include <deca_spi.h>
 #include <port.h>
 #include <shared_defines.h>
 
 // Zephyr includes
 #include <zephyr/kernel.h>
 #include <zephyr/sys/printk.h>
 #include <SEGGER_RTT.h>
 
 #define LOG_LEVEL 3
 #include <zephyr/logging/log.h>
 LOG_MODULE_REGISTER(simple_tx);
 
 /* Example application name */
 #define APP_NAME "SIMPLE TX v1.0"
 
 /* Default communication configuration. We use default non-STS DW mode. */
 static dwt_config_t config = {
     .chan            = 5,               /* Channel number. */
     .txPreambLength  = DWT_PLEN_128,    /* Preamble length. Used in TX only. */
     .rxPAC           = DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
     .txCode          = 9,               /* TX preamble code. Used in TX only. */
     .rxCode          = 9,               /* RX preamble code. Used in RX only. */
     .sfdType         = DWT_SFD_DW_8,    /* 0 to use standard 8 symbol SFD */
     .dataRate        = DWT_BR_6M8,      /* Data rate. */
     .phrMode         = DWT_PHRMODE_STD, /* PHY header mode. */
     .phrRate         = DWT_PHRRATE_STD, /* PHY header rate. */
     .sfdTO           = (129 + 8 - 8),   /* SFD timeout */
     .stsMode         = DWT_STS_MODE_OFF,
     .stsLength       = DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
     .pdoaMode        = DWT_PDOA_M0      /* PDOA mode off */
 };
 
 /* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
  *     - byte 0: frame type (0xC5 for a blink).
  *     - byte 1: sequence number, incremented for each new frame.
  *     - byte 2 -> 9: device ID, see NOTE 1 below.
  */
static uint8_t tx_msg[12];

 /* Index to access to sequence number of the blink frame in the tx_msg array. */
 #define BLINK_FRAME_SN_IDX 1
 
 #define FRAME_LENGTH    (sizeof(tx_msg) + FCS_LEN) //The real length that is going to be transmitted
 
 /* Inter-frame delay period, in milliseconds. */
 #define TX_DELAY_MS 10
 
 /* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
  * power of the spectrum at the current temperature. These values can be
  * calibrated prior to taking reference measurements.
  * See NOTE 2 below. */
 extern dwt_txconfig_t txconfig_options;
 
 /**
  * Function to read user input via RTT and update the tx_msg.
  */
 void update_tx_msg(void) {
    
     // Define a buffer for input
     char input_buffer[sizeof(tx_msg) - 1];
     
     // Attempt to read data from RTT
     int bytes_read = SEGGER_RTT_Read(0, input_buffer, sizeof(tx_msg) - 2);
     
     if (bytes_read > 0) {
         // Update the tx_msg with the new input (ensure proper bounds checking)
         for (int i = 2; i < sizeof(tx_msg); i++) {
            if(i <= bytes_read + 1)
            {
                tx_msg[i] = input_buffer[i-2];
            } 
            else
            {
                tx_msg[i] = 0;
            }
         }
         tx_msg[0] = 0xc5;
     }
     
 }
 
 /**
  * Application entry point.
  */
 int app_main(void)
 {
     tx_msg[1] = 0;
     /* Display application name. */
     LOG_INF(APP_NAME);
 
     /* Configure SPI rate, DW3000 supports up to 38 MHz */
     port_set_dw_ic_spi_fastrate();
 
     /* Reset DW IC */
     reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */
 
     /* Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC,
      *( or could wait for SPIRDY event) */
     Sleep(2);
 
     /* Need to make sure DW IC is in IDLE_RC before proceeding */
     while (!dwt_checkidlerc()) { /* spin */ };
 
     if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
         LOG_ERR("INIT FAILED");
         while (1) { /* spin */ };
     }
 
     /* Enabling LEDs here for debug so that for each TX the D1 LED will flash
      * on DW3000 red eval-shield boards. */
     dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
 
     /* Configure DW IC. See NOTE 5 below. */
     if (dwt_configure(&config))  {
         LOG_ERR("CONFIG FAILED");
         while (1) { /* spin */ };
     }
 
     /* Configure the TX spectrum parameters (power PG delay and PG Count) */
     dwt_configuretxrf(&txconfig_options);
 
     LOG_INF("Sending started");
 
     /* Loop forever sending frames periodically. */
     while (1) {
         // Update tx_msg with RTT input
         update_tx_msg();
         
         if (tx_msg[0] != 0) {
            // Print updated message length to log (optional)
            char len[sizeof(tx_msg)];
            sprintf(len, "len %d", FRAME_LENGTH - FCS_LEN);
            LOG_HEXDUMP_INF((char*)&tx_msg, sizeof(tx_msg), (char*)&len);
    
            /* Write frame data to DW IC and prepare transmission. */
            dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */
    
            /* Transmit the frame with no ranging. */
            dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */
    
            /* Start transmission. */
            dwt_starttx(DWT_START_TX_IMMEDIATE);
    
            /* Poll DW IC until TX frame sent event set. */
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { /* spin */ };
    
            /* Clear TX frame sent event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    
            /* Execute a delay between transmissions. */
            Sleep(TX_DELAY_MS);
    
            /* Increment the blink frame sequence number (modulo 256). */
            tx_msg[BLINK_FRAME_SN_IDX]++;
            tx_msg[0] = 0;
         }
     }
 }
 
 