
//! imports
#include <stdlib.h>
#include <math.h>
#include "spin1_api.h"
#include "common-typedefs.h"
#include <data_specification.h>
#include <recording.h>
#include <simulation.h>
#include <debug.h>
#include <circular_buffer.h>

#define X_MASK(x) x&0x1FF
#define Y_MASK(x) (x>>9)&0xFF
#define REMOVE_FLAG(x) x&0xFFEFFFFF

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;
static uint32_t update_count = 0;
static uint32_t received_count = 0;


//! timer period
uint32_t timer_period;
uint32_t max_counter;
uint32_t events_processed = 0;

//! parameters for this c code
static uint32_t row_number;
static uint32_t number_of_cols;
static unsigned char *LUT;

//! transmission key
static uint32_t i_has_key;
static uint32_t base_key;
static uint32_t my_tdma_id;

//! recpetion key params
static uint32_t retina_base_key;
static uint32_t aggregation_base_key;

//! The recording flags
static uint32_t recording_flags = 0;

//! key bases

//! human readable definitions of each region in SDRAM
typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA_REGION,
    RECEPTION_BASE_KEYS_REGION,
    CONFIG_REGION
} regions_e;

//! values for the priority for each callback
typedef enum callback_priorities {
    MC_PACKET = -1, MCPL_PACKET = 0, SDP_DMA = 1, USER = 4, TIMER = 3
} callback_priorities;

//! human readable definitions of each element in the transmission region
typedef enum transmission_region_elements {
    HAS_KEY = 0, MY_KEY = 1, TDMA_ID = 2
} transmission_region_elements;

//! human readable definitions of each element in the reception region
typedef enum reception_region_elements {
    RETINA_BASE_KEY = 0, AGGREGATION_BASE_KEY = 1
} reception_region_elements;

//! human readable definitions of each element in the config region
typedef enum config_region_elements {
    X_COORD = 0, Y_COORD = 1, RADIUS = 2, PACKET_THRESHOLD = 3
} config_region_elements;

//! \brief callback for when packet has payload (agg)
//! \param[in] key: the key received
//! \param[in] payload: the payload received
void receive_data_payload(uint key, uint payload)
{
    //here we receive the region of interest and need to set the values

    //our LUT is all 0s if the row doesn't fall in the ROI
    int y = Y_MASK(key);
    int r = payload;
    int x = X_MASK(key);
    if(row_number < y + r && row_number > y - r) {
        int x_start = std::max(x - r, 0);
        int y_start = std::min(x + r, number_of_cols-1);
        int i = 0;
        while(i < x_start)
            LUT[i++] = 0;
        while(i < x_end)
            LUT[i++] = 1;
        while(i < number_of_cols-1)
            LUT[i++] = 0;
    }


}

//! \brief callback when packet with no payload is received (retina)
//! \param[in] key: the key received
//! \param[in] payload: unused. is set to 0
void receive_data_no_payload(uint key, uint payload) {

    use(payload);
    received_count++;
    //here we need to filter the events based on the LUT
    if(!LUT[X_MASK(key)])
        return;

    events_processed++;
    //send on the data (masking out the flag bit
    while (!spin1_send_mc_packet(REMOVE_FLAG(x), 0, NO_PAYLOAD)) {
            spin1_delay_us(1);
    }

}

//! \brief callback for when resuming
void resume_callback() {
    time = UINT32_MAX;
}

//! \brief timer tick callback
//! \param[in] ticks the number of tiemr tick callbacks (not accurate)
//! \param[in] b: unknown
void update(uint ticks, uint b) {
    use(b);
    use(ticks);

    time++;

    log_debug("on tick %d of %d", time, simulation_ticks);

    // check that the run time hasn't already elapsed and thus needs to be
    // killed
    if ((infinite_run != TRUE) && (time >= simulation_ticks)) {
        log_info("Simulation complete.\n");

        if (recording_flags > 0) {
            log_info("updating recording regions");
        }

        // falls into the pause resume mode of operating
        simulation_handle_pause_resume(resume_callback);

        return;

    }

    if(time % 1000 == 0) {
        log_info("Received = %d | Processed = %d | Period 1s",
            received_count, events_processed);
        events_processed = 0;
        received_count = 0;
    }

    if(time == 0) {
        log_info("my key = %d : my aggregator key = %d",
                 base_key, aggregation_base_key);
    }


}

//! \brief reads the config data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_config(address_t address){
    x = address[X_COORD];
    y = address[Y_COORD];
    r = address[RADIUS];
    n = address[PACKET_THRESHOLD];
    return true;
}

//! \brief reads the transmission keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_transmission_keys(address_t address){
    i_has_key = address[HAS_KEY];
    base_key = address[MY_KEY];
    my_tdma_id = address[TDMA_ID];
    return true;
}


//! \brief reads the reception keys keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_reception_keys(address_t address){
    retina_base_key = address[RETINA_BASE_KEY];
    aggregation_base_key = address[AGGREGATION_BASE_KEY];
    return true;
}


//! \brief main initisation method
//! \param[in] timer_period. the time set for the timer
//! \return bool true if successful, false otherwise
static bool initialize(uint32_t *timer_period) {
    log_info("Initialise: started\n");

    // Get the address this core's DTCM data starts at from SRAM
    address_t address = data_specification_get_data_address();

    // Read the header
    if (!data_specification_read_header(address)) {
        log_error("failed to read the data spec header");
        return false;
    }

    // Get the timing details and set up the simulation interface
    if (!simulation_initialise(
            data_specification_get_region(SYSTEM_REGION, address),
            APPLICATION_NAME_HASH, timer_period, &simulation_ticks,
            &infinite_run, SDP_DMA, SDP_DMA)) {
        return false;
    }

    // get config data
    if (!read_config(data_specification_get_region(CONFIG_REGION, address))){
        return false;
    }

    // get config data
    if (!read_reception_keys(data_specification_get_region(
            RECEPTION_BASE_KEYS_REGION, address))){
        return false;
    }

    // get config data
    if (!read_transmission_keys(data_specification_get_region(
            TRANSMISSION_DATA_REGION, address))){
        return false;
    }

    // initialise my input_buffer for receiving packets
    log_info("build buffer");
    LUT = spin1_malloc(number_of_cols * sizeof(unsigned char));
    if (LUT == 0){
        log_info("Could not allocate LUT");
        return false;
    }

    return true;
}


//! \brief main entrance method
void c_main() {
    log_info("starting roi filter\n");

    // initialise the model
    if (!initialize(&timer_period)) {
        log_error("failed to init");
        rt_error(RTE_SWERR);
    }

    // set timer tick value to configured value
    log_info("setting timer to execute every %d microseconds", timer_period);
    spin1_set_timer_tick(timer_period);

    // register callbacks
    spin1_callback_on(MCPL_PACKET_RECEIVED, receive_data_payload, MCPL_PACKET);
    spin1_callback_on(MC_PACKET_RECEIVED, receive_data_no_payload, MC_PACKET);
    spin1_callback_on(TIMER_TICK, update, TIMER);
    //spin1_callback_on(USER_EVENT, user_callback, USER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
