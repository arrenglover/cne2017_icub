
//! imports
#include "spin1_api.h"
#include "common-typedefs.h"
#include <data_specification.h>
#include <recording.h>
#include <simulation.h>
#include <debug.h>

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;

//! parameters for this c code
static uint32_t x_coord;
static uint32_t y_coord;
static uint32_t radius;
static uint32_t packet_threshold;

//! transmission key
static uint32_t has_key;
static uint32_t base_key;

//! recpetion key params
static uint32_t retina_base_key;


//! The recording flags
static uint32_t recording_flags = 0;

//! human readable definitions of each region in SDRAM
typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA_REGION,
    RECEPTION_BASE_KEYS_REGION,
    CONFIG_REGION
} regions_e;

//! values for the priority for each callback
typedef enum callback_priorities{
    MC_PACKET = -1, SDP_DMA = 0, USER = 3, TIMER = 2
} callback_priorities;

//! human readable definitions of each element in the transmission region
typedef enum transmission_region_elements {
    HAS_KEY = 0, MY_KEY = 1
} transmission_region_elements;

//! human readable definitions of each element in the reception region
typedef enum reception_region_elements {
    RETINA_BASE_KEY = 0
} reception_region_elements;

//! human readable definitions of each element in the config region
typedef enum config_region_elements {
    X_COORD = 0, Y_COORD = 1, RADIUS = 2, PACKET_THRESHOLD = 3
} config_region_elements;

void receive_data_payload(uint key, uint payload) {
    use(key);
    use(payload);
}

void receive_data_no_payload(uint key, uint payload) {
    use(payload);
    use(key);
}


void resume_callback() {
    time = UINT32_MAX;
}

void user_callback(uint user0, uint user1){
    use(user0);
    use(user1);
}

/****f*
 *
 * SUMMARY
 *
 * SYNOPSIS
 *  void update (uint ticks, uint b)
 *
 * SOURCE
 */
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
}

//! \brief reads the config data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_config(address_t address){
    x_coord = address[X_COORD];
    y_coord = address[Y_COORD];
    radius = address[RADIUS];
    packet_threshold = address[PACKET_THRESHOLD];
    return true;
}

//! \brief reads the transmission keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_transmission_keys(address_t address){
    has_key = address[HAS_KEY];
    base_key = address[MY_KEY];
    return true;
}


//! \brief reads the reception keys keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_reception_keys(address_t address){
    retina_base_key = address[RETINA_BASE_KEY];
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

    return true;
}

/****f*
 *
 * SUMMARY
 *  This function is called at application start-up.
 *  It is used to register event callbacks and begin the simulation.
 *
 * SYNOPSIS
 *  int c_main()
 *
 * SOURCE
 */
void c_main() {
    log_info("starting heat_demo\n");

    // Load DTCM data
    uint32_t timer_period;

    // initialise the model
    if (!initialize(&timer_period)) {
        log_error("failed to init");
        rt_error(RTE_SWERR);
    }

    // set timer tick value to configured value
    log_info("setting timer to execute every %d microseconds", timer_period);
    spin1_set_timer_tick(timer_period);

    // register callbacks
    spin1_callback_on(MCPL_PACKET_RECEIVED, receive_data_payload, MC_PACKET);
    spin1_callback_on(MC_PACKET_RECEIVED, receive_data_no_payload, MC_PACKET);
    spin1_callback_on(TIMER_TICK, update, TIMER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
