
//! imports
#include "spin1_api.h"
#include "common-typedefs.h"
#include <data_specification.h>
#include <recording.h>
#include <simulation.h>
#include <debug.h>
#include <circular_buffer.h>

//! data format
typedef struct data_items_t{
    uint32_t x;
    uint32_t y;
    uint32_t r;
    uint32_t l;
    uint32_t w;
    uint32_t n;
}data_items_t;

static circular_buffer particle_buffer;
static data_items_t *particle_data;

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;

//! The recording flags
static uint32_t recording_flags = 0;

//! base keys for the transmission of data
static uint32_t has_key;
static uint32_t base_transmission_key;
static uint32_t has_record_key;
static uint32_t base_record_key;

//! flag for recording
static uint32_t do_record;

//! data items
static uint32_t *reception_base_keys = NULL;
static data_items_t *stored_data = NULL;
static uint32_t n_particles = 0;

//! key bases
typedef enum packet_identifiers{
    COORDS = 0, RADIUS = 1, L = 2, W = 3, N = 4, N_KEYS_RECEIVED = 5
}packet_identifiers;

//! human readable definitions of each region in SDRAM
typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA,
    RECEPTION_BASE_KEYS,
    RECORDED_DATA,
    CONFIG
} regions_e;

//! values for the priority for each callback
typedef enum callback_priorities{
    MC_PACKET = -1, SDP_DMA = 0, USER = 3, TIMER = 2
} callback_priorities;

//! human readable definitions of each element in the transmission region
typedef enum transmission_region_elements {
    HAS_KEY, MY_KEY, RECORD_HAS_KEY, RECORD_KEY
} transmission_region_elements;

//! human readable definitions of each element in the reception base key
//! region.
typedef enum reception_base_keys_region_items{
    N_KEYS, START_OF_KEYS
}reception_base_keys_region_items;

//! human readable definitions of each element in the config region
typedef enum config_region_items {
    DO_RECORD
}config_region_items;


//! \brief callback for when packet has payload (agg)
//! \param[in] key: the key received
//! \param[in] payload: the payload received
void receive_data_payload(uint key, uint payload) {

    if (!circular_buffer_add(particle_buffer, key)) {
        log_error("Could not add particle key");
    }
    if (!circular_buffer_add(particle_buffer, payload)) {
        log_error("Could not add particle payload");
    }

    if(circular_buffer_size(particle_buffer) > 2 * 5 * n_particles) {
        spin1_trigger_user_event(0, 0);
    }

}

//! \brief callback when packet with no payload is received (retina)
//! \param[in] key: the key received
//! \param[in] payload: unused. is set to 0
void receive_data_no_payload(uint key, uint payload) {
    use(payload);
    use(key);
    log_error("Should never have received this packet. %d"
              "Aggr's only receive packets with payloads.", key);
}

//! \brief records data via the record interface
void record_data() {

}


//! \brief Initialises the recording parts of the model
//! \return True if recording initialisation is successful, false otherwise
static bool initialise_recording(){
    address_t address = data_specification_get_data_address();
    address_t recording_region = data_specification_get_region(
        RECORDED_DATA, address);

    bool success = recording_initialize(recording_region, &recording_flags);
    log_info("Recording flags = 0x%08x", recording_flags);
    return success;
}

//! \brief callback for when resuming
void resume_callback() {
    time = UINT32_MAX;
}

//! \brief converts a int to a float via bit wise conversion
//! \param[in] y: the int to convert
//! \param[out] the converted float
static inline float int_to_float( int data){
    union { float x; int y; } cast_union;
    cast_union.y = data;
    return cast_union.x;
}
static inline int float_to_int( float data){
    union { float x; int y; } cast_union;
    cast_union.x = data;
    return cast_union.y;
}

uint32_t codexy(float x, float y)
{
    return ((int)x & 0x1FF) + (((int)y & 0xFF) << 9);
}

void decodexy(uint32_t coded, float *x, float *y) {

    *x = coded & 0x1FF;
    *y = (coded >> 9) & 0xFF;

}

//! \brief callback for user
//! \param[in] random param1
//! \param[in] random param2
void user_callback(uint user0, uint user1) {

    log_info("Aggregator user callback");

    use(user0);
    use(user1);

//    for(uint32_t i = 0; i < /*nparticles*/; i++) {
//        uint32_t key, payload;
//        if(!circular_buffer_get_next(particle_buffer, &key))
//            log_error("Could not get key from buffer");
//        if(!circular_buffer_get_next(particle_buffer, &payload))
//            log_error("Could not get payload from buffer");

//        START_OF_KEYS

//        switch(key) {
//        case(AGGREGATION_BASE_KEY + COORDS_KEY_OFFSET):
//            decodexy(payload, &nx, &ny);
//            break;
//        case(AGGREGATION_BASE_KEY + RADIUS_KEY_OFFSET):
//            nr = payload;
//            break;
//        case(AGGREGATION_BASE_KEY + L_KEY_OFFSET):
//            nl = payload;
//            break;
//        case(AGGREGATION_BASE_KEY + W_KEY_OFFSET):
//            nw = payload;
//            break;
//        case(AGGREGATION_BASE_KEY + N_KEY_OFFSET):
//            nn = payload;
//            break;
//        default:
//            log_error("Switch on Key (%d) not working", key);
//        }

//    }


    if(has_key) {

        //send a message out
        while (!spin1_send_mc_packet(base_transmission_key + COORDS, codexy(64, 64), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(base_transmission_key + RADIUS, float_to_int(30.0), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(base_transmission_key + L, float_to_int(10.0), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(base_transmission_key + W, float_to_int(1.0), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(base_transmission_key + N, 10, WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
    }

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
            recording_finalise();
        }

        // falls into the pause resume mode of operating
        simulation_handle_pause_resume(resume_callback);
        return;
    }

    // trigger buffering_out_mechanism
    if (recording_flags > 0) {
        recording_do_timestep_update(time);
    }

    if(time % 1000 == 0) {
        log_info("Data recieved: %d", circular_buffer_size(particle_buffer));
    }

}

//! \brief reads the transmission data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
static bool read_transmission_region(address_t address){
    has_key = address[HAS_KEY];
    base_transmission_key = address[MY_KEY];
    has_record_key = address[RECORD_HAS_KEY];
    base_record_key = address[RECORD_KEY];
    return true;
}

//! \brief reads the reception data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
static bool read_reception_region(address_t address){
    uint32_t n_keys = address[N_KEYS];
    reception_base_keys = (uint32_t*) spin1_malloc(n_keys * sizeof(uint32_t));
    stored_data = (data_items_t*) spin1_malloc(n_keys * sizeof(data_items_t));
    n_particles = n_keys;
    return true;
}

//! \brief reads the config data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
static bool read_config_region(address_t address){
    do_record = address[DO_RECORD];
    return true;
}



//! \brief main init function
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

    // get transmission data
    if (!read_transmission_region(
            data_specification_get_region(TRANSMISSION_DATA, address))){
        return false;
    }

    // get reception data
    if (!read_reception_region(
            data_specification_get_region(RECEPTION_BASE_KEYS, address))){
        return false;
    }

    // get reception data
    if (!read_config_region(data_specification_get_region(CONFIG, address))){
        return false;
    }

    // Get the timing details and set up the simulation interface
    if (!simulation_initialise(
            data_specification_get_region(SYSTEM_REGION, address),
            APPLICATION_NAME_HASH, timer_period, &simulation_ticks,
            &infinite_run, SDP_DMA, SDP_DMA)) {
        return false;
    }

    // initialise my input_buffer for receiving packets
    log_info("build buffer");
    particle_buffer = circular_buffer_initialize(2 * 6 * n_particles);
    if (particle_buffer == 0){
        return false;
    }
    log_info("particle_buffer initialised");

    particle_data = spin1_malloc(n_particles * sizeof(data_items_t));
    if(!particle_data)
        return false;
    log_info("particle data array initialised");


    return true;
}

//! \brief main entrance method
void c_main() {
    log_info("starting heat_demo\n");

    // Load DTCM data
    uint32_t timer_period;

    // initialise the model
    if (!initialize(&timer_period)) {
        log_error("failed to init");
        rt_error(RTE_SWERR);
    }

    // initialise the recording section
    // set up recording data structures
    if(!initialise_recording()){
        log_error("faield to init recording");
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
