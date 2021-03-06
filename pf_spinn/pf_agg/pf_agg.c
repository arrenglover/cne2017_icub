
//! imports
#include <stdlib.h>
#include "spin1_api.h"
#include "common-typedefs.h"
#include <data_specification.h>
#include <recording.h>
#include <simulation.h>
#include <debug.h>
#include <circular_buffer.h>

#define PACKETS_PER_PARTICLE 6
#define RECORDING_DATA_REGION_ID 0

//! data format
typedef struct data_items_t {
    float x;
    float y;
    float r;
    float l;
    float w;
    uint32_t n;
} data_items_t;

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
static uint32_t partner_base_key;
static uint32_t partner_i;

//! flag for recording
static uint32_t do_record;

//! data items
static float sumsqr = 0;
static uint32_t *reception_base_keys = NULL;
static uint32_t n_particles = 0;
static data_items_t resampled_data;
static data_items_t average_data;
static circular_buffer particle_buffer;
static data_items_t *particle_data;
static uint32_t maximum_n = 0;

//! key bases offsets
typedef enum packet_identifiers{
    COORDS_X = 0, COORDS_Y = 1, RADIUS = 2, L = 3, W = 4, N = 5,
    N_KEYS_RECEIVED = 6
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
    DO_RECORD, PARTNER_KEY
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

    if(circular_buffer_size(particle_buffer) >=
            2 * PACKETS_PER_PARTICLE * n_particles) {
        //log_info("set off user event");
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
   if (do_record){

       average_data.x = 0;
       average_data.y = 0;
       average_data.r = 0;
       average_data.l = 0;
       average_data.w = 1.0;
       average_data.n = 0;

       for(uint32_t i = 0; i < n_particles; i++) {
            float w = particle_data[i].w;
            average_data.x += particle_data[i].x * w;
            average_data.y += particle_data[i].y * w;
            average_data.r += particle_data[i].r * w;
            average_data.l += particle_data[i].l * w;
            average_data.n += particle_data[i].n * w;
       }


       recording_record(
           RECORDING_DATA_REGION_ID,
           &average_data,
           sizeof(data_items_t));
   }
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

void normalise() {

    sumsqr = 0;
    maximum_n = 0;

    float total = 0;
    for(uint32_t i = 0; i < n_particles; i++) {
        total += particle_data[i].w;
        if(particle_data[i].n > maximum_n)
            maximum_n = particle_data[i].n;
    }
    total = 1.0 / total;

    for(uint32_t i = 0; i < n_particles; i++) {
        particle_data[i].w *= total;
        sumsqr += particle_data[i].w * particle_data[i].w;
    }


}

void send_resample_message()
{
    if(has_key) {

        //log_info("sending packet %d", time);

        //send a message out
        while (!spin1_send_mc_packet(
            base_transmission_key + COORDS_X, float_to_int(resampled_data.x), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
            base_transmission_key + COORDS_Y, float_to_int(resampled_data.y), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
            base_transmission_key + RADIUS, float_to_int(resampled_data.r), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
            base_transmission_key + L, float_to_int(resampled_data.l), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
            base_transmission_key + W, float_to_int(resampled_data.w), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
            base_transmission_key + N, resampled_data.n, WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
    }


}

void send_position_out()
{
    float average_x = 0, average_y = 0;
    if(has_record_key) {

        for(uint32_t i = 0; i < n_particles; i++) {
            average_x += particle_data[i].x * particle_data[i].w;
            average_y += particle_data[i].y * particle_data[i].w;
        }

        //send a message out
        while (!spin1_send_mc_packet(
            base_record_key + (codexy(particle_data[partner_i].x, particle_data[partner_i].y) << 1), 0, NO_PAYLOAD)) {
            spin1_delay_us(1);
        }
        static int dropper = 0;
        if(dropper % 10000 == 0)
            log_info("Sending output: %f %f", particle_data[partner_i].x, particle_data[partner_i].y);

    }

}

void resample() {

    if(sumsqr * n_particles > 2.0f && maximum_n > 4) {

        float rn = 1.0 * (double)rand() / RAND_MAX;
        if(rn > 1.0) {

            //set resampled data to random values
            resampled_data.x = 10 + rand() % 284;
            resampled_data.y = 10 + rand() % 220;
            resampled_data.r = 20.0 + rand() % 10;
            resampled_data.l = particle_data[partner_i].l;
            resampled_data.w = particle_data[partner_i].w;
            resampled_data.n = particle_data[partner_i].n;

        } else {

            //set resampled according to distribution of weights
            float accumed_sum = 0.0;
            uint32_t j = 0;
            for(j = 0; j < n_particles; j++) {
                accumed_sum += particle_data[j].w;
                if(accumed_sum > rn) break;
            }
            resampled_data = particle_data[j];

        }


    } else {

        //remained at same value (resample to partner)
        //set l to 0 so the particle doesn't do the averaging step
        resampled_data = particle_data[partner_i];
        resampled_data.l = 0.0;
    }
}

//! \brief callback for user
//! \param[in] random param1
//! \param[in] random param2
void user_callback(uint user0, uint user1) {

    //log_info("Aggregator user callback");
    //circular_buffer_clear(particle_buffer);
    use(user0);
    use(user1);

    //float x, y;
    uint32_t n_packets = circular_buffer_size(particle_buffer) / 2;
    uint32_t pi = 0, particle_key;


    for(uint32_t i = 0; i < n_packets; i++) {

        uint32_t key, payload;
        if(!circular_buffer_get_next(particle_buffer, &key))
            log_error("Could not get key from buffer");
        if(!circular_buffer_get_next(particle_buffer, &payload))
            log_error("Could not get payload from buffer");


        particle_key = key & 0xFFFFFFF8;
        if(particle_key != reception_base_keys[pi]) {
            for(pi = 0; pi < n_particles; pi++) {
                if(reception_base_keys[pi] == particle_key)
                    break;
            }
        }

        if(pi == n_particles) {
            log_error("Could not find particle index from the key");
            continue;
        }

        switch(key & 0x07) {
        case(COORDS_X):
            particle_data[pi].x = int_to_float(payload);
            break;
        case(COORDS_Y):
            particle_data[pi].y = int_to_float(payload);
            break;
        case(RADIUS):
            particle_data[pi].r = int_to_float(payload);
            break;
        case(L):
            particle_data[pi].l = int_to_float(payload);
            break;
        case(W):
            particle_data[pi].w = int_to_float(payload);
            break;
        case(N):
            particle_data[pi].n = payload;
            break;
        default:
            log_error("incorrect key value received at aggregator");

        }

    }

    normalise();
    send_position_out(); //will only work for 1 aggregator
    resample();
    //resampled_data = particle_data[partner_i];
    send_resample_message();

}

//! \brief timer tick callback
//! \param[in] ticks the number of tiemr tick callbacks (not accurate)
//! \param[in] b: unknown
void update(uint ticks, uint b) {
    use(b);
    use(ticks);

    time++;

    //log_info("on tick %d of %d", time, simulation_ticks);

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
        log_info("Current size of input buffer: %d", circular_buffer_size(particle_buffer));
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

    //! read in base keys
    for (uint32_t entry = 0; entry < n_keys; entry++) {
        reception_base_keys[entry] = address[START_OF_KEYS + entry];
    }

    //! create data holder for particles
    particle_data =
        (data_items_t*) spin1_malloc(n_keys * sizeof(data_items_t));

    //! store n particles
    n_particles = n_keys;
    return true;
}

//! \brief reads the config data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
static bool read_config_region(address_t address){
    do_record = address[DO_RECORD];
    partner_base_key = address[PARTNER_KEY];
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

    for(partner_i = 0; partner_i < n_particles; partner_i++) {
        if(reception_base_keys[partner_i] == partner_base_key)
            break;
    }

    if(partner_i == n_particles) {
        log_error("Could not find partner_base_keys in reception_base_keys");
        return false;
    } else {
        log_info("Partner_key: %d, Partner Index: %d", partner_base_key, partner_i);
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
    particle_buffer = circular_buffer_initialize(2 * PACKETS_PER_PARTICLE * n_particles);
    if (particle_buffer == 0){
        return false;
    }
    log_info("particle_buffer initialised");

    if(has_record_key)
        log_info("Output should be [%d]",
                 base_record_key + codexy(64.0f, 64.0f));

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
    spin1_callback_on(USER_EVENT, user_callback, USER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
