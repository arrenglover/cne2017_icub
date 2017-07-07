
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

#define ANG_BUCKETS 64
#define INLIER_PAR 2
#define MIN_LIKE 10
#define SIGMA_SCALER 4.0f
#define TDMA_WAIT_PERIOD = 300

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;
static uint32_t update_count = 0;

//! timer period
uint32_t timer_period;
uint32_t max_counter;

//! parameters for this c code
static float x = 64.0f, nx = -1.0f;
static float y = 64.0f, ny = -1.0f;
static float r = 30.0f, nr = -1.0f;
static float l = 0.0f, nl = -1.0f;
static float w = 1.0f, nw = -1.0f;
static uint32_t n = 0, nn = 0;

static float L[ANG_BUCKETS];
static circular_buffer retina_buffer, agg_buffer;

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
typedef enum packet_identifiers {
    COORDS_X_KEY_OFFSET = 0, COORDS_Y_KEY_OFFSET = 1, RADIUS_KEY_OFFSET = 2,
    L_KEY_OFFSET = 3, W_KEY_OFFSET = 4, N_KEY_OFFSET = 5
}packet_identifiers;

//! human readable definitions of each region in SDRAM
typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA_REGION,
    RECEPTION_BASE_KEYS_REGION,
    CONFIG_REGION
} regions_e;

//! values for the priority for each callback
typedef enum callback_priorities {
    MC_PACKET = -1, SDP_DMA = 0, USER = 3, TIMER = 2
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
void receive_data_payload(uint key, uint payload) {

    if (!circular_buffer_add(agg_buffer, key)) {
        log_error("Could not add agg data");
    }
    if (!circular_buffer_add(agg_buffer, payload)) {
        log_error("Could not add agg data");
    }

    if(circular_buffer_size(agg_buffer) >= 10) {//or 40?
        spin1_trigger_user_event(0, 0);
    }

}

//! \brief callback when packet with no payload is received (retina)
//! \param[in] key: the key received
//! \param[in] payload: unused. is set to 0
void receive_data_no_payload(uint key, uint payload) {
    use(payload);

    if (!circular_buffer_add(retina_buffer, key)) {
        log_error("Could not add event");
    }

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

void concludeLikelihood() {

    l = 0;
    for(int i = 0; i < ANG_BUCKETS; i++) {
        l += L[i];
    }

    if(l < MIN_LIKE) {
        w = MIN_LIKE * w;
    } else {
        w = l * w;
    }

}

uint32_t codexy(float x, float y)
{
    return ((int)x & 0x1FF) + (((int)y & 0xFF) << 9);
}

void decodexy(uint32_t coded, float *x, float *y) {

    *x = coded & 0x1FF;
    *y = (coded >> 9) & 0xFF;

}

void predict(float sigma) {

    //use a flat distribution?
    x += 2.0 * sigma * rand() / RAND_MAX - sigma;
    y += 2.0 * sigma * rand() / RAND_MAX - sigma;
    r += 2.0 * sigma * rand() / RAND_MAX - sigma;

}

void incLikelihood(float vx, float vy) {

    //get the next input packet in the queue


    //update the L vector based on the event
    float dx = vx - x;
    float dy = vy - y;

    float d = sqrt(dx * dx + dy * dy) - r;
    if(d < INLIER_PAR) {

        int a = 0.5 + (ANG_BUCKETS-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);

        if(d > -INLIER_PAR) {
            //inlier event
            if(L[a] < 1.0) {
                L[a] = 1.0;
                n++;
            }
        } else {
            //outlier event
            if(L[a] > 0.0) {
                L[a] = 0;
                n++;
            }
        }
    }
}

void processInput() {

    uint32_t * qcopy;
    uint32_t current_key, buffersize;
    float vx, vy;

    uint cpsr = spin1_int_disable();

    buffersize = circular_buffer_size(retina_buffer);
    //copy data
    qcopy = spin1_malloc(buffersize * sizeof(uint32_t));

    for(uint32_t i = 0; i < buffersize; i++) {
        if(!circular_buffer_get_next(retina_buffer, &current_key))
            log_error("Could not get key from buffer");
        qcopy[i] = current_key;
    }

    spin1_mode_restore(cpsr);


    for(uint32_t i = 0; i < buffersize; i++) {
        decodexy(qcopy[i], &vx, &vy);
        incLikelihood(vx, vy);
    }

}

void particleResample() {

    w = nw;

    if(nl > 0) {
        x = nx;
        y = ny;
        r = nr;
        l = nl;

        for(int i = 0; i < ANG_BUCKETS; i++) {
            L[i] = l / ANG_BUCKETS;
        }

    }

}

void sendstate() {

    //log_info("sending state, %d", sv->cpu_clk);
    if(i_has_key == 1) {
       uint32_t current_time = tc[T1_COUNT];

       //calculate max_counter;
       max_counter = timer_period * sv->cpu_clk;


       int dt = current_time - tc[T1_COUNT];
       if(dt < 0) dt += max_counter;
       while(dt < my_tdma_id * TDMA_WAIT_PERIOD) {
            dt = current_time - tc[T1_COUNT];
            if(dt < 0) dt += max_counter;
       }

        //send a message out
        //log_info("sending packets");
        while (!spin1_send_mc_packet(
                base_key + COORDS_X_KEY_OFFSET, float_to_int(x),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + COORDS_Y_KEY_OFFSET, float_to_int(y),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + RADIUS_KEY_OFFSET, float_to_int(r),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + L_KEY_OFFSET, float_to_int(l), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + W_KEY_OFFSET, float_to_int(w), WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + N_KEY_OFFSET, n, WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
    }
}

//! \brief callback for user
//! \param[in] random param1
//! \param[in] random param2
void user_callback(uint user0, uint user1) {

    //log_info("User Callback (particle processing) run");
    use(user0);
    use(user1);

    for(uint32_t i = 0; i < 5; i++) {
        uint32_t key, payload;
        if(!circular_buffer_get_next(agg_buffer, &key))
            log_error("Could not get key from buffer");
        if(!circular_buffer_get_next(agg_buffer, &payload))
            log_error("Could not get payload from buffer");

        if(key == aggregation_base_key + COORDS_X_KEY_OFFSET)
            nx = int_to_float(payload);
        if(key == aggregation_base_key + COORDS_Y_KEY_OFFSET)
            ny = int_to_float(payload);
        else if(key == aggregation_base_key + RADIUS_KEY_OFFSET)
            nr = int_to_float(payload);
        else if(key == aggregation_base_key + L_KEY_OFFSET)
            nl = int_to_float(payload);
        else if(key == aggregation_base_key + W_KEY_OFFSET)
            nw = int_to_float(payload);
        else if(key == aggregation_base_key + N_KEY_OFFSET)
            nn = payload;
        else
            log_error("Switch on Key (%d) not working", key);
    }

    if(nx < 0 || ny < 0 || nr < 0 || nw < 0 || nl < 0) {
        log_error("Packet Loss from Aggregator");
        return;
    }

    particleResample();
    nx = ny = nr = nl = nw = -1;
    nn = 0;

    predict(SIGMA_SCALER * n / ANG_BUCKETS);

    processInput();
    concludeLikelihood();


    sendstate();

    update_count++;


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
        log_info("Update Rate = %d / second", update_count * timer_period);
        update_count = 0;
    }

    if(time == 0) {
        log_info("my key = %d : my aggregator key = %d",
                 base_key, aggregation_base_key);
        sendstate();
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
    retina_buffer = circular_buffer_initialize(512);
    if (retina_buffer == 0){
        return false;
    }
    log_info("retina_buffer initialised");

    // initialise my input_buffer for receiving packets
    log_info("build buffer");
    agg_buffer = circular_buffer_initialize(32);
    if (agg_buffer == 0){
        return false;
    }
    log_info("agg_buffer initialised");

    return true;
}


//! \brief main entrance method
void c_main() {
    log_info("starting particle filter\n");

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
    spin1_callback_on(USER_EVENT, user_callback, USER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
