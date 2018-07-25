
//! imports
#include <stdlib.h>
#include <math.h>
#include "spin1_api.h"
#include "common-typedefs.h"
#include <string.h>
#include <data_specification.h>
#include <recording.h>
#include <simulation.h>
#include <debug.h>
#include <circular_buffer.h>
#include <sqrt.h>

#include <stdfix.h>

#define MY_RAND int_to_accum(spin1_rand() & 0x00007FFF)
#define NEG_BIAS_CONSTANT 40.74k //2.0 * 64 / pi r^2

#define X_MASK(x) (accum)(x&0x1FF)
#define Y_MASK(y) (accum)((y>>9)&0xFF)
#define XY_CODE(x, y) (x&0x1FF)|((y&0xFF)<<9)

#define ANG_BUCKETS 64
#define INLIER_PAR_PLUS1 2.0k
#define INV_INLIER_PAR 1.0k
#define MIN_LIKE 6.4k //64 * 0.2k
#define SIGMA 2.0k
#define DIV_VALUE 200
#define EVENT_WINDOW_SIZE 1024
#define RETINA_BUFFER_SIZE 4096
#define TARGET_ELEMENTS 3
#define SAVE_VECTOR_ELEMENTS TARGET_ELEMENTS
#define MAX_RADIUS 40.0k
#define MAX_RADIUS_PLUS2 42
#define MAX_RADIUS_PLUS2_SQRD 1764
#define K_PI 3.14159265359k
#define K_PI_4 0.78539816k	/* pi/4 */
#define LOG_COUNTER_PERIOD 1000000


//! SYSTEM VARIABLES
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;
static uint32_t timer_period;
static uint32_t log_counter = LOG_COUNTER_PERIOD;
static uint32_t recording_flags = 0;
static float save_vector[SAVE_VECTOR_ELEMENTS];

typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA_REGION,
    CONFIG_REGION,
    RECORDING
} regions_e;

typedef enum transmission_region_elements {
    HAS_KEY = 0, P2P_KEY = 1, FILTER_UPDATE_KEY = 2, OUTPUT_KEY = 3
} transmission_region_elements;

typedef enum config_region_elements {
    X_COORD = 0, Y_COORD = 1, RADIUS = 2, P2P_ID = 3, IS_MAIN = 4,
    N_PARTICLES = 5
} config_region_elements;

typedef enum callback_priorities {
    PACKET = 0, SDP_DMA = 1, SEND = 2, FILTER_UPDATE = 3, TIMER = 3,
} callback_priorities;

//! ALGORITHM VARIABLES
static uint32_t event_window[EVENT_WINDOW_SIZE];
//static accum event_window_x[EVENT_WINDOW_SIZE];
//static accum event_window_y[EVENT_WINDOW_SIZE];
static uint32_t start_window = 0;
static uint32_t size_window = 0;

static accum *LUT_SQRT;
static accum L[ANG_BUCKETS];
static accum score;
static accum negativeScaler;


static accum x = 64.0k;
static accum y = 64.0k;
static accum r = 30.0k;
static accum l = MIN_LIKE;
static accum w = 1.0f;
static accum n = 0.0f;

static accum target[TARGET_ELEMENTS];
static uint32_t random_part_i;

//! SENDING/RECEIVING VARIABLES

static bool is_main = false;
static uint32_t filter_update_key;
static uint32_t output_key;

static uint32_t i_has_key;
static uint32_t p2p_key;
static uint32_t my_p2p_id;

static uint32_t n_particles;
static uint32_t last_index;
static accum **proc_data, **work_data;
static circular_buffer retina_buffer;

static uint32_t full_buffer;
static uint32_t my_turn;

#define PACKETS_PER_PARTICLE 4
typedef enum packet_identifiers{
    X_IND = 0, Y_IND = 1, R_IND = 2, W_IND = 3
}packet_identifiers;


//! DEBUG VARIABLES
static uint32_t received_count = 0;
static uint32_t dropped_count = 0;
static uint32_t events_processed = 0;
static uint32_t events_unprocessed = 0;
static uint32_t update_count = 0;
static uint32_t packets_received = 0;
static uint32_t double_processed = 0;
static uint32_t n_neg_events = 0;

static bool packet_sending_turn = false;
static bool finished_processing = true;
static bool tried_to_call_my_turn = false;
static bool tried_to_call_proc_done = false;


//! \brief converts a int to a float via bit wise conversion
//! \param[in] y: the int to convert
//! \param[out] the converted float
static inline accum int_to_accum( int data){
    union { accum x; int y; } cast_union;
    cast_union.y = data;
    return cast_union.x;
}
static inline int accum_to_int( accum data){
    union { accum x; int y; } cast_union;
    cast_union.x = data;
    return cast_union.y;
}



//SOME FORWARD DECLARATIONS
void particle_filter_update_step(uint do_p2p, uint do_calc);
void ready_to_send(uint proc_msg, uint pack_msg);

////////////////////////////////////////////////////////////////////////////////
// SEND/RECEIVE
////////////////////////////////////////////////////////////////////////////////

//! \brief callback when packet with no payload is received (retina)
//! \param[in] key: the key received
//! \param[in] payload: unused. is set to 0
void receive_retina_event(uint key, uint payload) {

    use(payload);

    //this will be the events
    if (!circular_buffer_add(retina_buffer, key))
        dropped_count++;
    received_count++;

}

//! \brief callback for when packet has payload (agg)
//! \param[in] key: the key received
//! \param[in] payload: the payload received
void receive_particle_data_packet(uint key, uint payload) {

    //load in data
    work_data[packets_received++ / PACKETS_PER_PARTICLE][key&0x7] =
        int_to_accum(payload);

    if(packets_received == my_turn) { //it is our turn to send data
        tried_to_call_my_turn = true;
        if(!spin1_schedule_callback(ready_to_send, 0, 1, SEND))
            log_error("Couldn't make my_turn callback!");
    }

    if(packets_received == full_buffer) { //perform update

        accum **temp = work_data;
        work_data = proc_data;
        proc_data = temp;

        packets_received = 0;

        spin1_schedule_callback(particle_filter_update_step, 0, 0, FILTER_UPDATE);
    }

}

//! \brief send this particles state to the other particles
void send_p2p() {

    //make sure we actually have a key
    if(!i_has_key) {
        log_info("Particle tried to send a packet without a key");
        return;
    }

    //do the sending
    while(!spin1_send_mc_packet(p2p_key + X_IND, accum_to_int(x), WITH_PAYLOAD))
        spin1_delay_us(1);
    while(!spin1_send_mc_packet(p2p_key + Y_IND, accum_to_int(y), WITH_PAYLOAD))
        spin1_delay_us(1);
    while(!spin1_send_mc_packet(p2p_key + R_IND, accum_to_int(r), WITH_PAYLOAD))
        spin1_delay_us(1);
    while(!spin1_send_mc_packet(p2p_key + W_IND, accum_to_int(w), WITH_PAYLOAD))
        spin1_delay_us(1);

    //update the diagnostics for particle filter update rate.
    update_count++;

}

//! \brief callback to conditionally send data on to other particles
//!         requires a call from finished processing, and also from
//!         the correct particle order
void ready_to_send(uint proc_msg, uint pack_msg)
{

    if(proc_msg)
        finished_processing = true;
    if(pack_msg)
        packet_sending_turn = true;

    if(finished_processing && packet_sending_turn) {
        send_p2p();
        packet_sending_turn = false;
        finished_processing = false;
        tried_to_call_my_turn = false;
        tried_to_call_proc_done = false;
    }

}

//! \brief send the region of interest to the filter
void send_roi()
{

    if(!is_main)
        return;

    //this will send to the filters the updated ROI
    //only if the main_particle
    while (!spin1_send_mc_packet(filter_update_key + (XY_CODE((int)x, (int)y)),
                (int)(r+15.0k), WITH_PAYLOAD)) {
            spin1_delay_us(1);
    }

}

void send_position_out()
{

    if(!is_main)
        return;

    //send a message out
    while (!spin1_send_mc_packet(output_key + (XY_CODE((uint32_t)x, (uint32_t)y) << 1), 0, NO_PAYLOAD)) {
        spin1_delay_us(1);
    }

//        static int dropper = 0;
//        if(dropper % 100 == 0)
//            log_info("Sending output: %u %u", uint32_t(x), uint32_t(y));
//        dropper++;



}

////////////////////////////////////////////////////////////////////////////////
// ALGORITHM FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

//! \brief move local particle data into particle array data
void load_particle_into_next_array() {

    work_data[last_index][X_IND] = x;
    work_data[last_index][Y_IND] = y;
    work_data[last_index][R_IND] = r;
    work_data[last_index][W_IND] = w;

}

//! \brief perform weigth normalisation, calculate average target and
//!     window size.
void normalise() {

    target[0] = 0.0k; target[1] = 0.0k; target[2] = 0.0k;
    accum total = 0.0k;

    for(uint32_t i = 0; i < n_particles; i++) {
        total += proc_data[i][W_IND];
    }
    total = 1.0k / total;

    for(uint32_t i = 0; i < n_particles; i++) {
        proc_data[i][W_IND] *= total;
        target[0] += proc_data[i][X_IND] * proc_data[i][W_IND];
        target[1] += proc_data[i][Y_IND] * proc_data[i][W_IND];
        target[2] += proc_data[i][R_IND] * proc_data[i][W_IND];
    }

}

//! \brief find a random particle to unload (this is the resample step)
void unload_weighted_random_particle() {

    accum rn = MY_RAND;

    //set resampled according to distribution of weights
    accum accumed_sum = 0.0;
    for(random_part_i = 0; random_part_i < n_particles; random_part_i++) {
        accumed_sum += proc_data[random_part_i][W_IND];
        if(accumed_sum > rn) break;
    }
    if(random_part_i == n_particles) random_part_i--;

    x = proc_data[random_part_i][X_IND];
    y = proc_data[random_part_i][Y_IND];
    r = proc_data[random_part_i][R_IND];
    w = proc_data[random_part_i][W_IND];

//    static int divisor = 0;
//    if(divisor++ % DIV_VALUE == 0) {
//        log_info("Chosen Particle: %d", i);
//    }

}

void predict(float sigma) {

    //should this be changed to a gaussian distribution?

    x += 2.0 * sigma * MY_RAND - sigma;
    y += 2.0 * sigma * MY_RAND - sigma;
    r += 0.2 * (2.0 * sigma * MY_RAND - sigma);

    if(r < 10.0k)      r = 10.0k;
    if(r > MAX_RADIUS)      r = MAX_RADIUS;
    if(x < -r)      x = -r;
    if(x > 304.0k+r)   x = 304.0k+r;
    if(y < -r)      y = -r;
    if(y > 240.0k+r)   y = 240.0k + r;

}

static inline accum approxatan2(accum y, accum x) {

    accum absy = y > 0.0k ? y : -y;
    accum absx = x > 0.0k ? x : -x;
    accum a = absy < absx ? absy / absx : absx / absy;
    accum r = a * (K_PI_4 - (a - 1.0k) * 0.2733185k);
    if(absy > absx) r = 1.57079637k - r;
    if(x < 0.0k) r = 3.14159274k - r;
    if(y < 0.0k) r = -r;

    return r;

}

void calculate_likelihood() {

    //load in new data
    //uint cpsr = spin1_int_disable();
    uint32_t num_new_events = circular_buffer_size(retina_buffer);
    uint32_t min_batch_size = (uint32_t)(K_PI * r + 0.5k);
    uint32_t num_batch_events = min_batch_size < num_new_events ? min_batch_size : num_new_events;
    for(uint32_t i = 0; i < num_batch_events; i++) {
        start_window = (start_window + 1) % EVENT_WINDOW_SIZE;
        circular_buffer_get_next(retina_buffer, &event_window[start_window]);
    }
    //spin1_mode_restore(cpsr);


    //set the new window size
    size_window = size_window + num_batch_events;
    if(size_window > EVENT_WINDOW_SIZE) size_window = EVENT_WINDOW_SIZE;

   // int events_processed_temp = 0;
//    if(num_new_events < size_window)
//        events_processed_temp = num_new_events;
//    else
//        events_processed_temp = size_window;
    //events_unprocessed += num_new_events - events_processed_temp;
    events_processed += num_batch_events;
    if(num_batch_events < min_batch_size)
        double_processed += min_batch_size - num_new_events;

    //initialise the likelihood calculation
    l = MIN_LIKE;
    score = 0.0k;
    n = (accum)size_window;
    memset(L, 0, ANG_BUCKETS * sizeof(accum));
    n_neg_events = 0;
//    for(uint32_t i = 0; i < ANG_BUCKETS; i++) {
//        L[i] = 0.0k;
//    }
    negativeScaler = NEG_BIAS_CONSTANT / (r * r);

    //calculate the likelihood;
    accum dx, dy, D2, D, ABSDR, cval;
    uint32_t L_i;

    uint32_t count = 0;
    uint32_t i = start_window;
    while(count < size_window) {

        dx = X_MASK(event_window[i]) - x;
        dy = Y_MASK(event_window[i]) - y;
        D2 = dx * dx + dy * dy;

        if(D2 <= MAX_RADIUS_PLUS2_SQRD) {

            D = LUT_SQRT[(uint32_t)(D2+0.5k)];

            if(D < r + INLIER_PAR_PLUS1) {
                if(D > r) ABSDR = D - r;
                else ABSDR = r - D;

                if(ABSDR <= INLIER_PAR_PLUS1) {
                    L_i = (uint32_t)(0.5k + 10.026769884k * (approxatan2(dy, dx) + K_PI));
                    cval = ABSDR < 1.0k ? 1.0k : (INLIER_PAR_PLUS1 - ABSDR)*INV_INLIER_PAR;
                    if(cval > L[L_i]) {
                        score = (score + cval) - L[L_i];
                        L[L_i] = cval;
                        if(score > l) {
                            l = score;
                        }
                    }
                } else {
                    n_neg_events++;
                    score -= negativeScaler;
                }
            }
        }

        //update our counters
        if(i == 0) i = EVENT_WINDOW_SIZE;
        i--;
        count++;
    }

    w = w * l;

}

void particle_filter_update_step(uint do_p2p, uint do_calc) {

    use(do_p2p);
    use(do_calc);

    normalise();

//    static int divisor = 0;
//    if(divisor++ % DIV_VALUE == 0) {
//        for(uint32_t i = 0; i < n_particles; i++) {
//            uint32_t wdecimal1 = (int)(proc_data[i][W_IND]*1000)%10;
//            uint32_t wdecimal2 = (int)(proc_data[i][W_IND]*10000)%10;
//            log_info("[%d %d %d] L:%d W:%d.%d%d #%d", (int)proc_data[i][X_IND], (int)proc_data[i][Y_IND],
//            (int)proc_data[i][R_IND], (int)(proc_data[i][L_IND]), (int)(proc_data[i][W_IND]*100),
//            wdecimal1, wdecimal2, (int)proc_data[i][N_IND]);
//        }
//        log_info("Target: [%d %d]", (int)x_target, (int)y_target);
//    }

    unload_weighted_random_particle();
    predict(SIGMA);
    calculate_likelihood();
    //w = 0.5k;


    //do final tasks
    send_roi();
    send_position_out();
    load_particle_into_next_array();

    tried_to_call_proc_done = true;
    if(!spin1_schedule_callback(ready_to_send, 1, my_p2p_id == 0, SEND)) {
        log_error("Could not call proc done callback");
    }

    //uint cpsr = spin1_fiq_disable();
    //spin1_mode_restore(cpsr);
    //spin1_delay_us(100);

}

////////////////////////////////////////////////////////////////////////////////
// SYSTEM INITIALISATION
////////////////////////////////////////////////////////////////////////////////

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

    //accum a = MY_RAND;
    //log_info("Using Fixed Point (%d %d 0.%d%d%d)", sizeof(accum), (int)a, (int)(a*10)%10, (int)(a*100)%10, (int)(a*1000)%10);


    //log_debug("on tick %d of %d", time, simulation_ticks);

    // check that the run time hasn't already elapsed and thus needs to be
    // killed
    if ((infinite_run != TRUE) && (time >= simulation_ticks)) {
        log_info("Simulation complete.\n");

        if (recording_flags > 0) {
            recording_finalise();
            log_info("updating recording regions");
        }

        // falls into the pause resume mode of operating
        simulation_handle_pause_resume(resume_callback);

        return;

    }

    if (recording_flags > 0) {
        for(int i = 0; i < TARGET_ELEMENTS; i++)
            save_vector[i] = (float)target[i];
        recording_record(0, save_vector, sizeof(save_vector));
    }

    if(time == 0) {
        log_info("my key = %d", p2p_key);
    }

    if(time*timer_period >= log_counter) {
        log_counter += LOG_COUNTER_PERIOD;
        float avg_period = 1000.0f/(float)update_count;

        log_info("Update: %d Hz / %d.%d%d ms | Events: %d/%d "
            "(%d dropped / %d unprocessed / %d overprocessed)",
            update_count, (int)avg_period, (int)(avg_period*10)%10,
            (int)(avg_period*100)%10, events_processed, received_count,
            dropped_count, events_unprocessed, double_processed);

        log_info("Score %d.%d (%d)", (int)score, (int)(score*10)%10, random_part_i);
        log_info("Negative Scaler %d.%d%d (%d)", (int)negativeScaler,
            (int)(negativeScaler*10)%10, (int)(negativeScaler*100)%10,
            n_neg_events);
        //log_info("Window Size: %d, Start Index: %d", size_window, start_window);

        //log_info("Received Particle Messages: %d", packets_received);
        //log_info("%d %d (%d %d)", finished_processing, packet_sending_turn, tried_to_call_proc_done, tried_to_call_my_turn);

        update_count = 0;
        events_processed = 0;
        events_unprocessed = 0;
        received_count = 0;
        dropped_count = 0;
        double_processed = 0;


    }

    if(time == 0 && my_p2p_id == 0) {
        //sendstate();
        //spin1_trigger_user_event(0, 1);
        spin1_schedule_callback(ready_to_send, 0, 1, SEND);
    }


}

//! \brief reads the config data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_config(address_t address){

    //read data
    my_p2p_id = address[P2P_ID];
    n_particles = address[N_PARTICLES];
    if (address[IS_MAIN]) is_main = true;

    x = address[X_COORD];
    y = address[Y_COORD];
    r = address[RADIUS];

    //compute some constants
    my_turn = PACKETS_PER_PARTICLE * my_p2p_id;
    last_index = n_particles - 1;
    full_buffer = PACKETS_PER_PARTICLE * (n_particles-1);

    //print some info
    log_info("\n==Particle Information==");
    if(is_main) log_info("Main Particle");
    log_info("ID: %d / %d, (my_turn: %d)", my_p2p_id, n_particles, my_turn);
    log_info("x, y, r: %u, %u, %u", (uint32_t)x, (uint32_t)y, (uint32_t)r);

    return true;
}

//! \brief reads the transmission keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_transmission_keys(address_t address){
    i_has_key = address[HAS_KEY];
    p2p_key = address[P2P_KEY];
    filter_update_key = address[FILTER_UPDATE_KEY];
    output_key = address[OUTPUT_KEY];

    return true;
}

//! \brief main initisation method
//! \param[in] timer_period. the time set for the timer
//! \return bool true if successful, false otherwise
bool initialize(uint32_t *timer_period) {

    log_info("Initialise: started");

    accum temp = -10.0;
    accum temp2 = int_to_accum(accum_to_int(temp) & 0x7FFFFFFF);
    log_info("abs? %d %d", (int)temp, (int)temp2);

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
    if (!read_transmission_keys(data_specification_get_region(
            TRANSMISSION_DATA_REGION, address))){
        return false;
    }

    // Setup recording
    if(is_main) {
        if (!recording_initialize(
            data_specification_get_region(RECORDING, address),
            &recording_flags)) {
                rt_error(RTE_SWERR);
        }
        log_info("Recording Flags: 0x%08x", recording_flags);
    }


//    int test_position[3] = {152, 120, 40};
//    recording_record(0, test_position, sizeof(test_position));

    // initialise my input_buffer for receiving packets
    retina_buffer = circular_buffer_initialize(RETINA_BUFFER_SIZE); //in ints
    if (retina_buffer == 0){
        log_info("Could not create retina buffer");
        return false;
    }

    proc_data = spin1_malloc(n_particles * sizeof(accum*));
    work_data = spin1_malloc(n_particles * sizeof(accum*));
    for(uint32_t i = 0; i < n_particles; i++) {
        proc_data[i] = spin1_malloc(PACKETS_PER_PARTICLE * sizeof(accum));
        work_data[i] = spin1_malloc(PACKETS_PER_PARTICLE * sizeof(accum));
        if(!proc_data[i] || !work_data[i]) {
            log_error("not enough space to create p2p data");
            return false;
        }
    }

    uint32_t n_indices = MAX_RADIUS_PLUS2_SQRD + 1;
    LUT_SQRT = spin1_malloc(n_indices * sizeof(accum));
    for(uint32_t i = 0; i < n_indices; i++)
            LUT_SQRT[i] = sqrtk((accum)i);


    load_particle_into_next_array();

    spin1_srand (p2p_key);

    log_info("Initialisation successful");

    return true;
}


//! \brief main entrance method
void c_main() {
    log_info("starting particle filter\n");

    // initialise the model
    if (!initialize(&timer_period)) {
        log_error("failed to init");
        rt_error(RTE_SWERR);
        return;
    }

    // set timer tick value to configured value
    log_info("setting timer to execute every %d microseconds", timer_period);
    spin1_set_timer_tick(timer_period);

    // register callbacks
    spin1_callback_on(MCPL_PACKET_RECEIVED, receive_particle_data_packet, PACKET);
    spin1_callback_on(MC_PACKET_RECEIVED, receive_retina_event, PACKET);
    spin1_callback_on(TIMER_TICK, update, TIMER);
    //spin1_callback_on(USER_EVENT, sendstate, USER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
