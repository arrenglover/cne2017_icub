
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

#define TYPE_SELECT 1 //0-fixed, 1-float
#if TYPE_SELECT == 0

#define CALC_TYPE accum
#define SUFFIX k
#include <stdfix.h>

#elif TYPE_SELECT == 1

#define CALC_TYPE float
#define SUFFIX f

#endif

#define SPIN_RAND_MAX UINT32_MAX

#define X_MASK(x) x&0x1FF
#define Y_MASK(y) (y>>9)&0xFF
#define XY_CODE(x, y) (x&0x1FF)|((y&0xFF)<<9)

#define ANG_BUCKETS 64
#define INLIER_PAR 1
#define MIN_LIKE 0.2f
#define SIGMA 1.0f
#define NEGATIVE_BIAS 0.2f;
#define PACKETS_PER_PARTICLE 6
#define DIV_VALUE 200
#define EVENT_WINDOW_SIZE 500

#define CONSTANT1 (ANG_BUCKETS - 1) / (M_PI * 2.0)
#define INV_INLIER_PAR 1.0/INLIER_PAR


//! SYSTEM VARIABLES
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;
static uint32_t timer_period;
static uint32_t log_counter = 0;
static uint32_t recording_flags = 0;

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
static uint32_t start_window = 0;
static uint32_t size_window = 0;

static float L[ANG_BUCKETS];
static float score;
static float negativeScaler;


static float x = 64.0f;
static float y = 64.0f;
static float r = 30.0f;
static float l = MIN_LIKE;
static float w = 1.0f;
static float n = 0.0f;

static float target[3];

//! SENDING/RECEIVING VARIABLES

static bool is_main = false;
static uint32_t filter_update_key;
static uint32_t output_key;

static uint32_t i_has_key;
static uint32_t p2p_my_key;
static uint32_t my_p2p_id;

static uint32_t n_particles;
static uint32_t last_index;
static float **proc_data, **work_data;
static circular_buffer retina_buffer;

static uint32_t full_buffer;
static uint32_t my_turn;

typedef enum packet_identifiers{
    X_IND = 0, Y_IND = 1, R_IND = 2, L_IND = 3, W_IND = 4, N_IND = 5
}packet_identifiers;


//! DEBUG VARIABLES
static uint32_t events_processed = 0;
static uint32_t update_count = 0;
static uint32_t received_count = 0;
static uint32_t dropped_count = 0;
static uint32_t packets_received = 0;

static bool packet_sending_turn = false;
static bool finished_processing = true;
static bool tried_to_call_my_turn = false;
static bool tried_to_call_proc_done = false;


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
        int_to_float(payload);

    if(packets_received == my_turn) { //it is our turn to send data
        tried_to_call_my_turn = true;
        if(!spin1_schedule_callback(ready_to_send, 0, 1, SEND))
            log_error("Couldn't make my_turn callback!");
    }

    if(packets_received == full_buffer) { //perform update

        float **temp = work_data;
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
    while (!spin1_send_mc_packet(p2p_my_key + X_IND, float_to_int(x),
            WITH_PAYLOAD))
        spin1_delay_us(1);
    while (!spin1_send_mc_packet(p2p_my_key + Y_IND, float_to_int(y),
            WITH_PAYLOAD))
        spin1_delay_us(1);
    while (!spin1_send_mc_packet(p2p_my_key + R_IND, float_to_int(r),
            WITH_PAYLOAD))
        spin1_delay_us(1);
    while (!spin1_send_mc_packet(p2p_my_key + L_IND, float_to_int(l),
            WITH_PAYLOAD))
        spin1_delay_us(1);
    while (!spin1_send_mc_packet(p2p_my_key + W_IND, float_to_int(w),
            WITH_PAYLOAD))
        spin1_delay_us(1);
    while (!spin1_send_mc_packet(p2p_my_key + N_IND, float_to_int(n),
            WITH_PAYLOAD))
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
                (int)(r*1.4f), WITH_PAYLOAD)) {
            spin1_delay_us(1);
    }

}

void send_position_out()
{

    if(!is_main)
        return;




        //send a message out
//        while (!spin1_send_mc_packet(
//            output_key + (codexy(x, y) << 1), 0, NO_PAYLOAD)) {
//            spin1_delay_us(1);
//        }

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
    work_data[last_index][L_IND] = l;
    work_data[last_index][W_IND] = w;
    work_data[last_index][N_IND] = n;

}

//! \brief perform weigth normalisation, calculate average target and
//!     window size.
void normalise() {

    target[0] = 0; target[1] = 0; target[2] = 0;
    float new_n = 0;
    float total = 0;

    for(uint32_t i = 0; i < n_particles; i++) {
        total += proc_data[i][W_IND];
    }
    total = 1.0 / total;

    for(uint32_t i = 0; i < n_particles; i++) {
        proc_data[i][W_IND] *= total;
        target[0] += proc_data[i][X_IND] * proc_data[i][W_IND];
        target[1] += proc_data[i][Y_IND] * proc_data[i][W_IND];
        target[2] += proc_data[i][R_IND] * proc_data[i][W_IND];
        new_n += proc_data[i][N_IND] * proc_data[i][W_IND];
    }

    if(size_window > new_n + 50)
        size_window = new_n;

}

//! \brief find a random particle to unload (this is the resample step)
void unload_weighted_random_particle() {

    float rn = (float)spin1_rand() / SPIN_RAND_MAX;

    //set resampled according to distribution of weights
    float accumed_sum = 0.0;
    uint32_t i = 0;
    for(i = 0; i < n_particles; i++) {
        accumed_sum += proc_data[i][W_IND];
        if(accumed_sum > rn) break;
    }
    if(i == n_particles) i--;

    x = proc_data[i][X_IND];
    y = proc_data[i][Y_IND];
    r = proc_data[i][R_IND];
    w = proc_data[i][W_IND];
    l = proc_data[i][L_IND];
    n = proc_data[i][N_IND];

//    static int divisor = 0;
//    if(divisor++ % DIV_VALUE == 0) {
//        log_info("Chosen Particle: %d", i);
//    }

}

void predict(float sigma) {

    //TODO: should this be changed to a gaussian distribution?
    x += 2.0 * sigma * ((float)spin1_rand() / SPIN_RAND_MAX) - sigma;
    y += 2.0 * sigma * ((float)spin1_rand() / SPIN_RAND_MAX) - sigma;
    r += 2.0 * sigma * ((float)spin1_rand() / SPIN_RAND_MAX) - sigma;

    if(r < 10)      r = 10;
    if(r > 40)      r = 40;
    if(x < -r)      x = -r;
    if(x > 304+r)   x = 304+r;
    if(y < -r)      y = -r;
    if(y > 240+r)   y = 240 + r;

}

static inline void incremental_calculation(uint32_t vx, uint32_t vy, uint32_t count) {

    //update the L vector based on the event
    float dx = vx - x;
    float dy = vy - y;

    float sqrd = sqrt(dx * dx + dy * dy) - r;
    if(sqrd > 1.0 + INLIER_PAR)
        return;

    float fsqrd = sqrd > 0 ? sqrd : -sqrd;

    int L_i = 0.5 + (CONSTANT1 * (atan2f(dy, dx) + M_PI));
    float cval = 0.0;

    if(fsqrd < 1.0)
        cval = 1.0;
    else if(fsqrd < 1.0 + INLIER_PAR)
        cval = (1.0 + INLIER_PAR - fsqrd) * INV_INLIER_PAR;

    if(cval > 0.0) {
        float improved = cval - L[L_i];
        if(improved > 0) {
            L[L_i] = cval;
            score += improved;
            if(score >= l) {
                l = score;
                n = count;
            }
        }
    } else {
        score -= negativeScaler;
    }
}

void calculate_likelihood() {

    //load in new data
    uint cpsr = spin1_int_disable();
    uint32_t num_new_events = circular_buffer_size(retina_buffer);
    for(uint32_t i = 0; i < num_new_events; i++) {
        start_window = (start_window + 1) % EVENT_WINDOW_SIZE;
        circular_buffer_get_next(retina_buffer, &event_window[start_window]);
    }
    spin1_mode_restore(cpsr);
    events_processed += num_new_events;

    //set the new window size
    size_window = size_window + num_new_events;
    if(size_window > EVENT_WINDOW_SIZE) size_window = EVENT_WINDOW_SIZE;

    //initialise the likelihood calculation
    l = MIN_LIKE;
    score = 0;
    n = size_window;
    for(uint32_t i = 0; i < ANG_BUCKETS; i++) {
        L[i] = 0;
    }
    negativeScaler = (ANG_BUCKETS / (M_PI * r * r)) * NEGATIVE_BIAS;

    //calculate the likelihood;
    uint32_t count = 0;
    uint32_t i = start_window;
    while(count < size_window) {

        incremental_calculation(X_MASK(event_window[i]), Y_MASK(event_window[i]), count);
        //spin1_delay_us(1);

        //update our counters
        if(i == 0) i = EVENT_WINDOW_SIZE;
        i--;
        count++;
    }

    if(n < 100) n = 100;
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

    if (recording_flags > 0)
        recording_record(0, target, sizeof(target));

    if(time == 0) {
        log_info("my key = %d", p2p_my_key);
    }

    if(time*timer_period >= log_counter) {
        log_counter += 1000000;
        log_info("Update Rate = %d Hz | # EventProcessed = %d Hz | "
            "Events Received = %d Hz  | Dropped: %d Hz",
            update_count, events_processed, received_count, dropped_count);
        //log_info("Received Particle Messages: %d", packets_received);
        update_count = 0;
        events_processed = 0;
        received_count = 0;
        dropped_count = 0;

        //log_info("%d %d (%d %d)", finished_processing, packet_sending_turn, tried_to_call_proc_done, tried_to_call_my_turn);
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
    p2p_my_key = address[P2P_KEY];
    filter_update_key = address[FILTER_UPDATE_KEY];
    output_key = address[OUTPUT_KEY];

    return true;
}

//! \brief main initisation method
//! \param[in] timer_period. the time set for the timer
//! \return bool true if successful, false otherwise
bool initialize(uint32_t *timer_period) {

    log_info("Initialise: started");

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
    retina_buffer = circular_buffer_initialize(512); //in ints
    if (retina_buffer == 0){
        log_info("Could not create retina buffer");
        return false;
    }

    proc_data = spin1_malloc(n_particles * sizeof(float*));
    work_data = spin1_malloc(n_particles * sizeof(float*));
    for(uint32_t i = 0; i < n_particles; i++) {
        proc_data[i] = spin1_malloc(PACKETS_PER_PARTICLE * sizeof(float));
        work_data[i] = spin1_malloc(PACKETS_PER_PARTICLE * sizeof(float));
        if(!proc_data[i] || !work_data[i]) {
            log_error("not enough space to create p2p data");
            return false;
        }
    }

    load_particle_into_next_array();

    //spin1_srand (p2p_my_key);

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
