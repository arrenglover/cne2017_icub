
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
#define Y_MASK(y) (y>>9)&0xFF
#define XY_CODE(x, y) (x&0x1FF)|((y&0xFF)<<9)

#define ANG_BUCKETS 64
#define INLIER_PAR 1
#define MIN_LIKE 0.2f
#define SIGMA_SCALER 4.0f
#define NEGATIVE_BIAS 0.2f;
#define PACKETS_PER_PARTICLE 6
#define DIV_VALUE 5000
#define EVENT_WINDOW_SIZE 1000

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;

//! timer period
uint32_t timer_period;

uint32_t events_processed = 0;
uint32_t update_count = 0;
uint32_t received_count = 0;
uint32_t dropped_count = 0;

static uint32_t event_window[EVENT_WINDOW_SIZE];
static uint32_t start_window = 0;
static uint32_t size_window = 0;

static float L[ANG_BUCKETS];
static float score;
static float negativeScaler;

//! parameters for this c code

float x = 64.0f;
float y = 64.0f;
float r = 30.0f;
float l = 0.0f;
float w = 1.0f;
float n = 20.0f;

static uint32_t LAST_INDEX;

float **proc_data, **work_data;

static circular_buffer retina_buffer;


uint32_t n_particles;

static uint32_t packets_received = 0;

float x_target;
float y_target;

//! transmission key
static uint32_t i_has_key;
static uint32_t p2p_my_key;
static uint32_t my_tdma_id;

static bool is_main = false;
static uint32_t filter_update_key;
static uint32_t output_key;

static bool computing = false;
static bool okay_to_send = true; //enforce p2p only once per update
static uint32_t full_buffer;
static uint32_t my_turn;

//! The recording flags
static uint32_t recording_flags = 0;

//! key bases offsets
typedef enum packet_identifiers{
    X_IND = 0, Y_IND = 1, R_IND = 2, L_IND = 3, W_IND = 4, N_IND = 5
}packet_identifiers;

//! human readable definitions of each region in SDRAM
typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA_REGION,
    CONFIG_REGION
} regions_e;

//! values for the priority for each callback
typedef enum callback_priorities {
    MC_PACKET = -1, MCPL_PACKET = -1, SDP_DMA = 1, USER = 4, TIMER = 3
} callback_priorities;

//! human readable definitions of each element in the transmission region
typedef enum transmission_region_elements {
    HAS_KEY = 0, MY_KEY = 1, TDMA_ID = 2
} transmission_region_elements;

//! human readable definitions of each element in the config region
typedef enum config_region_elements {
    X_COORD = 0, Y_COORD = 1, RADIUS = 2, PACKET_THRESHOLD = 3, IS_MAIN = 4,
    FILTER_UPDATE_KEY = 5, OUTPUT_KEY = 6, N_PARTICLES = 7
} config_region_elements;

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
void unload_weighted_random_particle();
void predict(float sigma);
void normalise();
void calculate_likelihood();
void load_particle_into_next_array();
void sendstate();
void send_roi();

////////////////////////////////////////////////////////////////////////////////
// SEND/RECEIVE
////////////////////////////////////////////////////////////////////////////////

//! \brief callback when packet with no payload is received (retina)
//! \param[in] key: the key received
//! \param[in] payload: unused. is set to 0
void receive_data_no_payload(uint key, uint payload) {

    use(payload);

    //this will be the events
    if (!circular_buffer_add(retina_buffer, key)) {
        //log_error("Could not add 1000 events");
        dropped_count++;
    }
    received_count++;

}

//! \brief callback for when packet has payload (agg)
//! \param[in] key: the key received
//! \param[in] payload: the payload received
void receive_data_payload(uint key, uint payload) {

    //load in data
    work_data[packets_received++ / PACKETS_PER_PARTICLE][key&0x7] =
        int_to_float(payload);
    okay_to_send = true;

    if(computing) return; //don't send anything if we haven't correctly updated

    if(packets_received == my_turn) //perform p2p
        sendstate();

    if(packets_received == full_buffer) { //perform update

        computing = true;

        float **temp = work_data;
        work_data = proc_data;
        proc_data = temp;
        packets_received = 0;

        spin1_trigger_user_event(0, 0);
    }

//    if(send || compute)
//        spin1_trigger_user_event(send, compute);

}

//! \brief callback for user
//! \param[in] random param1
//! \param[in] random param2
void user_callback(uint do_p2p, uint do_calc) {

    use(do_p2p);
    use(do_calc);

    normalise();

    static int divisor = 0;
    if(divisor++ % DIV_VALUE == 0) {
        for(uint32_t i = 0; i < n_particles; i++) {
            uint32_t wdecimal1 = (int)(proc_data[i][W_IND]*1000)%10;
            uint32_t wdecimal2 = (int)(proc_data[i][W_IND]*10000)%10;
            log_info("[%d %d %d] L:%d/100 W:%d.%d%d #%d", (int)proc_data[i][X_IND], (int)proc_data[i][Y_IND],
            (int)proc_data[i][R_IND], (int)(proc_data[i][L_IND]*100), (int)(proc_data[i][W_IND]*100),
            wdecimal1, wdecimal2, (int)proc_data[i][N_IND]);
        }
        log_info("Target: [%d %d]", (int)x_target, (int)y_target);
    }

    unload_weighted_random_particle();

    //perform prediction/observation steps
    predict(0.1);

    calculate_likelihood();

    //processInput (copy list out, append to other, perform observation)
    //l = 0.5 + (2.0 * 0.1 * (float)rand() / RAND_MAX) - 0.1;

    //conclude likelihood (e.g w = w*l)
    w = w * l;

    computing = false;

    //do final tasks
    load_particle_into_next_array();
    circular_buffer_clear(retina_buffer);
    send_roi();
    if(packets_received == my_turn) {
        sendstate();
    }

    //uint cpsr = spin1_fiq_disable();
    //spin1_mode_restore(cpsr);
    //spin1_delay_us(100);

}

void sendstate() {

    //we need to send X, Y, R, W, N

    if(!okay_to_send) return;
    okay_to_send = false;

    if(i_has_key == 1) {

        while (!spin1_send_mc_packet(
                p2p_my_key + X_IND, float_to_int(x),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                p2p_my_key + Y_IND, float_to_int(y),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                p2p_my_key + R_IND, float_to_int(r),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                p2p_my_key + L_IND, float_to_int(l),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                p2p_my_key + W_IND, float_to_int(w),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                p2p_my_key + N_IND, float_to_int(n),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
    } else {
        log_info("Particle tried to send a packet without a key");
    }
    update_count++;



}

void send_roi() {

    //this will send to the filters the updated ROI
    //only if the main_particle

    if(is_main) {
        while (!spin1_send_mc_packet(filter_update_key + (XY_CODE((int)x, (int)y)),
                    (int)r, WITH_PAYLOAD)) {
                spin1_delay_us(1);
        }
    }


}


void send_position_out()
{
//    float average_x = 0, average_y = 0;
    if(is_main) {

//        for(uint32_t i = 0; i < n_particles; i++) {
//            average_x += particle_data[i].x * particle_data[i].w;
//            average_y += particle_data[i].y * particle_data[i].w;
//        }

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

}

////////////////////////////////////////////////////////////////////////////////
// ALGORITHM FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void normalise();
void resample();
void predict(float sigma);
void incLikelihood(float vx, float vy);

void load_particle_into_next_array() {

    work_data[LAST_INDEX][X_IND] = x;
    work_data[LAST_INDEX][Y_IND] = y;
    work_data[LAST_INDEX][R_IND] = r;
    work_data[LAST_INDEX][L_IND] = l;
    work_data[LAST_INDEX][W_IND] = w;
    work_data[LAST_INDEX][N_IND] = n;

}


void normalise() {



    x_target = 0; y_target = 0;
    float new_n = 0;
    float total = 0;
    for(uint32_t i = 0; i < n_particles; i++) {
        total += proc_data[i][W_IND];
    }
    total = 1.0 / total;

    for(uint32_t i = 0; i < n_particles; i++) {
        proc_data[i][W_IND] *= total;
        x_target += proc_data[i][X_IND] * proc_data[i][W_IND];
        y_target += proc_data[i][Y_IND] * proc_data[i][W_IND];
        new_n += proc_data[i][N_IND] * proc_data[i][W_IND];
    }

    if(size_window > new_n + 30)
        size_window = new_n;

}

void unload_weighted_random_particle() {


    float rn = (float)rand() / RAND_MAX;

    //set resampled according to distribution of weights
    float accumed_sum = 0.0;
    uint32_t i = 0;
    for(i = 0; i < n_particles; i++) {
        accumed_sum += proc_data[i][W_IND];
        if(accumed_sum > rn) break;
    }
    if(i == n_particles) i--;
    static int divisor = 0;
    if(divisor++ % DIV_VALUE == 0) {
        log_info("Chosen Particle: %d", i);
    }

    x = proc_data[i][X_IND];
    y = proc_data[i][Y_IND];
    r = proc_data[i][R_IND];
    w = proc_data[i][W_IND];
    l = proc_data[i][L_IND];
    n = proc_data[i][N_IND];

}

void predict(float sigma) {

    //use a flat distribution?
    x += 2.0 * sigma * (float)rand() / RAND_MAX - sigma;
    y += 2.0 * sigma * (float)rand() / RAND_MAX - sigma;
    r += 2.0 * sigma * (float)rand() / RAND_MAX - sigma;

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
    if(sqrd > 1.0 + INLIER_PAR) return;

    float fsqrd = sqrd > 0 ? sqrd : -sqrd;
    int a = 0.5 + (ANG_BUCKETS-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
    float cval = 0.0;

    if(fsqrd < 1.0)
        cval = 1.0;
    else if(fsqrd < 1.0 + INLIER_PAR)
        cval = (1.0 + INLIER_PAR - fsqrd) / INLIER_PAR;

    if(cval) {
        float improved = cval - L[a];
        if(improved > 0) {
            L[a] = cval;
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
        circular_buffer_get_next(retina_buffer, &event_window[start_window++]);
        start_window %= EVENT_WINDOW_SIZE;
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
    negativeScaler = ANG_BUCKETS / (M_PI * r * r);
    negativeScaler *= NEGATIVE_BIAS;

    //calculate the likelihood;
    uint32_t count = 0;
    uint32_t i = start_window;
    while(count < size_window) {

        incremental_calculation(X_MASK(event_window[i]), Y_MASK(event_window[i]), count);

        //update our counters
        if(i == 0) i = EVENT_WINDOW_SIZE;
        i--;
        count++;
    }


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
            log_info("updating recording regions");
        }

        // falls into the pause resume mode of operating
        simulation_handle_pause_resume(resume_callback);

        return;

    }

    if(time == 0) {
        log_info("my key = %d", p2p_my_key);
    }

    if(time % 1000 == 999) {
        log_info("Update Rate = %d Hz | # EventProcessed = %d Hz | "
            "Events Received = %d Hz  | Dropped: %d Hz",
            update_count, events_processed, received_count, dropped_count);
        log_info("Received Particle Messages: %d", packets_received);
        update_count = 0;
        events_processed = 0;
        received_count = 0;
        dropped_count = 0;
    }

    if(time == 0 && my_tdma_id == 0) {
        sendstate();
    }


}

//! \brief reads the config data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_config(address_t address){
//    uint32_t xtemp = address[X_COORD];
//    uint32_t ytemp = address[Y_COORD];
//    uint32_t rtemp = address[RADIUS];
    x = address[X_COORD];
    y = address[Y_COORD];
    r = address[RADIUS];
    log_info("x, y, r: %u, %u, %u", (uint32_t)x, (uint32_t)y, (uint32_t)r);
    n = address[PACKET_THRESHOLD];
    log_info("packet_threshold: %d", n);
    if (address[IS_MAIN] == 0){
        is_main = true;
        filter_update_key = address[FILTER_UPDATE_KEY];
        output_key = address[OUTPUT_KEY];
        log_info("Main Particle: 0x%08x 0x%08x", filter_update_key, output_key);
    }
    else{
        log_info("non-main particle");
        is_main = false;
        filter_update_key = 0;
        output_key = 0;
    }
    n_particles = address[N_PARTICLES];
    LAST_INDEX = n_particles - 1;
    full_buffer = PACKETS_PER_PARTICLE * (n_particles-1);
    log_info("n_particles: %d", n_particles);
    return true;
}

//! \brief reads the transmission keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_transmission_keys(address_t address){
    i_has_key = address[HAS_KEY];
    p2p_my_key = address[MY_KEY];
    my_tdma_id = address[TDMA_ID];

    my_turn = PACKETS_PER_PARTICLE * my_tdma_id;
    log_info("ID: %d, my_turn: %d", my_tdma_id, my_turn);

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

    // initialise my input_buffer for receiving packets
    retina_buffer = circular_buffer_initialize(256); //in ints
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


    srand(p2p_my_key);
    computing = false;

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
    spin1_callback_on(MCPL_PACKET_RECEIVED, receive_data_payload, MCPL_PACKET);
    spin1_callback_on(MC_PACKET_RECEIVED, receive_data_no_payload, MC_PACKET);
    spin1_callback_on(TIMER_TICK, update, TIMER);
    spin1_callback_on(USER_EVENT, user_callback, USER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
