
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

#define ANG_BUCKETS 64
#define INLIER_PAR 2
#define MIN_LIKE 10
#define SIGMA_SCALER 4.0f
#define TDMA_WAIT_PERIOD 300
#define PACKETS_PER_PARTICLE 6

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;
static uint32_t update_count = 0;
static uint32_t received_count = 0;

//! data format
typedef struct data_items_t {
    float x;
    float y;
    float r;
    float l;
    float w;
    uint32_t n;
} data_items_t;
static data_items_t *particle_data;

//! timer period
uint32_t timer_period;
uint32_t max_counter;
uint32_t events_processed = 0;

//! parameters for this c code
static float x = 64.0f, nx = -1.0f;
static float y = 64.0f, ny = -1.0f;
static float r = 30.0f, nr = -1.0f;
static float l = 0.0f, nl = -1.0f;
static float w = 1.0f, nw = -1.0f;
static uint32_t n = 0;

static float L[ANG_BUCKETS];
static circular_buffer retina_buffer, particle_buffer;
static uint32_t *qcopy;

uint32_t n_particles;

float x_target;
float y_target;
float new_n;

//! transmission key
static uint32_t i_has_key;
static uint32_t base_key;

static bool is_main = false;
static uint32_t main_key;
static uint32_t output_key;

static uint32_t my_tdma_id;

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
    MC_PACKET = -1, MCPL_PACKET = 0, SDP_DMA = 1, USER = 4, TIMER = 3
} callback_priorities;

//! human readable definitions of each element in the transmission region
typedef enum transmission_region_elements {
    HAS_KEY = 0, MY_KEY = 1, TDMA_ID = 2
} transmission_region_elements;

//! human readable definitions of each element in the config region
typedef enum config_region_elements {
    X_COORD = 0, Y_COORD = 1, RADIUS = 2, PACKET_THRESHOLD = 3, IS_MAIN = 4,
    MAIN_KEY = 5, OUTPUT_KEY = 6, N_PARTICLES = 7
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
void performFullUpdate();

////////////////////////////////////////////////////////////////////////////////
// SEND/RECEIVE
////////////////////////////////////////////////////////////////////////////////


//! \brief callback for when packet has payload (agg)
//! \param[in] key: the key received
//! \param[in] payload: the payload received
void receive_data_payload(uint key, uint payload) {

    //this will be the values of the other particles

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

    //this will be the events
    if (!circular_buffer_add(retina_buffer, key)) {
        //log_error("Could not add 1000 events");
    }
    received_count++;

}

//! \brief callback for user
//! \param[in] random param1
//! \param[in] random param2
void user_callback(uint user0, uint user1) {

    //circular_buffer_clear(particle_buffer);
    use(user0);
    use(user1);

    //float x, y;
    uint32_t n_packets = circular_buffer_size(particle_buffer) / 2;


    for(uint32_t i = 0; i < n_packets; i++) {

        uint32_t key, payload;
        if(!circular_buffer_get_next(particle_buffer, &key))
            log_error("Could not get key from buffer");
        if(!circular_buffer_get_next(particle_buffer, &payload))
            log_error("Could not get payload from buffer");

        //THIS WILL HAVE TO CHANGE!! ASK ABOUT KEY SPACE!!
        switch(key & 0x07) {
        case(X_IND):
            particle_data[i].x = int_to_float(payload);
            break;
        case(Y_IND):
            particle_data[i].y = int_to_float(payload);
            break;
        case(R_IND):
            particle_data[i].r = int_to_float(payload);
            break;
        case(L_IND):
            particle_data[i].l = int_to_float(payload);
            break;
        case(W_IND):
            particle_data[i].w = int_to_float(payload);
            break;
        case(N_IND):
            particle_data[i].n = payload;
            break;
        default:
            log_error("incorrect key value received at aggregator");

        }

    }

    performFullUpdate();

    //sendstate to other particles
//
//    //if(special) sendROI to filters
//
//    //if(special) sendXY out
//
//    particleResample();
//    nx = ny = nr = nl = nw = -1;
//    nn = 0;
//
//    predict(SIGMA_SCALER * n / ANG_BUCKETS);
//    n = 0;
//
//    processInput();
//
//    concludeLikelihood();
//
//    sendstate();
//
//    update_count++;


}

void sendstate() {

    //we need to send X, Y, R, W, N

    //log_info("sending state, %d", sv->cpu_clk);
    if(i_has_key == 1) {
       uint32_t current_time = tc[T1_COUNT];

       //calculate max_counter;
       max_counter = timer_period * sv->cpu_clk;


       int dt = current_time - tc[T1_COUNT];
       if(dt < 0) dt += max_counter;
       while(dt < (int)(my_tdma_id * TDMA_WAIT_PERIOD)) {
            dt = current_time - tc[T1_COUNT];
            if(dt < 0) dt += max_counter;
       }

        //send a message out
        //log_info("sending packets %d", time);
        while (!spin1_send_mc_packet(
                base_key + X_IND, float_to_int(x),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + Y_IND, float_to_int(y),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + R_IND, float_to_int(r),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + L_IND, float_to_int(l),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + W_IND, float_to_int(w),
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
        while (!spin1_send_mc_packet(
                base_key + N_IND, n,
                WITH_PAYLOAD)) {
            spin1_delay_us(1);
        }
    }
}

void send_roi() {


}

void decodexy(uint32_t coded, float *x, float *y) {

    *x = coded & 0x1FF;
    *y = (coded >> 9) & 0xFF;

}

uint32_t codexy(float x, float y)
{
    return ((int)x & 0x1FF) + (((int)y & 0xFF) << 9);
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
        while (!spin1_send_mc_packet(
            output_key + (codexy(x, y) << 1), 0, NO_PAYLOAD)) {
            spin1_delay_us(1);
        }
        static int dropper = 0;
        if(dropper % 10000 == 0)
            log_info("Sending output: %f %f", x, y);

    }

}

////////////////////////////////////////////////////////////////////////////////
// ALGORITHM FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void normalise();
void incLikelihood(float vx, float vy);

void performFullUpdate()
{
    //normalise all the weights, get target position
    normalise();
    //get average n-window
        //clear the local buffer if too large
    //if(new_n) > current_buffer_size - 30;
    //  new_n - current_buffer_size


    //performResample

    //performPrediction

    //processInput (copy list out, append to other, perform observation)

    //conclude likelihood (e.g w = w*l)

}

void processInput() {

    uint32_t current_key, buffersize;
    float vx, vy;

    uint cpsr = spin1_int_disable();

    buffersize = circular_buffer_size(retina_buffer);
    //circular_buffer_clear(retina_buffer);
    //copy data

//
    for(uint32_t i = 0; i < buffersize; i++) {
        if(!circular_buffer_get_next(retina_buffer, &current_key))
            log_error("Could not get key from buffer");
        qcopy[i] = current_key;
    }
//
    spin1_mode_restore(cpsr);

    for(uint32_t i = 0; i < buffersize; i++) {
        decodexy(qcopy[i], &vx, &vy);
        incLikelihood(vx, vy);
    }


    events_processed += buffersize;

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

void normalise() {

    x_target = 0; y_target = 0; new_n = 0;
    float total = 0;
    for(uint32_t i = 0; i < n_particles; i++) {
        total += particle_data[i].w;
    }
    total = 1.0 / total;

    for(uint32_t i = 0; i < n_particles; i++) {
        particle_data[i].w *= total;
        x_target += particle_data[i].x * particle_data[i].w;
        y_target += particle_data[i].y * particle_data[i].w;
        new_n += particle_data[i].n * particle_data[i].w;
    }


}

void predict(float sigma) {

    //use a flat distribution?
    x += 2.0 * sigma * rand() / RAND_MAX - sigma;
    y += 2.0 * sigma * rand() / RAND_MAX - sigma;
    r += 2.0 * sigma * rand() / RAND_MAX - sigma;

    if(r < 10)      r = 10;
    if(r > 40)      r = 40;
    if(x < -r)      x = -r;
    if(x > 304+r)   x = 304+r;
    if(y < -r)      y = -r;
    if(y > 240+r)   y = 240 + r;

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

void resample() {



//    float rn = 1.0 * (double)rand() / RAND_MAX;
//    if(rn > 1.0) {
//
//            //set resampled data to random values
//            resampled_data.x = 10 + rand() % 284;
//            resampled_data.y = 10 + rand() % 220;
//            resampled_data.r = 20.0 + rand() % 10;
//            resampled_data.l = particle_data[partner_i].l;
//            resampled_data.w = particle_data[partner_i].w;
//            resampled_data.n = particle_data[partner_i].n;
//
//        } else {
//
//            //set resampled according to distribution of weights
//            float accumed_sum = 0.0;
//            uint32_t j = 0;
//            for(j = 0; j < n_particles; j++) {
//                accumed_sum += particle_data[j].w;
//                if(accumed_sum > rn) break;
//            }
//            resampled_data = particle_data[j];
//
//        }



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

    if(time % 1000 == 0) {
        log_info("Update Rate = %d Hz | # EventProcessed = %d Hz | Events Received = %d Hz", update_count, events_processed, received_count);
        update_count = 0;
        events_processed = 0;
        received_count = 0;
    }

    if(time == 0) {
        log_info("my key = %d : my aggregator key = %d", base_key);
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
    if (address[IS_MAIN] == 0){
        is_main = true;
        main_key = address[MAIN_KEY];
        output_key = address[OUTPUT_KEY];
    }
    else{
        is_main = false;
        main_key = 0;
        output_key = 0;
    }
    n_particles = address[N_PARTICLES];
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
    if (!read_transmission_keys(data_specification_get_region(
            TRANSMISSION_DATA_REGION, address))){
        return false;
    }

    // initialise my input_buffer for receiving packets
    log_info("build buffer");
    retina_buffer = circular_buffer_initialize(256);
    if (retina_buffer == 0){
        return false;
    }
    log_info("retina_buffer initialised");
    qcopy = spin1_malloc(256 * sizeof(uint32_t));

    particle_data =
        (data_items_t*) spin1_malloc(n_particles * sizeof(data_items_t));


    // initialise my input_buffer for receiving packets
    log_info("build buffer");
    particle_buffer = circular_buffer_initialize(PACKETS_PER_PARTICLE * 8);
    if (particle_buffer == 0){
        return false;
    }
    log_info("particle_buffer initialised");

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
