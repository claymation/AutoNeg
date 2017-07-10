#ifndef AutoNeg_h
#define AutoNeg_h

#include "Arduino.h"

namespace AutoNeg {

// Code word size (extended pages not supported)
static const unsigned char PAGE_SIZE = 16;

// Code word special bits
static const unsigned char RF  = 13;
static const unsigned char ACK = 14;
static const unsigned char NP  = 15;

/** @brief Timer abstraction with IEEE 802.3 state diagram semantics
 *
 * Timer is a thin wrapper around micros(), providing an ability to
 * statically define a timer's timeout, start a timer, and test if
 * a timer is done, i.e., if it's timeout has arrived.
 */
class Timer {
public:
    Timer(unsigned long period_in_us) : period_(period_in_us) {}

    void start()
    {
        start(micros());
    }

    void start(unsigned long epoch)
    {
        timeout_ = epoch + period_;
    }

    bool done() const
    {
        return micros() >= timeout_;
    }

private:
    unsigned long period_;
    unsigned long timeout_;
};

/** @brief IEEE 802.3 Clause 28 Auto-Negotiation receiver
 *
 * Implements the Receive state diagram from Figure 28-17.
 */
class Receiver {
private:
    enum class State {
        IDLE = 1,
        LINK_PULSE_DETECT,
        LINK_PULSE_COUNT,
        FLP_PASS,
        FLP_CHECK,
        FLP_CAPTURE,
        FLP_CLOCK,
        FLP_DATA_0,
        FLP_DATA_1,
    };

public:
    Receiver();

    void start() {
        enter(State::IDLE);
    }
    void tick();
    void received_pulse();
    void reset();

private:
    void enter(State state);

    static const char FLP_CNT_DONE = 6;
    static const char RX_BIT_CNT_CHECK = 10;

    State state_;
    volatile char linkpulse_ = 0;
    volatile unsigned long linkpulse_time_;
    char flp_cnt_;
    bool flp_receive_idle_;
    bool ability_match_;
    char ability_match_counter_;
    bool acknowledge_match_;
    char acknowledge_match_counter_;
    bool consistency_match_;
    char rx_bit_cnt_;
    char rx_link_code_word_[AutoNeg::PAGE_SIZE];
    char ability_match_word_[AutoNeg::PAGE_SIZE];
    Timer flp_test_min_timer_{25};
    Timer flp_test_max_timer_{165};
    Timer nlp_test_min_timer_{7000};
    Timer nlp_test_max_timer_{150000};
    Timer data_detect_min_timer_{47};
    Timer data_detect_max_timer_{120};  // from experimentation

    friend class Arbitrator;
};

/** @brief IEEE 802.3 Clause 28 Auto-Negotiation transmitter
 *
 * Implements the Transmit state diagram from Figure 28-16.
 */
class Transmitter {
private:
    enum class State {
        IDLE,
        TRANSMIT_REMAINING_ACKNOWLEDGE,
        TRANSMIT_ABILITY,
        TRANSMIT_CLOCK_BIT,
        TRANSMIT_DATA_BIT,
        TRANSMIT_COUNT_ACK,
    };

public:
    Transmitter();

    void start() {
        enter(State::IDLE);
    }
    void tick();

private:
    void enter(State state);
    void send_pulse() const;

    static const char REMAINING_ACK_CNT_DONE = 6;
    static const char TX_BIT_CNT_DONE = PAGE_SIZE + 1;

    State state_;
    char remaining_ack_cnt_;
    bool ack_finished_ = false;
    bool complete_ack_ = false;
    bool transmit_ability_ = false;
    bool transmit_ack_ = false;
    char tx_bit_cnt_;
    char tx_link_code_word_[PAGE_SIZE];
    Timer transmit_link_burst_timer_{14000};
    Timer interval_timer_{48};  // from experimentation

    friend class Arbitrator;
};

/** @brief IEEE 802.3 Clause 28 Auto-Negotiation arbitrator
 *
 * Implements (most of) the Arbitration state diagram from Figure 28-18.
 *
 * Next Page and Extended Page support not (yet) implemented.
 */
class Arbitrator {
public:
    Arbitrator();

    void start();
    void tick();
    void received_pulse() {
        receiver_.received_pulse();
    }
    bool complete() const {
        return mr_autoneg_complete_;
    }
    const char *get_ability_match_word() const {
        return receiver_.ability_match_word_;
    }

private:
    enum class State {
        AUTO_NEGOTIATION_ENABLE,
        TRANSMIT_DISABLE,
        ABILITY_DETECT,
        ACKNOWLEDGE_DETECT,
        COMPLETE_ACKNOWLEDGE,
        NEXT_PAGE_WAIT,
        AUTO_NEGOTIATION_COMPLETE,
    };

    void enter(State state);

    State state_;
    char mr_adv_ability_[PAGE_SIZE] = { 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0 };
    bool mr_autoneg_complete_;
    bool mr_next_page_loaded_;
    bool mr_lp_autoneg_able_;
    bool mr_lp_np_able_;
    bool mr_np_able_;
    char mr_np_tx_[PAGE_SIZE];
    bool mr_page_rx_;
    bool base_page_;
    bool desire_np_;
    bool np_rx_;
    bool toggle_rx_;
    bool toggle_tx_;
    Receiver receiver_;
    Transmitter transmitter_;
};

};

#endif
