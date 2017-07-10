#include "AutoNeg.h"

namespace AutoNeg {

Receiver::Receiver()
{
    // Analog Comparator
    ADCSRB = ADCSRB & ~bit(ACME);         // disable Analog Comparator Multiplexer Enable
    ACSR =    bit(ACI)                                    // clear Analog Comparator Interrupt Flag
                    | bit(ACIE)                             // set Analog Comparator Interrupt Enable
                    | bit(ACIS1) | bit(ACIS0) // select rising edge: ACIS1/ACIS0 Analog Comparator Interrupt Mode Select
                    ;

    // Analog Comparator pins
    DDRD &= ~(bit(6) | bit(7));             // set D7/8 (AIN0/1) to input
    PORTD &= ~(bit(6) | bit(7));            // disable pull-ups on D7/8 (AIN0/1)
    DIDR1 = bit(AIN0D) | bit(AIN1D);    // disable digital input buffers on AIN0/1

    // Status LEDs
    DDRD |= bit(3) | bit(2);
}

void
Receiver::reset()
{
    ability_match_ = false;
    ability_match_counter_ = 0;
    acknowledge_match_ = false;
    acknowledge_match_counter_ = 0;
    consistency_match_ = false;
}

void
Receiver::received_pulse()
{
    ++linkpulse_;
    linkpulse_time_ = micros();
}

void
Receiver::enter(State state)
{
    state_ = state;

    register unsigned long linkpulse_time = linkpulse_time_;

    switch (state_) {
        case State::IDLE:
            PORTD = 0;
            flp_cnt_ = 0;
            flp_receive_idle_ = true;
            reset();
            break;

        case State::LINK_PULSE_DETECT:
            flp_test_min_timer_.start(linkpulse_time);
            flp_test_max_timer_.start(linkpulse_time);
            flp_receive_idle_ = true;
            break;

        case State::LINK_PULSE_COUNT:
            ++flp_cnt_;
            flp_receive_idle_ = true;
            if (flp_cnt_ < FLP_CNT_DONE) {
                enter(State::LINK_PULSE_DETECT);
            } else {
                enter(State::FLP_PASS);
            }
            break;

        case State::FLP_PASS:
            asm("sbi 0x0b, 2"); // PORTD[2] high
            nlp_test_max_timer_.start(linkpulse_time);
            flp_test_max_timer_.start(linkpulse_time);
            rx_bit_cnt_ = 0;
            break;

        case State::FLP_CHECK:
            if (rx_bit_cnt_ >= RX_BIT_CNT_CHECK) {
                nlp_test_max_timer_.start(linkpulse_time);

                if (rx_bit_cnt_ > PAGE_SIZE)
                    rx_bit_cnt_ = PAGE_SIZE;

                bool ability_bits_match = true;
                for (int i = 1; i <= rx_bit_cnt_; ++i) {
                    if (i == ACK)
                        continue;
                    if (ability_match_word_[i - 1] != rx_link_code_word_[i - 1]) {
                        ability_bits_match = false;
                        break;
                    }
                }

                if (ability_bits_match) {
                    ++ability_match_counter_;
                }

                bool ack_bits_match = (ability_match_word_[ACK] == rx_link_code_word_[ACK]);

                if (ack_bits_match && ability_bits_match && ability_match_word_[ACK] == 1) {
                    asm("sbi 0x0b, 3"); // PORTD[3] high
                    ++acknowledge_match_counter_;
                }

                ability_match_ = (ability_match_counter_ >= 3);
                acknowledge_match_ = (acknowledge_match_counter_ >= 3);
                consistency_match_ = ability_match_ && acknowledge_match_;

                memcpy(ability_match_word_, rx_link_code_word_, sizeof(ability_match_word_));
            } else {
                memset(ability_match_word_, -1, sizeof(ability_match_word_));
            }

            memset(rx_link_code_word_, -1, sizeof(rx_link_code_word_));

            break;

        case State::FLP_CAPTURE:
            rx_bit_cnt_ = 0;
            nlp_test_min_timer_.start(linkpulse_time);
            enter(State::FLP_CLOCK);
            break;

        case State::FLP_CLOCK:
            data_detect_min_timer_.start(linkpulse_time);
            data_detect_max_timer_.start(linkpulse_time);
            ++rx_bit_cnt_;
            break;

        case State::FLP_DATA_0:
            rx_link_code_word_[rx_bit_cnt_ - 1] = 0;
            enter(State::FLP_CLOCK);
            break;

        case State::FLP_DATA_1:
            rx_link_code_word_[rx_bit_cnt_ - 1] = 1;
            data_detect_min_timer_.start(linkpulse_time);
            break;
    }
}

void
Receiver::tick()
{
    noInterrupts();
    register char linkpulse = linkpulse_ > 0;
    linkpulse_ = 0;
    interrupts();

    switch (state_) {
        case State::IDLE:
            if (linkpulse) {
                enter(State::LINK_PULSE_DETECT);
            }
            break;

        case State::LINK_PULSE_DETECT:
            if (linkpulse && flp_test_min_timer_.done() && !flp_test_max_timer_.done()) {
                enter(State::LINK_PULSE_COUNT);
            } else if (linkpulse && !flp_test_min_timer_.done()) {
                enter(State::IDLE);
            } else if (flp_test_max_timer_.done()) {
                enter(State::IDLE);
            }
            break;

        case State::LINK_PULSE_COUNT:
            // no-op
            break;

        case State::FLP_PASS:
            if (linkpulse) {
                enter(State::FLP_PASS);
            } else if (flp_test_max_timer_.done()) {
                enter(State::FLP_CHECK);
            }
            break;

        case State::FLP_CHECK:
            if (linkpulse) {
                enter(State::FLP_CAPTURE);
            } else if (nlp_test_max_timer_.done()) {
                enter(State::IDLE);
            }
            break;

        case State::FLP_CAPTURE:
            // no-op
            break;

        case State::FLP_CLOCK:
            if (linkpulse && data_detect_max_timer_.done()) {
                enter(State::FLP_DATA_0);
         } else if (linkpulse && data_detect_min_timer_.done() && !data_detect_max_timer_.done()) {
                enter(State::FLP_DATA_1);
                break;
            } else if (nlp_test_min_timer_.done()) {
                enter(State::FLP_CHECK);
            }
            break;

        case State::FLP_DATA_0:
            // no-op
            break;

        case State::FLP_DATA_1:
            if (linkpulse && !nlp_test_min_timer_.done() && data_detect_min_timer_.done()) {
                enter(State::FLP_CLOCK);
            } else if (nlp_test_min_timer_.done()) {
                enter(State::FLP_CHECK);
            }
            break;
    }
}

Transmitter::Transmitter()
{
    // Transmit Data (TD) pin
    PORTB &= ~bit(0);         // drive low
    DDRB |= bit(0);             // set as output
}

void
Transmitter::enter(State state)
{
    state_ = state;

    switch (state_) {
        case State::IDLE:
            transmit_link_burst_timer_.start();
            remaining_ack_cnt_ = REMAINING_ACK_CNT_DONE;
            break;

        case State::TRANSMIT_REMAINING_ACKNOWLEDGE:
            remaining_ack_cnt_ = 0;
            enter(State::TRANSMIT_ABILITY);
            break;

        case State::TRANSMIT_ABILITY:
            tx_bit_cnt_ = 1;
            enter(State::TRANSMIT_CLOCK_BIT);
            break;

        case State::TRANSMIT_CLOCK_BIT:
            interval_timer_.start();
            send_pulse();
            break;

        case State::TRANSMIT_DATA_BIT:
            interval_timer_.start();
            if (tx_link_code_word_[tx_bit_cnt_ - 1] == 1) {
                send_pulse();
            } else if (transmit_ack_ && tx_bit_cnt_ == ACK) {
                send_pulse();
            }
            ++tx_bit_cnt_;
            break;

        case State::TRANSMIT_COUNT_ACK:
            transmit_link_burst_timer_.start();
            ++remaining_ack_cnt_;
            if (remaining_ack_cnt_ >= REMAINING_ACK_CNT_DONE) {
                ack_finished_ = true;
            }
            break;
    }
}

void
Transmitter::tick()
{
    switch (state_) {
        case State::IDLE:
            if (complete_ack_ && transmit_link_burst_timer_.done()) {
                enter(State::TRANSMIT_REMAINING_ACKNOWLEDGE);
            } else if (!complete_ack_ && transmit_ability_ && transmit_link_burst_timer_.done()) {
                enter(State::TRANSMIT_ABILITY);
            }
            break;

        case State::TRANSMIT_REMAINING_ACKNOWLEDGE:
            // no-op
            break;

        case State::TRANSMIT_ABILITY:
            // no-op
            break;

        case State::TRANSMIT_CLOCK_BIT:
            if (tx_bit_cnt_ >= TX_BIT_CNT_DONE && remaining_ack_cnt_ < REMAINING_ACK_CNT_DONE) {
                enter(State::TRANSMIT_COUNT_ACK);
            } else if (tx_bit_cnt_ >= TX_BIT_CNT_DONE && remaining_ack_cnt_ >= REMAINING_ACK_CNT_DONE) {
                enter(State::IDLE);
            } else if (interval_timer_.done()) {
                enter(State::TRANSMIT_DATA_BIT);
            }
            break;

        case State::TRANSMIT_DATA_BIT:
            if (interval_timer_.done()) {
                enter(State::TRANSMIT_CLOCK_BIT);
            }
            break;

        case State::TRANSMIT_COUNT_ACK:
            if (transmit_link_burst_timer_.done()) {
                enter(State::TRANSMIT_ABILITY);
            } else if (remaining_ack_cnt_ >= REMAINING_ACK_CNT_DONE || ack_finished_ || !complete_ack_) {
                enter(State::IDLE);
            }
            break;
    }
}

void
Transmitter::send_pulse() const
{
    asm("sbi 0x05, 0"); // PORTB[0] high
    asm("cbi 0x05, 0"); // PORTB[0] low
}

Arbitrator::Arbitrator()
{
    // Status LEDs
    DDRD |= bit(4);
}

void
Arbitrator::start()
{
    receiver_.start();
    transmitter_.start();
    enter(State::AUTO_NEGOTIATION_ENABLE);
}

void
Arbitrator::enter(State state)
{
    state_ = state;

    switch (state_) {
        case State::AUTO_NEGOTIATION_ENABLE:
            mr_page_rx_ = false;
            mr_autoneg_complete_ = false;
            enter(State::TRANSMIT_DISABLE);
            break;

        case State::TRANSMIT_DISABLE:
            mr_page_rx_ = false;
            mr_autoneg_complete_ = false;
            mr_next_page_loaded_ = false;
            enter(State::ABILITY_DETECT);
            break;

        case State::ABILITY_DETECT:
            mr_page_rx_ = false;
            mr_lp_autoneg_able_ = false;
            mr_lp_np_able_ = false;
            base_page_ = true;
            desire_np_ = false;
            toggle_tx_ = (mr_adv_ability_[12] == 1);
            transmitter_.ack_finished_ = false;
            transmitter_.transmit_ability_ = true;
            memcpy(transmitter_.tx_link_code_word_, mr_adv_ability_, sizeof(mr_adv_ability_));
            receiver_.reset();
            break;

        case State::ACKNOWLEDGE_DETECT:
            mr_lp_autoneg_able_ = true;
            transmitter_.transmit_ability_ = true;
            transmitter_.transmit_ack_ = true;
            break;

        case State::COMPLETE_ACKNOWLEDGE:
            transmitter_.complete_ack_ = true;
            transmitter_.transmit_ability_ = true;
            transmitter_.transmit_ack_ = true;

            if (base_page_ && receiver_.rx_link_code_word_[NP]) {
                mr_lp_np_able_ = true;
            }

            if (base_page_ && transmitter_.tx_link_code_word_[NP]) {
                desire_np_ = true;
            }

            toggle_rx_ = receiver_.rx_link_code_word_[12];
            toggle_tx_ = !toggle_tx_;
            mr_page_rx_ = true;
            np_rx_ = receiver_.rx_link_code_word_[NP];
            break;

        case State::NEXT_PAGE_WAIT:
            mr_next_page_loaded_ = false;
            mr_page_rx_ = false;
            base_page_ = false;
            transmitter_.ack_finished_ = false;
            transmitter_.transmit_ability_ = true;
            for (int i = 13; i <= PAGE_SIZE; ++i) {
                transmitter_.tx_link_code_word_[i - 1] = mr_np_tx_[i - 1];
            }
            transmitter_.tx_link_code_word_[12] = toggle_tx_;
            for (int i = 1; i <= 11; ++i) {
                transmitter_.tx_link_code_word_[i - 1] = mr_np_tx_[i - 1];
            }
            break;

        case State::AUTO_NEGOTIATION_COMPLETE:
            asm("sbi 0x0b, 4"); // PORTD[4] high
            mr_autoneg_complete_ = true;
            break;
    }
}

void
Arbitrator::tick()
{
    receiver_.tick();
    transmitter_.tick();

    switch (state_) {
        case State::AUTO_NEGOTIATION_ENABLE:
            // no-op
            break;

        case State::TRANSMIT_DISABLE:
            // no-op
            break;

        case State::ABILITY_DETECT:
            if (receiver_.ability_match_) {
                enter(State::ACKNOWLEDGE_DETECT);
            }
            break;

        case State::ACKNOWLEDGE_DETECT:
            if (receiver_.acknowledge_match_ && receiver_.consistency_match_) {
                enter(State::COMPLETE_ACKNOWLEDGE);
            } else if ((receiver_.acknowledge_match_ && !receiver_.consistency_match_) || receiver_.flp_receive_idle_) {
                enter(State::TRANSMIT_DISABLE);
            }
            break;

        case State::COMPLETE_ACKNOWLEDGE:
            if ((transmitter_.ack_finished_ && (!mr_np_able_ || !desire_np_ || !mr_lp_np_able_)) ||
                    (transmitter_.ack_finished_ && mr_np_able_ && mr_lp_np_able_ && transmitter_.tx_link_code_word_[NP] == 0 && np_rx_ == 0)) {
                enter(State::AUTO_NEGOTIATION_COMPLETE);
            } else if (transmitter_.ack_finished_ && mr_np_able_ && desire_np_ && mr_lp_np_able_ && mr_next_page_loaded_ &&
                                 ((transmitter_.tx_link_code_word_[NP] == 1) || (np_rx_ == 1))) {
                enter(State::ACKNOWLEDGE_DETECT);
            }
            break;

        case State::NEXT_PAGE_WAIT:
            if (receiver_.ability_match_ && ((toggle_rx_ ^ receiver_.ability_match_word_[12]) == 1)) {
                enter(State::ACKNOWLEDGE_DETECT);
            } else if (receiver_.flp_receive_idle_) {
                enter(State::TRANSMIT_DISABLE);
            }
            break;

        case State::AUTO_NEGOTIATION_COMPLETE:
            // no-op
            break;
    }
}

};
