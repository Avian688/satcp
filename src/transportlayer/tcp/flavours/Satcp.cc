// Satcp.cc
//
// Minimal "Satcp" flavour implementation that overrides TcpCubic.
// All overrides currently forward to TcpCubic;
//

#include "Satcp.h"

namespace inet {
namespace tcp {

Register_Class(Satcp);

Satcp::Satcp()
    : TcpCubic()
{
}

void Satcp::initialize()
{
    TcpCubic::initialize();
    handover_status = false;
}

void Satcp::established(bool active)
{
    TcpCubic::established(active);
}

void Satcp::updateCubicCwnd(uint32_t acked) {

    uint64_t offs, t;
    uint32_t delta, bic_target, max_cnt;

    uint32_t cwnd = state->snd_cwnd / state->snd_mss;

    //In the kernel code this is the number of jiffies.
    //The number of jiffies is incremented HZ times per second
    //tcp_time_stamp is in ms unit. The jiffy variable would match if HZ = 1000
    uint32_t tcp_time_stamp = simTime().inUnit(SIMTIME_MS);

    state->ack_cnt++; /* count the number of ACKs */

    if (state->last_cwnd == cwnd
            && (int32_t) (tcp_time_stamp - state->last_time) <= HZ / 32)
        return;


    if (!(state->epoch_start && tcp_time_stamp == state->last_time)) {

        state->last_cwnd = cwnd;
        state->last_time = tcp_time_stamp;

        if (state->epoch_start == 0) {
            state->epoch_start = tcp_time_stamp; /* record the beginning of an epoch */
            state->ack_cnt = 1; /* start counting */
            state->tcp_cwnd = cwnd; /* syn with cubic */

            if (state->last_max_cwnd <= cwnd) {
                state->bic_K = 0;
                state->bic_origin_point = cwnd;
            } else {
                /* Compute new K based on
                 * (wmax-cwnd) * (srtt>>3 / HZ) / c * 2^(3*bictcp_HZ)
                 */
//                state->bic_K = calculateCubicRoot(
//                        state->cube_factor * (state->last_max_cwnd - cwnd));
                // shrink the recovery time
                if (handover_status) {
                    // handover is happening, so drastically reduce the recovery time K to last max point
                    state->bic_K = calculateCubicRoot(state->cube_factor * (state->last_max_cwnd - cwnd)) - (calculateCubicRoot(state->cube_factor * (state->last_max_cwnd - cwnd)) * 8 / 9);
                }
                else{
                    state->bic_K = calculateCubicRoot(state->cube_factor * (state->last_max_cwnd - cwnd));
                }
                state->bic_origin_point = state->last_max_cwnd;
            }
        }

        /* cubic function - calc*/
        /* calculate c * time^3 / rtt,
         *  while considering overflow in calculation of time^3
         * (so time^3 is done by using 64 bit)
         * and without the support of division of 64bit numbers
         * (so all divisions are done by using 32 bit)
         *  also NOTE the unit of those veriables
         *    time  = (t - K) / 2^bictcp_HZ
         *    c = bic_scale >> 10
         * rtt  = (srtt >> 3) / HZ
         * !!! The following code does not have overflow problems,
         * if the cwnd < 1 million packets !!!
         */

        t = (int32_t)(tcp_time_stamp - state->epoch_start);
        t += state->delay_min/1000;
        /* change the unit from HZ to bictcp_HZ */
        t <<= BICTCP_HZ;
        t /= HZ;

        if (t < state->bic_K) /* t - K */
            offs = state->bic_K - t;
        else
            offs = t - state->bic_K;

        /* c/rtt * (t-K)^3 */
        delta = (state->cube_rtt_scale * offs * offs * offs)
                >> (10 + 3 * BICTCP_HZ);
        if (t < state->bic_K) /* below origin*/
            bic_target = state->bic_origin_point - delta;
        else
            /* above origin*/
            bic_target = state->bic_origin_point + delta;

        /* cubic function - calc bictcp_cnt*/
        if (bic_target > cwnd) {
            state->cnt = cwnd / (bic_target - cwnd);
        } else {
            state->cnt = 100 * cwnd; /* very small increment*/
        }


        if (state->last_max_cwnd == 0 && state->cnt > 20)
            state->cnt = 20;
    }

    if (state->tcp_friendliness) {
            uint32_t scale = state->beta_scale;

            delta = (cwnd * scale) >> 3;
            while (state->ack_cnt > delta) {       /* update tcp cwnd */
                state->ack_cnt -= delta;
                state->tcp_cwnd++;
            }

            if (state->tcp_cwnd > cwnd) {  /* if bic is slower than tcp */
                delta = state->tcp_cwnd - cwnd;
                max_cnt = cwnd / delta;
                if (state->cnt > max_cnt)
                    state->cnt = max_cnt;
            }
    }
    state->cnt = std::max(state->cnt, 2U);

    conn->emit(cntSignal, state->cnt);
}

void Satcp::processRexmitTimer(TcpEventCode &event) {
    TcpPacedFamily::processRexmitTimer(event);

    std::cerr << "RTO at " << simTime() << std::endl;
    std::cerr << "cwnd=: " << state->snd_cwnd / state->snd_mss << ", in-flight="
            << (state->snd_max - state->snd_una) / state->snd_mss << std::endl;
    if(!handover_status){
        reset();
        recalculateSlowStartThreshold();
        state->snd_cwnd = state->snd_mss;
    }
//    if(state->snd_cwnd > 0){
//        if(state->snd_cwnd < state->ssthresh/2){
//            dynamic_cast<PacedTcpConnection*>(conn)->changeIntersendingTime(state->srtt.dbl()/(((double) state->snd_cwnd/(double)state->snd_mss)* 2));
//        }
//        else{
//            dynamic_cast<PacedTcpConnection*>(conn)->changeIntersendingTime(state->srtt.dbl()/(((double) state->snd_cwnd/(double)state->snd_mss)* 2));
//        }
//    }

    conn->emit(cwndSignal, state->snd_cwnd);

    state->afterRto = true;
    dynamic_cast<TcpPacedConnection*>(conn)->cancelPaceTimer();
    sendData(false);

    conn->emit(ssthreshSignal, state->ssthresh);
    conn->emit(cwndSegSignal, state->snd_cwnd / state->snd_mss);
}

} // namespace tcp
} // namespace inet
