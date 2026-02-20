// Satcp.cc
//
// "Satcp" flavour implementation that overrides TcpCubic.
// Parses a <scenario> XML and toggles `handover_status`:
//  - set TRUE when a <crash> happens
//  - set FALSE SATCP_DURATION seconds after the MOST RECENT crash
//
// This mirrors report_relay.c behavior: every "start" resets the expiration timer.
//

#include "../Satcp.h"
#include "../../satcp/flavours/SatcpFlavour.h"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace inet {
namespace tcp {

Register_Class(SatcpFlavour);

static constexpr double SATCP_DURATION_S = 2.3;

SatcpFlavour::SatcpFlavour()
    : TcpCubic()
{
}

static bool handoverEventLess(const SatcpFlavour::HandoverEvent& a,
                              const SatcpFlavour::HandoverEvent& b)
{
    if (a.t != b.t)
        return a.t < b.t;

    // Deterministic ordering when timestamps match
    return a.type < b.type;
}

void SatcpFlavour::parseHandoverEventsFromScenario(cXMLElement *scenario)
{
    handoverEvents.clear();
    nextHandoverEventIndex = 0;

    if (!scenario)
        return;

    for (cXMLElement *at = scenario->getFirstChild(); at; at = at->getNextSibling()) {
        if (std::strcmp(at->getTagName(), "at") != 0)
            continue;

        const char *tAttr = at->getAttribute("t");
        if (!tAttr)
            throw cRuntimeError("SatcpFlavour: <at> missing attribute 't' at %s", at->getSourceLocation());

        simtime_t t = SimTime::parse(tAttr);

        for (cXMLElement *cmd = at->getFirstChild(); cmd; cmd = cmd->getNextSibling()) {
            const char *tag = cmd->getTagName();

            if (std::strcmp(tag, "crash") == 0) {
                handoverEvents.push_back(HandoverEvent{HandoverEvent::Crash, t});
            }
            else if (std::strcmp(tag, "connect") == 0) {
                handoverEvents.push_back(HandoverEvent{HandoverEvent::Connect, t});
            }
        }
    }

    std::sort(handoverEvents.begin(), handoverEvents.end(), handoverEventLess);

    updateHandoverStatusFromScenario();
}

void SatcpFlavour::updateHandoverStatusFromScenario()
{
    const simtime_t now = simTime();

    // Timer-based end (like report_relay.c): end after SATCP_DURATION since last crash/start
    if (handover_status && now >= handover_end_time) {
        std::cout << "\n ENDING HANDOVER (TIMER) AT SIMTIME: " << now << std::endl;
        handover_status = false;
    }

    while (nextHandoverEventIndex < handoverEvents.size() &&
           handoverEvents[nextHandoverEventIndex].t <= now)
    {
        const auto& ev = handoverEvents[nextHandoverEventIndex];

        if (ev.type == HandoverEvent::Crash) {
            // Like report_relay.c: every start/crash refreshes the expiration time
            handover_status = true;
            handover_end_time = ev.t + SimTime(SATCP_DURATION_S, SIMTIME_S);
        }
        else {
            // Connect events do NOT end handover in this model (timer-based end instead)
        }

        nextHandoverEventIndex++;
    }

    // If we advanced time exactly past expiry due to processing events, enforce end
    if (handover_status && now >= handover_end_time) {
        handover_status = false;
    }
}

void SatcpFlavour::initialize()
{
    TcpCubic::initialize();

    cXMLElement *scenario = conn->getTcpMain()->par("scenario").xmlValue();
    if (!scenario)
        throw cRuntimeError("SatcpFlavour: tcpMain parameter 'scenario' is null or not an XML parameter");

    handover_status = false;
    handover_end_time = SIMTIME_ZERO;   // add this member in the class if not present
    parseHandoverEventsFromScenario(scenario);
}

void SatcpFlavour::established(bool active)
{
    TcpCubic::established(active);
    updateHandoverStatusFromScenario();
}

void SatcpFlavour::updateCubicCwnd(uint32_t acked)
{
    updateHandoverStatusFromScenario();

    uint64_t offs, t;
    uint32_t delta, bic_target, max_cnt;

    uint32_t cwnd = state->snd_cwnd / state->snd_mss;

    uint32_t tcp_time_stamp = simTime().inUnit(SIMTIME_MS);

    state->ack_cnt++;

    if (state->last_cwnd == cwnd
            && (int32_t)(tcp_time_stamp - state->last_time) <= HZ / 32)
        return;

    if (!(state->epoch_start && tcp_time_stamp == state->last_time)) {

        state->last_cwnd = cwnd;
        state->last_time = tcp_time_stamp;

        if (state->epoch_start == 0) {
            state->epoch_start = tcp_time_stamp;
            state->ack_cnt = 1;
            state->tcp_cwnd = cwnd;

            if (state->last_max_cwnd <= cwnd) {
                state->bic_K = 0;
                state->bic_origin_point = cwnd;
            }
            else {
                if (handover_status) {
                    uint32_t K = calculateCubicRoot(state->cube_factor * (state->last_max_cwnd - cwnd));
                    state->bic_K = K - (K * 8 / 9);
                }
                else {
                    state->bic_K = calculateCubicRoot(state->cube_factor * (state->last_max_cwnd - cwnd));
                }
                state->bic_origin_point = state->last_max_cwnd;
            }
        }

        t = (int32_t)(tcp_time_stamp - state->epoch_start);
        t += state->delay_min / 1000;
        t <<= BICTCP_HZ;
        t /= HZ;

        if (t < state->bic_K)
            offs = state->bic_K - t;
        else
            offs = t - state->bic_K;

        delta = (state->cube_rtt_scale * offs * offs * offs)
                >> (10 + 3 * BICTCP_HZ);

        if (t < state->bic_K)
            bic_target = state->bic_origin_point - delta;
        else
            bic_target = state->bic_origin_point + delta;

        if (bic_target > cwnd) {
            state->cnt = cwnd / (bic_target - cwnd);
        }
        else {
            state->cnt = 100 * cwnd;
        }

        if (state->last_max_cwnd == 0 && state->cnt > 20)
            state->cnt = 20;
    }

    if (state->tcp_friendliness) {
        uint32_t scale = state->beta_scale;

        delta = (cwnd * scale) >> 3;
        while (state->ack_cnt > delta) {
            state->ack_cnt -= delta;
            state->tcp_cwnd++;
        }

        if (state->tcp_cwnd > cwnd) {
            delta = state->tcp_cwnd - cwnd;
            max_cnt = cwnd / delta;
            if (state->cnt > max_cnt)
                state->cnt = max_cnt;
        }
    }

    state->cnt = std::max(state->cnt, 2U);

    conn->emit(cntSignal, state->cnt);
}

void SatcpFlavour::processRexmitTimer(TcpEventCode &event)
{
    updateHandoverStatusFromScenario();

    TcpPacedFamily::processRexmitTimer(event);

    std::cerr << "RTO at " << simTime() << std::endl;
    std::cerr << "cwnd=: " << state->snd_cwnd / state->snd_mss << ", in-flight="
              << (state->snd_max - state->snd_una) / state->snd_mss << std::endl;

    if (!handover_status) {
        reset();
        recalculateSlowStartThreshold();
        state->snd_cwnd = state->snd_mss;
    }

    conn->emit(cwndSignal, state->snd_cwnd);

    state->afterRto = true;
    dynamic_cast<TcpPacedConnection *>(conn)->cancelPaceTimer();
    sendData(false);

    conn->emit(ssthreshSignal, state->ssthresh);
    conn->emit(cwndSegSignal, state->snd_cwnd / state->snd_mss);
}

} // namespace tcp
} // namespace inet
