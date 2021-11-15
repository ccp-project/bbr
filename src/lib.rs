//! Linux source: `net/ipv4/tcp_bbr.c`
//!
//! model of the network path:
//! ```no-run
//!    bottleneck_bandwidth = windowed_max(delivered / elapsed, 10 round trips)
//!    min_rtt = windowed_min(rtt, 10 seconds)
//! ```
//! ```no-run
//! pacing_rate = pacing_gain * bottleneck_bandwidth
//! cwnd = max(cwnd_gain * bottleneck_bandwidth * min_rtt, 4)
//! ```
//!
//! A BBR flow starts in STARTUP, and ramps up its sending rate quickly.
//! When it estimates the pipe is full, it enters DRAIN to drain the queue.
//! In steady state a BBR flow only uses `PROBE_BW` and `PROBE_RTT`.
//! A long-lived BBR flow spends the vast majority of its time remaining
//! (repeatedly) in `PROBE_BW`, fully probing and utilizing the pipe's bandwidth
//! in a fair manner, with a small, bounded queue. *If* a flow has been
//! continuously sending for the entire `min_rtt` window, and hasn't seen an RTT
//! sample that matches or decreases its `min_rtt` estimate for 10 seconds, then
//! it briefly enters `PROBE_RTT` to cut inflight to a minimum value to re-probe
//! the path's two-way propagation delay (`min_rtt`). When exiting `PROBE_RTT`, if
//! we estimated that we reached the full bw of the pipe then we enter `PROBE_BW`;
//! otherwise we enter STARTUP to try to fill the pipe.
//!
//! The goal of `PROBE_RTT` mode is to have BBR flows cooperatively and
//! periodically drain the bottleneck queue, to converge to measure the true
//! `min_rtt` (unloaded propagation delay). This allows the flows to keep queues
//! small (reducing queuing delay and packet loss) and achieve fairness among
//! BBR flows.
//!
//! The `min_rtt` filter window is 10 seconds. When the `min_rtt` estimate expires,
//! we enter `PROBE_RTT` mode and cap the cwnd at `bbr_cwnd_min_target=4` packets.
//! After at least `bbr_probe_rtt_mode_ms=200ms` and at least one packet-timed
//! round trip elapsed with that flight size <= 4, we leave `PROBE_RTT` mode and
//! re-enter the previous mode. BBR uses 200ms to approximately bound the
//! performance penalty of `PROBE_RTT`'s cwnd capping to roughly 2% (200ms/10s).
//!
//! Portus note:
//! This implementation does `PROBE_BW` and `PROBE_RTT`, but leaves as future work
//! an implementation of the finer points of other BBR implementations
//! (e.g. policing detection).

use portus::ipc::Ipc;
use portus::lang::Scope;
use portus::{CongAlg, Datapath, DatapathInfo, DatapathTrait, Report};
use std::collections::HashMap;
use std::time::{Duration, Instant};
use tracing::{info, warn};

pub struct Bbr<T: Ipc> {
    control_channel: Datapath<T>,
    sc: Scope,
    probe_rtt_interval: Duration,
    bottle_rate: f64,
    bottle_rate_timeout: Instant,
    min_rtt_us: u32,
    min_rtt_timeout: Instant,
    curr_mode: BbrMode,
    mss: u32,
    init: bool,
    start: Instant,
}

enum BbrMode {
    ProbeBw,
    ProbeRtt,
}

pub const PROBE_RTT_INTERVAL_SECONDS: i64 = 10;

#[derive(Clone)]
pub struct BbrConfig {
    pub probe_rtt_interval: Duration,
    // TODO make more things configurable
}

impl<T: Ipc> Bbr<T> {
    fn install_update(&self, update: &[(&str, u32)]) {
        if let Err(err) = self.control_channel.update_field(&self.sc, update) {
            warn!(?err, "Cwnd and rate update error");
        }
    }

    // replaces the variables in the probe bw program if the bottle rate or min_rtt changes
    fn replace_probe_bw_rate(&self) {
        let three_fourths_rate = (self.bottle_rate * 0.75) as u32;
        let rate = self.bottle_rate as u32;
        let five_fourths_rate = (self.bottle_rate * 1.25) as u32;
        let cwnd_cap = (self.bottle_rate * 2.0 * f64::from(self.min_rtt_us) / 1e6) as u32;
        self.install_update(&[
            ("bottleRate", rate),
            ("threeFourthsRate", three_fourths_rate),
            ("fiveFourthsRate", five_fourths_rate),
            ("cwndCap", cwnd_cap),
        ]);
        info!(
            cwnd = cwnd_cap,
            down_rate = three_fourths_rate as f64 / 125_000.0,
            bottle_rate = self.bottle_rate / 125_000.0,
            up_rate = five_fourths_rate as f64 / 125_000.0,
            "PROBE_BW: updating rate"
        );
    }

    fn install_probe_bw(&mut self) -> Scope {
        // first, install the rate and cwnd for state 0 for state 0
        let min_rtt = self.min_rtt_us as u32;
        let three_fourths_rate = (self.bottle_rate * 0.75) as u32;
        let rate = self.bottle_rate as u32;
        let five_fourths_rate = (self.bottle_rate * 1.25) as u32;
        let cwnd_cap = (self.bottle_rate * 2.0 * f64::from(self.min_rtt_us) / 1e6) as u32;

        info!(
            cwnd = cwnd_cap,
            down_rate = three_fourths_rate as f64 / 125_000.0,
            bottle_rate_Mbps = self.bottle_rate / 125_000.0,
            up_rate = five_fourths_rate as f64 / 125_000.0,
            min_rtt_us = min_rtt,
            "switching to PROBE_BW"
        );

        self.install_update(&[("Cwnd", cwnd_cap), ("Rate", five_fourths_rate)]);
        self.control_channel
            .set_program(
                "probe_bw",
                Some(&[
                    ("cwndCap", cwnd_cap),
                    ("bottleRate", rate),
                    ("threeFourthsRate", three_fourths_rate),
                    ("fiveFourthsRate", five_fourths_rate),
                ]),
            )
            .unwrap()
    }

    fn get_probe_bw_fields(&mut self, m: &Report) -> Option<(u32, u32, f64, u32)> {
        let rtt = m
            .get_field(&String::from("Report.minrtt"), &self.sc)
            .expect("expected minrtt field in returned measurement") as u32;
        let loss = m
            .get_field(&String::from("Report.loss"), &self.sc)
            .expect("expected loss field in returned measurement") as u32;
        let rate = m
            .get_field(&String::from("Report.rate"), &self.sc)
            .expect("expected rate field in returned measurement") as f64;
        let state = m
            .get_field(&String::from("Report.pulseState"), &self.sc)
            .expect("expected state field in returned measurement") as u32;
        Some((loss, rtt, rate, state))
    }

    fn get_probe_minrtt(&mut self, m: &Report) -> u32 {
        m.get_field("Report.minrtt", &self.sc)
            .expect("expected minrtt field in returned measurement") as u32
    }
}

impl<T: Ipc> CongAlg<T> for BbrConfig {
    type Flow = Bbr<T>;

    fn name() -> &'static str {
        "bbr"
    }

    fn datapath_programs(&self) -> HashMap<&'static str, String> {
        vec![
            (
                "init_program",
                String::from(
                    "
                (def
                    (Report 
                        (volatile loss 0)
                        (minrtt +infinity)
                        (volatile rate 0) 
                        (pulseState 0)
                    )
                )
                (when true
                    (:= Report.loss (+ Report.loss Ack.lost_pkts_sample))
                    (:= Report.minrtt (min Report.minrtt Flow.rtt_sample_us))
                    (:= Report.rate (max Report.rate (min Flow.rate_outgoing Flow.rate_incoming)))
                    (:= Report.pulseState 5)
                    (fallthrough)
                )
                (when (> Micros Report.minrtt)
                    (report)
                )
            ",
                ),
            ),
            (
                "probe_rtt",
                String::from(
                    "
		(def 
		    (Report (volatile minrtt +infinity))
		    (volatile target_inflight_reached 0)
		)
		(when true
		    (:= Report.minrtt (min Report.minrtt Flow.rtt_sample_us))
		    (fallthrough)
		)
		(when (&& (== target_inflight_reached 0)
			  (|| (< Flow.packets_in_flight 4) (== Flow.packets_in_flight 4)))
		    (:= target_inflight_reached 1)
		    (:= Micros 0)
		)
		(when (&& (== target_inflight_reached 1) 
		          (&& (> Micros Flow.rtt_sample_us) (> Micros 200000))
                      )
                    (:= Micros 0)
		    (report)
		)
            ",
                ),
            ),
            (
                "probe_bw",
                String::from(
                    "
                (def
                    (Report 
                        (volatile loss 0)
                        (volatile minrtt +infinity)
                        (volatile rate 0) 
                        (pulseState 0)
                    )
                    (pulseState 0)
                    (cwndCap 0)
                    (bottleRate 0)
                    (threeFourthsRate 0)
                    (fiveFourthsRate 0)
                )
                (when true
                    (:= Report.loss (+ Report.loss Ack.lost_pkts_sample))
                    (:= Report.minrtt (min Report.minrtt Flow.rtt_sample_us))
                    (:= Report.pulseState pulseState)
                    (:= Report.rate (max Report.rate (min Flow.rate_outgoing Flow.rate_incoming)))
                    (fallthrough)
                )
                (when (&& (> Micros Report.minrtt) (== pulseState 0))
                    (:= Rate threeFourthsRate)
                    (:= pulseState 1)
                    (report)
                )
                (when (&& (> Micros (* Report.minrtt 2)) (== pulseState 1))
                    (:= Rate bottleRate)
                    (:= pulseState 2)
                    (report)
                )
                (when (&& (> Micros (* Report.minrtt 8)) (== pulseState 2))
                    (:= pulseState 0)
                    (:= Cwnd cwndCap)
                    (:= Rate fiveFourthsRate)
                    (:= Micros 0)
                    (report)
                )
	    ",
                ),
            ),
        ]
        .into_iter()
        .collect()
    }

    fn new_flow(&self, control: Datapath<T>, info: DatapathInfo) -> Self::Flow {
        let now = std::time::Instant::now();
        let mut s = Bbr {
            control_channel: control,
            sc: Scope::new(),
            probe_rtt_interval: self.probe_rtt_interval,
            bottle_rate: 125_000.0,
            bottle_rate_timeout: now + self.probe_rtt_interval,
            min_rtt_us: 1_000_000,
            min_rtt_timeout: now + self.probe_rtt_interval,
            curr_mode: BbrMode::ProbeBw,
            mss: info.mss,
            init: true,
            start: now,
        };

        s.sc = s
            .control_channel
            .set_program("init_program", Some(&[("Cwnd", info.init_cwnd)]))
            .unwrap();
        s
    }
}

impl<T: Ipc> portus::Flow for Bbr<T> {
    fn on_report(&mut self, _sock_id: u32, m: Report) {
        // if report is not for the current scope, please return
        if self.sc.program_uid != m.program_uid {
            return;
        }
        let now = std::time::Instant::now();
        match self.curr_mode {
            BbrMode::ProbeRtt => {
                self.min_rtt_us = self.get_probe_minrtt(&m);
                self.min_rtt_timeout = now + self.probe_rtt_interval;

                self.sc = self.install_probe_bw();
                self.curr_mode = BbrMode::ProbeBw;

                info!(min_rtt_us = self.min_rtt_us, "PROBE_RTT");
            }
            BbrMode::ProbeBw => {
                let fields = self.get_probe_bw_fields(&m);
                if fields.is_none() {
                    return;
                }

                let (_loss, minrtt, rate, _state) = fields.unwrap();
                let elapsed = now - self.start;
                info!(
                    elapsed_s = elapsed.as_secs_f32(),
                    rate_Mbps = rate / 125_000.0,
                    bottle_rate_Mbps = self.bottle_rate / 125_000.0,
                    "probe_bw"
                );

                // reset probe rtt counter and update cwnd cap
                if minrtt < self.min_rtt_us {
                    // datapath automatically uses minrtt for when condition (non volatile),
                    // this isn't reset, so no need to install again
                    self.min_rtt_us = minrtt;
                    self.min_rtt_timeout = now + self.probe_rtt_interval;
                    info!(
                        min_rtt_us = self.min_rtt_us,
                        bottle_rate_Mbps = self.bottle_rate / 125_000.0,
                        "new min_rtt"
                    );

                    if !(self.init) {
                        // probe bw program is installed
                        self.install_update(&[
                            (
                                "cwndCap",
                                (self.bottle_rate * 2.0 * f64::from(self.min_rtt_us) / 1e6) as u32,
                            ), // reinstall cwnd cap value
                        ]);
                    }
                }

                if now > self.min_rtt_timeout {
                    self.curr_mode = BbrMode::ProbeRtt;
                    info!(
                        min_rtt_us = self.min_rtt_us,
                        bottle_rate_Mbps = self.bottle_rate / 125_000.0,
                        "switching to PROBE_RTT"
                    );

                    self.min_rtt_us = 0x3fff_ffff;
                    self.sc = self.control_channel.set_program("probe_rtt", None).unwrap();
                    self.install_update(&[("Cwnd", (4 * self.mss) as u32)]);
                    return;
                }

                if self.bottle_rate < rate {
                    self.bottle_rate = rate;
                    self.bottle_rate_timeout = now + self.probe_rtt_interval;
                    // restart the pulse state
                    // here, we must reinstall the program for substitution with the correct values
                    if !(self.init) {
                        self.replace_probe_bw_rate();
                    }
                }

                if self.init {
                    info!("new_flow");
                    self.sc = self.install_probe_bw();
                    self.init = false;
                }
            }
        }
    }
}
