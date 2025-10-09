use ccp_bbr::BbrConfig;
use clap::Arg;
use tracing::{info, warn};

fn make_args() -> Result<(BbrConfig, String), String> {
    let matches = clap::Command::new("CCP BBR")
        .version("0.3.0")
        .author("Akshay Narayan <akshayn@brown.edu>")
        .about("Implementation of BBR Congestion Control")
        .arg(Arg::new("ipc")
             .long("ipc")
             .help("Sets the type of ipc to use: (netlink|unix)")
             .default_value("unix")
             .value_parser(|s: &str| portus::algs::ipc_valid(s.to_owned())))
        .arg(Arg::new("probe_rtt_interval")
             .long("probe_rtt_interval")
             .help("Sets the BBR probe RTT interval in seconds, after which BBR drops its congestion window to potentially observe a new minimum RTT."))
        .get_matches();

    let probe_rtt_interval_arg = std::time::Duration::from_secs(
        match matches
            .get_one::<String>("probe_rtt_interval")
            .map(|v| u64::from_str_radix(v, 10).map_err(|e| format!("{:?}", e)))
        {
            Some(Ok(x)) if x > 0 => x,
            Some(Ok(i)) => return Err(format!("probe_rtt_interval must be positive: {}", i)),
            Some(Err(e)) => return Err(e),
            None => ccp_bbr::PROBE_RTT_INTERVAL_SECONDS as u64,
        },
    );

    Ok((
        BbrConfig {
            probe_rtt_interval: probe_rtt_interval_arg,
        },
        matches.get_one::<String>("ipc").unwrap().clone(),
    ))
}

fn main() {
    tracing_subscriber::fmt::init();
    let (cfg, ipc) = make_args()
        .map_err(|e| warn!(err = ?e, "bad argument"))
        .unwrap();

    info!(?ipc, probe_rtt_interval = ?cfg.probe_rtt_interval, "configured BBR");
    portus::start!(ipc.as_str(), cfg).unwrap()
}
