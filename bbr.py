import sys
import portus
import time

class BBR(portus.AlgBase):
	PROBE_BW  = "probebw"
	PROBE_RTT = "probertt"
	STARTUP   = "startup"
	PROBE_RTT_INTERVAL = 10


	def on_create(self):
		sys.stdout.write("on_create()\n")

		self.bottle_rate         = 125000
		self.bottle_rate_timeout = time.time() + BBR.PROBE_RTT_INTERVAL
		self.min_rtt_us          = 1000000
		self.min_rtt_timeout     = time.time() + BBR.PROBE_RTT_INTERVAL

		self.set_mode(BBR.STARTUP)


	def update_rate(self):
		self.datapath.update_fields([
			("bottleRate", int(self.bottle_rate)),
			("threeFourthsRate", int(self.bottle_rate * 0.75)),
			("fiveFourthsRate", int(self.bottle_rate * 1.25)),
			("cwndCap", int(self.bottle_rate * 2 * (self.min_rtt_us / 1e6)))
		])
		

	def set_mode(self, new_mode):
		sys.stdout.write("> switching to {}\n".format(new_mode))
		self.curr_mode = new_mode

		if self.curr_mode == BBR.STARTUP:
			self.datapath.install("""
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
			""")

		elif self.curr_mode == BBR.PROBE_RTT:
			self.datapath.update_field("Cwnd", int((4 * self.datapath_info.mss)))
			self.datapath.install("""
                (def
                    (Report (volatile minrtt +infinity))
                )
                (when true
                    (:= Report.minrtt (min Report.minrtt Flow.rtt_sample_us))
                    (fallthrough)
                )
                (when (> Micros 2000000)
                    (report)
                )
                (when (< Flow.packets_in_flight 4)
                    (report)
                )
			""")

		elif self.curr_mode == BBR.PROBE_BW:
			self.datapath.install("""
                (def
                    (Report 
                        (volatile minrtt +infinity)
                        (volatile loss 0)
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
			""", [
					("Report.minrtt", self.min_rtt_us),
					("cwndCap", int(self.bottle_rate * 2 * (self.min_rtt_us / 1e6))),
					("bottleRate", int(self.bottle_rate)),
					("threeFourthsRate", int(self.bottle_rate * 0.75)),
					("fiveFourthsRate", int(self.bottle_rate * 1.25))
			])
		sys.stdout.write("! installed program\n")


	def on_report(self, r):
                sys.stdout.write("on_report()\n")
		if self.curr_mode == BBR.PROBE_BW or self.curr_mode == BBR.STARTUP:
			loss, minrtt, rate, state = r.loss, r.minrtt, r.rate, r.pulseState
			sys.stdout.write("[probe_bw] loss={} min_rtt={} rate={} setRate={}, state={}\n".format(loss, minrtt, rate / 125000.0, self.bottle_rate / 125000.0, state))

			if minrtt < self.min_rtt_us:
				self.min_rtt_us = minrtt
				self.min_rtt_timeout = time.time() + BBR.PROBE_RTT_INTERVAL

			if time.time() > self.min_rtt_timeout:
				self.min_rtt_us = 0x3fffffff
				self.set_mode(BBR.PROBE_RTT)

			if self.bottle_rate < rate:
				self.bottle_rate = rate
				self.bottle_rate_timeout = time.time() + BBR.PROBE_RTT_INTERVAL
				if self.curr_mode != BBR.STARTUP:
					self.update_rate()

			if self.curr_mode == BBR.STARTUP:
				self.set_mode(BBR.PROBE_BW)

		elif self.curr_mode == BBR.PROBE_RTT:
			self.min_rtt_us = r.minrtt
			sys.stdout.write("[probe_rtt] minrtt={}\n".format(self.min_rtt_us))

			if time.time() > self.bottle_rate_timeout:
				self.bottle_rate_timeout = time.time() + BBR.PROBE_RTT_INTERVAL
				self.bottle_rate = 125000

			self.set_mode(BBR.PROBE_BW)


portus.connect("netlink", BBR, debug=False, blocking=True)
