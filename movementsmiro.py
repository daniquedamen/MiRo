		# all easy movement options
		if key == "forever":
			self.forever = True
		elif key == "wheels":
			self.wheels = max_fwd_spd
		elif key == "wheels-":
			self.wheels = -max_fwd_spd
		elif key == "wheelsc":
			self.wheels = max_fwd_spd
			self.report_wheels = True
		elif key == "wheelsc-":
			self.wheels = -max_fwd_spd
			self.report_wheels = True
		elif key == "wheelsf":
			self.wheelsf = max_fwd_spd
			self.report_wheels = True
		elif key == "wheelsf-":
			self.wheelsf = -max_fwd_spd
			self.report_wheels = True
		elif input == "stall":
			self.wheels = 4.0
			self.report_wheels = True
		elif key == "stall-":
			self.wheels = -4.0
			self.report_wheels = True
		elif key == "spin":
			self.spin = 1.0
		elif key == "spin-":
			self.spin = -1.0
		elif key == "push":
			self.push = True
		elif key == "kin":
			self.kin = "lyp"
		elif key == "lift":
			self.kin = "l"
		elif key == "yaw":
			self.kin = "y"
		elif key == "pitch":
			self.kin = "p"
		elif key == "cos":
			self.cos = "hw"
		elif key == "cosl":
			self.cos = "l"
		elif key == "cosr":
			self.cos = "r"
		elif key == "eyes":
			self.cos = "y"
		elif key == "ears":
			self.cos = "e"
		elif key == "wag":
			self.cos = "w"
		elif key == "droop":
			self.cos = "d"
		elif key == "wagdroop":
			self.cos = "x"
		elif key == "illum":
			self.illum = True
		elif key == "workout":
			self.cos = "lrx"
			self.kin = "lyp"